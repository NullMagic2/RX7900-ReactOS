/*
Module: rx7900.tests.user_queue_fixture
Purpose: Bring MES up over the process-queue model and provide UserQueues with GTT, page-table, clock and doorbell services.
Created: 2026-10-22
Architecture: mes_process_model.hpp engines and the shared synthetic Navi31 machine;
shared by the user-queue and scheduler tests. It is not hardware evidence.
*/
#pragma once

#include <memory>
#include <vector>

#include "mes_process_model.hpp"
#include "user_queue.hpp"

namespace rx7900_test::user_queue_fixture {

using namespace rx7900::hardware;
using namespace rx7900_test::process_model;
using rx7900_test::mes_model::gp3_lo;
using rx7900_test::mes_model::kiq_uc;
using rx7900_test::mes_model::sched_select;
using rx7900_test::mes_model::sched_uc;

constexpr GpuVmMapFlags read_write{rx7900::memory::Access::read_write, false, GpuVmMtype::nc, false};
constexpr GpuVmMapFlags read_only{rx7900::memory::Access::read, true, GpuVmMtype::nc, false};

struct Fixture final {
    MesModel model;
    rx7900_test::VramBackend vram_backend{};
    GttBackend gtt_backend{};
    ProcessEngines engines{};
    std::unique_ptr<rx7900::memory::Manager> vram = std::make_unique<rx7900::memory::Manager>();
    std::unique_ptr<rx7900::memory::Manager> gtt = std::make_unique<rx7900::memory::Manager>();
    Nbio nbio{};
    NbioRegisters nbio_registers{};
    Mes11Registers registers{};
    std::unique_ptr<Mes11> mes = std::make_unique<Mes11>();
    Mes11Memory memory{};
    std::unique_ptr<Gmc11> gmc = std::make_unique<Gmc11>();
    Ih6Registers osssys{};
    std::unique_ptr<GpuVmInvalidator> invalidator = std::make_unique<GpuVmInvalidator>();
    std::unique_ptr<PasidAllocator> pasids = std::make_unique<PasidAllocator>();
    std::unique_ptr<UserQueues> queues = std::make_unique<UserQueues>();
    GartLayout gart{};
    uint64_t now = 1000;
    bool reset_confirmed = false;
    bool fail_user_doorbell = false;
    bool fail_wptr_store = false;
    unsigned wptr_stores = 0;

    static bool confirm(void* context) noexcept { return *static_cast<bool*>(context); }
    static uint64_t clock(void* context) noexcept { return static_cast<Fixture*>(context)->now; }

    static bool store(void* context, uint8_t* address, uint64_t value) noexcept {
        auto& self = *static_cast<Fixture*>(context);

        if (self.fail_wptr_store) {
            return false;
        }

        ++self.wptr_stores;
        return rx7900::sdma::CommandBuffer(address, 8).write_u32(0, static_cast<uint32_t>(value)) &&
               rx7900::sdma::CommandBuffer(address, 8).write_u32(4, static_cast<uint32_t>(value >> 32));
    }

    static bool doorbell64(void* context, uint32_t index, uint64_t value) noexcept {
        auto& self = *static_cast<Fixture*>(context);

        if (self.fail_user_doorbell && index >= aggregated_base) {
            return false;
        }

        return ProcessEngines::doorbell64(&self.engines, index, value);
    }

    static bool doorbell32(void* context, uint32_t index, uint32_t value) noexcept {
        return ProcessEngines::doorbell32(&static_cast<Fixture*>(context)->engines, index, value);
    }

    // gtt_size: GART-backed system memory of the model (default gtt_bytes);
    // larger only for tests of big process buffers.
    explicit Fixture(uint32_t gp3 = 0x00001063u, uint32_t api_timeout = 2100000, uint64_t gtt_size = gtt_bytes) {
        if (gtt_size > gtt_backend.pool.size()) {
            gtt_backend.pool.assign(static_cast<size_t>(gtt_size), 0x6b);
        }
        model.bank[{sched_select, gp3_lo}] = gp3;
        model.device->reg(context15_cntl) = vm_cntx_cntl;
        model.device->reg(sdma_ib_cntl) = 0x00000100u;
        vram_backend.device = model.device;
        vram->initialize({rx7900_test::fb_base + rx7900_test::arena_offset, rx7900_test::arena_bytes},
                         vram_backend.backend());
        compute_gart_layout(rx7900_test::navi31_vram(), gart);
        gtt_backend.arena = gart.start;
        gtt->initialize({gart.start, gtt_backend.pool.size()}, gtt_backend.backend());
        engines.model = &model;
        engines.gtt = &gtt_backend;
        const auto binary = rx7900_test::navi31_discovery();
        validate_nbio_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                nbio_registers);
        validate_mes11_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                 registers);
        validate_ih6_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, osssys);
        nbio.initialize(nbio_registers, model.device->io());
        nbio.program_common();
        allocate(*vram, mes11_kiq_ring_bytes, memory.kiq_ring);
        allocate(*vram, mes11_scheduler_ring_bytes, memory.scheduler_ring);
        allocate(*vram, mes11_mqd_bytes, memory.kiq_mqd);
        allocate(*vram, mes11_mqd_bytes, memory.scheduler_mqd);
        allocate(*vram, mes11_eop_bytes, memory.kiq_eop);
        allocate(*vram, mes11_eop_bytes, memory.scheduler_eop);
        allocate(*vram, mes11_writeback_bytes, memory.writeback);
        mes->initialize(registers, model.io(), {this, 0x10000, doorbell32, doorbell64}, nbio,
                        rx7900_test::navi31_vram(), {sched_uc, kiq_uc, 2}, {1000, api_timeout},
                        {&reset_confirmed, confirm});
        invalidator->initialize(*gmc, osssys, model.io(), {nullptr, nullptr}, false);
    }

    UserQueueConfig config() {
        return {mes.get(), pasids.get(), invalidator.get(), &nbio, vram.get(), gtt.get(),
                rx7900_test::navi31_vram(), gart, nbio_registers,
                {this, 0x10000, doorbell32, doorbell64}, {this, store}, {this, clock}};
    }

    bool up() {
        return mes->kiq_hw_init(*vram, memory) == Mes11Status::scheduler_ready &&
               queues->initialize(config()) == UserQueueStatus::ready;
    }

    rx7900::memory::Handle allocate(rx7900::memory::Manager& manager, uint64_t bytes,
                                    rx7900::memory::Handle& handle) {
        manager.allocate((bytes + 0xfffu) & ~uint64_t{0xfff}, 0x1000, rx7900::memory::Access::read_write,
                         true, handle);
        return handle;
    }

    rx7900::memory::Handle gtt_buffer(uint64_t bytes) {
        rx7900::memory::Handle handle{};
        return allocate(*gtt, bytes, handle);
    }

    rx7900::memory::AllocationInfo info(rx7900::memory::Manager& manager, rx7900::memory::Handle handle) {
        rx7900::memory::AllocationInfo result{};
        manager.describe(handle, result);
        return result;
    }

    uint64_t gart_of(rx7900::memory::Handle handle) { return info(*gtt, handle).gpu.virtual_address; }
    uint8_t* cpu_of(rx7900::memory::Handle handle) { return info(*gtt, handle).backing.cpu_bytes; }

    // The DMA bus pages of a GTT allocation (bus_base + arena offset), LE 64-bit.
    std::vector<uint8_t> dma(rx7900::memory::Handle handle) {
        const auto allocation = info(*gtt, handle);
        std::vector<uint8_t> pages(allocation.gpu.byte_length / 4096 * 8, 0);

        for (uint64_t page = 0; page < allocation.gpu.byte_length / 4096; ++page) {
            const uint64_t bus = bus_base + (allocation.gpu.virtual_address - gart.start) + page * 4096;
            rx7900::sdma::CommandBuffer out(pages.data(), pages.size());
            out.write_u32(page * 8, static_cast<uint32_t>(bus));
            out.write_u32(page * 8 + 4, static_cast<uint32_t>(bus >> 32));
        }

        return pages;
    }

    struct Process final {
        uint32_t pasid = 0;
        rx7900::memory::Handle tables{};
        rx7900::memory::Handle context{};
        rx7900::memory::Handle backup{};
        UserQueueStatus status = UserQueueStatus::unavailable;
    };

    Process process(uint32_t tables = 16, bool recoverable = false) {
        Process result{};
        allocate(*vram, uint64_t{tables} * 4096, result.tables);
        result.context = gtt_buffer(mes11_process_context_bytes);
        if (recoverable) {
            result.backup = gtt_buffer(uint64_t{tables} * gpuvm_table_bytes);
        }
        result.status = queues->create_process({result.tables, tables, result.context, result.backup},
                                               result.pasid);
        return result;
    }

    struct Gang final {
        uint32_t id = 0;
        rx7900::memory::Handle context{};
        UserQueueStatus status = UserQueueStatus::unavailable;
    };

    Gang gang(uint32_t pasid, uint32_t global = mes11_priority_normal,
              uint32_t inprocess = mes11_priority_normal, uint64_t quantum = 0) {
        Gang result{};
        result.context = gtt_buffer(mes11_gang_context_bytes);
        result.status = queues->add_gang(pasid, {global, inprocess, quantum, result.context}, result.id);
        return result;
    }

    struct Queue final {
        uint32_t id = 0;
        rx7900::memory::Handle buffer{};
        rx7900::memory::Handle mqd{};
        std::vector<uint8_t> pages;
        UserQueueStatus status = UserQueueStatus::unavailable;
    };

    Queue queue(uint32_t gang_id, Mes11QueueType type, uint64_t va, uint32_t ring = 8192,
                uint32_t timeout_ms = 0) {
        Queue result{};
        result.buffer = gtt_buffer(uint64_t{ring} + user_queue_control_bytes);
        result.mqd = gtt_buffer(type == Mes11QueueType::sdma ? mes11_sdma_mqd_bytes : mes11_mqd_bytes);
        result.pages = dma(result.buffer);
        result.status = queues->add_queue(gang_id, {type, ring, va, result.buffer,
            {result.pages.data(), result.pages.size()}, result.mqd, timeout_ms}, result.id);
        return result;
    }

    // A GTT data buffer mapped into the process at va (IBs and targets).
    struct Mapping final {
        rx7900::memory::Handle buffer{};
        std::vector<uint8_t> pages;
        GpuVmStatus status = GpuVmStatus::unavailable;
    };

    Mapping map(uint32_t pasid, uint64_t va, uint64_t bytes, GpuVmMapFlags flags = read_write) {
        Mapping result{};
        result.buffer = gtt_buffer(bytes);
        result.pages = dma(result.buffer);
        result.status = queues->map_system(pasid, va, {result.pages.data(), result.pages.size()}, true,
                                           flags);
        return result;
    }

    std::vector<std::vector<uint32_t>> frames(uint32_t opcode) {
        std::vector<std::vector<uint32_t>> result;

        for (const auto& frame : model.mes_frames) {
            if (((frame.at(0) >> 4) & 0xff) == opcode && (opcode != 2 || (frame.at(36) & (1u << 13)) == 0)) {
                result.push_back(frame);
            }
        }

        return result;
    }
};

// gfx/compute IB: WRITE_DATA of 0xDEADBEEF to target (gfx11_write_test_ib).
inline bool write_gfx_ib(uint8_t* cpu, uint64_t target) {
    return gfx11_write_test_ib(rx7900::sdma::CommandBuffer(cpu, 64), target);
}

// SDMA IB: WRITE_LINEAR of value to target, three NOPs (8 dwords).
inline void write_sdma_ib(uint8_t* cpu, uint64_t target, uint32_t value) {
    put(cpu, 0, 2);
    put(cpu, 4, static_cast<uint32_t>(target));
    put(cpu, 8, static_cast<uint32_t>(target >> 32));
    put(cpu, 12, 0);
    put(cpu, 16, value);
    put(cpu, 20, 0);
    put(cpu, 24, 0);
    put(cpu, 28, 0);
}

} // namespace rx7900_test::user_queue_fixture

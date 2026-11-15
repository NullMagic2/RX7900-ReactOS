/*
Module: rx7900.tests.mes_selftest
Purpose: Verify the amdgpu_mes_self_test VM, MQDs, ADD/REMOVE_QUEUE frames, ring/IB packets, faults and quarantine.
Created: 2026-10-21
Architecture: The shared MES/KIQ model (mes_test_model.hpp) brings MES up; a
test-local GTT backend and user-queue engine model walk the 4-level process page
tables from VRAM and execute gfx/compute PM4 and SDMA packets at process VAs. It
is not hardware evidence. Expected values are hand-derived from Linux v6.12
amdgpu_mes.c, mes_v11_0.c, mes_v11_api_def.h, amdgpu_mes_ctx.h, amdgpu_vm*.c,
gmc_v11_0.c, gfx_v11_0.c, sdma_v6_0.c, v11_structs.h and gc_11_0_0 headers.
*/
#include <map>
#include <memory>
#include <vector>

#include "mes_test_model.hpp"
#include "mes_selftest_vm.hpp"

namespace {

using namespace rx7900::hardware;
using namespace rx7900_test::mes_model;
using rx7900::ByteView;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

constexpr uint64_t gtt_bytes = 4u << 20;
constexpr uint64_t bus_base = 0x0000003000000000ULL;
constexpr uint32_t context15_cntl = gc0 + 0x1697;
constexpr uint32_t sdma_ib_cntl = gc0 + 0x008a;
constexpr uint32_t vm_cntx_cntl = 0x01fffe07u;
constexpr uint32_t aggregated_base = 0x800;
constexpr uint64_t meta_va = 0x10000;

uint32_t u32(const uint8_t* bytes, uint64_t offset) {
    uint32_t value = 0;
    ByteView(bytes + offset, 4).read_u32(0, value);
    return value;
}

uint64_t u64(const uint8_t* bytes, uint64_t offset) {
    return (static_cast<uint64_t>(u32(bytes, offset + 4)) << 32) | u32(bytes, offset);
}

void put(uint8_t* bytes, uint64_t offset, uint32_t value) {
    rx7900::sdma::CommandBuffer(bytes + offset, 4).write_u32(0, value);
}

// GART-mapped system memory: the GPU address is arena + offset, the CPU bytes
// are pool + offset and, unless remapped by a test, the DMA bus address is
// bus_base + offset.
struct GttBackend final {
    uint64_t arena = 0;
    std::vector<uint8_t> pool = std::vector<uint8_t>(gtt_bytes, 0x6b);

    static rx7900::memory::CreateResult create(void* context, rx7900::sdma::GpuAllocation gpu,
        rx7900::memory::Access, bool, rx7900::memory::Backing& backing) noexcept {
        auto& self = *static_cast<GttBackend*>(context);
        const uint64_t offset = gpu.virtual_address - self.arena;

        if (offset + gpu.byte_length > self.pool.size()) {
            return rx7900::memory::CreateResult::no_resources;
        }

        backing = {self.pool.data() + offset, self.pool.data() + offset,
                   static_cast<size_t>(gpu.byte_length)};
        return rx7900::memory::CreateResult::created;
    }

    static bool destroy(void*, const rx7900::memory::Backing&, rx7900::sdma::GpuAllocation) noexcept {
        return true;
    }

    rx7900::memory::Backend backend() noexcept { return {this, create, destroy}; }

    uint8_t* cpu(uint64_t gart) {
        return gart >= arena && gart - arena < pool.size() ? pool.data() + (gart - arena) : nullptr;
    }
};

// MES user-queue engines: on a user doorbell, the queue's ADD_QUEUE frame gives
// the MQD (GART) and page table base; rings, IBs and every target are process
// VAs translated through PDB2/PDB1/PDB0/PTB in VRAM and system PTEs.
struct UserEngines final {
    MesModel* model = nullptr;
    GttBackend* gtt = nullptr;
    std::map<uint64_t, uint8_t*> bus_pages;
    std::map<uint32_t, uint64_t> rptr;
    uint32_t hang_type = UINT32_MAX;
    uint32_t corrupt_ib_type = UINT32_MAX;
    uint32_t no_fence_type = UINT32_MAX;
    std::vector<uint32_t> fence_contexts;
    std::vector<uint32_t> ib_controls;
    std::vector<uint32_t> doorbells;
    std::vector<uint64_t> doorbell_values;
    unsigned translation_faults = 0;

    uint8_t* translate(uint64_t root_pa, uint64_t va, bool write) {
        const auto pa_to_cpu = [&](uint64_t pa) {
            return model->device->cpu(pa - (0x20ULL << 24) + rx7900_test::fb_base);
        };
        uint64_t table = root_pa;
        const uint32_t shifts[4]{39, 30, 21, 12};

        for (uint32_t level = 0; level < 4; ++level) {
            uint8_t* bytes = pa_to_cpu(table);

            if (bytes == nullptr) {
                ++translation_faults;
                return nullptr;
            }

            const uint64_t entry = u64(bytes, ((va >> shifts[level]) & 511) * 8);

            if ((entry & 1) == 0 || (level < 3 && (entry & (1ULL << 54)) != 0)) {
                ++translation_faults;
                return nullptr;
            }

            if (level == 3) {
                const bool system = (entry & 2) != 0;
                const bool allowed = (entry & 0x20) != 0 && (!write || (entry & 0x40) != 0);
                const auto page = bus_pages.find(entry & 0x0000fffffffff000ULL);

                if (!system || !allowed || page == bus_pages.end()) {
                    ++translation_faults;
                    return nullptr;
                }

                return page->second + (va & 0xfff);
            }

            table = entry & 0x0000fffffffff000ULL;
        }

        return nullptr;
    }

    uint32_t read(uint64_t root, uint64_t va) {
        uint8_t* bytes = translate(root, va, false);
        return bytes == nullptr ? 0 : u32(bytes, 0);
    }

    void store(uint64_t root, uint64_t va, uint32_t value) {
        uint8_t* bytes = translate(root, va, true);

        if (bytes != nullptr) {
            put(bytes, 0, value);
        }
    }

    // The newest ADD_QUEUE (non-legacy) or REMOVE_QUEUE frame for doorbell.
    const std::vector<uint32_t>* active(uint32_t doorbell) {
        const std::vector<uint32_t>* found = nullptr;

        for (const auto& frame : model->mes_frames) {
            const uint32_t opcode = (frame.at(0) >> 4) & 0xff;

            if (opcode == 2 && frame.at(18) == doorbell && (frame.at(36) & (1u << 13)) == 0) {
                found = &frame;
            } else if (opcode == 3 && frame.at(1) == doorbell) {
                found = nullptr;
            }
        }

        return found;
    }

    void run_pm4_ib(uint64_t root, uint64_t ib, uint32_t dwords, uint32_t type) {
        for (uint32_t position = 0; position < dwords;) {
            const uint32_t header = read(root, ib + position * 4);
            const uint32_t count = ((header >> 16) & 0x3fff) + 1;

            if (((header >> 8) & 0xff) == 0x37) {
                const uint64_t target = (static_cast<uint64_t>(read(root, ib + (position + 3) * 4)) << 32) |
                                        read(root, ib + (position + 2) * 4);
                store(root, target, type == corrupt_ib_type ? 0x0badf00du :
                                                              read(root, ib + (position + 4) * 4));
            }

            position += count + 1;
        }
    }

    void run_pm4(uint32_t doorbell, uint64_t root, const uint8_t* mqd, uint32_t type, uint64_t wptr) {
        const uint64_t ring = ((static_cast<uint64_t>(u32(mqd, 548)) << 32) | u32(mqd, 544)) << 8;
        const uint32_t dwords = 2u << (u32(mqd, 580) & 0x3f);
        uint64_t& position = rptr[doorbell];

        while (position < wptr) {
            const auto dw = [&](uint64_t index) {
                return read(root, ring + ((position + index) & (dwords - 1)) * 4);
            };
            const uint32_t header = dw(0);

            if (header == kiq_nop) {
                ++position;
                continue;
            }

            const uint32_t count = ((header >> 16) & 0x3fff) + 1;
            const uint32_t opcode = (header >> 8) & 0xff;

            if (opcode == 0x79) {
                model->device->reg(0xc000 + dw(1)) = dw(2);
            } else if (opcode == 0x3f) {
                ib_controls.push_back(dw(3));
                run_pm4_ib(root, (static_cast<uint64_t>(dw(2)) << 32) | dw(1), dw(3) & 0xfffff, type);
            } else if (opcode == 0x49 && type != no_fence_type) {
                fence_contexts.push_back(dw(7));
                store(root, (static_cast<uint64_t>(dw(4)) << 32) | dw(3), dw(5));
            }

            position += count + 1;
        }

        store(root, (static_cast<uint64_t>(u32(mqd, 560)) << 32) | u32(mqd, 556),
              static_cast<uint32_t>(position));
    }

    void run_sdma_ib(uint64_t root, uint64_t ib, uint32_t dwords) {
        for (uint32_t position = 0; position < dwords;) {
            const uint32_t header = read(root, ib + position * 4);

            if ((header & 0xff) == 2) {
                const uint64_t target = (static_cast<uint64_t>(read(root, ib + (position + 2) * 4)) << 32) |
                                        read(root, ib + (position + 1) * 4);
                store(root, target, corrupt_ib_type == 2 ? 0x0badf00du :
                                                           read(root, ib + (position + 4) * 4));
                position += 5;
            } else {
                ++position;
            }
        }
    }

    void run_sdma(uint32_t doorbell, uint64_t root, const uint8_t* mqd, uint64_t wptr_bytes) {
        const uint64_t ring = ((static_cast<uint64_t>(u32(mqd, 8)) << 32) | u32(mqd, 4)) << 8;
        const uint32_t dwords = 1u << ((u32(mqd, 0) >> 1) & 0x1f);
        uint64_t& position = rptr[doorbell];

        while (position < wptr_bytes / 4) {
            const auto dw = [&](uint64_t index) {
                return read(root, ring + ((position + index) & (dwords - 1)) * 4);
            };
            const uint32_t op = dw(0) & 0xff;
            uint32_t length = 1;

            if (op == 2) {
                const uint32_t count = dw(3) + 1;
                const uint64_t target = (static_cast<uint64_t>(dw(2)) << 32) | dw(1);

                for (uint32_t index = 0; index < count; ++index) {
                    store(root, target + index * 4, dw(4 + index));
                }

                length = 4 + count;
            } else if (op == 4) {
                ib_controls.push_back(dw(3));
                run_sdma_ib(root, (static_cast<uint64_t>(dw(2)) << 32) | dw(1), dw(3));
                length = 6;
            } else if (op == 5) {
                if (no_fence_type != 2) {
                    store(root, (static_cast<uint64_t>(dw(2)) << 32) | dw(1), dw(3));
                }

                length = 4;
            } else if (op == 6) {
                fence_contexts.push_back(dw(1));
                length = 2;
            } else if (op == 8) {
                length = 6;
            } else if (op == 9) {
                length = 5;
            }

            position += length;
        }

        const uint64_t rptr_va = (static_cast<uint64_t>(u32(mqd, 28)) << 32) | u32(mqd, 32);
        store(root, rptr_va, static_cast<uint32_t>(position * 4));
    }

    bool doorbell(uint32_t index, uint64_t value) {
        const std::vector<uint32_t>* frame = active(index);

        if (frame == nullptr) {
            return false;
        }

        doorbells.push_back(index);
        doorbell_values.push_back(value);
        const uint32_t type = frame->at(27);
        const uint64_t root = (static_cast<uint64_t>(frame->at(3)) << 32) | frame->at(2);
        const uint8_t* mqd = gtt->cpu((static_cast<uint64_t>(frame->at(20)) << 32) | frame->at(19));

        if (mqd == nullptr || type == hang_type) {
            return true;
        }

        if (type == 2) {
            if ((u32(mqd, 76) >> 2) == index && (u32(mqd, 68) & (1u << 28)) != 0) {
                run_sdma(index, root, mqd, value);
            }
        } else if (((u32(mqd, 572) & 0x0ffffffcu) >> 2) == index && (u32(mqd, 572) & 0x40000000u) != 0) {
            run_pm4(index, root, mqd, type, value);
        }

        return true;
    }
};

struct Fixture final {
    MesModel model;
    rx7900_test::VramBackend vram_backend{};
    GttBackend gtt_backend{};
    UserEngines engines{};
    std::unique_ptr<rx7900::memory::Manager> vram = std::make_unique<rx7900::memory::Manager>();
    std::unique_ptr<rx7900::memory::Manager> gtt = std::make_unique<rx7900::memory::Manager>();
    Nbio nbio{};
    NbioRegisters nbio_registers{};
    Mes11Registers registers{};
    std::unique_ptr<Mes11> mes = std::make_unique<Mes11>();
    Mes11Memory memory{};
    MesSelfTestVmMemory tables{};
    Mes11SelfTestMemory queues{};
    rx7900::memory::Handle metadata{};
    std::vector<uint8_t> dma;
    GartLayout gart{};
    bool reset_confirmed = false;
    std::unique_ptr<MesSelfTestVm> vm = std::make_unique<MesSelfTestVm>();

    static bool confirm(void* context) noexcept { return *static_cast<bool*>(context); }

    static bool doorbell64(void* context, uint32_t index, uint64_t value) noexcept {
        auto& self = *static_cast<Fixture*>(context);

        if (index >= aggregated_base && self.engines.doorbell(index, value)) {
            return true;
        }

        return MesModel::doorbell64(&self.model, index, value);
    }

    static bool doorbell32(void* context, uint32_t index, uint32_t value) noexcept {
        return MesModel::doorbell32(&static_cast<Fixture*>(context)->model, index, value);
    }

    explicit Fixture(uint32_t gp3 = 0x00001050u) {
        model.bank[{sched_select, gp3_lo}] = gp3;
        model.device->reg(context15_cntl) = vm_cntx_cntl;
        model.device->reg(sdma_ib_cntl) = 0x00000100u;
        vram_backend.device = model.device;
        vram->initialize({rx7900_test::fb_base + rx7900_test::arena_offset, rx7900_test::arena_bytes},
                         vram_backend.backend());
        compute_gart_layout(rx7900_test::navi31_vram(), gart);
        gtt_backend.arena = gart.start;
        gtt->initialize({gart.start, gtt_bytes}, gtt_backend.backend());
        engines.model = &model;
        engines.gtt = &gtt_backend;
        const auto binary = rx7900_test::navi31_discovery();
        validate_nbio_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                nbio_registers);
        validate_mes11_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                 registers);
        nbio.initialize(nbio_registers, model.device->io());
        nbio.program_common();
        allocate(*vram, mes11_kiq_ring_bytes, memory.kiq_ring);
        allocate(*vram, mes11_scheduler_ring_bytes, memory.scheduler_ring);
        allocate(*vram, mes11_mqd_bytes, memory.kiq_mqd);
        allocate(*vram, mes11_mqd_bytes, memory.scheduler_mqd);
        allocate(*vram, mes11_eop_bytes, memory.kiq_eop);
        allocate(*vram, mes11_eop_bytes, memory.scheduler_eop);
        allocate(*vram, mes11_writeback_bytes, memory.writeback);

        for (auto& table : tables.tables) {
            allocate(*vram, 4096, table);
        }

        allocate(*gtt, mes11_ctx::bytes, metadata);
        allocate(*gtt, mes11_process_context_bytes, queues.process_context);

        for (uint32_t index = 0; index < 3; ++index) {
            allocate(*gtt, mes11_gang_context_bytes, queues.gang_context[index]);
            allocate(*gtt, index == 2 ? mes11_sdma_mqd_bytes : mes11_mqd_bytes, queues.mqd[index]);
        }

        set_dma_pages(contiguous_pages());
        mes->initialize(registers, model.io(), {this, 0x10000, doorbell32, doorbell64}, nbio,
                        rx7900_test::navi31_vram(), {sched_uc, kiq_uc, 2}, {1000, 2100000},
                        {&reset_confirmed, confirm});
    }

    void allocate(rx7900::memory::Manager& manager, uint32_t bytes, rx7900::memory::Handle& handle) {
        manager.allocate((bytes + 0xfffu) & ~0xfffu, 0x1000, rx7900::memory::Access::read_write,
                         true, handle);
    }

    rx7900::memory::AllocationInfo info(rx7900::memory::Manager& manager,
                                        rx7900::memory::Handle handle) {
        rx7900::memory::AllocationInfo result{};
        manager.describe(handle, result);
        return result;
    }

    uint64_t meta_gart() { return info(*gtt, metadata).gpu.virtual_address; }
    uint8_t* meta() { return info(*gtt, metadata).backing.cpu_bytes; }

    std::vector<uint64_t> contiguous_pages() {
        std::vector<uint64_t> pages;

        for (uint32_t page = 0; page < 52; ++page) {
            pages.push_back(bus_base + (meta_gart() - gart.start) + page * 4096ULL);
        }

        return pages;
    }

    void set_dma_pages(const std::vector<uint64_t>& pages) {
        dma.assign(pages.size() * 8, 0);
        engines.bus_pages.clear();

        for (size_t page = 0; page < pages.size(); ++page) {
            rx7900::sdma::CommandBuffer(dma.data(), dma.size()).write_u32(page * 8, static_cast<uint32_t>(pages[page]));
            rx7900::sdma::CommandBuffer(dma.data(), dma.size()).write_u32(page * 8 + 4, static_cast<uint32_t>(pages[page] >> 32));
            engines.bus_pages[pages[page]] = meta() + page * 4096;
        }
    }

    MesSelfTestInputs inputs(uint32_t timeout = mes11_self_test_ib_timeout_us) {
        return {vram.get(), tables, gtt.get(),
                {rx7900_test::navi31_vram(), gart, metadata, {dma.data(), dma.size()},
                 {&reset_confirmed, confirm}},
                queues, {nbio_registers, gart, timeout}, &nbio};
    }

    bool bring_up() { return mes->kiq_hw_init(*vram, memory) == Mes11Status::scheduler_ready; }

    uint64_t table(uint32_t level, uint32_t index) {
        return u64(info(*vram, tables.tables[level]).backing.cpu_bytes, index * 8);
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

    uint64_t gtt_address(rx7900::memory::Handle handle) { return info(*gtt, handle).gpu.virtual_address; }
    uint8_t* gtt_cpu(rx7900::memory::Handle handle) { return info(*gtt, handle).backing.cpu_bytes; }
};

uint64_t frame64(const std::vector<uint32_t>& frame, uint32_t byte) {
    return (static_cast<uint64_t>(frame.at(byte / 4 + 1)) << 32) | frame.at(byte / 4);
}

void success_path() {
    Fixture fixture;
    expect(fixture.bring_up(), "MES scheduler ready before the self-test");
    const uint64_t pt_base_expected = fixture.info(*fixture.vram, fixture.tables.tables[0]).gpu.virtual_address -
                                      rx7900_test::fb_base + (0x20ULL << 24);
    const auto outcome = run_mes_self_test(*fixture.mes, *fixture.vm, fixture.inputs());
    const Mes11SelfTestReport report = fixture.mes->self_test_report();
    expect(outcome.map_status == MesSelfTestVmStatus::mapped &&
           outcome.test_status == Mes11Status::completed &&
           outcome.teardown_status == MesSelfTestVmStatus::released,
           "self-test maps, passes every ring and IB test and tears down");
    expect(report.ran && report.queues_added == 3 && report.rings_passed == 3 &&
           report.ibs_passed == 3 && report.failed_queue == UINT32_MAX && !report.memory_retained &&
           !report.memory_quarantined && report.pasid == 0x8000 &&
           report.page_table_base_addr == pt_base_expected &&
           report.process_va_end == 0x0000fffffffff000ULL && report.vm_context_cntl == vm_cntx_cntl &&
           report.api_version == 1, "report: pasid 0x8000, PD physical base, VA end, API 1");
    expect(fixture.engines.translation_faults == 0, "every engine access translated through the process VM");

    // ADD_QUEUE frames, exact dwords.
    const auto adds = fixture.frames(2);
    const auto removes = fixture.frames(3);
    expect(adds.size() == 3 && removes.size() == 3, "three ADD_QUEUE and three REMOVE_QUEUE frames");
    const uint32_t doorbells[3]{aggregated_base + 10, aggregated_base + 12, aggregated_base + 512};
    bool add_ok = adds.size() == 3;

    for (uint32_t index = 0; add_ok && index < 3; ++index) {
        const auto& frame = adds[index];
        const uint64_t slots = meta_va + (index == 0 ? 86016 : index == 1 ? 108544 : 188480);
        add_ok = frame.at(0) == 0x00040021u && frame.at(1) == 0x8000 &&
            frame64(frame, 8) == pt_base_expected && frame64(frame, 16) == 0 &&
            frame64(frame, 24) == 0x0000fffffffff000ULL && frame64(frame, 32) == 0 &&
            frame64(frame, 40) == fixture.gtt_address(fixture.queues.process_context) &&
            frame64(frame, 48) == 0 &&
            frame64(frame, 56) == fixture.gtt_address(fixture.queues.gang_context[index]) &&
            frame.at(16) == 1 && frame.at(17) == 1 && frame.at(18) == doorbells[index] &&
            frame64(frame, 76) == fixture.gtt_address(fixture.queues.mqd[index]) &&
            frame64(frame, 84) == slots + 32 && frame64(frame, 92) == 0 && frame64(frame, 100) == 0 &&
            frame.at(27) == index && frame.at(29) == 0 && frame.at(35) == vm_cntx_cntl &&
            frame.at(36) == 0 && frame64(frame, 148) != 0 && frame64(frame, 156) == 1;

        for (uint32_t dword = 37; add_ok && dword < 64; ++dword) {
            add_ok = dword == 37 || dword == 38 || dword == 39 || dword == 40 || frame.at(dword) == 0;
        }

        for (uint32_t dword = 23; add_ok && dword < 35; ++dword) {
            add_ok = dword == 27 || frame.at(dword) == 0;
        }
    }

    expect(add_ok, "ADD_QUEUE: pid 0x8000, PT base, VA 0..0xfffffffff000, quanta 0, process/gang "
                   "contexts, priorities 1/1, MES doorbells 0x80a/0x80c/0xa00, MQD, VA wptr (API 1), "
                   "type 0/1/2, gds_size 0, vm_context_cntl, flags 0, rest zero");
    bool remove_ok = removes.size() == 3;

    for (uint32_t index = 0; remove_ok && index < 3; ++index) {
        const auto& frame = removes[index];
        remove_ok = frame.at(0) == 0x00040031u && frame.at(1) == doorbells[index] &&
            frame64(frame, 8) == fixture.gtt_address(fixture.queues.gang_context[index]) &&
            frame.at(4) == 0 && frame.at(9) == 0 && frame.at(10) == 0 && frame64(frame, 44) == 0 &&
            frame.at(13) == 0 && frame.at(14) == 0;
    }

    expect(remove_ok, "REMOVE_QUEUE gfx, compute, SDMA: doorbell and gang context only");
    // Order: three adds, then (tests), then three removes.
    std::vector<uint32_t> order;

    for (const auto& frame : fixture.model.mes_frames) {
        const uint32_t opcode = (frame.at(0) >> 4) & 0xff;

        if (opcode == 2 || opcode == 3) {
            order.push_back(opcode);
        }
    }

    expect(order == std::vector<uint32_t>{2, 2, 2, 3, 3, 3}, "adds precede removes");

    // MQDs (still readable: the backing outlives the released leases).
    const uint8_t* gfx = fixture.gtt_cpu(fixture.queues.mqd[0]);
    const uint8_t* compute = fixture.gtt_cpu(fixture.queues.mqd[1]);
    const uint8_t* sdma = fixture.gtt_cpu(fixture.queues.mqd[2]);
    const uint64_t gfx_mqd = fixture.gtt_address(fixture.queues.mqd[0]);
    const uint64_t compute_mqd = fixture.gtt_address(fixture.queues.mqd[1]);
    expect(u32(gfx, 512) == (static_cast<uint32_t>(gfx_mqd) & 0xfffffffcu) &&
           u32(gfx, 516) == static_cast<uint32_t>(gfx_mqd >> 32) && u32(gfx, 520) == 1 &&
           u32(gfx, 524) == 0 && u32(gfx, 536) == 0 && u32(gfx, 540) == 1 &&
           u32(gfx, 544) == (meta_va >> 8) && u32(gfx, 548) == 0 &&
           u32(gfx, 556) == meta_va + 86016 && u32(gfx, 560) == 0 &&
           u32(gfx, 564) == meta_va + 86048 && u32(gfx, 572) == ((doorbells[0] << 2) | 0x40000000u) &&
           u32(gfx, 580) == (10u | (8u << 8)) && u32(gfx, 648) == 0x100 && u32(gfx, 596) == 0,
           "gfx MQD: GART MQD base, VA ring/rptr/wptr, MES doorbell, bufsz 10, normal priority");
    expect(u32(compute, 0) == 0xC0310800u && u32(compute, 512) == static_cast<uint32_t>(compute_mqd) &&
           u32(compute, 520) == 0 && u32(compute, 532) == 1 && u32(compute, 536) == 0 &&
           u32(compute, 544) == ((meta_va + 90112) >> 8) &&
           u32(compute, 556) == meta_va + 108544 && u32(compute, 564) == meta_va + 108576 &&
           u32(compute, 572) == ((doorbells[1] << 2) | 0x40000000u) &&
           u32(compute, 660) == ((meta_va + 106496) >> 8) && (u32(compute, 580) & 0x2000003fu) == 10 &&
           (u32(compute, 580) & 0xd0000000u) == 0xd0000000u && u32(compute, 668) == 8,
           "compute MQD: MEC HPD VA EOP, PIPE_PRIO_NORMAL 1, inactive, no tunnelling, privileged KMD queue");
    expect(u32(sdma, 0) == 0x00041816u && u32(sdma, 4) == ((meta_va + 172032) >> 8) &&
           u32(sdma, 8) == 0 && u32(sdma, 32) == meta_va + 188480 && u32(sdma, 28) == 0 &&
           u32(sdma, 108) == meta_va + 188512 && u32(sdma, 104) == 0 && u32(sdma, 36) == 0x100 &&
           u32(sdma, 76) == (doorbells[2] << 2) && u32(sdma, 68) == 0x10000000u &&
           u32(sdma, 112) == 0x4000 && u32(sdma, 100) == 0xf && u32(sdma, 60) == 0 &&
           u32(sdma, 64) == 0 && u32(sdma, 72) == 0,
           "SDMA MQD: RB_CNTL 0x41816, VA base/rptr/wptr poll, IB_CNTL copied, doorbell, AQL/DUMMY defaults");

    // Ring contents in the metadata.
    const uint8_t* meta = fixture.meta();
    expect(u32(meta, 0) == 0xC0017900u && u32(meta, 4) == 0x40 && u32(meta, 8) == 0xDEADBEEFu &&
           u32(meta, 12) == 0xC0FB1000u && u32(meta, 255 * 4) == 0xffff1000u,
           "gfx ring test: SET_UCONFIG_REG SCRATCH_REG0 0xDEADBEEF, NOP packet pad to 256");
    const uint32_t cond_lo = static_cast<uint32_t>(meta_va + 86016 + 96);
    expect(u32(meta, 256 * 4) == 0xC0032200u && u32(meta, 257 * 4) == cond_lo &&
           u32(meta, 259 * 4) == 0 && u32(meta, 260 * 4) == 24 &&
           u32(meta, 261 * 4) == 0xC0053C00u && u32(meta, 262 * 4) == 0x143 &&
           u32(meta, 265 * 4) == 1 && u32(meta, 268 * 4) == 0xC0023F00u &&
           u32(meta, 269 * 4) == meta_va + 86272 && u32(meta, 271 * 4) == (5u | 0x00400000u) &&
           u32(meta, 272 * 4) == 0xC0032200u && u32(meta, 277 * 4) == 0xC0064900u &&
           u32(meta, 280 * 4) == meta_va + 86016 + 64 && u32(meta, 282 * 4) == 1 &&
           u32(meta, 284 * 4) == 0x01000001u,
           "gfx IB frame: no ACQUIRE_MEM, COND_EXEC count 24, HDP WAIT_REG_MEM PFP cp0, IB inherit "
           "vmid, second COND_EXEC, RELEASE_MEM seq 1 ctx queue 1 | MES flag");
    expect(u32(meta, 86272) == 0xC0033700u && u32(meta, 86276) == 0x00100500u &&
           u32(meta, 86280) == meta_va + 87296 && u32(meta, 86288) == 0xDEADBEEFu &&
           u32(meta, 87296) == 0xDEADBEEFu && u32(meta, 86016 + 64) == 1 && u32(meta, 86016 + 96) == 1,
           "gfx IB in ib[] writes the padding dword; fence 1; cond_exec 1");
    const uint32_t c = 90112;
    expect(u32(meta, c) == 0xC0017900u && u32(meta, c + 256 * 4) == 0xC0053C00u &&
           u32(meta, c + 257 * 4) == 0x43 && u32(meta, c + 260 * 4) == 4 &&
           u32(meta, c + 263 * 4) == 0xC0023F00u &&
           u32(meta, c + 266 * 4) == (5u | (1u << 23) | 0x40000000u) &&
           u32(meta, c + 267 * 4) == 0xC0064900u && u32(meta, c + 274 * 4) == 0x01000002u &&
           u32(meta, 109824) == 0xDEADBEEFu && u32(meta, 108544 + 64) == 1,
           "compute frames: ring test, HDP ME cp2, IB VALID|inherit vmid, RELEASE_MEM ctx queue 2");
    const uint32_t s = 172032;
    expect(u32(meta, s) == 2 && u32(meta, s + 4) == meta_va + 189696 && u32(meta, s + 12) == 0 &&
           u32(meta, s + 16) == 0xDEADBEEFu && u32(meta, s + 20) == 0 && u32(meta, s + 60) == 0,
           "SDMA ring test: WRITE_LINEAR count 0 to the padding, plain NOP pad to 16");
    expect(u32(meta, s + 64) == 9 && u32(meta, s + 68) == meta_va + 188480 + 96 &&
           u32(meta, s + 76) == 1 && u32(meta, s + 80) == 25 && u32(meta, s + 84) == 0x34000008u &&
           u32(meta, s + 88) == (rx7900_test::nbif_bases[2] + 0x107) * 4 &&
           u32(meta, s + 92) == (rx7900_test::nbif_bases[2] + 0x106) * 4 &&
           u32(meta, s + 96) == 0x400 && u32(meta, s + 104) == 0x0fff000au &&
           u32(meta, s + 108) == 0 && u32(meta, s + 132) == 0 && u32(meta, s + 136) == 4 &&
           u32(meta, s + 140) == ((meta_va + 188672) & 0xffffffe0u) && u32(meta, s + 148) == 8 &&
           u32(meta, s + 152) == 0 && u32(meta, s + 156) == 0 &&
           u32(meta, s + 160) == 0x00030005u && u32(meta, s + 164) == meta_va + 188480 + 64 &&
           u32(meta, s + 172) == 1 && u32(meta, s + 176) == 6 && u32(meta, s + 180) == 0x01000003u,
           "SDMA IB frame: COND_EXE count 25, HDP POLL_REGMEM DONE/REQ, 7 NOPs so INDIRECT ends at 40, "
           "INDIRECT vmid 0 len 8, FENCE UC seq 1, TRAP ctx queue 3 | MES flag");
    expect(u32(meta, 188672) == 2 && u32(meta, 188672 + 28) == 0 && u32(meta, 189696) == 0xDEADBEEFu,
           "SDMA IB: WRITE_LINEAR then three NOPs; padding written");
    expect(fixture.engines.doorbells.size() == 6 && fixture.engines.doorbells[0] == doorbells[0] &&
           fixture.engines.doorbell_values[0] == 256 && fixture.engines.doorbell_values[1] == 512 &&
           fixture.engines.doorbells[4] == doorbells[2] && fixture.engines.doorbell_values[4] == 64 &&
           fixture.engines.doorbell_values[5] == 192 && u64(meta, 188480 + 32) == 192,
           "doorbells: gfx/compute wptr in dwords, SDMA wptr << 2 in bytes, shadow matches");

    // Page tables after teardown: PDs intact, PTEs rewritten as unmapped.
    const uint64_t pdb1 = fixture.info(*fixture.vram, fixture.tables.tables[1]).gpu.virtual_address;
    const uint64_t pdb0 = fixture.info(*fixture.vram, fixture.tables.tables[2]).gpu.virtual_address;
    const uint64_t ptb = fixture.info(*fixture.vram, fixture.tables.tables[3]).gpu.virtual_address;
    const auto pa = [](uint64_t mc) { return mc - rx7900_test::fb_base + (0x20ULL << 24); };
    expect(fixture.table(0, 0) == (pa(pdb1) | 1) && fixture.table(1, 0) == (pa(pdb0) | 1) &&
           fixture.table(2, 0) == (pa(ptb) | 1) && fixture.table(0, 1) == (1ULL << 54) &&
           fixture.table(0, 511) == (1ULL << 54) && fixture.table(1, 7) == (1ULL << 54) &&
           fixture.table(2, 511) == (1ULL << 54) && fixture.table(3, 15) == 0x10 &&
           fixture.table(3, 68) == 0x10 && fixture.table(3, 511) == 0x10,
           "PDEs: VRAM PA | VALID; cleared PD entries PDE_PTE; cleared PTEs EXECUTABLE");
    bool unmapped = true;

    for (uint32_t pfn = 16; pfn < 68; ++pfn) {
        const uint64_t flags = pfn < 32 ? 0x210 : pfn < 64 ? 0x290 : 0x110;
        unmapped = unmapped && fixture.table(3, pfn) == ((static_cast<uint64_t>(pfn - 16) << 12) | flags);
    }

    expect(unmapped, "amdgpu_vm_clear_freed PTEs: address increments, FRAG 4/5/2, EXECUTABLE");
    expect(!fixture.mes->self_test_report().memory_retained && !fixture.vm->report().memory_retained &&
           fixture.mes->report().scheduler_ready, "every self-test lease returned; MES still ready");
    expect(fixture.mes->self_test(*fixture.gtt, fixture.queues, fixture.vm->process_vm(),
                                  fixture.inputs().config) == Mes11Status::busy,
           "the self-test runs once per bring-up");
    expect(fixture.mes->kiq_hw_fini() == Mes11Status::disabled &&
           fixture.mes->release() == Mes11Status::released, "MES teardown unaffected");
}

void mapping_entries() {
    // Contiguous pages: fragments 4, 5 and 2 at pfn 16, 32 and 64.
    {
        Fixture fixture;
        fixture.bring_up();
        const auto outcome = fixture.vm->map(*fixture.vram, fixture.tables, *fixture.gtt,
                                             fixture.inputs().vm, fixture.nbio);
        const auto pages = fixture.contiguous_pages();
        bool ok = outcome == MesSelfTestVmStatus::mapped;

        for (uint32_t pfn = 16; ok && pfn < 68; ++pfn) {
            const uint64_t frag = pfn < 32 ? 4 : pfn < 64 ? 5 : 2;
            ok = fixture.table(3, pfn) == (pages[pfn - 16] | 0x77 | (frag << 7));
        }

        expect(ok && fixture.vm->report().mapping_runs == 1 && fixture.vm->report().ptes_written == 52,
               "contiguous metadata: V|S|SN|X|R|W, MTYPE NC, FRAG by VA alignment");
        const Mes11ProcessVm vm = fixture.vm->process_vm();
        expect(vm.valid() && vm.metadata_va() == meta_va && vm.metadata_gart() == fixture.meta_gart() &&
               vm.metadata_cpu() == fixture.meta(), "process VM token");
        bool zeroed = true;

        for (uint32_t offset = 0; offset < mes11_ctx::bytes; offset += 4096) {
            zeroed = zeroed && u32(fixture.meta(), offset) == 0;
        }

        expect(zeroed, "metadata zeroed by amdgpu_mes_ctx_alloc_meta_data's memset");
        Mes11SelfTestReport none{};
        expect(fixture.vm->teardown(none) == MesSelfTestVmStatus::released &&
               !fixture.vm->process_vm().valid(), "teardown without a run unmaps and releases");
    }

    // Scattered pages: one contiguous run of 4, 47 single pages, a final page.
    {
        Fixture fixture;
        fixture.bring_up();
        std::vector<uint64_t> pages;
        const uint64_t base = 0x0000004000000000ULL;

        for (uint32_t page = 0; page < 4; ++page) {
            pages.push_back(base + page * 4096ULL);
        }

        for (uint32_t page = 4; page < 52; ++page) {
            pages.push_back(base + 0x100000 - page * 4096ULL);
        }

        fixture.set_dma_pages(pages);
        bool ok = fixture.vm->map(*fixture.vram, fixture.tables, *fixture.gtt, fixture.inputs().vm,
                                  fixture.nbio) == MesSelfTestVmStatus::mapped;

        for (uint32_t pfn = 16; ok && pfn < 68; ++pfn) {
            const uint64_t frag = pfn < 20 ? 2 : 0;
            ok = fixture.table(3, pfn) == (pages[pfn - 16] | 0x77 | (frag << 7));
        }

        expect(ok && fixture.vm->report().mapping_runs == 3,
               "scattered metadata: runs 4 (FRAG 2), 47 per-page, 1; amdgpu_vm_map_gart addresses");
        const auto result = fixture.mes->self_test(*fixture.gtt, fixture.queues,
                                                   fixture.vm->process_vm(), fixture.inputs().config);
        expect(result == Mes11Status::completed && fixture.engines.translation_faults == 0,
               "self-test runs through the scattered mapping");
        expect(fixture.vm->teardown(fixture.mes->self_test_report()) == MesSelfTestVmStatus::released,
               "scattered mapping torn down");
    }
}

void api_version_two() {
    Fixture fixture(0x00002050u);
    fixture.bring_up();
    const auto outcome = run_mes_self_test(*fixture.mes, *fixture.vm, fixture.inputs());
    const auto adds = fixture.frames(2);
    expect(outcome.test_status == Mes11Status::completed && adds.size() == 3 &&
           frame64(adds[0], 84) == fixture.meta_gart() + 86016 + 32 &&
           frame64(adds[1], 84) == fixture.meta_gart() + 108544 + 32 &&
           frame64(adds[2], 84) == fixture.meta_gart() + 188480 + 32 &&
           fixture.mes->self_test_report().api_version == 2,
           "MES API >= 2: ADD_QUEUE wptr_addr is the GART (MC) address of the wptr slot");
}

void test_failures() {
    // Hung gfx queue: ring test times out, testing stops, every queue is still
    // removed, the buffers and VM stay quarantined, MES itself stays usable.
    {
        Fixture fixture;
        fixture.bring_up();
        fixture.engines.hang_type = 0;
        const auto outcome = run_mes_self_test(*fixture.mes, *fixture.vm, fixture.inputs());
        const auto report = fixture.mes->self_test_report();
        expect(outcome.map_status == MesSelfTestVmStatus::mapped &&
               outcome.test_status == Mes11Status::ring_test_failed &&
               outcome.teardown_status == MesSelfTestVmStatus::retained,
               "gfx ring timeout: diagnostic failure, VM retained");
        expect(report.queues[0].ring_test == Mes11SelfTestResult::timeout &&
               report.queues[1].ring_test == Mes11SelfTestResult::not_run &&
               report.queues[2].ring_test == Mes11SelfTestResult::not_run && report.failed_queue == 0 &&
               fixture.frames(3).size() == 3 && report.memory_quarantined && report.memory_retained,
               "testing stops at the first failure; all three REMOVE_QUEUE frames still sent");
        expect(fixture.mes->report().scheduler_ready &&
               fixture.mes->query_sched_status() == Mes11Status::completed,
               "MES stays operational: the self-test result does not fail bring-up");
        expect(fixture.mes->kiq_hw_fini() == Mes11Status::disabled &&
               fixture.mes->release() == Mes11Status::quarantined &&
               fixture.vm->release_after_reset() == MesSelfTestVmStatus::quarantined,
               "quarantined buffers block release until a confirmed reset");
        fixture.reset_confirmed = true;
        expect(fixture.mes->release_after_reset() == Mes11Status::released &&
               fixture.vm->release_after_reset() == MesSelfTestVmStatus::released &&
               !fixture.mes->self_test_report().memory_retained,
               "confirmed reset returns every self-test lease");
        expect(fixture.vram->release(fixture.tables.tables[0]) == rx7900::memory::Status::success &&
               fixture.gtt->release(fixture.metadata) == rx7900::memory::Status::success &&
               fixture.gtt->release(fixture.queues.mqd[0]) == rx7900::memory::Status::success,
               "released self-test allocations are free again");
    }

    // Wrong IB result on the compute queue.
    {
        Fixture fixture;
        fixture.bring_up();
        fixture.engines.corrupt_ib_type = 1;
        const auto outcome = run_mes_self_test(*fixture.mes, *fixture.vm, fixture.inputs());
        const auto report = fixture.mes->self_test_report();
        expect(outcome.test_status == Mes11Status::ring_test_failed &&
               report.queues[0].ib_test == Mes11SelfTestResult::passed &&
               report.queues[1].ring_test == Mes11SelfTestResult::passed &&
               report.queues[1].ib_test == Mes11SelfTestResult::failed &&
               report.queues[2].ring_test == Mes11SelfTestResult::not_run &&
               report.memory_quarantined && outcome.teardown_status == MesSelfTestVmStatus::retained,
               "compute IB writes a wrong value: failed, SDMA untested, memory quarantined");
    }

    // SDMA IB never fences: bounded by the IB timeout, reported as a timeout.
    {
        Fixture fixture;
        fixture.bring_up();
        fixture.engines.no_fence_type = 2;
        const auto outcome = run_mes_self_test(*fixture.mes, *fixture.vm, fixture.inputs(50));
        const auto report = fixture.mes->self_test_report();
        expect(outcome.test_status == Mes11Status::ring_test_failed &&
               report.queues[2].ring_test == Mes11SelfTestResult::passed &&
               report.queues[2].ib_test == Mes11SelfTestResult::timeout &&
               report.queues[2].ib_polls == 50 && report.failed_queue == 2 &&
               fixture.frames(3).size() == 3 && report.memory_quarantined,
               "SDMA IB fence never written: 50 us budget, timeout, queues removed, quarantined");
    }

    // MES never completes ADD_QUEUE: MES quarantined, no tests, no removes.
    {
        Fixture fixture;
        fixture.bring_up();
        fixture.model.ignore_opcode = 2;
        const auto outcome = run_mes_self_test(*fixture.mes, *fixture.vm, fixture.inputs());
        const auto report = fixture.mes->self_test_report();
        expect(outcome.test_status == Mes11Status::timeout && !report.queues[0].added &&
               report.queues[0].add_status == Mes11Status::timeout &&
               report.queues[1].add_status == Mes11Status::quarantined &&
               report.queues[2].add_status == Mes11Status::quarantined &&
               fixture.frames(2).size() == 1 && fixture.frames(3).empty() &&
               report.memory_quarantined && outcome.teardown_status == MesSelfTestVmStatus::retained,
               "ADD_QUEUE timeout: later adds refused without frames, everything quarantined");
        expect(!fixture.mes->report().scheduler_ready &&
               fixture.mes->query_sched_status() == Mes11Status::quarantined,
               "an unanswered MES frame quarantines MES as every other MES API failure does");
    }

    // REMOVE_QUEUE unanswered.
    {
        Fixture fixture;
        fixture.bring_up();
        fixture.model.ignore_opcode = 3;
        const auto outcome = run_mes_self_test(*fixture.mes, *fixture.vm, fixture.inputs());
        const auto report = fixture.mes->self_test_report();
        expect(outcome.test_status == Mes11Status::timeout && report.ibs_passed == 3 &&
               !report.queues[0].removed && report.queues[1].remove_status == Mes11Status::quarantined &&
               report.memory_quarantined && outcome.teardown_status == MesSelfTestVmStatus::retained,
               "REMOVE_QUEUE timeout: queues may still run, memory quarantined");
    }

    // Failed SDMA IB_CNTL read during the SDMA MQD build.
    {
        Fixture fixture;
        fixture.bring_up();
        fixture.model.device->reg(sdma_ib_cntl) = UINT32_MAX;
        const auto outcome = run_mes_self_test(*fixture.mes, *fixture.vm, fixture.inputs());
        const auto report = fixture.mes->self_test_report();
        expect(outcome.test_status == Mes11Status::mmio_failure &&
               report.queues[2].add_status == Mes11Status::mmio_failure &&
               report.queues[0].added && report.queues[1].added && fixture.frames(3).empty() &&
               report.memory_quarantined, "MMIO failure quarantines MES and the added queues' memory");
    }
}

void refusals() {
    Fixture fixture;
    expect(fixture.mes->self_test(*fixture.gtt, fixture.queues, {}, fixture.inputs().config) ==
               Mes11Status::unavailable && fixture.model.mes_frames.empty(),
           "self-test before the scheduler is ready refused");
    fixture.bring_up();
    const size_t frames = fixture.model.mes_frames.size();
    expect(fixture.mes->self_test(*fixture.gtt, fixture.queues, {}, fixture.inputs().config) ==
               Mes11Status::invalid_argument, "invalid process VM token refused");

    auto inputs = fixture.inputs();
    std::vector<uint8_t> short_list(fixture.dma.begin(), fixture.dma.end() - 8);
    inputs.vm.metadata_dma_pages = {short_list.data(), short_list.size()};
    expect(fixture.vm->map(*fixture.vram, fixture.tables, *fixture.gtt, inputs.vm, fixture.nbio) ==
               MesSelfTestVmStatus::invalid_argument, "51-entry DMA list refused");
    std::vector<uint8_t> unaligned = fixture.dma;
    rx7900::sdma::CommandBuffer(unaligned.data(), unaligned.size()).write_u32(8, 0x123);
    inputs.vm.metadata_dma_pages = {unaligned.data(), unaligned.size()};
    expect(fixture.vm->map(*fixture.vram, fixture.tables, *fixture.gtt, inputs.vm, fixture.nbio) ==
               MesSelfTestVmStatus::invalid_argument, "unaligned DMA page refused before any store");
    inputs = fixture.inputs();
    MesSelfTestVmMemory gtt_tables{};

    for (auto& table : gtt_tables.tables) {
        fixture.allocate(*fixture.gtt, 4096, table);
    }

    expect(fixture.vm->map(*fixture.gtt, gtt_tables, *fixture.gtt, inputs.vm, fixture.nbio) ==
               MesSelfTestVmStatus::invalid_memory, "page tables in GTT refused (root must be VRAM)");
    inputs = fixture.inputs();
    inputs.vm.metadata = fixture.queues.mqd[0];
    expect(fixture.vm->map(*fixture.vram, fixture.tables, *fixture.gtt, inputs.vm, fixture.nbio) ==
               MesSelfTestVmStatus::invalid_memory && !fixture.vm->report().memory_retained,
           "4 KiB metadata allocation refused, table leases rolled back");

    expect(fixture.vm->map(*fixture.vram, fixture.tables, *fixture.gtt, fixture.inputs().vm,
                           fixture.nbio) == MesSelfTestVmStatus::mapped, "valid map");
    auto config = fixture.inputs().config;
    config.ib_timeout_microseconds = mes11_self_test_ib_timeout_max_us + 1;
    expect(fixture.mes->self_test(*fixture.gtt, fixture.queues, fixture.vm->process_vm(), config) ==
               Mes11Status::invalid_argument, "IB timeout above 40 s refused");
    Mes11SelfTestMemory swapped = fixture.queues;
    swapped.mqd[0] = fixture.queues.mqd[2];
    expect(fixture.mes->self_test(*fixture.gtt, swapped, fixture.vm->process_vm(),
                                  fixture.inputs().config) == Mes11Status::invalid_memory &&
               !fixture.mes->self_test_report().memory_retained,
           "aliased MQD buffers refused, nothing retained");
    Mes11SelfTestMemory vram_buffer = fixture.queues;
    vram_buffer.process_context = fixture.memory.writeback;
    expect(fixture.mes->self_test(*fixture.vram, vram_buffer, fixture.vm->process_vm(),
                                  fixture.inputs().config) == Mes11Status::invalid_memory,
           "process context outside the GART aperture refused");
    fixture.model.device->reg(context15_cntl) = 0x01fffe05u;
    expect(fixture.mes->self_test(*fixture.gtt, fixture.queues, fixture.vm->process_vm(),
                                  fixture.inputs().config) == Mes11Status::unsupported_ip &&
               fixture.mes->self_test_report().vm_context_cntl == 0x01fffe05u,
           "VM contexts not 3-level/block 9 refused (vm_context_cntl must describe this VM)");
    expect(fixture.model.mes_frames.size() == frames, "no refusal sent an MES frame");
    fixture.model.device->reg(context15_cntl) = vm_cntx_cntl;
    expect(fixture.mes->self_test(*fixture.gtt, fixture.queues, fixture.vm->process_vm(),
                                  fixture.inputs().config) == Mes11Status::completed,
           "after refusals the self-test still runs");
    expect(fixture.vm->teardown(fixture.mes->self_test_report()) == MesSelfTestVmStatus::released,
           "and tears down");
}

} // namespace

int main() {
    success_path();
    mapping_entries();
    api_version_two();
    test_failures();
    refusals();
    return checks.finish("mes_selftest_tests");
}

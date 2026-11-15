/*
Module: rx7900.tests.gfx11_queues
Purpose: Verify KGQ/KCQ MQDs, MES map/unmap frames, clear state, ring and IB tests, faults and leases.
Created: 2026-10-21
Architecture: Test-local MES/KIQ/CP model over the shared synthetic Navi31 machine
and a real FenceManager on a model platform for the submission path; it is not
hardware evidence. Expected values are hand-derived from Linux v6.12
gfx_v11_0.c, amdgpu_gfx.c, amdgpu_ring.c, amdgpu_ib.c, amdgpu_fence.c, nvd.h,
nbio_v4_3.c, mes_v11_0.c, mes_v11_api_def.h, v11_structs.h and the gc_11_0_0
and nbio_4_3_0 offset, sh_mask and default headers.
*/
#include <functional>
#include <map>
#include <memory>
#include <utility>

#include "asic_test_device.hpp"
#include "gfx11_queues.hpp"

namespace {

using namespace rx7900::hardware;
using rx7900_test::SimulatedNavi31;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

constexpr uint32_t gc0 = rx7900_test::gc_base;
constexpr uint32_t gc1 = 0xa000;
constexpr uint32_t grbm = gc1 + 0x0900;
constexpr uint32_t scratch = gc1 + 0x2040;
constexpr uint32_t gp3_lo = gc1 + 0x2849;
constexpr uint32_t pc_start = gc1 + 0x2800;
constexpr uint32_t pc_start_hi = gc1 + 0x289d;
constexpr uint32_t hqd_active = gc0 + 0x1fab;
constexpr uint32_t hqd_doorbell = gc0 + 0x1fb8;
constexpr uint32_t hqd_pq_base = gc0 + 0x1fb1;
constexpr uint32_t hqd_pq_base_hi = gc0 + 0x1fb2;
constexpr uint32_t hqd_pq_control = gc0 + 0x1fba;
constexpr uint32_t hqd_dequeue = gc0 + 0x1fc1;
constexpr uint32_t cp_max_context = gc0 + 0x1e4e;
constexpr uint32_t cp_device_id = gc0 + 0x1deb;
constexpr uint32_t kiq_select = 0xd;
constexpr uint32_t sched_select = 0xc;
constexpr uint32_t ring_nop = 0xffff1000u;
constexpr uint32_t kiq_doorbell = 0x18;
constexpr uint32_t gfx_doorbell = 0x116;
// BIF_BX_PF0_GPU_HDP_FLUSH_REQ/DONE: NBIF segment 2 + 0x106/0x107.
constexpr uint32_t hdp_flush_req = rx7900_test::nbif_bases[2] + 0x106;
constexpr uint32_t hdp_flush_done = rx7900_test::nbif_bases[2] + 0x107;

// soc21_grbm_select value of compute ring i (MEC1 = ME1, pipe i % 4, queue i / 4).
constexpr uint32_t compute_select(uint32_t index) {
    return (index % 4) | (1u << 2) | ((index / 4) << 8);
}

bool banked(uint32_t index) {
    return (index >= gc0 + 0x1e80 && index <= gc0 + 0x1e9a) ||
           (index >= gc0 + 0x1fa9 && index <= gc0 + 0x1fe0) || index == pc_start ||
           index == pc_start_hi || index == gp3_lo;
}

bool gc_register(uint32_t index) {
    return (index >= gc0 && index < gc0 + 0x2000) || (index >= gc1 && index < gc1 + 0x3000);
}

// gc_11_0_0_default.h reset values of the banked HQD registers Linux reads.
uint32_t reset_value(uint32_t index) {
    switch (index - gc0) {
    case 0x1e85: return 0x00000a01u; // CP_GFX_HQD_QUANTUM
    case 0x1e8f: return 0x00a00000u; // CP_GFX_HQD_CNTL
    case 0x1e9a: return 0x00000100u; // CP_GFX_MQD_CONTROL
    case 0x1fad: return 0x0be05501u; // CP_HQD_PERSISTENT_STATE
    case 0x1fba: return 0x00308509u; // CP_HQD_PQ_CONTROL
    case 0x1fbe: return 0x00300000u; // CP_HQD_IB_CONTROL
    case 0x1fcb: return 0x00000100u; // CP_MQD_CONTROL
    case 0x1fd0: return 0x00000006u; // CP_HQD_EOP_CONTROL
    default: return 0;
    }
}

struct CpQueue final {
    uint64_t mqd = 0;
    uint32_t type = 0;
    uint64_t rptr = 0;
    std::vector<std::vector<uint32_t>> packets;
    std::vector<std::vector<uint32_t>> ib_packets;
    std::vector<uint64_t> doorbells;
    std::vector<uint32_t> hdp_flush_refs;
    std::vector<uint32_t> fence_interrupts;
    std::vector<uint32_t> ib_vmids;
    std::vector<uint64_t> ib_faults;
};

// MES scheduler/KIQ firmware and CP model: the KIQ consumes its ring from the
// ME3 pipe1 HQD registers; MAP_QUEUES activates the scheduler; the scheduler
// completes 64-dword API frames and maps/unmaps legacy queues; a mapped legacy
// queue fetches its ring from its MQD on its doorbell and executes
// SET_UCONFIG_REG into the register file, COND_EXEC on its memory dword,
// WAIT_REG_MEM HDP flushes (NBIO reflects FLUSH_REQ into FLUSH_DONE),
// INDIRECT_BUFFER contents (WRITE_DATA to memory) and RELEASE_MEM 32/64-bit
// fence writes, skipping NOP, ACQUIRE_MEM and every other packet.
struct Model final {
    std::unique_ptr<SimulatedNavi31> device = std::make_unique<SimulatedNavi31>();
    std::map<std::pair<uint32_t, uint32_t>, uint32_t> bank;
    std::vector<std::pair<uint32_t, uint32_t>> writes;
    std::vector<std::pair<uint32_t, uint32_t>> banked_reads;
    std::vector<std::vector<uint32_t>> kiq_packets;
    std::vector<std::vector<uint32_t>> mes_frames;
    std::map<uint32_t, CpQueue> queues;
    std::vector<uint32_t> removed;
    uint32_t select = 0;
    uint32_t fail_write_index = UINT32_MAX;
    uint32_t fail_read_index = UINT32_MAX;
    uint32_t fail_doorbell = UINT32_MAX;
    uint32_t hung_doorbell = UINT32_MAX;
    // WRITE_DATA executed from an IB on this doorbell stores this value instead.
    uint32_t corrupt_ib_doorbell = UINT32_MAX;
    // WAIT_REG_MEM HDP flushes on this doorbell never see FLUSH_DONE (CP stalls).
    uint32_t stuck_hdp_doorbell = UINT32_MAX;
    uint32_t ignore_opcode = 0xff;
    // Process-VM jobs: ring-level WRITE_DATA register stores, GFXHUB
    // invalidation acknowledgements (bit vmid of every acknowledged request)
    // and the VA -> MC translation the test gives each VMID's page directory.
    std::vector<std::pair<uint32_t, uint32_t>> ring_register_writes;
    uint32_t acknowledged_vmids = 0;
    std::function<bool(uint32_t vmid, uint64_t va, uint64_t& mc)> vm_translate;
    uint32_t last_flush = 0;
    bool flushed_before_every_doorbell = true;
    bool mapped = false;
    uint64_t kiq_rptr = 0;
    uint64_t mes_rptr = 0;
    uint64_t mes_ring = 0;
    uint32_t mes_ring_dwords = 0;
    uint32_t mes_doorbell = UINT32_MAX;

    Model() {
        bank[{sched_select, gp3_lo}] = 0x00001050u;
        bank[{kiq_select, gp3_lo}] = 0x00001049u;
    }

    RegisterIo io() noexcept { return {this, rx7900_test::aperture_bytes, read, write, delay}; }
    DoorbellIo doorbells(uint32_t dwords = 0x10000) noexcept {
        return {this, dwords, doorbell32, doorbell64};
    }

    uint32_t& banked_reg(uint32_t selection, uint32_t index) {
        const auto key = std::make_pair(selection, index);

        if (bank.find(key) == bank.end()) {
            bank[key] = reset_value(index);
        }

        return bank[key];
    }

    static bool read(void* context, uint32_t offset, uint32_t& value) noexcept {
        auto& self = *static_cast<Model*>(context);
        const uint32_t index = offset / 4;

        if (index == self.fail_read_index) {
            return false;
        }

        if (banked(index)) {
            self.banked_reads.push_back({self.select, index});
            value = self.banked_reg(self.select, index);
            return true;
        }

        return SimulatedNavi31::read(self.device.get(), offset, value);
    }

    static bool write(void* context, uint32_t offset, uint32_t value) noexcept {
        auto& self = *static_cast<Model*>(context);
        const uint32_t index = offset / 4;

        if (index == self.fail_write_index) {
            return false;
        }

        if (gc_register(index)) {
            self.writes.push_back({index, value});
        }

        if (index == grbm) {
            self.select = value;
        }

        if (banked(index)) {
            self.banked_reg(self.select, index) = value;

            if (index == hqd_dequeue && value == 1) {
                self.banked_reg(self.select, hqd_active) = 0;
            }

            return true;
        }

        return SimulatedNavi31::write(self.device.get(), offset, value);
    }

    static void delay(void* context, uint32_t microseconds) noexcept {
        SimulatedNavi31::delay(static_cast<Model*>(context)->device.get(), microseconds);
    }

    static bool doorbell32(void*, uint32_t, uint32_t) noexcept { return false; }

    static bool doorbell64(void* context, uint32_t index, uint64_t value) noexcept {
        auto& self = *static_cast<Model*>(context);

        if (index == self.fail_doorbell) {
            return false;
        }

        if (self.device->hdp_flushes <= self.last_flush) {
            self.flushed_before_every_doorbell = false;
        }

        self.last_flush = self.device->hdp_flushes;

        if (index == kiq_doorbell) {
            self.run_kiq(value);
        } else if (index == self.mes_doorbell) {
            self.run_mes(value);
        } else {
            const auto queue = self.queues.find(index);

            if (queue != self.queues.end()) {
                queue->second.doorbells.push_back(value);

                if (index != self.hung_doorbell) {
                    self.run_cp(index, queue->second, value);
                }
            }
        }

        return true;
    }

    uint32_t vram(uint64_t address) { return device->vram_u32(address); }

    uint64_t vram64(uint64_t address) {
        return (static_cast<uint64_t>(vram(address + 4)) << 32) | vram(address);
    }

    void put64(uint64_t address, uint64_t value) {
        device->put_vram_u32(address, static_cast<uint32_t>(value));
        device->put_vram_u32(address + 4, static_cast<uint32_t>(value >> 32));
    }

    void run_kiq(uint64_t wptr) {
        const uint32_t control = banked_reg(kiq_select, hqd_doorbell);

        if (banked_reg(kiq_select, hqd_active) != 1 || (control & 0x40000000u) == 0) {
            return;
        }

        const uint64_t ring = ((static_cast<uint64_t>(banked_reg(kiq_select, hqd_pq_base_hi)) << 32) |
                               banked_reg(kiq_select, hqd_pq_base)) << 8;
        const uint32_t dwords = 2u << (banked_reg(kiq_select, hqd_pq_control) & 0x3f);
        const auto dw = [&](uint64_t position) { return vram(ring + (position & (dwords - 1)) * 4); };

        while (kiq_rptr < wptr) {
            const uint32_t header = dw(kiq_rptr);

            if (header == ring_nop) {
                ++kiq_rptr;
                continue;
            }

            const uint32_t count = ((header >> 16) & 0x3fff) + 1;
            std::vector<uint32_t> packet;

            for (uint32_t index = 0; index <= count; ++index) {
                packet.push_back(dw(kiq_rptr + index));
            }

            kiq_rptr += count + 1;
            kiq_packets.push_back(packet);
            const uint32_t opcode = (header >> 8) & 0xff;

            if (opcode == 0xa2) {
                const uint64_t mqd = (static_cast<uint64_t>(packet.at(4)) << 32) | packet.at(3);
                mes_ring = ((static_cast<uint64_t>(vram(mqd + 548)) << 32) | vram(mqd + 544)) << 8;
                mes_ring_dwords = 2u << (vram(mqd + 580) & 0x3f);
                mes_doorbell = (vram(mqd + 572) & 0x0ffffffcu) >> 2;
                banked_reg(sched_select, hqd_doorbell) = vram(mqd + 572);
                banked_reg(sched_select, hqd_active) = 1;
                mapped = true;
            } else if (opcode == 0x37) {
                device->reg(packet.at(2)) = packet.at(4);
            }
        }
    }

    void run_mes(uint64_t wptr) {
        if (!mapped) {
            return;
        }

        while (mes_rptr < wptr) {
            std::vector<uint32_t> frame;

            for (uint32_t index = 0; index < 64; ++index) {
                frame.push_back(vram(mes_ring + ((mes_rptr + index) & (mes_ring_dwords - 1)) * 4));
            }

            mes_rptr += 64;
            mes_frames.push_back(frame);
            const uint32_t opcode = (frame.at(0) >> 4) & 0xff;
            const uint32_t status = opcode == 0 ? 200 : opcode == 2 ? 148 : opcode == 3 ? 20 : 8;

            if (opcode == ignore_opcode) {
                continue;
            }

            if (opcode == 2) {
                CpQueue queue;
                queue.mqd = (static_cast<uint64_t>(frame.at(20)) << 32) | frame.at(19);
                queue.type = frame.at(27);
                queues[frame.at(18)] = queue;
            } else if (opcode == 3) {
                removed.push_back(frame.at(1));
                queues.erase(frame.at(1));
            }

            put64((static_cast<uint64_t>(frame.at(status / 4 + 1)) << 32) | frame.at(status / 4),
                  (static_cast<uint64_t>(frame.at(status / 4 + 3)) << 32) | frame.at(status / 4 + 2));
        }
    }

    void run_cp(uint32_t doorbell, CpQueue& queue, uint64_t wptr) {
        const uint64_t mqd = queue.mqd;

        if (((vram(mqd + 572) & 0x0ffffffcu) >> 2) != doorbell || (vram(mqd + 572) & 0x40000000u) == 0) {
            return;
        }

        const uint64_t ring = ((static_cast<uint64_t>(vram(mqd + 548)) << 32) | vram(mqd + 544)) << 8;
        const uint32_t dwords = 2u << (vram(mqd + 580) & 0x3f);
        const auto dw = [&](uint64_t position) { return vram(ring + (position & (dwords - 1)) * 4); };

        while (queue.rptr < wptr) {
            const uint32_t header = dw(queue.rptr);

            if (header == ring_nop) {
                ++queue.rptr;
                continue;
            }

            const uint32_t count = ((header >> 16) & 0x3fff) + 1;
            std::vector<uint32_t> packet;

            for (uint32_t index = 0; index <= count; ++index) {
                packet.push_back(dw(queue.rptr + index));
            }

            const uint32_t opcode = (header >> 8) & 0xff;

            if (opcode == 0x3c && packet.size() == 7 && (packet.at(1) & 0x70u) == 0x40u) {
                // Register write-then-wait (reg0 = ref, then reg1 & mask == mask):
                // NBIO reflects FLUSH_REQ into FLUSH_DONE; a VM hub acknowledges
                // the requested VMID bits (mask 1 << vmid) of an invalidation.
                const bool hdp = packet.at(4) == packet.at(5);
                device->reg(packet.at(2)) = packet.at(4);
                device->reg(packet.at(3)) = doorbell == stuck_hdp_doorbell ? 0 : packet.at(5);

                if (hdp) {
                    queue.hdp_flush_refs.push_back(packet.at(4));
                } else {
                    acknowledged_vmids |= packet.at(5);
                }

                if ((device->reg(packet.at(3)) & packet.at(5)) != packet.at(5)) {
                    return;
                }
            }

            // Memory wait (pipeline sync): the fence writeback must equal ref.
            if (opcode == 0x3c && packet.size() == 7 && (packet.at(1) & 0x70u) == 0x10u &&
                (vram((static_cast<uint64_t>(packet.at(3)) << 32) | packet.at(2)) & packet.at(5)) !=
                    packet.at(4)) {
                return;
            }

            queue.rptr += count + 1;
            queue.packets.push_back(packet);

            if (opcode == 0x79 && packet.size() == 3) {
                device->reg(0xc000 + packet.at(1)) = packet.at(2);
            } else if (opcode == 0x37 && packet.size() == 5 && ((packet.at(1) >> 8) & 0xf) == 0) {
                // Ring-level WRITE_DATA DST_SEL 0: a register store.
                device->reg(packet.at(2)) = packet.at(4);
                ring_register_writes.emplace_back(packet.at(2), packet.at(4));
            } else if (opcode == 0x22 && packet.size() == 5) {
                // COND_EXEC: discard the next count dwords when *addr == 0.
                if (vram((static_cast<uint64_t>(packet.at(2)) << 32) | packet.at(1)) == 0) {
                    queue.rptr += packet.at(4);
                }
            } else if (opcode == 0x3f && packet.size() == 4) {
                run_ib(doorbell, queue, (static_cast<uint64_t>(packet.at(2)) << 32) | packet.at(1),
                       packet.at(3));
            } else if (opcode == 0x49 && packet.size() == 8) {
                const uint64_t address = (static_cast<uint64_t>(packet.at(4)) << 32) | packet.at(3);
                const uint32_t data_sel = packet.at(2) >> 29;
                queue.fence_interrupts.push_back((packet.at(2) >> 24) & 7);

                if (data_sel == 1 || data_sel == 2) {
                    device->put_vram_u32(address, packet.at(5));
                }

                if (data_sel == 2) {
                    device->put_vram_u32(address + 4, packet.at(6));
                }
            }
        }

        // 32-bit RPTR report to the MQD's rptr writeback address.
        device->put_vram_u32((static_cast<uint64_t>(vram(mqd + 560)) << 32) | vram(mqd + 556),
                             static_cast<uint32_t>(queue.rptr));
    }

    // INDIRECT_BUFFER: IB_SIZE is control bits 19:0 (dwords), VMID bits 27:24.
    // VMID 0 IBs and their WRITE_DATA targets are MC addresses; a job VMID
    // must have had its invalidation acknowledged, and both the IB fetch and
    // WRITE_DATA DST_SEL 5 then go through vm_translate.
    void run_ib(uint32_t doorbell, CpQueue& queue, uint64_t address, uint32_t control) {
        const uint32_t length = control & 0xfffff;
        const uint32_t vmid = (control >> 24) & 0xf;
        const auto mc_of = [&](uint64_t va, uint64_t& mc) {
            if (vmid == 0) {
                mc = va;
                return true;
            }

            return vm_translate && (acknowledged_vmids & (1u << vmid)) != 0 && vm_translate(vmid, va, mc);
        };

        if ((address & 3) != 0) {
            return;
        }

        uint32_t position = 0;

        while (position < length) {
            uint64_t mc = 0;

            if (!mc_of(address + position * 4, mc)) {
                queue.ib_faults.push_back(address + position * 4);
                return;
            }

            const uint32_t header = vram(mc);
            const uint32_t count = ((header >> 16) & 0x3fff) + 1;
            std::vector<uint32_t> packet;

            for (uint32_t index = 0; index <= count && position + index < length; ++index) {
                uint64_t word = 0;
                packet.push_back(mc_of(address + (position + index) * 4, word) ? vram(word) : 0);
            }

            position += count + 1;
            queue.ib_packets.push_back(packet);
            queue.ib_vmids.push_back(vmid);

            if (((header >> 8) & 0xff) == 0x37 && packet.size() == 5 &&
                ((packet.at(1) >> 8) & 0xf) == 5) {
                uint64_t target = 0;

                if (mc_of((static_cast<uint64_t>(packet.at(3)) << 32) | packet.at(2), target)) {
                    device->put_vram_u32(target, doorbell == corrupt_ib_doorbell ? 0x0badf00du : packet.at(4));
                } else {
                    queue.ib_faults.push_back(target);
                }
            }
        }
    }
};

struct Fixture final {
    Model model;
    rx7900_test::VramBackend backend{};
    std::unique_ptr<rx7900::memory::Manager> manager = std::make_unique<rx7900::memory::Manager>();
    Nbio nbio{};
    Mes11Registers mes_registers{};
    rx7900::memory::GcRegisterBase gc{};
    std::unique_ptr<Mes11> mes = std::make_unique<Mes11>();
    std::unique_ptr<Gfx11Queues> queues = std::make_unique<Gfx11Queues>();
    Mes11Memory mes_memory{};
    Gfx11QueueMemory memory{};
    NbioRegisters nbio_registers{};
    rx7900::memory::Handle ib{};
    bool reset_confirmed = false;
    Gfx11QueuesStatus initialized = Gfx11QueuesStatus::unavailable;

    static bool confirm(void* context) noexcept { return *static_cast<bool*>(context); }

    explicit Fixture(bool start_mes = true, uint32_t compute = 8, uint32_t polls = 1000) {
        backend.device = model.device.get();
        manager->initialize({rx7900_test::fb_base + rx7900_test::arena_offset,
                             rx7900_test::arena_bytes}, backend.backend());
        const auto binary = rx7900_test::navi31_discovery();
        validate_nbio_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                nbio_registers);
        validate_mes11_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                 mes_registers);
        rx7900::memory::validate_gc_discovery({binary.data(), binary.size()},
                                              rx7900_test::aperture_bytes, gc);
        nbio.initialize(nbio_registers, model.device->io());
        nbio.program_common();
        allocate(mes11_kiq_ring_bytes, mes_memory.kiq_ring);
        allocate(mes11_scheduler_ring_bytes, mes_memory.scheduler_ring);
        allocate(mes11_mqd_bytes, mes_memory.kiq_mqd);
        allocate(mes11_mqd_bytes, mes_memory.scheduler_mqd);
        allocate(mes11_eop_bytes, mes_memory.kiq_eop);
        allocate(mes11_eop_bytes, mes_memory.scheduler_eop);
        allocate(mes11_writeback_bytes, mes_memory.writeback);
        allocate(gfx11_ring_bytes, memory.gfx_ring);
        allocate(gfx11_mqd_bytes, memory.gfx_mqd);

        for (uint32_t index = 0; index < compute; ++index) {
            allocate(gfx11_ring_bytes, memory.compute_ring[index]);
            allocate(gfx11_mqd_bytes, memory.compute_mqd[index]);
        }

        allocate(compute * gfx11_mec_hpd_bytes, memory.compute_eop);
        allocate(gfx11_writeback_bytes, memory.writeback);
        allocate(gfx11_ib_test_buffer_bytes, ib);
        mes->initialize(mes_registers, model.io(), model.doorbells(), nbio,
                        rx7900_test::navi31_vram(), {0x40000, 0x0000000100080000ULL, 2},
                        {1000, 2100000}, {&reset_confirmed, confirm});

        if (start_mes) {
            mes->kiq_hw_init(*manager, mes_memory);
        }

        initialized = queues->initialize(gc, model.io(), model.doorbells(), nbio,
                                         rx7900_test::navi31_vram(), {compute}, {polls},
                                         {&reset_confirmed, confirm});
    }

    void allocate(uint32_t bytes, rx7900::memory::Handle& handle) {
        const uint32_t pages = (bytes + 0xfff) & ~0xfffu;
        manager->allocate(pages, 0x1000, rx7900::memory::Access::read_write, true, handle);
    }

    uint64_t address(rx7900::memory::Handle handle) {
        rx7900::memory::AllocationInfo info{};
        manager->describe(handle, info);
        return info.gpu.virtual_address;
    }

    uint32_t vram(uint64_t address) { return model.vram(address); }

    std::vector<uint32_t> dwords(uint64_t address, uint32_t count) {
        std::vector<uint32_t> result;

        for (uint32_t index = 0; index < count; ++index) {
            result.push_back(vram(address + index * 4));
        }

        return result;
    }

    bool leases_free() {
        // Every allocation must be releasable with no remaining lease.
        std::vector<rx7900::memory::Handle> handles{mes_memory.kiq_ring, mes_memory.scheduler_ring,
            mes_memory.kiq_mqd, mes_memory.scheduler_mqd, mes_memory.kiq_eop,
            mes_memory.scheduler_eop, mes_memory.writeback, memory.gfx_ring, memory.gfx_mqd,
            memory.compute_eop, memory.writeback, ib};

        for (uint32_t index = 0; index < 8; ++index) {
            if (memory.compute_ring[index].slot != UINT32_MAX) {
                handles.push_back(memory.compute_ring[index]);
                handles.push_back(memory.compute_mqd[index]);
            }
        }

        bool ok = true;

        for (const auto& handle : handles) {
            ok = ok && manager->release(handle) == rx7900::memory::Status::success;
        }

        return ok && manager->empty();
    }

    Gfx11QueuesStatus test_ib(uint32_t timeout = gfx11_ib_test_timeout_us) {
        return queues->test_ib(*manager, ib, {nbio_registers, timeout});
    }

    uint32_t leases(rx7900::memory::Handle handle) {
        rx7900::memory::AllocationInfo info{};
        manager->describe(handle, info);
        return info.leases;
    }

    bool shutdown_all() {
        return queues->fini(*mes) == Gfx11QueuesStatus::stopped &&
               queues->release() == Gfx11QueuesStatus::released &&
               mes->kiq_hw_fini() == Mes11Status::disabled && mes->release() == Mes11Status::released &&
               leases_free();
    }
};

uint32_t lo(uint64_t value) { return static_cast<uint32_t>(value); }
uint32_t hi(uint64_t value) { return static_cast<uint32_t>(value >> 32); }

void initialize_refusals() {
    Fixture fixture(false);
    expect(fixture.initialized == Gfx11QueuesStatus::ready, "valid token, doorbells and config accepted");
    const Gfx11QueuesReport report = fixture.queues->report();
    expect(report.rings[0].type == Mes11QueueType::gfx && report.rings[0].doorbell_index == 0x116 &&
           report.rings[0].me == 0 && report.rings[0].pipe == 0 && report.rings[0].queue == 0 &&
           !report.rings[0].high_priority, "gfx ring: me0 pipe0 queue0, doorbell 0x8b << 1");
    bool layout = true;
    const uint32_t pipes[8]{0, 1, 2, 3, 0, 1, 2, 3};
    const uint32_t queue_ids[8]{0, 0, 0, 0, 1, 1, 1, 1};

    for (uint32_t index = 0; index < 8; ++index) {
        const Gfx11RingReport& ring = report.rings[1 + index];
        layout = layout && ring.type == Mes11QueueType::compute && ring.me == 1 &&
                 ring.pipe == pipes[index] && ring.queue == queue_ids[index] &&
                 ring.doorbell_index == (3 + index) * 2 && ring.high_priority == (index == 0);
    }

    expect(layout, "compute_queue_acquire multipipe order and doorbells (0x3 + i) << 1; ring 0 high priority");

    Gfx11Queues other;
    Model& model = fixture.model;
    const auto vram = rx7900_test::navi31_vram();
    const Gfx11QueuesRecovery recovery{&fixture.reset_confirmed, Fixture::confirm};
    expect(other.initialize({}, model.io(), model.doorbells(), fixture.nbio, vram, {8}, {1000},
                            recovery) == Gfx11QueuesStatus::unsupported_ip, "invalid token refused");
    expect(other.initialize(fixture.gc, model.io(), {nullptr, 0, nullptr, nullptr}, fixture.nbio,
                            vram, {8}, {1000}, recovery) == Gfx11QueuesStatus::unavailable,
           "no doorbell aperture refused (Linux BUG() on the non-doorbell compute path)");
    expect(other.initialize(fixture.gc, model.io(), model.doorbells(0x117), fixture.nbio, vram, {8},
                            {1000}, recovery) == Gfx11QueuesStatus::unavailable,
           "doorbell aperture without the gfx doorbell's upper dword refused");
    expect(other.initialize(fixture.gc, model.io(), model.doorbells(), fixture.nbio, vram, {0},
                            {1000}, recovery) == Gfx11QueuesStatus::invalid_argument &&
           other.initialize(fixture.gc, model.io(), model.doorbells(), fixture.nbio, vram, {9},
                            {1000}, recovery) == Gfx11QueuesStatus::invalid_argument,
           "0 or 9 compute rings refused");
    expect(other.initialize(fixture.gc, model.io(), model.doorbells(), fixture.nbio, vram, {8}, {0},
                            recovery) == Gfx11QueuesStatus::invalid_argument, "zero poll budget refused");
    expect(other.initialize(fixture.gc, model.io(), model.doorbells(), fixture.nbio, vram, {8},
                            {1000}, {nullptr, nullptr}) == Gfx11QueuesStatus::unavailable,
           "missing reset-quiescence callback refused");

    const auto shifted = rx7900_test::discovery_with({
        {discovery_gc_hardware_id, 11, 0, {gc0, 0x9000}}});
    rx7900::memory::GcRegisterBase low_gc{};
    rx7900::memory::validate_gc_discovery({shifted.data(), shifted.size()},
                                          rx7900_test::aperture_bytes, low_gc);
    expect(low_gc.valid() &&
           other.initialize(low_gc, model.io(), model.doorbells(), fixture.nbio, vram, {8}, {1000},
                            recovery) == Gfx11QueuesStatus::unsupported_ip,
           "SCRATCH_REG0 outside the SET_UCONFIG_REG window refused");

    expect(fixture.queues->resume(*fixture.manager, fixture.memory, *fixture.mes) ==
           Gfx11QueuesStatus::unavailable && model.writes.empty() &&
           !fixture.queues->report().memory_retained, "MES scheduler not ready: refused before any access");
    expect(fixture.queues->fini(*fixture.mes) == Gfx11QueuesStatus::unavailable &&
           fixture.queues->release() == Gfx11QueuesStatus::busy &&
           fixture.queues->release_after_reset() == Gfx11QueuesStatus::busy,
           "fini/release without resume refused");

    Fixture old(false);
    old.model.banked_reg(sched_select, gp3_lo) = 0x00001046u;
    old.mes->kiq_hw_init(*old.manager, old.mes_memory);
    old.mes->hw_init();
    old.model.writes.clear();
    expect(old.queues->resume(*old.manager, old.memory, *old.mes) ==
           Gfx11QueuesStatus::unsupported_firmware && old.model.writes.empty(),
           "MES firmware below 0x47: KIQ map path not implemented, refused");

    Fixture memory;
    memory.model.writes.clear();
    Gfx11QueueMemory wrong = memory.memory;
    wrong.compute_ring[3] = memory.memory.compute_mqd[3];
    expect(memory.queues->resume(*memory.manager, wrong, *memory.mes) ==
           Gfx11QueuesStatus::invalid_memory && memory.model.writes.empty() &&
           !memory.queues->report().memory_retained, "4 KiB allocation for an 8 KiB ring refused");
    wrong = memory.memory;
    wrong.compute_mqd[5] = memory.memory.compute_mqd[4];
    expect(memory.queues->resume(*memory.manager, wrong, *memory.mes) ==
           Gfx11QueuesStatus::invalid_memory && memory.model.writes.empty(),
           "one MQD for two rings refused as overlap, leases rolled back");
    wrong = memory.memory;
    wrong.compute_eop = memory.memory.writeback;
    expect(memory.queues->resume(*memory.manager, wrong, *memory.mes) ==
           Gfx11QueuesStatus::invalid_memory, "4 KiB EOP buffer for eight 2 KiB HPDs refused");
    expect(memory.queues->report().status == Gfx11QueuesStatus::invalid_memory &&
           memory.queues->resume(*memory.manager, memory.memory, *memory.mes) ==
           Gfx11QueuesStatus::running, "refusals do not quarantine; a valid resume then runs");
    expect(memory.shutdown_all(), "teardown returns every lease");
}

std::vector<uint32_t> expected_compute_mqd(uint32_t index, uint64_t mqd, uint64_t ring,
                                           uint64_t eop, uint64_t wb, uint32_t rptr,
                                           uint32_t pq_control) {
    std::vector<uint32_t> m(512, 0);
    const uint64_t rptr_addr = wb + (1 + index) * 160;
    const uint64_t wptr_addr = rptr_addr + 32;
    const bool high = index == 0;
    m.at(0) = 0xc0310800u;
    m.at(11) = 1;
    m.at(23) = m.at(24) = m.at(26) = m.at(27) = 0xffffffffu;
    m.at(32) = 7;
    m.at(128) = lo(mqd) & 0xfffffffcu;
    m.at(129) = hi(mqd);
    m.at(132) = 0x0be05501u;
    m.at(133) = high ? 2 : 0;
    m.at(134) = high ? 15 : 0;
    m.at(136) = lo(ring >> 8);
    m.at(137) = hi(ring >> 8);
    m.at(138) = rptr;
    m.at(139) = lo(rptr_addr) & 0xfffffffcu;
    m.at(140) = hi(rptr_addr) & 0xffffu;
    m.at(141) = lo(wptr_addr) & 0xfffffffcu;
    m.at(142) = hi(wptr_addr) & 0xffffu;
    m.at(143) = 0x40000000u | (((3 + index) * 2) << 2);
    // Reset 0x00308509 keeps PQ_EMPTY (bit 15) and MIN_AVAIL_SIZE; QUEUE_SIZE 10 (2048
    // dwords), RPTR_BLOCK_SIZE 9, UNORD, TUNNEL (high only), PRIV, KMD.
    m.at(145) = pq_control | (high ? 0x20000000u : 0);
    m.at(149) = 0x00300000u;
    m.at(162) = 0x100;
    m.at(165) = lo((eop + index * 2048) >> 8);
    m.at(166) = hi((eop + index * 2048) >> 8);
    m.at(167) = 8;
    return m;
}

void bring_up() {
    Fixture fixture;
    Model& model = fixture.model;
    expect(fixture.mes->report().scheduler_ready, "MES scheduler ready before cp_resume continues");
    // Distinct register contents under specific selections prove the reads
    // happen under the ring's own GRBM selection.
    model.banked_reg(compute_select(4), gc0 + 0x1fb3) = 0x44;
    model.banked_reg(compute_select(1), gc0 + 0x1fba) = 0x01308509u;
    model.banked_reg(0, gc0 + 0x1e88) = 0x12;
    model.writes.clear();
    model.banked_reads.clear();
    const size_t frames_before = model.mes_frames.size();
    const size_t kiq_before = model.kiq_packets.size();

    expect(fixture.queues->resume(*fixture.manager, fixture.memory, *fixture.mes) ==
           Gfx11QueuesStatus::running, "resume: KCQ, KGQ, clear state and nine ring tests");
    const Gfx11QueuesReport report = fixture.queues->report();
    const uint64_t wb = fixture.address(fixture.memory.writeback);
    const uint64_t eop = fixture.address(fixture.memory.compute_eop);

    std::vector<std::pair<uint32_t, uint32_t>> writes;

    for (uint32_t index = 0; index < 8; ++index) {
        writes.push_back({grbm, compute_select(index)});
        writes.push_back({grbm, 0});
    }

    writes.push_back({scratch, 0xcafedeadu});
    writes.push_back({grbm, 0});
    writes.push_back({grbm, 0});
    writes.push_back({cp_max_context, 7});
    writes.push_back({cp_device_id, 1});

    for (uint32_t ring = 0; ring < 9; ++ring) {
        writes.push_back({scratch, 0xcafedeadu});
    }

    expect(model.writes == writes,
           "Linux order: KCQ MQDs under GRBM select, KIQ test seed, KGQ MQD, CP_MAX_CONTEXT 7, "
           "CP_DEVICE_ID 1, then gfx and compute ring-test seeds");

    std::vector<std::pair<uint32_t, uint32_t>> reads;
    const uint32_t compute_reads[8]{0x1fd0, 0x1fb8, 0x1fcb, 0x1fba, 0x1fb8, 0x1fb3, 0x1fad, 0x1fbe};

    for (uint32_t index = 0; index < 8; ++index) {
        for (uint32_t reg : compute_reads) {
            reads.push_back({compute_select(index), gc0 + reg});
        }
    }

    const uint32_t gfx_reads[7]{0x1e9a, 0x1e81, 0x1e84, 0x1e85, 0x1e8f, 0x1e8d, 0x1e88};

    for (uint32_t reg : gfx_reads) {
        reads.push_back({0, gc0 + reg});
    }

    expect(model.banked_reads == reads, "compute_mqd_init and gfx_mqd_init register reads in order");
    expect(model.select == 0, "broadcast GRBM selection restored");

    bool compute_ok = true;

    for (uint32_t index = 0; index < 8; ++index) {
        const uint64_t mqd = fixture.address(fixture.memory.compute_mqd[index]);
        const uint64_t ring = fixture.address(fixture.memory.compute_ring[index]);
        const auto expected = expected_compute_mqd(index, mqd, ring, eop, wb, index == 4 ? 0x44 : 0,
                                                   index == 1 ? 0xd130890au : 0xd030890au);
        compute_ok = compute_ok && fixture.dwords(mqd, 512) == expected;
    }

    expect(compute_ok, "eight v11_compute_mqd images match the hand-derived Linux values byte for byte");
    const uint64_t mqd0 = fixture.address(fixture.memory.compute_mqd[0]);
    expect(fixture.vram(mqd0 + 580) == 0xf030890au && fixture.vram(mqd0 + 532) == 2 &&
           fixture.vram(mqd0 + 536) == 15 && fixture.vram(mqd0 + 572) == 0x40000018u &&
           fixture.vram(mqd0 + 668) == 8,
           "high-priority KCQ 0: tunnel dispatch, pipe priority HIGH, queue priority 15, doorbell 6");

    const uint64_t gfx_mqd = fixture.address(fixture.memory.gfx_mqd);
    const uint64_t gfx_ring = fixture.address(fixture.memory.gfx_ring);
    std::vector<uint32_t> g(512, 0);
    g.at(128) = lo(gfx_mqd);
    g.at(129) = hi(gfx_mqd);
    g.at(130) = 1;
    g.at(135) = 0x00000a01u;
    g.at(136) = lo(gfx_ring >> 8);
    g.at(137) = hi(gfx_ring >> 8);
    g.at(138) = 0x12;
    g.at(139) = lo(wb);
    g.at(140) = hi(wb) & 0xffff;
    g.at(141) = lo(wb + 32);
    g.at(142) = hi(wb + 32) & 0xffff;
    g.at(143) = 0x40000458u;
    g.at(145) = 0x00a0080au;
    g.at(162) = 0x100;
    expect(fixture.dwords(gfx_mqd, 512) == g,
           "v11_gfx_mqd: quantum 0xa01, RB_BUFSZ 10 / RB_BLKSZ 8, doorbell 0x116, rptr read, active");

    expect(model.kiq_packets.size() == kiq_before + 2, "one SET_RESOURCES plus its KIQ ring test");
    const std::vector<uint32_t> resources{0xc006a000u, 0xa0000u, 0xffffffffu, 0xffffffffu, 0, 0, 0, 0};
    expect(model.kiq_packets.at(kiq_before) == resources && report.set_resources_mask == ~0ULL &&
           report.set_resources_status == Mes11Status::completed,
           "amdgpu_gfx_mes_enable_kcq: gfx11_kiq_set_resources with queue_mask ~0");

    const uint64_t mes_wb = fixture.address(fixture.mes_memory.writeback);
    bool frames_ok = model.mes_frames.size() == frames_before + 18;

    for (uint32_t index = 0; frames_ok && index < 9; ++index) {
        const uint32_t ring = index < 8 ? 1 + index : 0;
        const Gfx11RingReport& info = report.rings[ring];
        std::vector<uint32_t> add(64, 0);
        add.at(0) = 0x40021;
        add.at(18) = info.doorbell_index;
        add.at(19) = lo(info.mqd_mc);
        add.at(20) = hi(info.mqd_mc);
        add.at(21) = lo(wb + ring * 160 + 32);
        add.at(22) = hi(wb + ring * 160 + 32);
        add.at(27) = ring == 0 ? 0 : 1;
        add.at(36) = 0x2000;
        add.at(37) = lo(mes_wb + 480);
        add.at(38) = hi(mes_wb + 480);
        add.at(39) = 1;
        add.at(48) = info.pipe;
        add.at(49) = info.queue;
        frames_ok = model.mes_frames.at(frames_before + index * 2) == add &&
                    model.mes_frames.at(frames_before + index * 2 + 1).at(0) == 0x400b1;
    }

    expect(frames_ok, "ADD_QUEUE map_legacy_kq frames: eight KCQs then the KGQ, each with a fence frame");

    const auto& gfx_queue = model.queues.at(gfx_doorbell);
    expect(gfx_queue.doorbells == std::vector<uint64_t>{1024, 1280} && report.rings[0].wptr == 1280,
           "gfx ring: clear state padded to 1024 dwords, ring test padded to 1280");
    const auto ring_words = fixture.dwords(gfx_ring, 1300);
    expect(ring_words.at(0) == 0xc0004a00u && ring_words.at(1) == 0x20000000u &&
           ring_words.at(2) == 0xc0012800u && ring_words.at(3) == 0x80000000u &&
           ring_words.at(4) == 0x80000000u && ring_words.at(5) == 0xc0d76900u && ring_words.at(6) == 0,
           "cp_gfx_start: PREAMBLE begin, CONTEXT_CONTROL, first SET_CONTEXT_REG extent (215 regs)");
    const std::vector<uint32_t> tail{0xc0016900u, 0xd7, 0, 0xc0004a00u, 0x30000000u, 0xc0001200u, 0};
    expect(std::vector<uint32_t>(ring_words.begin() + 953, ring_words.begin() + 960) == tail &&
           ring_words.at(960) == 0xc03e1000u && ring_words.at(961) == ring_nop &&
           ring_words.at(1023) == ring_nop,
           "steering override 0, PREAMBLE end, CLEAR_STATE; 64-dword NOP pad (header count 62)");
    expect(ring_words.at(1024) == 0xc0017900u && ring_words.at(1025) == 0x40 &&
           ring_words.at(1026) == 0xdeadbeefu && ring_words.at(1027) == 0xc0fb1000u &&
           ring_words.at(1028) == ring_nop && ring_words.at(1279) == ring_nop &&
           ring_words.at(1280) == ring_nop, "gfx ring test: SET_UCONFIG_REG scratch-0xc000, 253-dword pad");
    expect(gfx_queue.packets.size() == 10 + 3 + 2, "CP executed the clear-state stream and ring test");

    bool compute_rings = true;

    for (uint32_t index = 0; index < 8; ++index) {
        const uint32_t doorbell = (3 + index) * 2;
        const auto& queue = model.queues.at(doorbell);
        const auto words = fixture.dwords(fixture.address(fixture.memory.compute_ring[index]), 257);
        compute_rings = compute_rings && queue.doorbells == std::vector<uint64_t>{256} &&
            words.at(0) == 0xc0017900u && words.at(1) == 0x40 && words.at(2) == 0xdeadbeefu &&
            words.at(3) == 0xc0fb1000u && words.at(255) == ring_nop && words.at(256) == ring_nop &&
            report.rings[1 + index].wptr == 256 && model.vram64(wb + (1 + index) * 160 + 32) == 256 &&
            fixture.vram(wb + (1 + index) * 160) == 256 &&
            report.rings[1 + index].ring_test == Gfx11RingTest::passed && queue.type == 1;
    }

    expect(compute_rings, "every compute ring test: 3 dwords + PACKET3(NOP, 251) pad, doorbell 256, rptr reported");
    expect(model.vram64(wb + 32) == 1280 && fixture.vram(wb) == 1280 &&
           report.rings[0].ring_test == Gfx11RingTest::passed, "gfx wptr shadow and rptr writeback");
    bool cond = true;

    for (uint32_t ring = 0; ring < 9; ++ring) {
        cond = cond && fixture.vram(wb + ring * 160 + 128) == 1 && fixture.vram(wb + ring * 160 + 64) == 0;
    }

    expect(cond, "cond_exec slots 1, fence slots 0");
    expect(model.device->reg(scratch) == 0xdeadbeefu && report.failed_ring == UINT32_MAX &&
           report.queues_mapped == 9 && report.clear_state_emitted && report.memory_retained,
           "all rings passed; nine queues mapped");
    expect(model.flushed_before_every_doorbell, "HDP flushed before every doorbell");
    bool addresses = report.compute_eop_mc == eop && report.writeback_mc == wb &&
                     report.rings[0].ring_mc == gfx_ring && report.rings[0].mqd_mc == gfx_mqd &&
                     report.rings[0].eop_mc == 0;

    for (uint32_t index = 0; index < 8; ++index) {
        addresses = addresses && report.rings[1 + index].eop_mc == eop + index * 2048 &&
                    report.rings[1 + index].rptr_mc == wb + (1 + index) * 160 &&
                    report.rings[1 + index].wptr_mc == wb + (1 + index) * 160 + 32;
    }

    expect(addresses, "report MC addresses: rings, MQDs, EOP slices and writeback slots");
    expect(fixture.queues->resume(*fixture.manager, fixture.memory, *fixture.mes) ==
           Gfx11QueuesStatus::busy && fixture.queues->release() == Gfx11QueuesStatus::busy &&
           fixture.queues->initialize(fixture.gc, model.io(), model.doorbells(), fixture.nbio,
                                      rx7900_test::navi31_vram(), {8}, {1000},
                                      {nullptr, Fixture::confirm}) == Gfx11QueuesStatus::busy,
           "resume/release/initialize refused while running");

    // hw_fini: disable_kgq then disable_kcq through MES REMOVE_QUEUE.
    const size_t frames = model.mes_frames.size();
    model.writes.clear();
    expect(fixture.queues->fini(*fixture.mes) == Gfx11QueuesStatus::stopped &&
           model.writes.empty() && model.queues.empty(), "fini unmaps all nine queues without MMIO");
    bool removes = model.mes_frames.size() == frames + 18;

    for (uint32_t index = 0; removes && index < 9; ++index) {
        const Gfx11RingReport& info = report.rings[index];
        const auto& frame = model.mes_frames.at(frames + index * 2);
        removes = frame.at(0) == 0x40031 && frame.at(1) == info.doorbell_index && frame.at(4) == 8 &&
                  frame.at(9) == info.pipe && frame.at(10) == info.queue &&
                  frame.at(14) == (index == 0 ? 0u : 1u) && frame.at(2) == 0 && frame.at(3) == 0;
    }

    expect(removes, "REMOVE_QUEUE unmap_legacy_queue: KGQ first, then KCQs, with queue types");
    expect(fixture.queues->report().queues_mapped == 0 && fixture.queues->fini(*fixture.mes) ==
           Gfx11QueuesStatus::stopped && model.mes_frames.size() == frames + 18,
           "repeated fini returns stopped without MES traffic");
    expect(fixture.queues->release() == Gfx11QueuesStatus::released &&
           !fixture.queues->report().memory_retained, "release after fini");
    expect(fixture.mes->kiq_hw_fini() == Mes11Status::disabled && fixture.mes->release() ==
           Mes11Status::released && fixture.leases_free(), "MES halted afterwards; every lease returned");
}

void fewer_rings() {
    Fixture fixture(true, 1);
    expect(fixture.queues->resume(*fixture.manager, fixture.memory, *fixture.mes) ==
           Gfx11QueuesStatus::running, "one KCQ resumes");
    const uint64_t mqd = fixture.address(fixture.memory.compute_mqd[0]);
    expect(!fixture.queues->report().rings[1].high_priority && fixture.vram(mqd + 580) == 0xd030890au &&
           fixture.vram(mqd + 532) == 0 && fixture.vram(mqd + 536) == 0,
           "a single KCQ is not high priority (num_compute_rings > 1 policy)");
    expect(fixture.shutdown_all(), "single-KCQ teardown");
}

void ring_timeout() {
    Fixture fixture(true, 8, 50);
    fixture.model.hung_doorbell = (3 + 3) * 2;
    expect(fixture.queues->resume(*fixture.manager, fixture.memory, *fixture.mes) ==
           Gfx11QueuesStatus::ring_test_failed, "compute ring 3 never executes: ring test times out");
    const Gfx11QueuesReport report = fixture.queues->report();
    expect(report.failed_ring == 4 && report.rings[0].ring_test == Gfx11RingTest::passed &&
           report.rings[3].ring_test == Gfx11RingTest::passed &&
           report.rings[4].ring_test == Gfx11RingTest::timeout &&
           report.rings[5].ring_test == Gfx11RingTest::not_run,
           "tests stop at the first failure (amdgpu_ring_test_helper return); failed ring reported");
    size_t polls = 0;

    for (uint64_t delay : fixture.model.device->delays) {
        polls += delay == 1 ? 1 : 0;
    }

    expect(polls >= 50, "usec_timeout polls with 1 us delays");
    expect(fixture.queues->release() == Gfx11QueuesStatus::quarantined &&
           fixture.queues->resume(*fixture.manager, fixture.memory, *fixture.mes) ==
               Gfx11QueuesStatus::quarantined && report.memory_retained,
           "quarantined: no release, no resume, leases kept");
    expect(fixture.queues->fini(*fixture.mes) == Gfx11QueuesStatus::quarantined &&
           fixture.model.queues.empty() && fixture.queues->report().memory_retained,
           "fini still unmaps every queue but stays quarantined");
    expect(fixture.queues->release_after_reset() == Gfx11QueuesStatus::quarantined,
           "no reset proof: leases kept");
    fixture.reset_confirmed = true;
    expect(fixture.queues->release_after_reset() == Gfx11QueuesStatus::released &&
           !fixture.queues->report().memory_retained &&
           fixture.queues->report().failed_ring == UINT32_MAX, "confirmed reset releases every lease");
    expect(fixture.mes->kiq_hw_fini() == Mes11Status::disabled && fixture.mes->release() ==
           Mes11Status::released && fixture.leases_free(), "no lease leaked");
}

void hardware_failures() {
    Fixture read_fail;
    read_fail.model.fail_read_index = gc0 + 0x1fba;
    const size_t frames = read_fail.model.mes_frames.size();
    expect(read_fail.queues->resume(*read_fail.manager, read_fail.memory, *read_fail.mes) ==
           Gfx11QueuesStatus::mmio_failure && read_fail.model.select == 0 &&
           read_fail.model.mes_frames.size() == frames && read_fail.queues->report().memory_retained,
           "failed MQD register read: quarantined, selection restored, nothing mapped");
    expect(read_fail.queues->release() == Gfx11QueuesStatus::quarantined, "leases kept");

    Fixture write_fail;
    write_fail.model.fail_write_index = cp_max_context;
    expect(write_fail.queues->resume(*write_fail.manager, write_fail.memory, *write_fail.mes) ==
           Gfx11QueuesStatus::mmio_failure && write_fail.queues->report().queues_mapped == 9 &&
           !write_fail.queues->report().clear_state_emitted,
           "failed CP_MAX_CONTEXT store after the KGQ map quarantines");
    write_fail.reset_confirmed = true;
    expect(write_fail.queues->release_after_reset() == Gfx11QueuesStatus::released,
           "reset releases the quarantined leases");

    Fixture doorbell;
    doorbell.model.fail_doorbell = gfx_doorbell;
    expect(doorbell.queues->resume(*doorbell.manager, doorbell.memory, *doorbell.mes) ==
           Gfx11QueuesStatus::doorbell_failure && doorbell.queues->report().rings[0].mapped,
           "failed gfx doorbell store quarantines");

    Fixture seed;
    seed.model.fail_write_index = scratch;
    expect(seed.queues->resume(*seed.manager, seed.memory, *seed.mes) == Gfx11QueuesStatus::mes_failure &&
           seed.queues->report().set_resources_status == Mes11Status::mmio_failure &&
           seed.queues->report().mes_status == Mes11Status::quarantined &&
           seed.queues->report().queues_mapped == 0,
           "KIQ SET_RESOURCES failure: Linux continues, the quarantined MES refuses the first map");

    Fixture mes;
    mes.model.ignore_opcode = 2;
    expect(mes.queues->resume(*mes.manager, mes.memory, *mes.mes) == Gfx11QueuesStatus::mes_failure &&
           mes.queues->report().mes_status == Mes11Status::timeout &&
           mes.queues->report().queues_mapped == 0, "ADD_QUEUE without API status: MES timeout quarantines");
    expect(mes.queues->fini(*mes.mes) == Gfx11QueuesStatus::quarantined, "fini after a fault stays quarantined");

    Fixture unmap;
    unmap.queues->resume(*unmap.manager, unmap.memory, *unmap.mes);
    unmap.model.ignore_opcode = 3;
    expect(unmap.queues->fini(*unmap.mes) == Gfx11QueuesStatus::mes_failure &&
           unmap.queues->report().queues_mapped == 9 &&
           unmap.queues->release() == Gfx11QueuesStatus::quarantined,
           "REMOVE_QUEUE timeout: queues still counted mapped, leases kept");
}

// gfx_v11_0_ring_test_ib frame (amdgpu_ib_schedule, no job) expected on ring r,
// hand-derived from nvd.h and gfx_v11_0.c; seq is ++fence_drv.sync_seq.
std::vector<uint32_t> expected_ib_frame(uint32_t ring, uint32_t pipe, uint64_t wb, uint64_t ib,
                                        uint32_t seq, uint32_t ib_dwords = 5) {
    const bool gfx = ring == 0;
    const uint64_t cond = wb + ring * 160 + 128;
    const uint64_t fence = wb + ring * 160 + 64;
    const uint32_t ref = (gfx ? 1u : 4u) << pipe;
    // ACQUIRE_MEM: PACKET3(0x58, 6), COHER_CNTL 0, SIZE 0xffffffff/0xffffff,
    // BASE 0/0, POLL_INTERVAL 0xA, GCR_CNTL 0xC3B1.
    std::vector<uint32_t> f{0xc0065800u, 0, 0xffffffffu, 0x00ffffffu, 0, 0, 0xa, 0xc3b1u};

    if (gfx) {
        // COND_EXEC on cond_exe_gpu_addr; count patched to 7 + 4 + 5 + 8 = 24.
        f.insert(f.end(), {0xc0032200u, lo(cond), hi(cond), 0, 24});
    }

    // WAIT_REG_MEM: OPERATION 1, FUNCTION 3, ENGINE PFP (gfx) / ME (compute).
    f.insert(f.end(), {0xc0053c00u, gfx ? 0x143u : 0x43u, hdp_flush_req, hdp_flush_done, ref, ref,
                       0x20});
    // INDIRECT_BUFFER: length 5, vmid 0; compute adds INDIRECT_BUFFER_VALID.
    f.insert(f.end(), {0xc0023f00u, lo(ib), hi(ib), gfx ? ib_dwords : 0x00800000u | ib_dwords});

    if (gfx) {
        // Second COND_EXEC: its offset is discarded, count stays 0.
        f.insert(f.end(), {0xc0032200u, lo(cond), hi(cond), 0, 0});
    }

    // RELEASE_MEM: GCR_SEQ|GL2_WB|GLM_INV|GLM_WB|CACHE_POLICY 3|EVENT 0x14|INDEX 5;
    // DATA_SEL 1 (32-bit), INT_SEL 2; fence address, seq lo/hi, 0.
    f.insert(f.end(), {0xc0064900u, 0x06603514u, 0x22000000u, lo(fence), hi(fence), seq, 0, 0});
    const uint32_t pad = 256 - static_cast<uint32_t>(f.size());
    f.push_back(0xc0001000u | ((pad - 2) << 16));

    while (f.size() < 256) {
        f.push_back(ring_nop);
    }

    return f;
}

void ib_tests() {
    Fixture fixture;
    Model& model = fixture.model;
    expect(fixture.queues->resume(*fixture.manager, fixture.memory, *fixture.mes) ==
           Gfx11QueuesStatus::running, "ib: queues running before late-init IB tests");
    const uint64_t wb = fixture.address(fixture.memory.writeback);
    const uint64_t ib = fixture.address(fixture.ib);
    const size_t mes_frames = model.mes_frames.size();
    model.writes.clear();
    model.device->delays.clear();

    expect(fixture.test_ib() == Gfx11QueuesStatus::running,
           "amdgpu_ib_ring_tests: gfx ring then eight compute rings pass");
    Gfx11QueuesReport report = fixture.queues->report();
    expect(report.ib_rings_passed == 9 && report.ib_failed_ring == UINT32_MAX &&
           !report.ib_primary_failed && !report.ib_buffer_retained && fixture.leases(fixture.ib) == 0,
           "all nine passed; IB buffer lease returned after every fence signalled");
    expect(model.writes.empty() && model.mes_frames.size() == mes_frames,
           "IB tests use no GC register store and no MES traffic (KIQ and MES rings skipped)");

    const auto& gfx_queue = model.queues.at(gfx_doorbell);
    const auto gfx_words = fixture.dwords(fixture.address(fixture.memory.gfx_ring) + 1280 * 4, 256);
    expect(gfx_words == expected_ib_frame(0, 0, wb, ib, 1),
           "gfx ring frame: ACQUIRE_MEM, COND_EXEC(24), HDP WAIT_REG_MEM PFP cp0, IB(5), "
           "COND_EXEC(0), RELEASE_MEM seq 1, PACKET3(NOP, 217) pad");
    expect(gfx_queue.doorbells == std::vector<uint64_t>{1024, 1280, 1536} &&
           report.rings[0].wptr == 1536 && model.vram64(wb + 32) == 1536,
           "gfx IB frame padded to 256 dwords; wptr shadow and doorbell 1536");
    const std::vector<uint32_t> ib_words{0xc0033700u, 0x00100500u, lo(wb + 45 * 32), hi(wb + 45 * 32),
                                         0xdeadbeefu};
    expect(fixture.dwords(ib, 5) == ib_words && gfx_queue.ib_packets.size() == 1 &&
           gfx_queue.ib_packets.at(0) == ib_words,
           "gfx IB: PACKET3(WRITE_DATA, 3), DST_SEL 5 | WR_CONFIRM, wb slot 45, 0xDEADBEEF");
    expect(gfx_queue.hdp_flush_refs == std::vector<uint32_t>{1} &&
           gfx_queue.fence_interrupts == std::vector<uint32_t>{2},
           "gfx ring HDP flush ref_and_mask cp0, EOP interrupt requested (INT_SEL 2)");

    bool compute = true;

    for (uint32_t index = 0; index < 8; ++index) {
        const uint32_t ring = 1 + index;
        const auto& queue = model.queues.at((3 + index) * 2);
        const Gfx11RingReport& info = report.rings[ring];
        const auto words = fixture.dwords(fixture.address(fixture.memory.compute_ring[index]) + 256 * 4,
                                          256);
        const uint64_t ring_ib = ib + ring * 256;
        const uint64_t test = wb + (45 + ring) * 32;
        compute = compute && words == expected_ib_frame(ring, info.pipe, wb, ring_ib, 1) &&
                  queue.doorbells == std::vector<uint64_t>{256, 512} && info.wptr == 512 &&
                  fixture.dwords(ring_ib, 5) ==
                      std::vector<uint32_t>{0xc0033700u, 0x00100500u, lo(test), hi(test), 0xdeadbeefu} &&
                  queue.hdp_flush_refs == std::vector<uint32_t>{4u << info.pipe} &&
                  info.ib_test == Gfx11RingTest::passed && info.fence_seq == 1 &&
                  info.fence_value == 1 && fixture.vram(wb + ring * 160 + 64) == 1 &&
                  fixture.vram(test) == 0xdeadbeefu && info.ib_mc == ring_ib &&
                  info.ib_test_mc == test && info.fence_mc == wb + ring * 160 + 64 &&
                  !info.ib_disabled;
    }

    expect(compute, "compute frames: ACQUIRE_MEM, HDP WAIT_REG_MEM ME cp2 << pipe, "
                    "IB VALID | 5, RELEASE_MEM seq 1, PACKET3(NOP, 227); fences and destinations");
    expect(report.rings[0].ib_test == Gfx11RingTest::passed && report.rings[0].fence_seq == 1 &&
           fixture.vram(wb + 64) == 1 && fixture.vram(wb + 45 * 32) == 0xdeadbeefu &&
           fixture.vram(wb + 128) == 1, "gfx fence slot holds seq 1, destination 0xDEADBEEF, cond_exec 1");
    expect(model.flushed_before_every_doorbell, "CPU HDP flush before every IB-test doorbell");
    expect(model.device->delays.empty() && report.rings[0].ib_polls == 0,
           "fences already signalled: no poll delay");

    expect(fixture.test_ib() == Gfx11QueuesStatus::running &&
           fixture.queues->report().rings[0].fence_seq == 2 &&
           fixture.vram(wb + 64) == 2 && fixture.queues->report().rings[8].fence_seq == 2 &&
           fixture.dwords(fixture.address(fixture.memory.gfx_ring) + 1536 * 4, 256) ==
               expected_ib_frame(0, 0, wb, ib, 2), "a second run emits fence seq 2 at wptr 1536");
    expect(fixture.shutdown_all(), "IB-tested queues still unmap and release every lease");
}

void ib_refusals() {
    Fixture fixture;
    expect(fixture.test_ib() == Gfx11QueuesStatus::busy && !fixture.queues->report().ib_buffer_retained,
           "test_ib before resume refused");
    fixture.queues->resume(*fixture.manager, fixture.memory, *fixture.mes);
    Model& model = fixture.model;
    const auto doorbells = model.queues.at(gfx_doorbell).doorbells.size();
    const auto none = [&] {
        return model.queues.at(gfx_doorbell).doorbells.size() == doorbells &&
               fixture.leases(fixture.ib) == 0 && !fixture.queues->report().ib_buffer_retained;
    };

    expect(fixture.test_ib(0) == Gfx11QueuesStatus::invalid_argument && none(), "zero timeout refused");
    expect(fixture.test_ib(gfx11_ib_test_xgmi_timeout_us + 1) == Gfx11QueuesStatus::invalid_argument &&
           none(), "timeout above AMDGPU_IB_TEST_GFX_XGMI_TIMEOUT refused");
    expect(fixture.queues->test_ib(*fixture.manager, fixture.ib, {NbioRegisters{}, 1000}) ==
           Gfx11QueuesStatus::invalid_argument && none(), "invalid NBIO token refused");
    expect(fixture.queues->test_ib(*fixture.manager, fixture.memory.writeback,
                                   {fixture.nbio_registers, 1000}) == Gfx11QueuesStatus::invalid_memory &&
           none() && fixture.leases(fixture.memory.writeback) == 1,
           "IB buffer overlapping the writeback refused without an extra lease");
    expect(fixture.queues->test_ib(*fixture.manager, rx7900::memory::Handle{},
                                   {fixture.nbio_registers, 1000}) ==
           Gfx11QueuesStatus::invalid_memory && none(), "stale IB buffer handle refused");
    rx7900::memory::Manager other;
    expect(fixture.queues->test_ib(other, fixture.ib, {fixture.nbio_registers, 1000}) ==
           Gfx11QueuesStatus::invalid_memory && none(), "IB buffer from another manager refused");
    rx7900::memory::Handle read_only{};
    fixture.manager->allocate(4096, 4096, rx7900::memory::Access::read, true, read_only);
    expect(fixture.queues->test_ib(*fixture.manager, read_only, {fixture.nbio_registers, 1000}) ==
           Gfx11QueuesStatus::running && fixture.leases(read_only) == 0,
           "GPU read-only IB buffer accepted: the CP only fetches it");
    expect(fixture.manager->release(read_only) == rx7900::memory::Status::success &&
           fixture.shutdown_all(), "refusals left no lease behind");
}

void ib_compute_timeout() {
    Fixture fixture;
    fixture.queues->resume(*fixture.manager, fixture.memory, *fixture.mes);
    Model& model = fixture.model;
    model.hung_doorbell = (3 + 3) * 2;
    model.device->delays.clear();
    const uint32_t timeout = 50;

    expect(fixture.test_ib(timeout) == Gfx11QueuesStatus::ib_test_failed,
           "compute ring 3 never runs its IB frame: IB test fails");
    Gfx11QueuesReport report = fixture.queues->report();
    bool others = true;

    for (uint32_t ring = 0; ring < 9; ++ring) {
        others = others && (ring == 4 || (report.rings[ring].ib_test == Gfx11RingTest::passed &&
                                          !report.rings[ring].ib_disabled));
    }

    expect(report.rings[4].ib_test == Gfx11RingTest::timeout && report.rings[4].ib_disabled &&
           report.rings[4].ib_polls == timeout && report.rings[4].fence_seq == 1 &&
           report.rings[4].fence_value == 0 && others && report.ib_rings_passed == 8 &&
           report.ib_failed_ring == 4 && !report.ib_primary_failed,
           "Linux continues after a compute failure: ring 4 timed out, the other eight passed");
    size_t polls = 0;

    for (uint64_t delay : model.device->delays) {
        polls += delay == 1 ? 1 : 0;
    }

    expect(polls == timeout, "fence polled once per microsecond up to the timeout");
    expect(report.ib_buffer_retained && fixture.leases(fixture.ib) == 1 &&
           fixture.queues->report().status == Gfx11QueuesStatus::ib_test_failed,
           "timed-out ring keeps the IB buffer leased");
    const size_t gfx_doorbells = model.queues.at(gfx_doorbell).doorbells.size();
    expect(fixture.test_ib() == Gfx11QueuesStatus::quarantined &&
           model.queues.at(gfx_doorbell).doorbells.size() == gfx_doorbells,
           "no further IB submission while the IB buffer is quarantined");
    expect(fixture.queues->fini(*fixture.mes) == Gfx11QueuesStatus::stopped && model.queues.empty() &&
           fixture.queues->release() == Gfx11QueuesStatus::quarantined &&
           fixture.queues->report().memory_retained, "fini unmaps; release refused: ring quarantined");
    expect(fixture.queues->release_after_reset() == Gfx11QueuesStatus::quarantined &&
           fixture.leases(fixture.ib) == 1, "unconfirmed reset keeps the IB lease");
    fixture.reset_confirmed = true;
    expect(fixture.queues->release_after_reset() == Gfx11QueuesStatus::released &&
           fixture.leases(fixture.ib) == 0 && !fixture.queues->report().rings[4].ib_disabled &&
           fixture.queues->report().rings[4].ib_test == Gfx11RingTest::not_run,
           "confirmed reset releases the IB buffer and clears the ring quarantine");
    expect(fixture.mes->kiq_hw_fini() == Mes11Status::disabled &&
           fixture.mes->release() == Mes11Status::released && fixture.leases_free(), "no lease leaked");
}

void ib_gfx_failures() {
    Fixture fixture;
    fixture.queues->resume(*fixture.manager, fixture.memory, *fixture.mes);
    fixture.model.hung_doorbell = gfx_doorbell;
    expect(fixture.test_ib(20) == Gfx11QueuesStatus::ib_test_failed,
           "gfx ring IB timeout fails the IB tests");
    Gfx11QueuesReport report = fixture.queues->report();
    bool untested = true;

    for (uint32_t ring = 1; ring < 9; ++ring) {
        untested = untested && report.rings[ring].ib_test == Gfx11RingTest::not_run;
    }

    expect(report.ib_primary_failed && report.ib_failed_ring == 0 &&
           report.rings[0].ib_test == Gfx11RingTest::timeout && untested,
           "gfx_ring[0] failure returns at once (accel_working false): compute rings untested");

    Fixture corrupt;
    corrupt.queues->resume(*corrupt.manager, corrupt.memory, *corrupt.mes);
    corrupt.model.corrupt_ib_doorbell = (3 + 6) * 2;
    expect(corrupt.test_ib() == Gfx11QueuesStatus::ib_test_failed &&
           corrupt.queues->report().rings[7].ib_test == Gfx11RingTest::failed &&
           corrupt.queues->report().rings[7].ib_disabled &&
           corrupt.queues->report().rings[7].fence_value == 1 &&
           corrupt.queues->report().rings[8].ib_test == Gfx11RingTest::passed &&
           corrupt.queues->report().ib_failed_ring == 7,
           "fence signalled but wrong destination value: -EINVAL, ring disabled");

    Fixture stuck;
    stuck.queues->resume(*stuck.manager, stuck.memory, *stuck.mes);
    stuck.model.stuck_hdp_doorbell = (3 + 1) * 2;
    expect(stuck.test_ib(10) == Gfx11QueuesStatus::ib_test_failed &&
           stuck.queues->report().rings[2].ib_test == Gfx11RingTest::timeout &&
           stuck.model.queues.at((3 + 1) * 2).ib_packets.empty(),
           "HDP flush never completes: the CP stalls before the IB and the fence times out");

    Fixture doorbell;
    doorbell.queues->resume(*doorbell.manager, doorbell.memory, *doorbell.mes);
    doorbell.model.fail_doorbell = (3 + 2) * 2;
    expect(doorbell.test_ib() == Gfx11QueuesStatus::doorbell_failure &&
           doorbell.queues->report().ib_failed_ring == 3 &&
           doorbell.queues->report().ib_buffer_retained &&
           doorbell.test_ib() == Gfx11QueuesStatus::quarantined &&
           doorbell.queues->fini(*doorbell.mes) == Gfx11QueuesStatus::quarantined,
           "failed IB-test doorbell store quarantines the whole protocol");
    doorbell.reset_confirmed = true;
    expect(doorbell.queues->release_after_reset() == Gfx11QueuesStatus::released &&
           doorbell.leases(doorbell.ib) == 0, "reset releases the IB buffer after a protocol fault");
}


// The shared builders Mes11::self_test reuses for MES user queues.
void shared_builders() {
    std::vector<uint8_t> ring(4096, 0);
    Gfx11RingWriter writer{rx7900::sdma::CommandBuffer(ring.data(), ring.size()), 1020};
    const auto dword = [&](uint64_t position) {
        uint32_t value = 0;
        rx7900::ByteView(ring.data(), ring.size()).read_u32((position & 1023) * 4, value);
        return value;
    };
    const Gfx11IbFrame compute{false, false, true, 2, 0, 0, hdp_flush_req, hdp_flush_done,
                               0x10100, 5, 0x10040, 7};
    expect(gfx11_emit_ib_frame(writer, compute) && writer.wptr == 1020 + 7 + 4 + 8 &&
           dword(1020) == 0xC0053C00u && dword(1021) == 0x43 && dword(1024) == 4 &&
           dword(1027) == 0xC0023F00u && dword(1030) == (5u | (1u << 23) | 0x40000000u) &&
           dword(1031) == 0xC0064900u && dword(1036) == 7 && dword(1038) == 0x01000002u,
           "MES compute IB frame wraps the ring: no ACQUIRE_MEM, inherit-vmid IB, queue context");
    writer.wptr = 2;
    const Gfx11IbFrame gfx{true, false, true, 1, 0, 0x10060, hdp_flush_req, hdp_flush_done,
                           0x10100, 5, 0x10040, 1};
    expect(gfx11_emit_ib_frame(writer, gfx) && dword(2) == 0xC0032200u && dword(6) == 24 &&
           dword(17) == (5u | 0x00400000u) && dword(30) == 0x01000001u,
           "MES gfx IB frame: COND_EXEC patched to 24, IB inherit-vmid 0x400000");
    const uint64_t before = writer.wptr;
    const Gfx11IbFrame bad_pipe{false, true, false, 0, 4, 0, 0, 0, 0x100, 5, 0x40, 1};
    const Gfx11IbFrame bad_ib{false, true, false, 0, 0, 0, 0, 0, 0x102, 5, 0x40, 1};
    const Gfx11IbFrame bad_queue{false, false, true, 0x01000000u, 0, 0, 0, 0, 0x100, 5, 0x40, 1};
    expect(!gfx11_emit_ib_frame(writer, bad_pipe) && !gfx11_emit_ib_frame(writer, bad_ib) &&
           !gfx11_emit_ib_frame(writer, bad_queue) && writer.wptr == before,
           "pipe > 3, unaligned IB or oversized queue id refused before any store");
    expect(!gfx11_emit_ring_test(writer, 0xc400) && !gfx11_emit_ring_test(writer, 0xbfff) &&
           writer.wptr == before, "SCRATCH outside the SET_UCONFIG_REG window refused");
    writer.wptr = 255;
    expect(gfx11_ring_pad(writer) && writer.wptr == 256 && dword(255) == ring_nop &&
           gfx11_ring_pad(writer) && writer.wptr == 256, "single-NOP pad and aligned no-op pad");
    Gfx11RingWriter odd{rx7900::sdma::CommandBuffer(ring.data(), 12), 0};
    expect(!gfx11_ring_write(odd, 1), "non-power-of-two ring refused");
    std::vector<uint8_t> mqd(gfx11_mqd_bytes, 0x5a);
    const Gfx11MqdProperties properties{0x1000, 0x2000, 0x3000, 0x3020, 0x4000, 3000, 0x80a, 1, 0,
                                        false, false};
    expect(!gfx11_compute_mqd_init({{}, gc0, gc1}, {1, 0, 0}, properties,
                                   rx7900::sdma::CommandBuffer(mqd.data(), mqd.size())) &&
           mqd[0] == 0x5a, "non-power-of-two queue size refused before the MQD is touched");
}

// FenceManager platform for the submission tests: lock flag, settable clock,
// events unused (waits poll), plus the atomic WPTR shadow store double.
struct FenceModel final {
    bool held = false;
    unsigned errors = 0;
    uint64_t now = 1000;
    unsigned stores = 0;
    uint64_t last_store = 0;

    FencePlatform platform() {
        return {this,
                [](void* c) noexcept { auto& p = *static_cast<FenceModel*>(c); p.errors += p.held; p.held = true; },
                [](void* c) noexcept { auto& p = *static_cast<FenceModel*>(c); p.errors += !p.held; p.held = false; },
                [](void* c) noexcept { return static_cast<FenceModel*>(c)->now; },
                [](void*, uint32_t) noexcept {}, [](void*, uint32_t) noexcept {},
                [](void* c, uint32_t, uint64_t timeout) noexcept {
                    static_cast<FenceModel*>(c)->now += timeout;
                    return false;
                }, nullptr, nullptr};
    }

    static bool store(void* context, uint8_t* address, uint64_t value) noexcept {
        auto& self = *static_cast<FenceModel*>(context);
        ++self.stores;
        self.last_store = value;
        const rx7900::sdma::CommandBuffer slot(address, 8);
        return slot.write_u32(0, lo(value)) && slot.write_u32(4, hi(value));
    }

    RingPointerStore wptr_store() noexcept { return {this, store}; }
};

struct SubmitFixture final {
    Fixture fixture;
    FenceModel model;
    FenceManager fences;
    rx7900::memory::Handle user{};
    uint64_t user_mc = 0;
    uint64_t wb = 0;

    explicit SubmitFixture(bool run_ib_tests = true) {
        fixture.queues->resume(*fixture.manager, fixture.memory, *fixture.mes);

        if (run_ib_tests) {
            fixture.test_ib();
        }

        fences.initialize(model.platform());
        fixture.allocate(4 * 4096, user);
        user_mc = fixture.address(user);
        wb = fixture.address(fixture.memory.writeback);
    }

    Gfx11QueuesStatus enable() {
        return fixture.queues->enable_submission(fences, {fixture.nbio_registers, model.wptr_store()});
    }

    // A WRITE_DATA IB at user byte offset storing value to user byte dest.
    void write_ib(uint32_t offset, uint32_t dest, uint32_t value) {
        const uint64_t target = user_mc + dest;
        const uint32_t words[5]{0xc0033700u, 0x00100500u, lo(target), hi(target), value};

        for (uint32_t index = 0; index < 5; ++index) {
            fixture.model.device->put_vram_u32(user_mc + offset + index * 4, words[index]);
        }
    }

    Gfx11QueuesStatus submit(uint32_t ring, uint32_t offset, uint32_t& seq, uint32_t dwords = 5) {
        return fixture.queues->submit({ring, user, offset, dwords}, seq);
    }

    uint32_t doorbell(uint32_t ring) const { return ring == 0 ? gfx_doorbell : (3 + ring - 1) * 2; }
};

void submission_enable() {
    SubmitFixture before(false);
    expect(before.enable() == Gfx11QueuesStatus::unavailable &&
           !before.fixture.queues->report().submission_enabled,
           "no ring passed an IB test: nothing to attach");
    SubmitFixture s;
    expect(s.fixture.queues->enable_submission(s.fences, {s.fixture.nbio_registers, {nullptr, nullptr}}) ==
           Gfx11QueuesStatus::invalid_argument, "WPTR store required");
    expect(s.enable() == Gfx11QueuesStatus::running, "enable after the IB tests");
    const Gfx11QueuesReport report = s.fixture.queues->report();
    bool attached = report.submission_enabled;

    for (uint32_t ring = 0; ring < 9; ++ring) {
        const FenceRingReport fence = s.fences.ring_report(ring);
        attached = attached && report.rings[ring].submit_enabled &&
                   fence.state == FenceRingState::active && fence.sync_seq == 1 &&
                   fence.last_seq == 1 && fence.retired_position == report.rings[ring].wptr &&
                   fence.me == report.rings[ring].me && fence.pipe == report.rings[ring].pipe &&
                   fence.queue == report.rings[ring].queue;
    }

    expect(attached, "every IB-tested ring attached with the IB test's sync_seq and position");
    expect(s.enable() == Gfx11QueuesStatus::busy && s.fixture.test_ib() == Gfx11QueuesStatus::busy,
           "enable once; no IB tests on fence-managed rings");
    // An IB test failure keeps that ring out of submission.
    SubmitFixture failed(false);
    failed.fixture.model.hung_doorbell = (3 + 3) * 2;
    failed.fixture.test_ib(50);
    expect(failed.enable() == Gfx11QueuesStatus::busy, "IB buffer still quarantined: refused");
    expect(s.fixture.queues->disable_submission() == Gfx11QueuesStatus::running &&
           s.fences.ring_report(0).state == FenceRingState::detached &&
           !s.fixture.queues->report().submission_enabled, "idle rings detach");
    expect(s.model.errors == 0, "fence lock balanced");
}

void submission_frames() {
    SubmitFixture s;
    s.enable();
    Model& model = s.fixture.model;
    s.write_ib(0, 2048, 0x11112222u);
    uint32_t seq = 0;
    expect(s.submit(0, 0, seq) == Gfx11QueuesStatus::running && seq == 2,
           "gfx submit returns fence sequence 2 (after the IB test's 1)");
    const Gfx11QueuesReport report = s.fixture.queues->report();
    const auto words = s.fixture.dwords(s.fixture.address(s.fixture.memory.gfx_ring) + 1536 * 4, 256);
    expect(words == expected_ib_frame(0, 0, s.wb, s.user_mc, 2),
           "amdgpu_ib_schedule frame of the IB-test layout with the caller's IB and seq 2");
    expect(report.rings[0].wptr == 1792 && model.queues.at(gfx_doorbell).doorbells.back() == 1792 &&
           s.model.stores == 1 && s.model.last_store == 1792 && model.vram64(s.wb + 32) == 1792,
           "atomic WPTR shadow store, then WDOORBELL64");
    expect(s.fixture.vram(s.user_mc + 2048) == 0x11112222u && s.fixture.vram(s.wb + 64) == 2 &&
           model.queues.at(gfx_doorbell).fence_interrupts.back() == 2,
           "CP ran the IB and wrote fence 2 with INT_SEL 2");
    expect(report.rings[0].ib_leases == 1 && s.fixture.leases(s.user) == 1,
           "IB leased until its fence signals");
    expect(s.fences.wait(0, seq, 1000) == FenceStatus::signalled, "wait signalled");
    expect(s.fixture.queues->retire() == Gfx11QueuesStatus::running && s.fixture.leases(s.user) == 0 &&
           s.fixture.queues->report().rings[0].ib_leases == 0, "retire returns the IB lease");
    // Compute ring 5 (MEC1 pipe 0 queue 1): VALID bit, cp2 HDP ref, EOP to IrqService.
    s.write_ib(256, 2052, 0x33334444u);
    expect(s.submit(5, 256, seq) == Gfx11QueuesStatus::running && seq == 2, "compute submit");
    const Gfx11RingReport ring5 = s.fixture.queues->report().rings[5];
    expect(s.fixture.dwords(s.fixture.address(s.fixture.memory.compute_ring[4]) + 512 * 4, 256) ==
               expected_ib_frame(5, ring5.pipe, s.wb, s.user_mc + 256, 2) &&
           s.fixture.vram(s.user_mc + 2052) == 0x33334444u, "compute frame and IB result");
    // Interrupt-driven completion through the IrqService record of that queue.
    IrqReport irq{};
    IrqVmFaultLog log{};
    ++irq.compute_eop[0][ring5.pipe][ring5.queue];
    s.fences.process_interrupts({&irq, &log, false});
    const FenceRingReport fence5 = s.fences.ring_report(5);
    expect(fence5.last_seq == 2 && fence5.interrupt_processes == 1 && !fence5.timeout_armed,
           "EOP record of MEC1 pipe/queue signals compute ring 5");
    expect(model.flushed_before_every_doorbell, "HDP flushed before every submission doorbell");
    expect(s.model.errors == 0, "fence lock balanced");
}

void submission_wrap() {
    SubmitFixture s;
    s.enable();
    bool ok = true;
    uint32_t seq = 0;

    // 40 frames of 256 dwords on a 2048-dword ring from wptr 512: five wraps.
    for (uint32_t index = 0; index < 40; ++index) {
        s.write_ib(index * 32, 8192 + index * 4, 0x5000u + index);
        ok = ok && s.submit(1, index * 32, seq) == Gfx11QueuesStatus::running && seq == 2 + index;
        ok = ok && s.fences.query(1, seq) == FenceStatus::signalled;
    }

    bool results = true;

    for (uint32_t index = 0; index < 40; ++index) {
        results = results && s.fixture.vram(s.user_mc + 8192 + index * 4) == 0x5000u + index;
    }

    const Gfx11RingReport info = s.fixture.queues->report().rings[1];
    expect(ok && results && info.wptr == 512 + 40 * 256 && info.submissions == 40 &&
           info.ib_leases <= fence_slots && s.fences.ring_report(1).last_seq == 41,
           "ring reused across five wraps; every IB ran and every fence signalled");
    expect(info.consumed_position == 512 + 39 * 256 && info.free_dwords == 2048,
           "space accounting: everything before the last frame was consumed");
    // Frame 38 (seq 40) is the last one written at ring offset 0 after the wraps.
    expect(s.fixture.dwords(s.fixture.address(s.fixture.memory.compute_ring[0]), 256) ==
               expected_ib_frame(1, 0, s.wb, s.user_mc + 32 * 38, 40),
           "wrapped frames are written modulo the ring size");
    s.fixture.queues->retire();
    expect(s.fixture.leases(s.user) == 0, "every lease returned after retire");
}

void submission_space_and_fences() {
    SubmitFixture s;
    s.enable();
    Model& model = s.fixture.model;
    const uint32_t ring = 2;
    model.hung_doorbell = s.doorbell(ring);
    uint32_t seq = 0;
    bool ok = true;

    for (uint32_t index = 0; index < 4; ++index) {
        s.write_ib(index * 32, 4096 + index * 4, 0x700u + index);
        ok = ok && s.submit(ring, index * 32, seq) == Gfx11QueuesStatus::running;
    }

    Gfx11RingReport info = s.fixture.queues->report().rings[ring];
    expect(ok && info.consumed_position == 512 && info.free_dwords == 2048 - 768 &&
           info.ib_leases == 4 && s.fixture.leases(s.user) == 4,
           "CP stalled: nothing consumed past the IB test, three frames in flight at the 4th");
    const auto doorbells = model.queues.at(s.doorbell(ring)).doorbells.size();
    expect(s.submit(ring, 0, seq) == Gfx11QueuesStatus::ring_full &&
           model.queues.at(s.doorbell(ring)).doorbells.size() == doorbells &&
           s.fixture.leases(s.user) == 4, "fifth outstanding fence refused without any write");
    // The CP RPTR writeback ahead of every retired fence frees space.
    const uint64_t rptr_slot = s.wb + ring * 160;
    model.device->put_vram_u32(rptr_slot, lo(info.wptr));
    model.hung_doorbell = UINT32_MAX;
    auto& queue = model.queues.at(s.doorbell(ring));
    // The CP catches up with the four frames: fences 2..5 signal.
    model.run_cp(s.doorbell(ring), queue, info.wptr);
    ok = s.submit(ring, 0, seq) == Gfx11QueuesStatus::running && seq == 6;
    info = s.fixture.queues->report().rings[ring];
    expect(ok && info.consumed_position == 512 + 4 * 256 && s.fixture.vram(s.user_mc + 4096 + 12) == 0x703u,
           "after the CP caught up the ring accepts work again");
    // A stale RPTR (behind the retired position) is ignored.
    SubmitFixture t;
    t.enable();
    t.fixture.model.hung_doorbell = t.doorbell(3);
    t.write_ib(0, 4096, 1);
    t.submit(3, 0, seq);
    t.fixture.model.device->put_vram_u32(t.wb + 3 * 160, 100);
    t.submit(3, 0, seq);
    info = t.fixture.queues->report().rings[3];
    expect(info.consumed_position == 512, "stale RPTR does not count as consumed");
    t.fixture.model.device->put_vram_u32(t.wb + 3 * 160, 512 + 256);
    t.submit(3, 0, seq);
    info = t.fixture.queues->report().rings[3];
    expect(info.consumed_position == 512 + 256 && info.free_dwords == 2048 - 256,
           "RPTR inside the unretired span counts (fetched, fence not yet written)");
}

void submission_timeout_quarantine() {
    SubmitFixture s;
    s.enable();
    Model& model = s.fixture.model;
    model.hung_doorbell = s.doorbell(4);
    s.write_ib(0, 4096, 9);
    uint32_t seq = 0;
    s.submit(4, 0, seq);
    s.model.now += 60 * 1000000ULL - 1;
    s.fences.poll();
    expect(s.fences.ring_report(4).state == FenceRingState::active, "compute: within 60 s");
    s.model.now += 1;
    s.fences.poll();
    const FenceRingReport fence = s.fences.ring_report(4);
    expect(fence.state == FenceRingState::hung && fence.hang_reason == FenceHangReason::job_timeout &&
           fence.hang_signalled_seq == 1 && fence.hang_emitted_seq == 2 &&
           s.fences.report().reset_required, "60000 ms compute lockup quarantines the ring");
    expect(s.submit(4, 0, seq) == Gfx11QueuesStatus::ring_hung && s.fixture.leases(s.user) == 1,
           "hung ring refuses work and keeps the IB leased");
    expect(s.submit(1, 0, seq) == Gfx11QueuesStatus::running, "other rings keep working");
    s.fixture.queues->retire();
    expect(s.fixture.queues->disable_submission() == Gfx11QueuesStatus::ring_hung,
           "disable refused while a ring is hung");
    expect(s.fixture.queues->fini(*s.fixture.mes) == Gfx11QueuesStatus::stopped &&
           s.fixture.queues->release() == Gfx11QueuesStatus::busy,
           "fini still unmaps; release refused while submission is enabled");
    expect(s.fixture.queues->release_after_reset() == Gfx11QueuesStatus::quarantined &&
           s.fixture.leases(s.user) == 1, "no reset proof: IB lease kept");
    s.fixture.reset_confirmed = true;
    expect(s.fixture.queues->release_after_reset() == Gfx11QueuesStatus::released &&
           s.fixture.leases(s.user) == 0 && s.fences.ring_report(4).state == FenceRingState::detached &&
           s.fences.ring_report(0).state == FenceRingState::detached &&
           !s.fixture.queues->report().submission_enabled,
           "confirmed reset detaches every fence ring and returns the IB lease");
    // A CP fault on a queue: drm_sched_fault -> ring_hung.
    SubmitFixture c;
    c.enable();
    IrqReport irq{};
    IrqVmFaultLog log{};
    const Gfx11RingReport ring7 = c.fixture.queues->report().rings[7];
    irq.cp_fault_count[1][ring7.pipe][ring7.queue] = 1;
    irq.cp_fault_source[1][ring7.pipe][ring7.queue] = IrqSource::cp_bad_opcode;
    c.fences.process_interrupts({&irq, &log, false});
    c.write_ib(0, 4096, 1);
    expect(c.submit(7, 0, seq) == Gfx11QueuesStatus::ring_hung &&
           c.fences.ring_report(7).hang_reason == FenceHangReason::cp_bad_opcode,
           "CP bad opcode on the ring's queue quarantines it");
}

void submission_refusals() {
    SubmitFixture s;
    uint32_t seq = 7;
    expect(s.submit(0, 0, seq) == Gfx11QueuesStatus::unavailable && seq == 0,
           "submit before enable_submission refused");
    s.enable();
    Model& model = s.fixture.model;
    const auto doorbells = model.queues.at(gfx_doorbell).doorbells.size();
    expect(s.submit(0, 0, seq, 0) == Gfx11QueuesStatus::invalid_argument &&
           s.submit(0, 0, seq, gfx11_max_ib_dwords + 1) == Gfx11QueuesStatus::invalid_argument &&
           s.submit(9, 0, seq) == Gfx11QueuesStatus::invalid_argument, "IB length and ring bounded");
    expect(s.submit(0, 2, seq) == Gfx11QueuesStatus::invalid_memory &&
           s.submit(0, 4 * 4096 - 16, seq) == Gfx11QueuesStatus::invalid_memory,
           "misaligned or out-of-allocation IB refused");
    expect(s.fixture.queues->submit({0, s.fixture.memory.writeback, 0, 5}, seq) ==
               Gfx11QueuesStatus::invalid_memory &&
           s.fixture.queues->submit({0, s.fixture.memory.gfx_ring, 0, 5}, seq) ==
               Gfx11QueuesStatus::invalid_memory, "IB overlapping module buffers refused");
    expect(s.fixture.queues->submit({0, {}, 0, 5}, seq) == Gfx11QueuesStatus::invalid_memory,
           "stale handle refused");
    expect(model.queues.at(gfx_doorbell).doorbells.size() == doorbells && s.fixture.leases(s.user) == 0 &&
           s.fences.ring_report(0).sync_seq == 1, "refusals write and publish nothing");
    // A failed doorbell after publication quarantines protocol and ring.
    model.fail_doorbell = gfx_doorbell;
    s.write_ib(0, 4096, 1);
    expect(s.submit(0, 0, seq) == Gfx11QueuesStatus::doorbell_failure &&
           s.fixture.queues->report().status == Gfx11QueuesStatus::doorbell_failure &&
           s.fences.ring_report(0).hang_reason == FenceHangReason::submit_failure &&
           s.fixture.leases(s.user) == 1, "doorbell failure: quarantined, IB stays leased");
    expect(s.submit(1, 0, seq) == Gfx11QueuesStatus::quarantined, "protocol quarantined");
}

void submission_shutdown() {
    SubmitFixture s;
    s.enable();
    uint32_t seq = 0;
    s.write_ib(0, 4096, 1);
    s.submit(0, 0, seq);
    expect(s.fixture.queues->disable_submission() == Gfx11QueuesStatus::running &&
           s.fixture.leases(s.user) == 0 && s.fences.ring_report(0).state == FenceRingState::detached,
           "signalled work: disable detaches and returns leases");
    s.fixture.manager->release(s.user);
    expect(s.fixture.shutdown_all(), "queues then unmap and release every lease");
}

// Process-VM jobs on the gfx ring (amdgpu_ib_schedule with a job): the frame
// hand-derived from gfx_v11_0.c/amdgpu_vm.c/amdgpu_ids.c, VMID reuse and LRU
// assignment, PASID LUT writes, context switches and refusals.
constexpr uint32_t osssys_lut = 0x4260;           // OSSSYS segment 0 + IH_VMID_0_LUT
constexpr uint32_t gfxhub_base_lo0 = 0x2953;      // GCVM_CONTEXT0_PAGE_TABLE_BASE_ADDR_LO32
constexpr uint32_t gfxhub_req0 = 0x290b;          // GCVM_INVALIDATE_ENG0_REQ
constexpr uint32_t gfxhub_ack0 = 0x291d;          // GCVM_INVALIDATE_ENG0_ACK

std::vector<uint32_t> expected_vm_frame(uint64_t wb, uint64_t ib_va, uint32_t dwords, uint32_t vmid,
                                        uint64_t pd, uint32_t pasid, bool mapping, bool context_switch,
                                        uint32_t previous, uint32_t seq) {
    const uint64_t cond = wb + 128;
    const uint64_t fence = wb + 64;
    std::vector<uint32_t> f{0xc0065800u, 0, 0xffffffffu, 0x00ffffffu, 0, 0, 0xa, 0xc3b1u};
    // amdgpu_vm_flush: COND_EXEC (count: pipeline sync 7, two WREG 10, REG_WRITE_REG_WAIT 7,
    // PFP_SYNC_ME 2 and the PASID WREG 5), WAIT_REG_MEM memory PFP equal (0x113, poll 4).
    f.insert(f.end(), {0xc0032200u, lo(cond), hi(cond), 0, mapping ? 31u : 26u});
    f.insert(f.end(), {0xc0053c00u, 0x113u, lo(fence), hi(fence), previous, 0xffffffffu, 4});
    // WRITE_DATA ENGINE_SEL(1) | WR_CONFIRM, DST_SEL 0: CONTEXTn base lo/hi.
    f.insert(f.end(), {0xc0033700u, 0x40100000u, gfxhub_base_lo0 + 2 * vmid, 0, lo(pd)});
    f.insert(f.end(), {0xc0033700u, 0x40100000u, gfxhub_base_lo0 + 2 * vmid + 1, 0, hi(pd)});
    // Engine 0 REQ (legacy flush, every L1/L2 level) and ACK bit vmid on PFP.
    f.insert(f.end(), {0xc0053c00u, 0x143u, gfxhub_req0, gfxhub_ack0, 0x00f80000u | (1u << vmid),
                       1u << vmid, 0x20});
    f.insert(f.end(), {0xc0004200u, 0});
    if (mapping) {
        f.insert(f.end(), {0xc0033700u, 0x40100000u, osssys_lut + vmid, 0, pasid});
    }
    // COND_EXEC (count 31: HDP 7, CONTEXT_CONTROL 3, FRAME_CONTROL 2+2, IB 4, COND 5, RELEASE 8).
    f.insert(f.end(), {0xc0032200u, lo(cond), hi(cond), 0, 31});
    f.insert(f.end(), {0xc0053c00u, 0x143u, hdp_flush_req, hdp_flush_done, 1, 1, 0x20});
    f.insert(f.end(), {0xc0012800u, context_switch ? 0x81018003u : 0x80000000u, 0});
    f.insert(f.end(), {0xc0009000u, 0});
    f.insert(f.end(), {0xc0023f00u, lo(ib_va), hi(ib_va), dwords | (vmid << 24)});
    f.insert(f.end(), {0xc0009000u, 0x10000000u});
    f.insert(f.end(), {0xc0032200u, lo(cond), hi(cond), 0, 0});
    f.insert(f.end(), {0xc0064900u, 0x06603514u, 0x22000000u, lo(fence), hi(fence), seq, 0, 0});
    const uint32_t pad = 256 - static_cast<uint32_t>(f.size());
    f.push_back(0xc0001000u | ((pad - 2) << 16));

    while (f.size() < 256) {
        f.push_back(ring_nop);
    }

    return f;
}

struct VmJobFixture final {
    SubmitFixture s;
    GmcRegisters gmc{};
    Ih6Registers osssys{};
    // Process VA va_base maps the test's user allocation (bytes 0..16 KiB).
    static constexpr uint64_t va_base = 0x0000123400000000ULL;
    std::map<uint32_t, uint64_t> pd_of_pasid;

    VmJobFixture() {
        const auto binary = rx7900_test::navi31_discovery();
        validate_gmc_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, gmc);
        validate_ih6_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, osssys);
        // The hub translates only for a VMID whose CONTEXT base names a known
        // page directory: the IB is fetched through the VM, never by MC address.
        s.fixture.model.vm_translate = [this](uint32_t vmid, uint64_t va, uint64_t& mc) {
            const uint64_t pd = (static_cast<uint64_t>(s.fixture.model.device->reg(
                                     gfxhub_base_lo0 + 2 * vmid + 1)) << 32) |
                                s.fixture.model.device->reg(gfxhub_base_lo0 + 2 * vmid);
            bool known = false;

            for (const auto& entry : pd_of_pasid) {
                known = known || entry.second == pd;
            }

            if (!known || va < va_base || va >= va_base + 4 * 4096) {
                return false;
            }

            mc = s.user_mc + (va - va_base);
            return true;
        };
    }

    Gfx11QueuesStatus enable(bool tokens = true) {
        return s.fixture.queues->enable_submission(s.fences, {s.fixture.nbio_registers, s.model.wptr_store(),
            tokens ? gmc : GmcRegisters{}, tokens ? osssys : Ih6Registers{}});
    }

    // A WRITE_DATA IB at user offset whose destination is a process VA.
    void write_vm_ib(uint32_t offset, uint32_t dest, uint32_t value) {
        const uint64_t target = va_base + dest;
        const uint32_t words[5]{0xc0033700u, 0x00100500u, lo(target), hi(target), value};

        for (uint32_t index = 0; index < 5; ++index) {
            s.fixture.model.device->put_vram_u32(s.user_mc + offset + index * 4, words[index]);
        }
    }

    Gfx11QueuesStatus submit(uint32_t pasid, uint32_t offset, uint32_t& seq, uint32_t ring = 0) {
        const uint64_t pd = 0x0000000200000001ULL + (static_cast<uint64_t>(pasid) << 12);
        pd_of_pasid[pasid] = pd;
        const Gfx11VmJob job{pasid, pd, va_base + offset};
        return s.fixture.queues->submit({ring, s.user, offset, 5, &job}, seq);
    }
};

void submission_vm_jobs() {
    VmJobFixture v;
    expect(v.enable() == Gfx11QueuesStatus::running && v.s.fixture.queues->vm_report().enabled,
           "GMC/OSSSYS tokens enable process-VM jobs on the gfx ring");
    Model& model = v.s.fixture.model;
    const uint64_t wb = v.s.wb;
    v.write_vm_ib(0, 2048, 0x51515151u);
    uint32_t seq = 0;
    expect(v.submit(0x8001, 0, seq) == Gfx11QueuesStatus::running && seq == 2,
           "first process-VM job submitted with fence 2");
    const uint64_t ring = v.s.fixture.address(v.s.fixture.memory.gfx_ring);
    const uint64_t pd1 = v.pd_of_pasid[0x8001];
    expect(v.s.fixture.dwords(ring + 1536 * 4, 256) ==
               expected_vm_frame(wb, VmJobFixture::va_base, 5, 1, pd1, 0x8001, true, true, 1, 2),
           "frame: ACQUIRE_MEM, vm flush (pipeline sync on seq 1, CONTEXT1 base, engine 0 REQ/ACK, "
           "PFP_SYNC_ME, IH_VMID_1_LUT), COND_EXEC, HDP, full CONTEXT_CONTROL, FRAME_CONTROL, IB vmid 1");
    const CpQueue& gfx = model.queues.at(gfx_doorbell);
    expect(v.s.fixture.vram(v.s.user_mc + 2048) == 0x51515151u && gfx.ib_vmids.back() == 1 &&
           gfx.ib_faults.empty() && model.device->reg(osssys_lut + 1) == 0x8001,
           "CP fetched the IB and wrote its target through VMID 1; LUT names the PASID");
    Gfx11QueuesReport report = v.s.fixture.queues->report();
    const Gfx11VmReport vm = v.s.fixture.queues->vm_report();
    expect(vm.submissions == 1 && vm.pasid_mappings == 1 && vm.vmid_pasid[0] == 0x8001 &&
           v.s.fixture.leases(v.s.user) == 1, "VMID 1 bound to the PASID; the IB backing is leased");
    expect(v.s.fences.wait(0, seq, 1000) == FenceStatus::signalled, "job fence signalled");

    // Same VM again: VMID 1 reused (amdgpu_vmid_grab_used), flushed, no LUT write,
    // no context switch.
    v.write_vm_ib(256, 2052, 0x52525252u);
    expect(v.submit(0x8001, 256, seq) == Gfx11QueuesStatus::running && seq == 3 &&
           v.s.fixture.dwords(ring + 1792 * 4, 256) ==
               expected_vm_frame(wb, VmJobFixture::va_base + 256, 5, 1, pd1, 0x8001, false, false, 2, 3),
           "same VM reuses VMID 1: VM flush kept, PASID mapping and context load skipped");
    // Another process takes the least recently used idle VMID (2) with a context switch.
    v.write_vm_ib(512, 2056, 0x53535353u);
    expect(v.submit(0x8002, 512, seq) == Gfx11QueuesStatus::running && seq == 4,
           "second process submitted");
    const uint64_t pd2 = v.pd_of_pasid[0x8002];
    expect(v.s.fixture.dwords(ring + 2048 * 4 % 8192, 256) ==
               expected_vm_frame(wb, VmJobFixture::va_base + 512, 5, 2, pd2, 0x8002, true, true, 3, 4) &&
           model.device->reg(osssys_lut + 2) == 0x8002 && v.s.fixture.vram(v.s.user_mc + 2056) == 0x53535353u,
           "another PASID: VMID 2, LUT write, full context load");
    // A VMID 0 kernel IB in between makes the next job reload its context.
    v.s.write_ib(768, 2060, 0x54545454u);
    expect(v.s.submit(0, 768, seq) == Gfx11QueuesStatus::running, "VMID 0 kernel IB on the same ring");
    v.write_vm_ib(1024, 2064, 0x55555555u);
    v.s.fences.wait(0, seq, 1000);
    expect(v.submit(0x8002, 1024, seq) == Gfx11QueuesStatus::running &&
           v.s.fixture.dwords(ring + (2560 * 4) % 8192, 256) ==
               expected_vm_frame(wb, VmJobFixture::va_base + 1024, 5, 2, pd2, 0x8002, false, true, 5, 6),
           "after VMID 0 work the next job loads a full context on its own VMID");
    v.s.fences.wait(0, seq, 1000);
    v.s.fixture.queues->retire();
    expect(v.s.fixture.leases(v.s.user) == 0, "signalled VM jobs return their IB leases");

    // Five more processes take the never-used VMIDs 3..7; the next one gets the
    // least recently used idle VMID (1, last owned by 0x8001), so its PASID
    // mapping is written again (amdgpu_vm_flush pasid_mapping_needed).
    for (uint32_t index = 0; index < 5; ++index) {
        v.write_vm_ib(1280 + index * 32, 2100 + index * 4, index);
        v.submit(0x8100 + index, 1280 + index * 32, seq);
        v.s.fences.wait(0, seq, 1000);
    }
    const uint64_t mappings = v.s.fixture.queues->vm_report().pasid_mappings;
    v.write_vm_ib(1536, 2200, 0x56565656u);
    expect(v.submit(0x8200, 1536, seq) == Gfx11QueuesStatus::running &&
           v.s.fixture.queues->vm_report().vmid_pasid[0] == 0x8200 &&
           v.s.fixture.queues->vm_report().pasid_mappings == mappings + 1 &&
           model.device->reg(osssys_lut + 1) == 0x8200 && v.s.fixture.vram(v.s.user_mc + 2200) == 0x56565656u,
           "with every VMID used, the least recently used idle one (VMID 1) changes owner and maps the new PASID");
    v.s.fences.wait(0, seq, 1000);
    v.s.fixture.queues->retire();

    // Refusals change nothing.
    report = v.s.fixture.queues->report();
    const uint64_t wptr = report.rings[0].wptr;
    const Gfx11VmJob zero{0, pd1, VmJobFixture::va_base};
    const Gfx11VmJob unaligned{0x8001, pd1, VmJobFixture::va_base + 2};
    const Gfx11VmJob wide{0x8001, pd1, (1ULL << 48) - 8};
    const Gfx11VmJob huge_pasid{0x10000, pd1, VmJobFixture::va_base};
    const Gfx11VmJob bad_pd{0x8001, 1ULL << 48, VmJobFixture::va_base};
    expect(v.s.fixture.queues->submit({1, v.s.user, 0, 5, &zero}, seq) == Gfx11QueuesStatus::invalid_argument &&
           v.submit(0x8001, 0, seq, 1) == Gfx11QueuesStatus::invalid_argument,
           "compute rings take no process-VM jobs");
    expect(v.s.fixture.queues->submit({0, v.s.user, 0, 5, &zero}, seq) == Gfx11QueuesStatus::invalid_argument &&
           v.s.fixture.queues->submit({0, v.s.user, 0, 5, &unaligned}, seq) ==
               Gfx11QueuesStatus::invalid_argument &&
           v.s.fixture.queues->submit({0, v.s.user, 0, 5, &wide}, seq) == Gfx11QueuesStatus::invalid_argument &&
           v.s.fixture.queues->submit({0, v.s.user, 0, 5, &huge_pasid}, seq) ==
               Gfx11QueuesStatus::invalid_argument &&
           v.s.fixture.queues->submit({0, v.s.user, 0, 5, &bad_pd}, seq) == Gfx11QueuesStatus::invalid_argument,
           "PASID 0 or above 16 bits, unaligned/48-bit-overflowing IB VA, invalid PD refused");
    const Gfx11VmJob outside{0x8001, pd1, VmJobFixture::va_base};
    expect(v.s.fixture.queues->submit({0, v.s.user, 4 * 4096 - 8, 5, &outside}, seq) ==
               Gfx11QueuesStatus::invalid_memory,
           "the leased backing must hold the whole IB");
    expect(v.s.fixture.queues->report().rings[0].wptr == wptr && v.s.fixture.leases(v.s.user) == 0,
           "refusals wrote nothing and kept no lease");
    expect(v.s.fixture.queues->disable_submission() == Gfx11QueuesStatus::running, "idle rings detach");

    VmJobFixture plain;
    expect(plain.enable(false) == Gfx11QueuesStatus::running && !plain.s.fixture.queues->vm_report().enabled &&
           plain.submit(0x8001, 0, seq) == Gfx11QueuesStatus::unavailable,
           "without the GMC/OSSSYS tokens only VMID 0 work is accepted");

    // A confirmed reset forgets every VMID owner: the next use maps again.
    VmJobFixture r;
    r.enable();
    r.write_vm_ib(0, 2048, 1);
    r.submit(0x8003, 0, seq);
    r.s.fixture.reset_confirmed = true;
    expect(r.s.fixture.queues->release_after_reset() == Gfx11QueuesStatus::released &&
           !r.s.fixture.queues->vm_report().enabled && r.s.fixture.queues->vm_report().vmid_pasid[0] == 0 &&
           r.s.fixture.leases(r.s.user) == 0, "reset releases the IB lease and every VMID binding");
    expect(v.s.model.errors == 0 && r.s.model.errors == 0, "fence lock balanced");
}

} // namespace

int main() {
    initialize_refusals();
    bring_up();
    fewer_rings();
    ring_timeout();
    hardware_failures();
    ib_tests();
    ib_refusals();
    ib_compute_timeout();
    ib_gfx_failures();
    shared_builders();
    submission_enable();
    submission_frames();
    submission_wrap();
    submission_space_and_fences();
    submission_timeout_quarantine();
    submission_refusals();
    submission_shutdown();
    submission_vm_jobs();
    return checks.finish("gfx11_queues_tests");
}

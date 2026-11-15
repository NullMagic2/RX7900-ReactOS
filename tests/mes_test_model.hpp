/*
Module: rx7900.tests.mes_test_model
Purpose: Model MES scheduler/KIQ firmware for MES11 and ASIC sequence tests.
Created: 2026-10-21
Architecture: Banked GC HQD registers and KIQ/API ring consumers over the shared
synthetic Navi31 machine; it is not hardware evidence.
*/
#pragma once

#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "asic_test_device.hpp"
#include "mes11.hpp"

namespace rx7900_test::mes_model {

using rx7900::hardware::DoorbellIo;
using rx7900::hardware::RegisterIo;
using rx7900_test::SimulatedNavi31;

constexpr uint32_t gc0 = rx7900_test::gc_base;
constexpr uint32_t gc1 = 0xa000;
constexpr uint32_t grbm = gc1 + 0x0900;
constexpr uint32_t mes_cntl = gc1 + 0x2807;
constexpr uint32_t rlc_cp_schedulers = gc1 + 0x098a;
constexpr uint32_t scratch = gc1 + 0x2040;
constexpr uint32_t gp3_lo = gc1 + 0x2849;
constexpr uint32_t pc_start = gc1 + 0x2800;
constexpr uint32_t pc_start_hi = gc1 + 0x289d;
constexpr uint32_t hqd_active = gc0 + 0x1fab;
constexpr uint32_t hqd_dequeue = gc0 + 0x1fc1;
constexpr uint32_t hqd_doorbell = gc0 + 0x1fb8;
constexpr uint32_t hqd_pq_base = gc0 + 0x1fb1;
constexpr uint32_t hqd_pq_base_hi = gc0 + 0x1fb2;
constexpr uint32_t hqd_pq_control = gc0 + 0x1fba;
constexpr uint32_t kiq_select = 0xd;   // ME3 pipe1
constexpr uint32_t sched_select = 0xc; // ME3 pipe0
constexpr uint32_t kiq_nop = 0xffff1000u;
constexpr uint64_t sched_uc = 0x0000000000040000ULL;
constexpr uint64_t kiq_uc = 0x0000000100080000ULL;

inline bool banked(uint32_t index) {
    return (index >= gc0 + 0x1fa9 && index <= gc0 + 0x1fe0) || index == pc_start ||
           index == pc_start_hi || index == gp3_lo;
}

inline bool gc_register(uint32_t index) {
    return (index >= gc0 && index < gc0 + 0x2000) || (index >= gc1 && index < gc1 + 0x3000);
}

// MES scheduler and KIQ firmware model: the KIQ consumes its ring from the HQD
// registers programmed for ME3 pipe1; MAP_QUEUES activates the scheduler HQD
// from its MQD; the scheduler consumes 64-dword API frames and writes their
// completion status as MES firmware does.
struct MesModel final {
    std::unique_ptr<SimulatedNavi31> owned;
    SimulatedNavi31* device = nullptr;
    std::map<std::pair<uint32_t, uint32_t>, uint32_t> bank;
    std::vector<std::pair<uint32_t, uint32_t>> writes;
    std::vector<std::vector<uint32_t>> kiq_packets;
    std::vector<std::vector<uint32_t>> mes_frames;
    std::vector<uint64_t> kiq_doorbells;
    std::vector<uint64_t> mes_doorbells;
    uint32_t select = 0;
    uint32_t fail_write_index = UINT32_MAX;
    uint32_t ignore_opcode = 0xff;
    uint32_t last_flush = 0;
    bool flushed_before_every_doorbell = true;
    bool kiq_executes = true;
    bool mes_dead = false;
    bool dequeue_hang = false;
    bool doorbell_ok = true;
    bool mapped = false;
    uint64_t kiq_rptr = 0;
    uint64_t mes_rptr = 0;
    uint64_t mes_ring = 0;
    uint64_t mes_mqd = 0;
    uint32_t mes_ring_dwords = 0;
    uint32_t mes_doorbell = UINT32_MAX;
    uint64_t wptr_poll = 0;
    // Legacy kernel queues mapped by MES ADD_QUEUE, keyed by doorbell dword.
    struct CpQueue final {
        uint64_t mqd = 0;
        uint32_t type = 0;
        uint64_t rptr = 0;
        uint32_t packets = 0;
    };
    std::map<uint32_t, CpQueue> queues;
    std::vector<uint32_t> removed_queues;
    uint32_t hung_queue_doorbell = UINT32_MAX;

    // Owns a fresh machine, or wraps one shared with other engine models (the
    // ASIC sequence test); doorbells it does not own fall through to SDMA.
    explicit MesModel(SimulatedNavi31* shared = nullptr)
        : owned(shared == nullptr ? std::make_unique<SimulatedNavi31>() : nullptr),
          device(shared == nullptr ? owned.get() : shared) {
        bank[{sched_select, gp3_lo}] = 0x00001050u; // API version 1, firmware 0x50
        bank[{kiq_select, gp3_lo}] = 0x00001049u;
    }

    RegisterIo io() noexcept { return {this, rx7900_test::aperture_bytes, read, write, delay}; }
    DoorbellIo doorbells(uint32_t dwords = 0x10000) noexcept {
        return {this, dwords, doorbell32, doorbell64};
    }

    uint32_t& banked_reg(uint32_t selection, uint32_t index) { return bank[{selection, index}]; }

    static bool read(void* context, uint32_t offset, uint32_t& value) noexcept {
        auto& self = *static_cast<MesModel*>(context);
        const uint32_t index = offset / 4;

        if (banked(index)) {
            value = self.bank[{self.select, index}];
            return true;
        }

        return SimulatedNavi31::read(self.device, offset, value);
    }

    static bool write(void* context, uint32_t offset, uint32_t value) noexcept {
        auto& self = *static_cast<MesModel*>(context);
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
            self.bank[{self.select, index}] = value;

            if (index == hqd_dequeue && value == 1 && !self.dequeue_hang) {
                self.bank[{self.select, hqd_active}] = 0;
            }

            return true;
        }

        return SimulatedNavi31::write(self.device, offset, value);
    }

    static void delay(void* context, uint32_t microseconds) noexcept {
        SimulatedNavi31::delay(static_cast<MesModel*>(context)->device, microseconds);
    }

    static bool doorbell32(void* context, uint32_t index, uint32_t value) noexcept {
        return SimulatedNavi31::doorbell32(static_cast<MesModel*>(context)->device, index, value);
    }

    static bool doorbell64(void* context, uint32_t index, uint64_t value) noexcept {
        auto& self = *static_cast<MesModel*>(context);

        if (!self.doorbell_ok) {
            return false;
        }

        if (self.device->hdp_flushes <= self.last_flush) {
            self.flushed_before_every_doorbell = false;
        }

        self.last_flush = self.device->hdp_flushes;

        if (index == 0x18) {
            self.kiq_doorbells.push_back(value);
            self.run_kiq(value);
        } else if (self.queues.count(index) != 0) {
            self.run_cp(index, self.queues[index], value);
        } else if (index == self.mes_doorbell) {
            self.mes_doorbells.push_back(value);
            self.run_mes(value);
        } else {
            return SimulatedNavi31::doorbell64(self.device, index, value);
        }

        return true;
    }

    uint64_t vram64(uint64_t address) {
        return (static_cast<uint64_t>(device->vram_u32(address + 4)) << 32) |
               device->vram_u32(address);
    }

    void put64(uint64_t address, uint64_t value) {
        device->put_vram_u32(address, static_cast<uint32_t>(value));
        device->put_vram_u32(address + 4, static_cast<uint32_t>(value >> 32));
    }

    void run_kiq(uint64_t wptr) {
        const uint32_t control = bank[{kiq_select, hqd_doorbell}];

        if (bank[{kiq_select, hqd_active}] != 1 || (control & 0x40000000u) == 0 ||
            (control & 0x0ffffffcu) >> 2 != 0x18) {
            return;
        }

        const uint64_t ring = ((static_cast<uint64_t>(bank[{kiq_select, hqd_pq_base_hi}]) << 32) |
                               bank[{kiq_select, hqd_pq_base}]) << 8;
        const uint32_t dwords = 2u << (bank[{kiq_select, hqd_pq_control}] & 0x3f);
        const auto dw = [&](uint64_t position) {
            return device->vram_u32(ring + (position & (dwords - 1)) * 4);
        };

        while (kiq_rptr < wptr) {
            const uint32_t header = dw(kiq_rptr);

            if (header == kiq_nop) {
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

            if (opcode == 0xa2 && ((packet.at(1) >> 18) & 3) == 2 && ((packet.at(1) >> 26) & 7) == 5) {
                mes_mqd = (static_cast<uint64_t>(packet.at(4)) << 32) | packet.at(3);
                wptr_poll = (static_cast<uint64_t>(packet.at(6)) << 32) | packet.at(5);
                mes_ring = ((static_cast<uint64_t>(device->vram_u32(mes_mqd + 548)) << 32) |
                            device->vram_u32(mes_mqd + 544)) << 8;
                mes_ring_dwords = 2u << (device->vram_u32(mes_mqd + 580) & 0x3f);
                mes_doorbell = (device->vram_u32(mes_mqd + 572) & 0x0ffffffcu) >> 2;
                // CP loads the MQD into the ME3 pipe0 HQD.
                bank[{sched_select, hqd_doorbell}] = device->vram_u32(mes_mqd + 572);
                bank[{sched_select, hqd_active}] = 1;
                mapped = true;
            } else if (opcode == 0x37 && kiq_executes) {
                device->reg(packet.at(2)) = packet.at(4);
            }
        }
    }

    void run_mes(uint64_t wptr) {
        if (!mapped || mes_dead) {
            return;
        }

        while (mes_rptr < wptr) {
            std::vector<uint32_t> frame;

            for (uint32_t index = 0; index < 64; ++index) {
                frame.push_back(device->vram_u32(mes_ring + ((mes_rptr + index) & (mes_ring_dwords - 1)) * 4));
            }

            mes_rptr += 64;
            mes_frames.push_back(frame);
            const uint32_t opcode = (frame.at(0) >> 4) & 0xff;
            // MESAPI api_status byte offsets (pack 4): SET_HW_RSRC 200, ADD_QUEUE 148,
            // REMOVE_QUEUE 20, SUSPEND 28, RESUME 16, MISC/QUERY_STATUS 8.
            const uint32_t status = opcode == 0 ? 200 : opcode == 2 ? 148 : opcode == 3 ? 20 :
                                    opcode == 6 ? 28 : opcode == 7 ? 16 : 8;

            if (opcode == 2 && opcode != ignore_opcode) {
                CpQueue queue;
                queue.mqd = (static_cast<uint64_t>(frame.at(20)) << 32) | frame.at(19);
                queue.type = frame.at(27);
                queues[frame.at(18)] = queue;
            } else if (opcode == 3 && opcode != ignore_opcode) {
                removed_queues.push_back(frame.at(1));
                queues.erase(frame.at(1));
            }

            if (opcode != ignore_opcode) {
                put64((static_cast<uint64_t>(frame.at(status / 4 + 1)) << 32) | frame.at(status / 4),
                      (static_cast<uint64_t>(frame.at(status / 4 + 3)) << 32) | frame.at(status / 4 + 2));
            }
        }
    }

    // Mode-1 reset: MES/KIQ/CP lose their queues and HQD state; firmware versions
    // (GP3) come back with the reloaded firmware.
    void reset() {
        const uint32_t scheduler = bank[{sched_select, gp3_lo}];
        const uint32_t kiq = bank[{kiq_select, gp3_lo}];
        bank.clear();
        bank[{sched_select, gp3_lo}] = scheduler;
        bank[{kiq_select, gp3_lo}] = kiq;
        queues.clear();
        mapped = false;
        kiq_rptr = 0;
        mes_rptr = 0;
        mes_doorbell = UINT32_MAX;
    }

    // Indirect buffer: WRITE_DATA to memory; every other packet is skipped.
    void run_ib(uint64_t address, uint32_t dwords) {
        uint32_t position = 0;

        while (position < dwords) {
            const uint32_t header = device->vram_u32(address + position * 4);
            const uint32_t count = ((header >> 16) & 0x3fff) + 1;

            if (((header >> 8) & 0xff) == 0x37 && count >= 4) {
                const uint64_t target = (static_cast<uint64_t>(device->vram_u32(address + (position + 3) * 4)) << 32) |
                                        device->vram_u32(address + (position + 2) * 4);
                device->put_vram_u32(target, device->vram_u32(address + (position + 4) * 4));
            }

            position += count + 1;
        }
    }

    // A mapped queue fetches its ring from its MQD (v11 gfx and compute MQDs
    // share these offsets) and executes SET_UCONFIG_REG; other packets are skipped.
    void run_cp(uint32_t doorbell, CpQueue& queue, uint64_t wptr) {
        const uint64_t mqd = queue.mqd;
        const uint32_t control = device->vram_u32(mqd + 572);

        if (doorbell == hung_queue_doorbell || ((control & 0x0ffffffcu) >> 2) != doorbell ||
            (control & 0x40000000u) == 0) {
            return;
        }

        const uint64_t ring = ((static_cast<uint64_t>(device->vram_u32(mqd + 548)) << 32) |
                               device->vram_u32(mqd + 544)) << 8;
        const uint32_t dwords = 2u << (device->vram_u32(mqd + 580) & 0x3f);
        const auto dw = [&](uint64_t position) {
            return device->vram_u32(ring + (position & (dwords - 1)) * 4);
        };

        while (queue.rptr < wptr) {
            const uint32_t header = dw(queue.rptr);

            if (header == kiq_nop) {
                ++queue.rptr;
                continue;
            }

            const uint32_t count = ((header >> 16) & 0x3fff) + 1;
            const uint32_t opcode = (header >> 8) & 0xff;

            if (opcode == 0x79 && count == 2) {
                device->reg(0xc000 + dw(queue.rptr + 1)) = dw(queue.rptr + 2);
            } else if (opcode == 0x3c && count == 6) {
                // WAIT_REG_MEM write-then-wait (HDP flush): REQ written, DONE follows.
                device->reg(dw(queue.rptr + 3)) = dw(queue.rptr + 4);
            } else if (opcode == 0x3f && count == 3) {
                run_ib((static_cast<uint64_t>(dw(queue.rptr + 2)) << 32) | dw(queue.rptr + 1),
                       dw(queue.rptr + 3) & 0xfffff);
            } else if (opcode == 0x49 && count >= 6) {
                // RELEASE_MEM, DATA_SEL 1: 32-bit sequence to the fence address.
                device->put_vram_u32((static_cast<uint64_t>(dw(queue.rptr + 4)) << 32) | dw(queue.rptr + 3),
                                     dw(queue.rptr + 5));
            }

            queue.rptr += count + 1;
            ++queue.packets;
        }

        device->put_vram_u32((static_cast<uint64_t>(device->vram_u32(mqd + 560)) << 32) |
                             device->vram_u32(mqd + 556), static_cast<uint32_t>(queue.rptr));
    }
};

} // namespace rx7900_test::mes_model

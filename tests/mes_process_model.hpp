/*
Module: rx7900.tests.mes_process_model
Purpose: Model MES process queues (SUSPEND/RESUME, user-queue engines in process VMs) for the MES11, user-queue and scheduler tests.
Created: 2026-10-22
Architecture: Wraps the shared MES/KIQ model (mes_test_model.hpp, unchanged) with
a GART-mapped GTT backend, correct SUSPEND/RESUME completion and gfx/compute/SDMA
engines that walk the 4-level process page tables. It is not hardware evidence.
Expected values are hand-derived from Linux v6.12 amdgpu_mes.c, mes_v11_0.c,
mes_v11_api_def.h, gfx_v11_0.c, sdma_v6_0.c and v11_structs.h.
*/
#pragma once

#include <map>
#include <memory>
#include <vector>

#include "mes_test_model.hpp"

namespace rx7900_test::process_model {

using rx7900::ByteView;
using rx7900_test::mes_model::MesModel;

constexpr uint64_t gtt_bytes = 8u << 20;
constexpr uint64_t bus_base = 0x0000003000000000ULL;
constexpr uint32_t aggregated_base = 0x800;
constexpr uint32_t context15_cntl = rx7900_test::mes_model::gc0 + 0x1697;
constexpr uint32_t sdma_ib_cntl = rx7900_test::mes_model::gc0 + 0x008a;
constexpr uint32_t vm_cntx_cntl = 0x01fffe07u;

inline uint32_t u32(const uint8_t* bytes, uint64_t offset) {
    uint32_t value = 0;
    ByteView(bytes + offset, 4).read_u32(0, value);
    return value;
}

inline uint64_t u64(const uint8_t* bytes, uint64_t offset) {
    return (static_cast<uint64_t>(u32(bytes, offset + 4)) << 32) | u32(bytes, offset);
}

inline void put(uint8_t* bytes, uint64_t offset, uint32_t value) {
    rx7900::sdma::CommandBuffer(bytes + offset, 4).write_u32(0, value);
}

inline uint64_t frame64(const std::vector<uint32_t>& frame, uint32_t byte) {
    return (static_cast<uint64_t>(frame.at(byte / 4 + 1)) << 32) | frame.at(byte / 4);
}

// GART-mapped system memory: GPU address arena + offset, CPU pool + offset, DMA
// bus address bus_base + offset.
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

    uint8_t* bus_cpu(uint64_t bus) {
        return bus >= bus_base && bus - bus_base < pool.size() ? pool.data() + (bus - bus_base) : nullptr;
    }
};

// MES process-queue engines and MES SUSPEND/RESUME completion.
struct ProcessEngines final {
    MesModel* model = nullptr;
    GttBackend* gtt = nullptr;
    std::map<uint32_t, uint64_t> rptr;
    std::map<uint32_t, uint64_t> pending;
    // Index of the ADD_QUEUE frame a doorbell's read pointer belongs to: a new
    // queue on a reused doorbell starts at 0.
    std::map<uint32_t, size_t> added_frame;
    std::vector<uint32_t> fence_contexts;
    std::vector<uint32_t> doorbells;
    std::vector<uint64_t> doorbell_values;
    std::vector<uint32_t> gang_opcodes;
    uint32_t hang_doorbell = UINT32_MAX;
    uint32_t drop_opcode = UINT32_MAX;
    uint32_t corrupt_fence_doorbell = UINT32_MAX;
    bool suspended = false;
    bool resumed = false;
    unsigned translation_faults = 0;
    unsigned ib_writes = 0;

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
                const bool allowed = (entry & 2) != 0 && (entry & 0x20) != 0 &&
                                     (!write || (entry & 0x40) != 0);
                uint8_t* page = gtt->bus_cpu(entry & 0x0000fffffffff000ULL);

                if (!allowed || page == nullptr) {
                    ++translation_faults;
                    return nullptr;
                }

                return page + (va & 0xfff);
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

    // The newest process ADD_QUEUE (not legacy) without a later REMOVE_QUEUE.
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

    void run_pm4_ib(uint64_t root, uint64_t ib, uint32_t dwords) {
        for (uint32_t position = 0; position < dwords;) {
            const uint32_t header = read(root, ib + position * 4);
            const uint32_t count = ((header >> 16) & 0x3fff) + 1;

            if (((header >> 8) & 0xff) == 0x37) {
                const uint64_t target = (static_cast<uint64_t>(read(root, ib + (position + 3) * 4)) << 32) |
                                        read(root, ib + (position + 2) * 4);
                store(root, target, read(root, ib + (position + 4) * 4));
                ++ib_writes;
            }

            position += count + 1;
        }
    }

    void run_pm4(uint32_t doorbell, uint64_t root, const uint8_t* mqd, uint64_t wptr) {
        const uint64_t ring = ((static_cast<uint64_t>(u32(mqd, 548)) << 32) | u32(mqd, 544)) << 8;
        const uint32_t dwords = 2u << (u32(mqd, 580) & 0x3f);
        uint64_t& position = rptr[doorbell];

        while (position < wptr) {
            const auto dw = [&](uint64_t index) {
                return read(root, ring + ((position + index) & (dwords - 1)) * 4);
            };
            const uint32_t header = dw(0);

            if (header == rx7900_test::mes_model::kiq_nop) {
                ++position;
                continue;
            }

            const uint32_t count = ((header >> 16) & 0x3fff) + 1;
            const uint32_t opcode = (header >> 8) & 0xff;

            if (opcode == 0x3f) {
                run_pm4_ib(root, (static_cast<uint64_t>(dw(2)) << 32) | dw(1), dw(3) & 0xfffff);
            } else if (opcode == 0x49) {
                fence_contexts.push_back(dw(7));
                store(root, (static_cast<uint64_t>(dw(4)) << 32) | dw(3),
                      doorbell == corrupt_fence_doorbell ? dw(5) + 100 : dw(5));
            }

            position += count + 1;
        }

        store(root, (static_cast<uint64_t>(u32(mqd, 560)) << 32) | u32(mqd, 556),
              static_cast<uint32_t>(position));
    }

    void run_sdma_ib(uint64_t root, uint64_t ib, uint32_t dwords) {
        for (uint32_t position = 0; position < dwords;) {
            const uint32_t header = read(root, ib + position * 4);

            if ((header & 0xff) == 1 || (header & 0xff) == 11) {
                // Linux SDMA linear COPY (7 dwords) / CONSTANT_FILL (5).
                // Independently walk actual process PTEs for every byte; the
                // tested owner supplies production packets, not expected data.
                const bool copy = (header & 0xff) == 1;
                const uint32_t words = copy ? 7 : 5;
                if (words > dwords - position) {
                    ++translation_faults;
                    break;
                }
                const uint32_t count = read(root, ib + (position + (copy ? 1 : 4)) * 4) + 1;
                if (count == 0 || count > (4u << 20)) {
                    ++translation_faults;
                    break;
                }
                const uint32_t destination_word = copy ? 5 : 1;
                const uint64_t destination = uint64_t{read(root, ib + (position + destination_word) * 4)} |
                    (uint64_t{read(root, ib + (position + destination_word + 1) * 4)} << 32);
                const uint64_t source = copy ? uint64_t{read(root, ib + (position + 3) * 4)} |
                    (uint64_t{read(root, ib + (position + 4) * 4)} << 32) : 0;
                const uint32_t value = copy ? 0 : read(root, ib + (position + 3) * 4);
                for (uint32_t byte = 0; byte < count; ++byte) {
                    uint8_t* output = translate(root, destination + byte, true);
                    const uint8_t* input = copy ? translate(root, source + byte, false) : nullptr;
                    if (output != nullptr && (!copy || input != nullptr)) {
                        *output = copy ? *input : static_cast<uint8_t>(value >> ((byte & 3u) * 8));
                    }
                }
                ++ib_writes;
                position += words;
            } else if ((header & 0xff) == 2) {
                const uint64_t target = (static_cast<uint64_t>(read(root, ib + (position + 2) * 4)) << 32) |
                                        read(root, ib + (position + 1) * 4);
                store(root, target, read(root, ib + (position + 4) * 4));
                ++ib_writes;
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

            if (op == 4) {
                run_sdma_ib(root, (static_cast<uint64_t>(dw(2)) << 32) | dw(1), dw(3));
                length = 6;
            } else if (op == 5) {
                store(root, (static_cast<uint64_t>(dw(2)) << 32) | dw(1),
                      doorbell == corrupt_fence_doorbell ? dw(3) + 100 : dw(3));
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

    void execute(uint32_t index, uint64_t value) {
        const std::vector<uint32_t>* frame = active(index);

        if (frame == nullptr || index == hang_doorbell) {
            return;
        }

        const size_t frame_index = static_cast<size_t>(frame - model->mes_frames.data());
        const auto known = added_frame.find(index);

        if (known == added_frame.end() || known->second != frame_index) {
            added_frame[index] = frame_index;
            rptr[index] = 0;
        }

        const uint32_t type = frame->at(27);
        const uint64_t root = (static_cast<uint64_t>(frame->at(3)) << 32) | frame->at(2);
        const uint8_t* mqd = gtt->cpu((static_cast<uint64_t>(frame->at(20)) << 32) | frame->at(19));

        if (mqd == nullptr) {
            return;
        }

        if (type == 2) {
            if ((u32(mqd, 76) >> 2) == index && (u32(mqd, 68) & (1u << 28)) != 0) {
                run_sdma(index, root, mqd, value);
            }
        } else if (((u32(mqd, 572) & 0x0ffffffcu) >> 2) == index &&
                   (u32(mqd, 572) & 0x40000000u) != 0) {
            run_pm4(index, root, mqd, value);
        }
    }

    // A user-queue doorbell: executed now, or after RESUME while gangs are suspended.
    bool doorbell(uint32_t index, uint64_t value) {
        if (active(index) == nullptr) {
            return false;
        }

        doorbells.push_back(index);
        doorbell_values.push_back(value);
        pending[index] = value;

        if (!suspended) {
            execute(index, value);
        }

        return true;
    }

    // Before the shared model consumes the MES ring: complete SUSPEND (status at
    // byte 28) and RESUME (status at byte 16), which the shared model does not know.
    void mes_frames(uint64_t wptr) {
        if (!model->mapped || model->mes_dead) {
            return;
        }

        for (uint64_t position = model->mes_rptr; position < wptr; position += 64) {
            const auto dw = [&](uint32_t index) {
                return model->device->vram_u32(model->mes_ring +
                    ((position + index) & (model->mes_ring_dwords - 1)) * 4);
            };
            const uint32_t opcode = (dw(0) >> 4) & 0xff;

            if (opcode != 6 && opcode != 7) {
                continue;
            }

            gang_opcodes.push_back(opcode);

            if (opcode == drop_opcode) {
                continue;
            }

            const uint32_t status = opcode == 6 ? 28 : 16;
            model->put64((static_cast<uint64_t>(dw(status / 4 + 1)) << 32) | dw(status / 4),
                         (static_cast<uint64_t>(dw(status / 4 + 3)) << 32) | dw(status / 4 + 2));
            suspended = opcode == 6;
            resumed = opcode == 7;
        }
    }

    // After a RESUME: queues fetch up to the last doorbell they were given.
    void resume_pending() {
        if (suspended || !resumed) {
            return;
        }

        resumed = false;

        for (const auto& entry : pending) {
            execute(entry.first, entry.second);
        }
    }

    // One DoorbellIo for Mes11 and user queues over the shared model.
    static bool doorbell64(void* context, uint32_t index, uint64_t value) noexcept {
        auto& self = *static_cast<ProcessEngines*>(context);

        if (index >= aggregated_base && self.doorbell(index, value)) {
            return true;
        }

        const bool scheduler = index == self.model->mes_doorbell;

        if (scheduler) {
            self.mes_frames(value);
        }

        const bool ok = MesModel::doorbell64(self.model, index, value);

        if (scheduler) {
            self.resume_pending();
        }

        return ok;
    }

    static bool doorbell32(void* context, uint32_t index, uint32_t value) noexcept {
        return MesModel::doorbell32(static_cast<ProcessEngines*>(context)->model, index, value);
    }

    rx7900::hardware::DoorbellIo io() noexcept { return {this, 0x10000, doorbell32, doorbell64}; }
};

} // namespace rx7900_test::process_model

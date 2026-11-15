/*
Module: rx7900.tests.process_memory
Purpose: Exercise actual process-buffer mappings, ownership, recovery and faults.
Created: 2026-10-22
Architecture: Reuses the MES process fixture and production ProcessMemory;
synthetic DMA/VRAM do not establish physical hardware execution.
*/
#include "user_queue_fixture.hpp"
#include "../src/reactos/process_memory.hpp"

namespace {

using namespace rx7900::reactos;
using namespace rx7900_test::user_queue_fixture;
using rx7900::memory::Access;
using rx7900::memory::Status;

rx7900_test::Checks checks;

void expect(bool value, const char* description) { checks.expect(value, description); }

struct Buffers final {
    Fixture device;
    Fixture::Process process{};
    std::unique_ptr<ProcessMemory> memory = std::make_unique<ProcessMemory>();
    rx7900::memory::Handle last_dma{};
    uint64_t dma_delta = 0;
    bool dma_available = true;

    static bool dma_bus(void* context, rx7900::memory::Handle handle, uint64_t& bus) noexcept {
        auto& self = *static_cast<Buffers*>(context);
        rx7900::memory::AllocationInfo allocation{};

        if (!self.dma_available || self.device.gtt->describe(handle, allocation) != Status::success) {
            return false;
        }

        self.last_dma = handle;
        bus = bus_base + allocation.gpu.virtual_address - self.device.gart.start + self.dma_delta;
        return true;
    }

    ProcessMemoryConfig config() {
        return {device.queues.get(), process.pasid, device.vram.get(), device.gtt.get(), this, dma_bus};
    }

    Buffers() {
        expect(device.up(), "MES fixture starts for the production process-buffer service");
        process = device.process();
        expect(process.status == UserQueueStatus::created &&
               memory->initialize(config()) == Status::success, "session binds only an existing process VM");
    }

    void close() {
        expect(device.queues->destroy_process(process.pasid) == UserQueueStatus::destroyed,
               "real process teardown proves the page-table walkers retired");
        expect(memory->close_after_idle() == Status::success && memory->empty(),
               "after VM retirement every buffer and its ownership lease is returned");
    }
};

void lifecycle_and_permissions() {
    Buffers f;
    uint64_t buffer = 99;
    expect(f.memory->allocate(8192, ProcessMemoryDomain::system, Access::read_write, buffer) ==
               Status::success && buffer != 0, "allocate actual GTT backing and opaque nonzero handle");
    uint8_t output[8]{0xff};
    expect(f.memory->read(buffer, 0, output, sizeof(output)) == Status::success && output[0] == 0 &&
               output[7] == 0, "allocation clears old physical backing before any client read");
    const uint8_t input[5]{11, 22, 33, 44, 55};
    expect(f.memory->write(buffer, 4094, {input, sizeof(input)}) == Status::success &&
           f.memory->read(buffer, 4094, output, sizeof(input)) == Status::success &&
           output[0] == 11 && output[4] == 55, "captured bounded copies may cross an owned page boundary");
    expect(f.memory->write(buffer, UINT64_MAX, {input, sizeof(input)}) == Status::invalid_argument &&
           f.memory->read(buffer, 8190, output, sizeof(output)) == Status::invalid_argument &&
           f.memory->read(buffer, 0, nullptr, 1) == Status::invalid_argument &&
           f.memory->read(buffer, 0, output, process_memory_copy_bytes + 1) == Status::invalid_argument,
           "overflow, extent, null and excessive copy boundaries are refused before access");
    expect(f.memory->map(buffer, 0x400000, Access::read) == Status::success,
           "real GpuVm system mapping plus successful TLB acknowledgement");
    const auto root = f.device.queues->vm_report(f.process.pasid).page_table_base_addr;
    uint8_t* cpu = f.device.engines.translate(root, 0x400000 + 4094, false);
    expect(cpu != nullptr && cpu[0] == 11 &&
           f.device.engines.translate(root, 0x400000, true) == nullptr,
           "hardware-visible page tables resolve the real backing and deny a write to read-only PTEs");
    const auto allocation = f.device.info(*f.device.gtt, f.last_dma);
    expect(allocation.leases == 1, "native Manager retains a permanent resource-ownership lease");
    expect(f.memory->retain(buffer, Access::write) == Status::access_denied &&
           f.memory->retain(buffer, Access::read) == Status::success,
           "submitted work can lease only rights actually granted by this mapping");
    expect(f.memory->unmap(buffer) == Status::busy && f.memory->release(buffer) == Status::busy &&
           f.memory->read(buffer, 0, output, 1) == Status::busy &&
           f.memory->write(buffer, 0, {input, 1}) == Status::busy &&
           f.memory->save_for_recovery() == Status::busy,
           "in-flight buffer leases exclude CPU access, remapping, destruction and recovery");
    expect(f.memory->unretain(buffer) == Status::success &&
           f.memory->unretain(buffer) == Status::invalid_argument &&
           f.memory->unmap(buffer) == Status::success &&
           f.memory->release(buffer) == Status::success,
           "fence retirement permits heavyweight TLB unmap and backing destruction");
    ProcessBufferInfo info{};
    expect(f.memory->describe(buffer, info) == Status::stale_handle &&
           f.memory->read(buffer, 0, output, 1) == Status::stale_handle,
           "destroyed opaque handles cannot name recycled backing");
    uint64_t read_only_buffer = 0;
    expect(f.memory->allocate(4096, ProcessMemoryDomain::system, Access::read, read_only_buffer) ==
               Status::success && read_only_buffer != buffer &&
           f.memory->map(read_only_buffer, 0x400000, Access::write) == Status::access_denied &&
           f.memory->write(read_only_buffer, 0, {input, sizeof(input)}) == Status::access_denied,
           "allocation grants constrain both user copies and process mappings");
    expect(f.memory->close_after_idle() == Status::busy,
           "a live process VM cannot be discarded on an unverified close request");
    f.close();
    f.process = f.device.process();
    expect(f.memory->initialize(f.config()) == Status::success &&
           f.memory->allocate(4096, ProcessMemoryDomain::system, Access::read, buffer) == Status::success &&
           buffer > read_only_buffer, "reused session storage preserves monotonic handle generation");
    f.close();
}

void mapping_boundaries() {
    Buffers f;
    uint64_t first = 0;
    uint64_t second = 0;
    expect(f.memory->allocate(4095, ProcessMemoryDomain::system, Access::read, first) ==
               Status::invalid_argument && first == 0 &&
           f.memory->allocate(process_memory_max_bytes + 4096, ProcessMemoryDomain::system,
                              Access::read, first) == Status::invalid_argument &&
           f.memory->allocate(4096, static_cast<ProcessMemoryDomain>(2), Access::read, first) ==
               Status::invalid_argument, "unaligned, oversized and unknown-domain allocations are rejected");
    expect(f.memory->allocate(8192, ProcessMemoryDomain::system, Access::read_write, first) ==
               Status::success &&
           f.memory->allocate(4096, ProcessMemoryDomain::system, Access::read_write, second) ==
               Status::success, "two independent backing allocations");
    expect(f.memory->map(first, 0, Access::read) == Status::invalid_argument &&
           f.memory->map(first, 0x100001, Access::read) == Status::invalid_argument &&
           f.memory->map(first, (gpuvm_max_pfn << 12) - 4096, Access::read) == Status::invalid_argument &&
           f.memory->map(first, UINT64_MAX, Access::read) == Status::invalid_argument,
           "null, unaligned, overflowing and end-of-GPUVA mappings fail before PTE mutation");
    f.dma_available = false;
    expect(f.memory->map(first, 0x500000, Access::read) == Status::backend_failure,
           "no process mapping without a trusted native DMA translation");
    f.dma_available = true;
    f.dma_delta = 1;
    expect(f.memory->map(first, 0x500000, Access::read) == Status::backend_failure,
           "misaligned DMA bus addresses never become process PTEs");
    f.dma_delta = 0;
    expect(f.memory->map(first, 0x500000, Access::read_write) == Status::success &&
           f.memory->map(second, 0x501000, Access::read) == Status::invalid_argument,
           "GPUVM rejects overlapping owned mappings with no second-buffer mutation");
    const auto gang = f.device.gang(f.process.pasid);
    const auto queue = f.device.queue(gang.id, Mes11QueueType::compute, 0x200000);
    expect(queue.status == UserQueueStatus::added &&
           f.memory->map(second, 0x200000, Access::read) == Status::invalid_argument,
           "client mappings cannot overwrite kernel-owned queue ring/control pages");
    expect(f.memory->map(second, 0x600000, Access::read) == Status::success &&
           f.memory->release(second) == Status::success, "refused map can be retried at an owned free range");
    f.close();
}

void recovery_and_quarantine() {
    {
        Buffers f;
        uint64_t local = 0;
        const uint8_t value[]{0x5a, 0xa5, 0x77, 0x88};
        uint8_t output[4]{};
        expect(f.memory->allocate(4096, ProcessMemoryDomain::vram, Access::read_write, local) ==
                   Status::success &&
               f.memory->write(local, 0, {value, sizeof(value)}) == Status::success &&
               f.memory->map(local, 0x700000, Access::read_write) == Status::success,
               "visible VRAM buffer owns a process map and an upfront pinned GTT recovery copy");
        expect(f.memory->save_for_recovery() == Status::success &&
               f.memory->read(local, 0, output, sizeof(output)) == Status::busy &&
               f.memory->unmap(local) == Status::busy,
               "saving buffers closes CPU and mapping admission until restore");
        // Simulate lost local-VRAM data without disturbing the retained VM image.
        auto& machine = *f.device.model.device;
        uint8_t* cpu = nullptr;

        for (size_t index = 0; index + sizeof(value) <= machine.vram.size(); ++index) {
            if (machine.vram[index] == value[0] && machine.vram[index + 1] == value[1] &&
                machine.vram[index + 2] == value[2] && machine.vram[index + 3] == value[3]) {
                cpu = machine.vram.data() + index;
                break;
            }
        }

        expect(cpu != nullptr, "test independently locates the mapped local-VRAM bytes");

        if (cpu != nullptr) {
            cpu[0] = 0;
            cpu[1] = 0;
        }

        expect(f.memory->restore_after_recovery() == Status::success &&
               f.memory->read(local, 0, output, sizeof(output)) == Status::success &&
               output[0] == value[0] && output[1] == value[1] && output[3] == value[3],
               "resume restores actual VRAM bytes from owned GTT before queues restart");
        f.close();
    }
    {
        Buffers f;
        uint64_t buffer = 0;
        expect(f.memory->allocate(4096, ProcessMemoryDomain::system, Access::read_write, buffer) ==
                   Status::success &&
               f.memory->map(buffer, 0x800000, Access::read_write) == Status::success,
               "quarantine test owns real backing and a published process mapping");
        const uint32_t lut = f.device.osssys.segment0_dword_base() + 1;
        f.device.model.device->reg(lut) = UINT32_MAX;
        expect(f.memory->unmap(buffer) == Status::quarantined,
               "unknown PASID/VMID invalidation state never reports successful unmapping");
        ProcessBufferInfo info{};
        uint8_t byte = 0;
        uint64_t other = 99;
        expect(f.memory->describe(buffer, info) == Status::quarantined && info.mapped &&
               info.va == 0x800000 &&
               f.memory->release(buffer) == Status::quarantined &&
               f.memory->read(buffer, 0, &byte, 1) == Status::quarantined &&
               f.memory->allocate(4096, ProcessMemoryDomain::system, Access::read, other) ==
                   Status::quarantined && other == 0,
               "uncertain TLB ACK retains the exact VA/backing and blocks new process-memory work");
        const auto allocation = f.device.info(*f.device.gtt, f.last_dma);
        expect(allocation.leases == 1 && f.memory->close_after_idle() == Status::busy,
               "uncertain mapping keeps the actual Manager lease until VM retirement");
        f.device.model.device->reg(lut) = 0;
        f.close();
    }
}

void capacity() {
    Buffers f;
    uint64_t handles[process_memory_capacity]{};

    for (auto& handle : handles) {
        expect(f.memory->allocate(4096, ProcessMemoryDomain::system, Access::read_write, handle) ==
                   Status::success, "fixed session buffer capacity accepts an owned page");
    }

    uint64_t overflow = 99;
    expect(f.memory->allocate(4096, ProcessMemoryDomain::system, Access::read_write, overflow) ==
               Status::out_of_memory && overflow == 0,
           "capacity exhaustion returns no handle and allocates no hidden backing");
    f.close();
}

// Confirmed destructive reset: GTT keeps contents and GPUVA records, VRAM
// becomes a tombstone answered once, and a failed rebind is retry-safe.
void reset_retirement() {
    Buffers f;
    uint64_t gtt = 0;
    uint64_t vram = 0;
    expect(f.memory->allocate(8192, ProcessMemoryDomain::system, Access::read_write, gtt) == Status::success &&
           f.memory->allocate(8192, ProcessMemoryDomain::vram, Access::read_write, vram) == Status::success &&
           f.memory->map(gtt, 0x400000, Access::read_write) == Status::success &&
           f.memory->map(vram, 0x800000, Access::read_write) == Status::success,
           "GTT and VRAM buffers mapped before the reset");
    const uint8_t pattern[4]{0x5a, 0x5b, 0x5c, 0x5d};
    expect(f.memory->write(gtt, 100, {pattern, sizeof(pattern)}) == Status::success &&
           f.memory->write(vram, 100, {pattern, sizeof(pattern)}) == Status::success, "contents written");
    expect(f.memory->retain(gtt, Access::read) == Status::success &&
           f.memory->retire_after_reset() == Status::busy && f.memory->unretain(gtt) == Status::success,
           "a buffer still leased by work is never retired");
    f.device.reset_confirmed = true;
    expect(f.device.queues->release_after_reset(Fixture::confirm, &f.device.reset_confirmed) ==
               UserQueueStatus::released, "the reset proof retired the old VM");
    expect(f.memory->retire_after_reset() == Status::success && f.memory->lost_buffers() == 1,
           "VRAM contents are lost on a mode-1 reset; GTT survives");
    ProcessBufferInfo info{};
    expect(f.memory->describe(gtt, info) == Status::success && !info.mapped && info.va == 0,
           "between the reset and the rebind no GPUVA is reported mapped");
    uint8_t output[4]{};
    expect(f.memory->describe(vram, info) == Status::stale_handle && f.memory->take_lost_touch() == vram &&
           f.memory->take_lost_touch() == 0, "a tombstone never resolves and is recorded once per lookup");
    expect(f.memory->read(gtt, 100, output, sizeof(output)) == Status::busy,
           "no CPU access before the new VM exists");
    uint32_t pasid = 0;
    expect(f.device.queues->create_process({f.process.tables, 16, f.process.context, {}}, pasid) ==
               UserQueueStatus::created, "a new process VM over the retained page-table allocation");
    f.dma_available = false;
    expect(f.memory->rebind_after_reset({f.device.queues.get(), pasid, f.device.vram.get(), f.device.gtt.get(),
               &f, Buffers::dma_bus}) != Status::success && f.memory->lost_buffers() == 1,
           "a failed rebind changes no record");
    expect(f.device.queues->destroy_process(pasid) == UserQueueStatus::destroyed, "the failed VM is destroyed");
    f.memory->abandon_rebind();
    f.dma_available = true;
    expect(f.device.queues->create_process({f.process.tables, 16, f.process.context, {}}, pasid) ==
               UserQueueStatus::created &&
           f.memory->rebind_after_reset({f.device.queues.get(), pasid, f.device.vram.get(), f.device.gtt.get(),
               &f, Buffers::dma_bus}) == Status::success, "the retry rebinds into a fresh VM");
    f.process.pasid = pasid;
    expect(f.memory->describe(gtt, info) == Status::success && info.mapped && info.va == 0x400000 &&
           f.memory->read(gtt, 100, output, sizeof(output)) == Status::success && output[0] == 0x5a &&
           output[3] == 0x5d, "the GTT buffer keeps its contents at the same GPUVA");
    expect(f.memory->read(vram, 100, output, sizeof(output)) == Status::stale_handle &&
           f.memory->take_lost_touch() == vram && f.memory->retire_lost(vram) &&
           !f.memory->retire_lost(vram) && f.memory->lost_buffers() == 0,
           "the owner answers the lost buffer exactly once");
    expect(f.memory->read(vram, 100, output, sizeof(output)) == Status::stale_handle &&
           f.memory->take_lost_touch() == 0, "afterwards the handle is simply dead");
    uint64_t next = 0;
    expect(f.memory->allocate(4096, ProcessMemoryDomain::vram, Access::read_write, next) == Status::success &&
           next > vram, "handles never recycle across a reset");
    f.close();
}

// Process exit after queue removal: every client GPUVA unmapped and one
// acknowledged TLB flush before any backing may be released.
void exit_unmap() {
    Buffers f;
    uint64_t first = 0;
    uint64_t second = 0;
    expect(f.memory->allocate(4096, ProcessMemoryDomain::system, Access::read_write, first) == Status::success &&
           f.memory->allocate(4096, ProcessMemoryDomain::vram, Access::read, second) == Status::success &&
           f.memory->map(first, 0x400000, Access::read_write) == Status::success &&
           f.memory->map(second, 0x600000, Access::read) == Status::success, "two client mappings");
    const auto mapped = f.device.queues->vm_report(f.process.pasid);
    expect(f.memory->retain(first, Access::read) == Status::success && f.memory->unmap_all() == Status::busy &&
           f.memory->unretain(first) == Status::success, "leased buffers keep their mappings");
    expect(f.memory->unmap_all() == Status::success, "every client mapping removed");
    const auto unmapped = f.device.queues->vm_report(f.process.pasid);
    ProcessBufferInfo info{};
    expect(unmapped.mappings + 2 == mapped.mappings && unmapped.tlb_seq == unmapped.flushed_seq &&
           f.memory->describe(first, info) == Status::success && !info.mapped && info.va == 0,
           "PTEs cleared and the TLB invalidation acknowledged before release");
    f.close();
}

} // namespace

int main() {
    lifecycle_and_permissions();
    mapping_boundaries();
    recovery_and_quarantine();
    capacity();
    reset_retirement();
    exit_unmap();
    return checks.finish("process_memory_tests");
}

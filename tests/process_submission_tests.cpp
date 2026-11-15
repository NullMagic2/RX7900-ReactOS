/*
Module: rx7900.tests.process_submission
Purpose: Verify semantic native submissions, actual SDMA pixels and fence leases.
Created: 2026-10-22
Architecture: Production ProcessSubmissions/ProcessMemory/UserQueues over the
shared synthetic MES engine and real page-table walker; no physical execution.
*/
#include "user_queue_fixture.hpp"
#include "../src/reactos/process_submission.hpp"

namespace {
using namespace rx7900::reactos;
using namespace rx7900_test::user_queue_fixture;
using rx7900::memory::Access;
using rx7900::memory::Status;
rx7900_test::Checks checks;
void expect(bool value, const char* description) { checks.expect(value, description); }

struct FixtureOwner final {
    Fixture device;
    Fixture::Process process{};
    std::unique_ptr<ProcessMemory> buffers = std::make_unique<ProcessMemory>();
    ProcessSubmissions work;
    uint32_t queue = 0;

    static bool dma_bus(void* context, rx7900::memory::Handle handle, uint64_t& bus) noexcept {
        auto& self = *static_cast<FixtureOwner*>(context);
        rx7900::memory::AllocationInfo info{};
        if (self.device.gtt->describe(handle, info) != Status::success) return false;
        bus = bus_base + info.gpu.virtual_address - self.device.gart.start;
        return true;
    }
    FixtureOwner() {
        expect(device.up(), "actual MES/GART initialization for process submission");
        process = device.process(32, true);
        expect(process.status == UserQueueStatus::created &&
            buffers->initialize({device.queues.get(), process.pasid, device.vram.get(),
                device.gtt.get(), this, dma_bus}) == Status::success,
            "native buffers bind one actual process VM");
        expect(work.initialize({device.queues.get(), buffers.get(), device.gtt.get(),
            process.pasid, this, dma_bus}) == Status::success,
            "driver-owned GTT indirect buffer maps read-only in that VM");
        const auto gang = device.gang(process.pasid);
        const auto result = device.queue(gang.id, Mes11QueueType::sdma, 1ULL << 44);
        queue = result.id;
        expect(result.status == UserQueueStatus::added, "actual private SDMA queue is ready");
    }
    uint64_t buffer(uint64_t va, Access access = Access::read_write) {
        uint64_t handle = 0;
        expect(buffers->allocate(8192, ProcessMemoryDomain::system, Access::read_write, handle) ==
            Status::success && buffers->map(handle, va, access) == Status::success,
            "owned native buffer has actual process PTEs");
        return handle;
    }
    void close() {
        expect(device.queues->destroy_process(process.pasid) == UserQueueStatus::destroyed,
            "process retirement removes actual queues and GPUVM");
        expect(work.close_after_idle() == Status::success && work.empty(),
            "private IB and any command leases retire after actual VM removal");
        expect(buffers->close_after_idle() == Status::success && buffers->empty(),
            "native backing retires after the submission owner returned its leases");
    }
};

void actual_transfers() {
    FixtureOwner f;
    const uint64_t source = f.buffer(0x400000, Access::read);
    const uint64_t destination = f.buffer(0x600000);
    uint8_t input[32]{};
    for (uint32_t index = 0; index < sizeof(input); ++index) input[index] = static_cast<uint8_t>(index * 7u);
    expect(f.buffers->write(source, 4090, {input, sizeof(input)}) == Status::success,
        "caller data is captured into owned source backing");
    uint32_t sequence = 0;
    expect(f.work.copy(f.queue, {source, destination, 4090, 4092, sizeof(input), 0}, sequence) ==
        Status::success && sequence != 0, "semantic copy submits production SDMA across page boundaries");
    ProcessBufferInfo info{};
    expect(f.buffers->describe(source, info) == Status::success && info.leases == 1,
        "completion cannot silently release a buffer before owner fence processing");
    expect(f.work.query(f.queue, sequence) == Status::success,
        "actual MES fence observation returns the command's source/destination leases");
    uint8_t output[32]{};
    expect(f.buffers->read(destination, 4092, output, sizeof(output)) == Status::success,
        "completed copy permits CPU readback");
    bool equal = true;
    for (uint32_t index = 0; index < sizeof(output); ++index) equal = equal && output[index] == input[index];
    expect(equal && f.device.engines.translation_faults == 0,
        "independent engine model walks production tables and copies exact input bytes");
    expect(f.work.fill(f.queue, {0, destination, 0, 128, 64, 0x12345678}, sequence) == Status::success &&
        f.work.query(f.queue, sequence) == Status::success &&
        f.buffers->read(destination, 128, output, sizeof(output)) == Status::success,
        "semantic fill retires through the same actual queue fence");
    equal = true;
    for (uint32_t index = 0; index < sizeof(output); ++index)
        equal = equal && output[index] == static_cast<uint8_t>(0x12345678u >> ((index & 3u) * 8));
    expect(equal, "constant-fill packet produces exact repeated pattern bytes");
    f.close();
}

void pending_and_authorization() {
    FixtureOwner f;
    const uint64_t buffer = f.buffer(0x400000);
    const auto report = f.device.queues->queue_report(f.queue);
    f.device.engines.hang_doorbell = report.doorbell_index;
    uint32_t sequence = 0;
    expect(f.work.fill(f.queue, {0, buffer, 0, 0, 32, 0xabcdef01}, sequence) == Status::success &&
        f.work.query(f.queue, sequence) == Status::busy, "delayed hardware fence leaves submission pending");
    uint8_t output[4]{};
    uint32_t other = 99;
    expect(f.buffers->read(buffer, 0, output, sizeof(output)) == Status::busy &&
        f.buffers->unmap(buffer) == Status::busy && f.buffers->release(buffer) == Status::busy &&
        f.work.fill(f.queue, {0, buffer, 0, 0, 4, 1}, other) == Status::busy && other == 0,
        "pending buffers and private IB cannot be read, unmapped, freed or overwritten");
    expect(f.work.query(f.queue, sequence + 1) == Status::invalid_argument &&
        f.work.retire_removed_queue(f.queue) == Status::busy &&
        f.work.close_after_idle() == Status::busy,
        "invented future fences and unproven removal never release real ownership");
    f.device.engines.hang_doorbell = UINT32_MAX;
    f.device.engines.execute(report.doorbell_index, f.device.engines.pending.at(report.doorbell_index));
    expect(f.work.poll() == Status::success &&
        f.buffers->read(buffer, 0, output, sizeof(output)) == Status::success && output[0] == 1,
        "actual delayed engine execution and fence completion release the lease");
    const uint64_t read_only = f.buffer(0x600000, Access::read);
    expect(f.work.fill(f.queue, {0, read_only, 0, 0, 4, 1}, sequence) == Status::access_denied,
        "semantic commands cannot exceed destination PTE grants");
    expect(f.work.copy(f.queue, {buffer, buffer, 0, 4, 32, 0}, sequence) == Status::invalid_argument &&
        f.work.fill(f.queue, {0, buffer, 0, UINT64_MAX, 4, 0}, sequence) == Status::invalid_argument &&
        f.work.fill(f.queue, {0, buffer, 0, 1, 4, 0}, sequence) == Status::invalid_argument,
        "overlap, overflow and fill misalignment are refused before GPU publication");
    const auto foreign = f.device.process();
    const auto gang = f.device.gang(foreign.pasid);
    const auto queue = f.device.queue(gang.id, Mes11QueueType::sdma, 0x200000);
    expect(f.work.fill(queue.id, {0, buffer, 0, 0, 4, 1}, sequence) == Status::invalid_argument &&
        f.device.queues->destroy_process(foreign.pasid) == UserQueueStatus::destroyed,
        "a valid queue owned by another PASID cannot use this session's buffers");
    uint64_t alias = 0;
    expect(f.buffers->allocate(4096, ProcessMemoryDomain::system, Access::read_write, alias) == Status::success &&
        f.buffers->map(alias, process_submission_va, Access::read_write) == Status::invalid_argument,
        "client buffer maps cannot overwrite the driver-owned executable IB mapping");
    f.close();
}

void uncertain_publication() {
    FixtureOwner f;
    const uint64_t buffer = f.buffer(0x400000);
    f.device.fail_user_doorbell = true;
    uint32_t sequence = 99;
    expect(f.work.fill(f.queue, {0, buffer, 0, 0, 4, 0x77}, sequence) == Status::quarantined && sequence == 0,
        "uncertain doorbell publication exposes no fictitious completion fence");
    ProcessBufferInfo info{};
    expect(f.buffers->describe(buffer, info) == Status::success && info.leases == 1 &&
        f.work.poll() == Status::quarantined && f.work.close_after_idle() == Status::busy,
        "failed publication keeps buffer and IB ownership under quarantine");
    f.device.fail_user_doorbell = false;
    expect(f.device.queues->remove_queue(f.queue) == UserQueueStatus::removed &&
        f.work.retire_removed_queue(f.queue) == Status::success &&
        f.buffers->describe(buffer, info) == Status::success && info.leases == 0,
        "acknowledged real REMOVE_QUEUE safely retires uncertain command references");
    f.close();
}

void destructive_reset_rebind() {
    FixtureOwner f;
    const uint64_t buffer = f.buffer(0x400000);
    uint32_t sequence = 0;
    expect(f.work.fill(f.queue, {0, buffer, 0, 0, 4, 0x11223344}, sequence) == Status::success &&
        f.work.query(f.queue, sequence) == Status::success,
        "pre-reset work has a real completion receipt");
    expect(f.device.queues->suspend_for_recovery() == UserQueueStatus::suspended &&
        f.buffers->save_for_recovery() == Status::success &&
        f.buffers->ready_for_reset_rebind() && f.work.ready_for_reset_rebind(),
        "quiescent session preserves its exact buffer and private IB owners");
    f.device.reset_confirmed = true;
    expect(f.device.queues->release_after_reset(Fixture::confirm,
        &f.device.reset_confirmed) == UserQueueStatus::released &&
        f.device.mes->release_after_reset() == Mes11Status::released,
        "confirmed destructive reset retires old PASID and MES ownership");
    const auto queue_restart = f.device.queues->initialize(f.device.config());
    expect(queue_restart == UserQueueStatus::ready,
        "same Managers permit fresh process-queue configuration");
    uint32_t pasid = 0;
    expect(f.device.queues->create_process({f.process.tables, 32, f.process.context,
        f.process.backup}, pasid) == UserQueueStatus::created && pasid != 0,
        "fresh process VM reuses owned page-table and context backing");
    f.process.pasid = pasid;
    expect(f.buffers->rebind_after_reset({f.device.queues.get(), pasid, f.device.vram.get(),
        f.device.gtt.get(), &f, FixtureOwner::dma_bus}) == Status::success &&
        f.work.rebind_after_reset({f.device.queues.get(), f.buffers.get(), f.device.gtt.get(),
        pasid, &f, FixtureOwner::dma_bus}) == Status::success,
        "retained opaque buffer and private IB map into the reconstructed GPUVM");
    uint8_t output[4]{};
    const auto vm = f.device.queues->vm_report(pasid);
    expect(vm.pasid == pasid && vm.mappings >= 2 &&
        f.buffers->read(buffer, 0, output, sizeof(output)) == Status::success &&
        output[0] == 0x44 && output[3] == 0x11,
        "rebound buffer and private IB retain contents and actual GPUVA mappings");
    f.close();
}

// Work in flight at a confirmed reset is lost, never replayed; its leases return.
void reset_lost_work() {
    FixtureOwner f;
    const uint64_t buffer = f.buffer(0x400000);
    f.device.engines.hang_doorbell = f.device.queues->queue_report(f.queue).doorbell_index;
    uint32_t sequence = 0;
    expect(f.work.fill(f.queue, {0, buffer, 0, 0, 4, 0x01010101}, sequence) == Status::success &&
        f.work.query(f.queue, sequence) == Status::busy, "work outstanding when the device hangs");
    expect(!f.work.ready_for_reset_rebind() && f.buffers->retire_after_reset() == Status::busy,
        "pending work keeps its owners before the reset proof");
    f.device.reset_confirmed = true;
    expect(f.device.queues->release_after_reset(Fixture::confirm, &f.device.reset_confirmed) ==
        UserQueueStatus::released, "confirmed reset retired the queue and VM");
    uint32_t lost_queue = 0;
    uint32_t lost_sequence = 0;
    ProcessBufferInfo info{};
    expect(f.work.retire_after_reset(lost_queue, lost_sequence) == Status::success &&
        lost_queue == f.queue && lost_sequence == sequence &&
        f.buffers->describe(buffer, info) == Status::success && info.leases == 0 &&
        f.work.ready_for_reset_rebind(), "the in-flight transfer is reported lost and its leases return");
    expect(f.work.retire_after_reset(lost_queue, lost_sequence) == Status::success &&
        lost_queue == 0 && lost_sequence == 0, "nothing is reported twice");
    expect(f.buffers->retire_after_reset() == Status::success, "buffers retire after the work owner");
    uint32_t pasid = 0;
    expect(f.device.mes->release_after_reset() == Mes11Status::released &&
        f.device.queues->initialize(f.device.config()) == UserQueueStatus::ready &&
        f.device.queues->create_process({f.process.tables, 32, f.process.context, f.process.backup}, pasid) ==
            UserQueueStatus::created, "a new incarnation and process VM");
    f.process.pasid = pasid;
    expect(f.buffers->rebind_after_reset({f.device.queues.get(), pasid, f.device.vram.get(),
        f.device.gtt.get(), &f, FixtureOwner::dma_bus}) == Status::success &&
        f.work.rebind_after_reset({f.device.queues.get(), f.buffers.get(), f.device.gtt.get(),
        pasid, &f, FixtureOwner::dma_bus}) == Status::success, "both owners rebind into it");
    expect(f.work.query(f.queue, sequence) == Status::invalid_argument,
        "the old sequence is never claimed complete by the rebuilt owner");
    f.close();
}
}

int main() {
    actual_transfers();
    pending_and_authorization();
    uncertain_publication();
    destructive_reset_rebind();
    reset_lost_work();
    return checks.finish("process_submission_tests");
}

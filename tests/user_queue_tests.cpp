/*
Module: rx7900.tests.user_queue
Purpose: Verify MES process/gang/queue lifecycle, ADD/REMOVE_QUEUE frames, MQDs, submission, fences, timeouts, suspension and quarantine.
Created: 2026-10-22
Architecture: user_queue_fixture.hpp over the process-queue model; engines execute
gfx/compute PM4 and SDMA packets at process VAs through the real page tables. It is
not hardware evidence. Expected values are hand-derived from Linux v6.12
amdgpu_mes.c, mes_v11_0.c, mes_v11_api_def.h, gfx_v11_0.c, sdma_v6_0.c,
amdgpu_ring.c, amdgpu_fence.c and v11_structs.h.
*/
#include "user_queue_fixture.hpp"

namespace {

using namespace rx7900_test::user_queue_fixture;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

constexpr uint64_t gfx_va = 0x100000;
constexpr uint64_t compute_va = 0x200000;
constexpr uint64_t sdma_va = 0x300000;
constexpr uint64_t ib_va = 0x400000;
constexpr uint64_t target_va = 0x500000;

void lifecycle() {
    Fixture fixture;
    UserQueues& queues = *fixture.queues;
    uint32_t id = 0;
    expect(queues.create_process({}, id) == UserQueueStatus::unavailable, "initialize first");
    expect(fixture.up(), "MES scheduler and UserQueues ready");

    const auto process = fixture.process();
    expect(process.status == UserQueueStatus::created && process.pasid == 0x8000 &&
           fixture.pasids->allocated(0x8000), "amdgpu_pasid_alloc(16): PASID 0x8000");
    const GpuVmReport vm = queues.vm_report(process.pasid);
    expect(vm.pasid == 0x8000 && vm.memory_retained && vm.page_table_base_addr ==
           fixture.info(*fixture.vram, process.tables).gpu.virtual_address - rx7900_test::fb_base +
               (0x20ULL << 24), "process VM: root PD physical base, PASID set");
    expect(u32(fixture.cpu_of(process.context), 0) == 0 && u32(fixture.cpu_of(process.context), 4092) == 0,
           "process context zeroed");

    const auto gang = fixture.gang(process.pasid, 2, 3, 0x5000);
    const auto other = fixture.gang(process.pasid);
    expect(gang.status == UserQueueStatus::added && gang.id == 1 && other.id == 2,
           "gang ids from 1 (gang_id_idr)");
    expect(fixture.gang(0x1234).status == UserQueueStatus::not_found &&
           queues.add_gang(process.pasid, {5, 1, 0, fixture.gtt_buffer(4096)}, id) ==
               UserQueueStatus::invalid_argument, "unknown PASID and priority 5 refused");

    const auto gfx = fixture.queue(gang.id, Mes11QueueType::gfx, gfx_va);
    const auto compute = fixture.queue(gang.id, Mes11QueueType::compute, compute_va, 16384);
    const auto sdma = fixture.queue(other.id, Mes11QueueType::sdma, sdma_va);
    expect(gfx.status == UserQueueStatus::added && compute.status == UserQueueStatus::added &&
           sdma.status == UserQueueStatus::added && gfx.id == 1 && compute.id == 2 && sdma.id == 3,
           "three queues added, MES queue ids 1..3");
    const auto adds = fixture.frames(2);
    bool add_ok = adds.size() == 3;
    const uint32_t doorbells[3]{aggregated_base + 10, aggregated_base + 12, aggregated_base + 512};
    const uint64_t vas[3]{gfx_va, compute_va, sdma_va};
    const uint32_t rings[3]{8192, 16384, 8192};
    const Fixture::Queue* added[3]{&gfx, &compute, &sdma};
    const uint64_t gang_gart[3]{fixture.gart_of(gang.context), fixture.gart_of(gang.context),
                                fixture.gart_of(other.context)};

    for (uint32_t index = 0; add_ok && index < 3; ++index) {
        const auto& f = adds[index];
        add_ok = f.at(0) == 0x00040021u && f.at(1) == 0x8000 && frame64(f, 8) == vm.page_table_base_addr &&
            frame64(f, 16) == 0 && frame64(f, 24) == user_queue_process_va_end &&
            frame64(f, 32) == 0 && frame64(f, 40) == fixture.gart_of(process.context) &&
            frame64(f, 48) == (index < 2 ? 0x5000u : 0u) && frame64(f, 56) == gang_gart[index] &&
            f.at(16) == (index < 2 ? 3u : 1u) && f.at(17) == (index < 2 ? 2u : 1u) &&
            f.at(18) == doorbells[index] && frame64(f, 76) == fixture.gart_of(added[index]->mqd) &&
            frame64(f, 84) == vas[index] + rings[index] + 32 && f.at(27) == index && f.at(29) == 0 &&
            f.at(35) == vm_cntx_cntl && f.at(36) == 0;
    }

    expect(add_ok, "ADD_QUEUE: PASID, PT base, VA end, gang quantum/priorities, contexts, MES "
                   "doorbells 0x80a/0x80c/0xa00, MQD GART, VA wptr shadow, type, vm_context_cntl");

    // MQDs and buffers.
    const uint8_t* gfx_mqd = fixture.cpu_of(gfx.mqd);
    const uint8_t* compute_mqd = fixture.cpu_of(compute.mqd);
    const uint8_t* sdma_mqd = fixture.cpu_of(sdma.mqd);
    expect(u32(gfx_mqd, 544) == (gfx_va >> 8) && u32(gfx_mqd, 556) == gfx_va + 8192 &&
           u32(gfx_mqd, 564) == gfx_va + 8192 + 32 &&
           u32(gfx_mqd, 572) == ((doorbells[0] << 2) | 0x40000000u) && u32(gfx_mqd, 520) == 1 &&
           (u32(gfx_mqd, 580) & 0x3f) == 10, "gfx MQD: ring/rptr/wptr VAs, MES doorbell, 8 KiB");
    expect(u32(compute_mqd, 544) == (compute_va >> 8) && u32(compute_mqd, 660) ==
           ((compute_va + 16384 + 2048) >> 8) && (u32(compute_mqd, 580) & 0x3f) == 11 &&
           u32(compute_mqd, 520) == 0 && u32(compute_mqd, 532) == 1,
           "compute MQD: EOP in the control page, 16 KiB ring, inactive, PIPE_PRIO_NORMAL");
    expect(u32(sdma_mqd, 4) == (sdma_va >> 8) && u32(sdma_mqd, 32) == sdma_va + 8192 &&
           u32(sdma_mqd, 108) == sdma_va + 8192 + 32 && u32(sdma_mqd, 76) == (doorbells[2] << 2) &&
           u32(sdma_mqd, 36) == 0x100, "SDMA MQD from the shared sdma_v6_0_mqd_init");
    const uint8_t* gfx_ring = fixture.cpu_of(gfx.buffer);
    const uint8_t* sdma_ring = fixture.cpu_of(sdma.buffer);
    expect(u32(gfx_ring, 0) == gfx11_ring_nop && u32(gfx_ring, 8188) == gfx11_ring_nop &&
           u32(gfx_ring, 8192 + 96) == 1 && u32(gfx_ring, 8192 + 64) == 0 && u32(sdma_ring, 0) == 0 &&
           u32(sdma_ring, 8192 + 96) == 1, "amdgpu_ring_init: NOP-cleared ring, cond_exec 1, fence 0");

    // Submission.
    const auto ib = fixture.map(process.pasid, ib_va, 0x3000, read_only);
    const auto target = fixture.map(process.pasid, target_va, 0x1000);
    expect(ib.status == GpuVmStatus::mapped && target.status == GpuVmStatus::mapped,
           "IB and target mapped in the process VM");
    uint8_t* ib_cpu = fixture.cpu_of(ib.buffer);
    write_gfx_ib(ib_cpu, target_va);
    write_gfx_ib(ib_cpu + 0x1000, target_va + 4);
    write_sdma_ib(ib_cpu + 0x2000, target_va + 8, 0x5d5d5d5du);
    uint32_t seq[3]{};
    expect(queues.submit(gfx.id, {ib_va, 5}, seq[0]) == UserQueueStatus::submitted &&
           queues.submit(compute.id, {ib_va + 0x1000, 5}, seq[1]) == UserQueueStatus::submitted &&
           queues.submit(sdma.id, {ib_va + 0x2000, 8}, seq[2]) == UserQueueStatus::submitted &&
           seq[0] == 1 && seq[1] == 1 && seq[2] == 1, "one IB on each queue, fence 1");
    const uint8_t* data = fixture.cpu_of(target.buffer);
    expect(u32(data, 0) == 0xDEADBEEFu && u32(data, 4) == 0xDEADBEEFu && u32(data, 8) == 0x5d5d5d5du &&
           fixture.engines.translation_faults == 0, "engines ran every IB through the process VM");
    expect(queues.query(gfx.id, 1) == UserQueueStatus::signalled &&
           queues.query(compute.id, 1) == UserQueueStatus::signalled &&
           queues.query(sdma.id, 1) == UserQueueStatus::signalled &&
           queues.query(gfx.id, 2) == UserQueueStatus::invalid_argument &&
           queues.query(9, 1) == UserQueueStatus::not_found, "fences signalled on each queue's slot");
    expect(fixture.engines.fence_contexts.size() == 3 &&
           fixture.engines.fence_contexts[0] == (1u | 0x01000000u) &&
           fixture.engines.fence_contexts[1] == (2u | 0x01000000u) &&
           fixture.engines.fence_contexts[2] == (3u | 0x01000000u),
           "RELEASE_MEM/TRAP carry queue id | AMDGPU_FENCE_MES_QUEUE_FLAG");
    expect(u32(gfx_ring, 0) == 0xC0065800u && fixture.engines.doorbell_values.size() == 3 &&
           fixture.engines.doorbell_values[0] == 256 && fixture.engines.doorbell_values[1] == 256 &&
           fixture.engines.doorbell_values[2] == 32 * 4 && u64(gfx_ring, 8192 + 32) == 256 &&
           u64(sdma_ring, 8192 + 32) == 128 && fixture.wptr_stores == 3,
           "ACQUIRE_MEM first (mem_sync), 256-dword gfx frames, 32-dword SDMA frame published "
           "in bytes, atomic shadows");
    const UserQueueReport report = queues.queue_report(gfx.id);
    expect(report.sync_seq == 1 && report.last_seq == 1 && report.retired_position == 256 &&
           !report.timeout_armed && report.timeout_ms == fence_gfx_timeout_ms &&
           queues.queue_report(compute.id).timeout_ms == fence_compute_timeout_ms,
           "retired position at the frame end, timeout disarmed when idle, Linux defaults");

    // Teardown order: gang busy with queues, queue removal, gang, process.
    expect(queues.remove_gang(gang.id) == UserQueueStatus::busy, "gang with queues is busy");
    expect(queues.remove_queue(gfx.id) == UserQueueStatus::removed &&
           !fixture.mes->doorbell_allocated(doorbells[0]) && fixture.frames(3).size() == 1 &&
           fixture.frames(3)[0].at(1) == doorbells[0] &&
           frame64(fixture.frames(3)[0], 8) == gang_gart[0], "REMOVE_QUEUE, doorbell freed");
    expect(fixture.gtt->release(gfx.buffer) == rx7900::memory::Status::success &&
           fixture.gtt->release(gfx.mqd) == rx7900::memory::Status::success,
           "removed queue's buffer and MQD leases returned");
    expect(queues.vm_report(process.pasid).mappings == 4, "queue buffer unmapped (4 mappings left)");
    expect(queues.destroy_process(process.pasid) == UserQueueStatus::destroyed &&
           fixture.frames(3).size() == 3 && !fixture.pasids->allocated(0x8000) &&
           fixture.mes->report().process_queues == 0 && fixture.mes->report().doorbells_allocated == 0 &&
           fixture.mes->report().queue_ids_allocated == 0 && queues.report().processes == 0 &&
           queues.report().gangs == 0, "destroy_process removes queues and gangs, frees PASID/VM");
    expect(fixture.vram->release(process.tables) == rx7900::memory::Status::success &&
           fixture.gtt->release(process.context) == rx7900::memory::Status::success &&
           fixture.gtt->release(gang.context) == rx7900::memory::Status::success &&
           fixture.gtt->release(sdma.buffer) == rx7900::memory::Status::success,
           "every lease returned");
    expect(fixture.mes->kiq_hw_fini() == Mes11Status::disabled &&
           fixture.mes->release() == Mes11Status::released, "MES teardown after the process");
}

void refusals() {
    Fixture fixture;
    fixture.up();
    UserQueues& queues = *fixture.queues;
    const auto process = fixture.process();
    const auto gang = fixture.gang(process.pasid);
    const size_t frames = fixture.model.mes_frames.size();
    expect(fixture.queue(gang.id, Mes11QueueType::gfx, gfx_va, 4096).status ==
               UserQueueStatus::invalid_argument &&
           fixture.queue(gang.id, Mes11QueueType::gfx, gfx_va + 8, 8192).status ==
               UserQueueStatus::invalid_argument &&
           fixture.queue(gang.id, Mes11QueueType::gfx, gpuvm_max_pfn << 12, 8192).status ==
               UserQueueStatus::invalid_argument &&
           fixture.queue(99, Mes11QueueType::gfx, gfx_va).status == UserQueueStatus::not_found,
           "ring below 8 KiB, unaligned VA, VA beyond max_pfn, unknown gang");
    rx7900::memory::Handle buffer = fixture.gtt_buffer(8192 + 4096);
    rx7900::memory::Handle mqd = fixture.gtt_buffer(4096);
    auto pages = fixture.dma(buffer);
    uint32_t id = 0;
    expect(queues.add_queue(gang.id, {Mes11QueueType::gfx, 8192, gfx_va, buffer,
                                      {pages.data(), pages.size() - 8}, mqd, 0}, id) ==
               UserQueueStatus::invalid_argument &&
           queues.add_queue(gang.id, {Mes11QueueType::gfx, 8192, gfx_va, buffer,
                                      {pages.data(), pages.size()}, buffer, 0}, id) ==
               UserQueueStatus::invalid_memory, "short DMA list; MQD aliasing the buffer");
    const auto first = fixture.queue(gang.id, Mes11QueueType::compute, gfx_va);
    const auto overlap = fixture.queue(gang.id, Mes11QueueType::compute, gfx_va + 0x1000);
    expect(first.status == UserQueueStatus::added && overlap.status == UserQueueStatus::invalid_argument &&
           fixture.mes->report().doorbells_allocated == 1 && fixture.frames(2).size() == 1 &&
           fixture.model.mes_frames.size() == frames + 2, "overlapping queue VA refused, rolled back");
    expect(fixture.gtt->release(overlap.buffer) == rx7900::memory::Status::success &&
           fixture.gtt->release(overlap.mqd) == rx7900::memory::Status::success,
           "refused queue holds no lease");
    uint32_t seq = 0;
    expect(queues.submit(first.id, {ib_va, 5}, seq) == UserQueueStatus::invalid_memory &&
           queues.submit(first.id, {ib_va + 2, 5}, seq) == UserQueueStatus::invalid_argument &&
           queues.submit(first.id, {ib_va, 0}, seq) == UserQueueStatus::invalid_argument,
           "IB not mapped, unaligned, empty");
    const auto ib = fixture.map(process.pasid, ib_va, 0x1000, read_only);
    expect(queues.submit(first.id, {ib_va + 0xff0, 5}, seq) == UserQueueStatus::invalid_memory,
           "IB crossing into an unmapped page");
    expect(queues.unmap(process.pasid, gfx_va) == GpuVmStatus::invalid_argument,
           "queue buffer cannot be unmapped by the caller");
    const auto sdma = fixture.queue(gang.id, Mes11QueueType::sdma, sdma_va);
    expect(queues.submit(sdma.id, {ib_va + 4, 8}, seq) == UserQueueStatus::invalid_argument &&
           queues.submit(sdma.id, {ib_va, 6}, seq) == UserQueueStatus::invalid_argument,
           "SDMA IB needs 32-byte alignment and a multiple of 8 dwords");
    (void)ib;

    UserQueues fresh;
    UserQueueConfig config = fixture.config();
    config.wptr_store = {nullptr, nullptr};
    expect(fresh.initialize(config) == UserQueueStatus::invalid_argument, "missing WPTR store refused");
    expect(queues.initialize(fixture.config()) == UserQueueStatus::busy, "no re-init with live processes");
}

void fences_and_timeouts() {
    Fixture fixture;
    fixture.up();
    UserQueues& queues = *fixture.queues;
    const auto process = fixture.process();
    const auto gang = fixture.gang(process.pasid);
    const auto gfx = fixture.queue(gang.id, Mes11QueueType::gfx, gfx_va, 8192, 100);
    const auto ib = fixture.map(process.pasid, ib_va, 0x1000, read_only);
    const auto target = fixture.map(process.pasid, target_va, 0x1000);
    write_gfx_ib(fixture.cpu_of(ib.buffer), target_va);
    (void)target;
    const UserQueueReport added = queues.queue_report(gfx.id);
    fixture.engines.hang_doorbell = added.doorbell_index;
    uint32_t seq = 0;

    for (uint32_t index = 0; index < 4; ++index) {
        queues.submit(gfx.id, {ib_va, 5}, seq);
    }

    expect(seq == 4 && queues.submit(gfx.id, {ib_va, 5}, seq) == UserQueueStatus::ring_full &&
           queues.query(gfx.id, 1) == UserQueueStatus::pending,
           "four fences outstanding (num_fences_mask 3): the fifth waits");
    const UserQueueReport armed = queues.queue_report(gfx.id);
    expect(armed.timeout_armed && armed.timeout_deadline_us == fixture.now + 100000 &&
           queues.next_deadline_us() == fixture.now + 100000, "job timeout armed at submission");
    expect(queues.remove_queue(gfx.id) == UserQueueStatus::busy, "removal waits for outstanding work");

    // Two fences signal: the timeout restarts for the rest.
    fixture.now += 50000;
    put(fixture.cpu_of(gfx.buffer), 8192 + 64, 2);
    queues.poll();
    const UserQueueReport progress = queues.queue_report(gfx.id);
    expect(progress.last_seq == 2 && progress.retired_position == 512 &&
           progress.timeout_deadline_us == fixture.now + 100000 &&
           queues.query(gfx.id, 2) == UserQueueStatus::signalled,
           "progress retires frames and restarts the timeout");

    // Suspended gangs: the deadline follows the clock.
    expect(fixture.mes->suspend_all() == Mes11Status::completed, "SUSPEND all gangs");
    fixture.now += 500000;
    queues.poll();
    expect(queues.queue_report(gfx.id).state == UserQueueState::active &&
           queues.submit(gfx.id, {ib_va, 5}, seq) == UserQueueStatus::suspended,
           "no timeout and no submission while suspended");
    expect(fixture.mes->resume_all() == Mes11Status::completed, "RESUME all gangs");

    fixture.now += 100001;
    queues.poll();
    const UserQueueReport hung = queues.queue_report(gfx.id);
    expect(hung.state == UserQueueState::hung && hung.hang_reason == FenceHangReason::job_timeout &&
           hung.hang_signalled_seq == 2 && hung.hang_emitted_seq == 4 &&
           queues.query(gfx.id, 3) == UserQueueStatus::hung &&
           queues.query(gfx.id, 2) == UserQueueStatus::signalled &&
           queues.submit(gfx.id, {ib_va, 5}, seq) == UserQueueStatus::hung &&
           queues.report().hung_queues == 1, "job timeout quarantines the queue (no reset)");
    expect(queues.remove_queue(gfx.id) == UserQueueStatus::removed &&
           queues.query(gfx.id, 3) == UserQueueStatus::not_found, "a hung queue may be removed");
    fixture.engines.hang_doorbell = UINT32_MAX;

    // A fence value outside the emitted window.
    const auto compute = fixture.queue(gang.id, Mes11QueueType::compute, compute_va);
    fixture.engines.corrupt_fence_doorbell = queues.queue_report(compute.id).doorbell_index;
    expect(queues.submit(compute.id, {ib_va, 5}, seq) == UserQueueStatus::submitted &&
           queues.query(compute.id, 1) == UserQueueStatus::hung &&
           queues.queue_report(compute.id).hang_reason == FenceHangReason::fence_out_of_window,
           "fence outside (last, sync] quarantines");

    // Failed doorbell after the frame.
    const auto sdma = fixture.queue(gang.id, Mes11QueueType::sdma, sdma_va);
    write_sdma_ib(fixture.cpu_of(ib.buffer) + 0x100, target_va, 1);
    fixture.fail_user_doorbell = true;
    expect(queues.submit(sdma.id, {ib_va + 0x100, 8}, seq) == UserQueueStatus::hung &&
           queues.queue_report(sdma.id).hang_reason == FenceHangReason::submit_failure,
           "failed doorbell: queue hung");
    fixture.fail_user_doorbell = false;

    UserQueueInterruptHint hint;
    rx7900::hardware::IrqReport irq{};
    expect(!hint.observe({&irq, nullptr, false}), "no MES fence interrupt yet");
    irq.sequence[static_cast<uint32_t>(IrqSource::mes_eop)] = 1;
    expect(hint.observe({&irq, nullptr, false}) && !hint.observe({&irq, nullptr, false}) &&
           hint.observe({&irq, nullptr, true}), "MES EOP or lost vectors request a poll");
}

void mes_failures() {
    // ADD_QUEUE that MES never completes.
    {
        Fixture fixture(0x00001063u, 200);
        fixture.up();
        UserQueues& queues = *fixture.queues;
        const auto process = fixture.process();
        const auto gang = fixture.gang(process.pasid);
        fixture.model.ignore_opcode = 2;
        const auto lost = fixture.queue(gang.id, Mes11QueueType::gfx, gfx_va);
        expect(lost.status == UserQueueStatus::quarantined && lost.id == 1 &&
               queues.queue_report(lost.id).state == UserQueueState::quarantined &&
               queues.report().reset_required && queues.report().queues == 1,
               "lost ADD_QUEUE: queue quarantined with its leases");
        expect(queues.destroy_process(process.pasid) == UserQueueStatus::quarantined &&
               fixture.info(*fixture.gtt, lost.buffer).leases == 1,
               "process kept; buffer still leased");
        expect(queues.release_after_reset(Fixture::confirm, &fixture.reset_confirmed) ==
                   UserQueueStatus::quarantined, "no reset proof, nothing released");
        fixture.reset_confirmed = true;
        expect(queues.release_after_reset(Fixture::confirm, &fixture.reset_confirmed) ==
                   UserQueueStatus::released && queues.report().processes == 0 &&
               !fixture.pasids->allocated(process.pasid) &&
               fixture.gtt->release(lost.buffer) == rx7900::memory::Status::success &&
               fixture.vram->release(process.tables) == rx7900::memory::Status::success,
               "confirmed reset returns every lease");
        expect(fixture.mes->release_after_reset() == Mes11Status::released &&
               fixture.mes->report().doorbells_allocated == 0, "then Mes11 forgets its allocations");
    }

    // REMOVE_QUEUE that MES never completes.
    {
        Fixture fixture(0x00001063u, 200);
        fixture.up();
        UserQueues& queues = *fixture.queues;
        const auto process = fixture.process();
        const auto gang = fixture.gang(process.pasid);
        const auto queue = fixture.queue(gang.id, Mes11QueueType::sdma, sdma_va);
        fixture.model.ignore_opcode = 3;
        expect(queues.remove_queue(queue.id) == UserQueueStatus::quarantined &&
               queues.queue_report(queue.id).state == UserQueueState::quarantined &&
               queues.remove_gang(gang.id) == UserQueueStatus::quarantined &&
               fixture.info(*fixture.gtt, queue.mqd).leases == 1,
               "lost REMOVE_QUEUE quarantines queue, gang and process");
    }

    // MES not ready / quarantined before any frame.
    {
        Fixture fixture;
        expect(fixture.queues->initialize(fixture.config()) == UserQueueStatus::ready,
               "UserQueues validates without MES");
        const auto process = fixture.process();
        const auto gang = fixture.gang(process.pasid);
        expect(process.status == UserQueueStatus::created &&
               fixture.queue(gang.id, Mes11QueueType::gfx, gfx_va).status == UserQueueStatus::unavailable &&
               fixture.mes->report().doorbells_allocated == 0,
               "no queue without the MES scheduler");
        expect(fixture.queues->destroy_process(process.pasid) == UserQueueStatus::destroyed,
               "empty process destroyed");
    }

    // Page-table pool exhaustion while mapping the queue buffer.
    {
        Fixture fixture;
        fixture.up();
        const auto process = fixture.process(2);
        const auto gang = fixture.gang(process.pasid);
        const auto queue = fixture.queue(gang.id, Mes11QueueType::gfx, gfx_va);
        expect(queue.status == UserQueueStatus::vm_failure &&
               fixture.mes->report().doorbells_allocated == 0 &&
               fixture.gtt->release(queue.buffer) == rx7900::memory::Status::success,
               "two-table pool cannot map: rolled back before any id or frame");
    }
}

void memory_recovery() {
    {
        Fixture fixture;
        expect(fixture.up(), "recovery fixture starts real queue implementation");
        auto& queues = *fixture.queues;
        const auto process = fixture.process(16, true);
        const auto gang = fixture.gang(process.pasid);
        const auto removed = fixture.queue(gang.id, Mes11QueueType::gfx, gfx_va);
        const auto retained = fixture.queue(gang.id, Mes11QueueType::sdma, sdma_va);
        expect(fixture.mes->kiq_hw_fini() == Mes11Status::busy &&
               fixture.mes->report().scheduler_ready,
               "MES cannot halt while the process owner still has registered queues");
        expect(queues.remove_queue(removed.id) == UserQueueStatus::removed && retained.id == 2,
               "recovery keeps a nonconsecutive logical queue identity");
        const auto ib = fixture.map(process.pasid, ib_va, 4096, read_only);
        const auto target = fixture.map(process.pasid, target_va, 4096);
        write_sdma_ib(fixture.cpu_of(ib.buffer), target_va, 0x12345678u);
        uint32_t sequence = 0;
        expect(queues.submit(retained.id, {ib_va, 8}, sequence) == UserQueueStatus::submitted &&
               queues.query(retained.id, sequence) == UserQueueStatus::signalled,
               "real process-VA SDMA work retires before suspend");
        const auto before = queues.vm_report(process.pasid);
        expect(queues.suspend_for_recovery() == UserQueueStatus::suspended &&
               queues.recovery_suspended() && fixture.mes->report().process_queues == 0 &&
               fixture.mes->report().doorbells_allocated == 0 &&
               fixture.info(*fixture.gtt, retained.buffer).leases == 1 &&
               fixture.info(*fixture.gtt, process.backup).leases == 1 &&
               fixture.pasids->allocated(process.pasid),
               "suspend removes MES registration while retaining every mapping, PASID and lease");
        uint32_t refused = 0;
        expect(queues.submit(retained.id, {ib_va, 8}, refused) == UserQueueStatus::suspended &&
               queues.destroy_process(process.pasid) == UserQueueStatus::suspended &&
               queues.unmap(process.pasid, ib_va) == GpuVmStatus::unavailable &&
               queues.cancel_recovery() == UserQueueStatus::quarantined,
               "all mutation stays blocked after detach and cannot cancel saved recovery");

        const auto tables = fixture.info(*fixture.vram, process.tables);
        std::fill(tables.backing.cpu_bytes,
                  tables.backing.cpu_bytes + tables.backing.cpu_byte_count, uint8_t{0xa5});
        expect(queues.resume_after_recovery() == UserQueueStatus::ready &&
               !queues.recovery_suspended() && queues.queue_report(retained.id).queue_id == 2 &&
               queues.queue_report(retained.id).sync_seq == sequence &&
               queues.vm_report(process.pasid).mappings == before.mappings &&
               queues.vm_report(process.pasid).flushed_seq > before.flushed_seq &&
               fixture.mes->report().process_queues == 1,
               "lost VRAM table image rebuilt with PASID invalidate and stable queue/fence ids");
        write_sdma_ib(fixture.cpu_of(ib.buffer), target_va, 0x87654321u);
        uint32_t after = 0;
        expect(queues.submit(retained.id, {ib_va, 8}, after) == UserQueueStatus::submitted &&
               after == sequence + 1 && queues.query(retained.id, after) == UserQueueStatus::signalled &&
               u32(fixture.cpu_of(target.buffer), 0) == 0x87654321u,
               "post-recovery work walks rebuilt actual tables and continues fence sequence");
        expect(queues.destroy_process(process.pasid) == UserQueueStatus::destroyed &&
               fixture.gtt->release(process.backup) == rx7900::memory::Status::success,
               "normal process teardown returns its retained recovery image");
    }
    {
        Fixture fixture;
        fixture.up();
        auto& queues = *fixture.queues;
        const auto process = fixture.process();
        expect(queues.suspend_for_recovery() == UserQueueStatus::invalid_memory &&
               queues.cancel_recovery() == UserQueueStatus::ready &&
               queues.destroy_process(process.pasid) == UserQueueStatus::destroyed,
               "missing backup refuses before hardware detach and permits explicit cancel");
    }
    {
        Fixture fixture(0x00001063u, 200);
        fixture.up();
        auto& queues = *fixture.queues;
        const auto process = fixture.process(16, true);
        const auto gang = fixture.gang(process.pasid);
        const auto queue = fixture.queue(gang.id, Mes11QueueType::gfx, gfx_va);
        fixture.model.ignore_opcode = 3;
        expect(queues.suspend_for_recovery() == UserQueueStatus::quarantined &&
               queues.report().reset_required &&
               fixture.info(*fixture.gtt, queue.buffer).leases == 1 &&
               fixture.info(*fixture.gtt, process.backup).leases == 1 &&
               queues.resume_after_recovery() == UserQueueStatus::unavailable,
               "uncertain REMOVE keeps all memory quarantined and forbids replay");
        expect(queues.release_after_reset(Fixture::confirm, &fixture.reset_confirmed) ==
                   UserQueueStatus::quarantined,
               "failed detach never supplies its own reset proof");
    }
    {
        Fixture fixture;
        fixture.up();
        auto& queues = *fixture.queues;
        const auto process = fixture.process(16, true);
        const auto gang = fixture.gang(process.pasid);
        const auto queue = fixture.queue(gang.id, Mes11QueueType::gfx, gfx_va);
        const auto ib = fixture.map(process.pasid, ib_va, 4096, read_only);
        write_gfx_ib(fixture.cpu_of(ib.buffer), target_va);
        fixture.engines.hang_doorbell = queues.queue_report(queue.id).doorbell_index;
        uint32_t sequence = 0;
        expect(queues.submit(queue.id, {ib_va, 5}, sequence) == UserQueueStatus::submitted &&
               queues.suspend_for_recovery() == UserQueueStatus::busy &&
               fixture.mes->report().process_queues == 1 &&
               queues.submit(queue.id, {ib_va, 5}, sequence) == UserQueueStatus::suspended,
               "inflight work refuses suspend before detach and blocks new submissions");
        expect(queues.cancel_recovery() == UserQueueStatus::ready &&
               queues.query(queue.id, 1) == UserQueueStatus::pending,
               "cancel only before detach preserves the unsignalled fence");
    }
    {
        Fixture fixture;
        fixture.up();
        auto& queues = *fixture.queues;
        const auto process = fixture.process(16, true);
        const auto gang = fixture.gang(process.pasid);
        const auto queue = fixture.queue(gang.id, Mes11QueueType::gfx, gfx_va);
        expect(queues.suspend_for_recovery() == UserQueueStatus::suspended &&
               fixture.mes->reserve_queue_id(queue.id) == Mes11Status::completed &&
               queues.resume_after_recovery() == UserQueueStatus::quarantined &&
               queues.report().reset_required &&
               fixture.info(*fixture.gtt, queue.buffer).leases == 1,
               "identity collision after restart fails closed and retains the table/buffer leases");
        fixture.reset_confirmed = true;
        expect(queues.release_after_reset(Fixture::confirm, &fixture.reset_confirmed) ==
                   UserQueueStatus::released &&
               fixture.mes->reserve_queue_id(queue.id) == Mes11Status::busy,
               "reset cleanup cannot free an identity reserved by a different owner");
    }
}

// Process-exit eviction (amdgpu_userq_cleanup after a bounded last-fence wait)
// and post-reset fence continuation.
void exit_eviction_and_fence_continuation() {
    {
        Fixture fixture;
        fixture.up();
        auto& queues = *fixture.queues;
        const auto process = fixture.process();
        const auto gang = fixture.gang(process.pasid);
        const auto queue = fixture.queue(gang.id, Mes11QueueType::sdma, sdma_va);
        const auto idle = fixture.queue(gang.id, Mes11QueueType::gfx, gfx_va);
        const auto ib = fixture.map(process.pasid, ib_va, 4096, read_only);
        const auto target = fixture.map(process.pasid, target_va, 4096);
        write_sdma_ib(fixture.cpu_of(ib.buffer), target_va, 0x13579bdfu);
        fixture.engines.hang_doorbell = queues.queue_report(queue.id).doorbell_index;
        uint32_t sequence = 0;
        expect(queues.submit(queue.id, {ib_va, 8}, sequence) == UserQueueStatus::submitted &&
               queues.destroy_process(process.pasid) == UserQueueStatus::busy,
               "a healthy queue with an outstanding fence refuses ordinary destruction");
        const size_t removes = fixture.frames(3).size();
        expect(queues.evict_process_queues(process.pasid) == UserQueueStatus::removed &&
               fixture.frames(3).size() == removes + 2 && queues.queue_report(queue.id).queue_id == 0 &&
               queues.queue_report(idle.id).queue_id == 0 && fixture.mes->report().process_queues == 0 &&
               fixture.info(*fixture.gtt, queue.buffer).leases == 0,
               "eviction marks the unfinished queue hung and removes every queue through REMOVE_QUEUE");
        expect(queues.vm_report(process.pasid).pasid == process.pasid &&
               queues.unmap(process.pasid, target_va) == GpuVmStatus::unmapped &&
               queues.flush_tlb(process.pasid, 2) == GpuVmStatus::flushed &&
               queues.destroy_process(process.pasid) == UserQueueStatus::destroyed,
               "the VM stays for the owner's unmaps and flush, then destroy_process finishes it");
        expect(queues.evict_process_queues(process.pasid) == UserQueueStatus::not_found,
               "an unknown PASID is refused");
        (void)target;
    }
    {
        Fixture fixture(0x00001063u, 200);
        fixture.up();
        auto& queues = *fixture.queues;
        const auto process = fixture.process();
        const auto gang = fixture.gang(process.pasid);
        const auto queue = fixture.queue(gang.id, Mes11QueueType::sdma, sdma_va);
        fixture.model.ignore_opcode = 3;
        expect(queues.evict_process_queues(process.pasid) == UserQueueStatus::quarantined &&
               queues.report().reset_required && fixture.info(*fixture.gtt, queue.buffer).leases == 1 &&
               queues.evict_process_queues(process.pasid) == UserQueueStatus::quarantined,
               "a REMOVE_QUEUE MES never completes keeps the queue and its leases until reset");
    }
    {
        Fixture fixture;
        fixture.up();
        auto& queues = *fixture.queues;
        const auto process = fixture.process();
        const auto gang = fixture.gang(process.pasid);
        const auto queue = fixture.queue(gang.id, Mes11QueueType::sdma, sdma_va);
        const auto ib = fixture.map(process.pasid, ib_va, 4096, read_only);
        const auto target = fixture.map(process.pasid, target_va, 4096);
        expect(queues.continue_fences(queue.id, 41) == UserQueueStatus::ready &&
               queues.queue_report(queue.id).sync_seq == 41 && queues.queue_report(queue.id).last_seq == 41 &&
               u32(fixture.cpu_of(queue.buffer), 8192 + user_queue_fence_offset) == 41,
               "a rebuilt queue continues the pre-reset fence values in its writeback slot");
        expect(queues.query(queue.id, 41) == UserQueueStatus::signalled &&
               queues.query(queue.id, 42) == UserQueueStatus::invalid_argument,
               "the continued window neither signals nor invents later fences");
        write_sdma_ib(fixture.cpu_of(ib.buffer), target_va, 0x2468ace0u);
        uint32_t sequence = 0;
        expect(queues.submit(queue.id, {ib_va, 8}, sequence) == UserQueueStatus::submitted && sequence == 42 &&
               queues.query(queue.id, sequence) == UserQueueStatus::signalled &&
               u32(fixture.cpu_of(target.buffer), 0) == 0x2468ace0u,
               "the next submission is 42 and its real engine fence signals");
        expect(queues.continue_fences(queue.id, 7) == UserQueueStatus::invalid_argument &&
               queues.continue_fences(999, 7) == UserQueueStatus::not_found,
               "only a fresh queue may be continued");
        expect(queues.destroy_process(process.pasid) == UserQueueStatus::destroyed, "teardown");
    }
}

} // namespace

int main() {
    lifecycle();
    refusals();
    fences_and_timeouts();
    mes_failures();
    memory_recovery();
    exit_eviction_and_fence_continuation();
    return checks.finish("user_queue_tests");
}

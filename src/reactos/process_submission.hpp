/*
Module: rx7900.reactos.process_submission
Purpose: Own semantic SDMA work and buffer leases through actual MES fences.
Created: 2026-10-22
Architecture: One serialized process owner reuses ProcessMemory, UserQueues,
Manager and the SDMA encoder. No user command stream or private IB is exposed.
*/
#pragma once

#include "process_memory.hpp"

namespace rx7900::reactos {

constexpr uint64_t process_submission_va = 1ULL << 45;

struct ProcessTransfer final {
    uint64_t source = 0;
    uint64_t destination = 0;
    uint64_t source_offset = 0;
    uint64_t destination_offset = 0;
    uint64_t bytes = 0;
    uint32_t value = 0;
};

// PASSIVE_LEVEL, serialized by the native process service. The DMA callback
// resolves only this owner's real common-buffer allocation. Config owners and
// CPU backing remain alive through close_after_idle, including failed submits.
struct ProcessSubmissionConfig final {
    hardware::UserQueues* queues;
    ProcessMemory* buffers;
    memory::Manager* gtt;
    uint32_t pasid;
    void* context;
    bool (*dma_bus)(void*, memory::Handle, uint64_t&) noexcept;
};

class ProcessSubmissions final {
public:
    memory::Status initialize(ProcessSubmissionConfig config) noexcept;
    // One outstanding transfer per process bounds retained command memory.
    // success acknowledges submission, not completion; query owns retirement.
    memory::Status copy(uint32_t queue, const ProcessTransfer& transfer,
                        uint32_t& sequence) noexcept;
    memory::Status fill(uint32_t queue, const ProcessTransfer& transfer,
                        uint32_t& sequence) noexcept;
    memory::Status query(uint32_t queue, uint32_t sequence) noexcept;
    memory::Status poll() noexcept;
    // Rebind the retained private IB after a confirmed reset retired the old
    // process VM. There can be no pending command or uncertain publication.
    memory::Status rebind_after_reset(ProcessSubmissionConfig config) noexcept;
    bool ready_for_reset_rebind() const noexcept;
    // Inside a confirmed destructive reset window (every queue and the VM are
    // retired): an outstanding transfer was neither proven complete nor is it
    // replayed (Linux amdgpu_userq_pre_reset force-completes such fences with
    // an error); its buffer leases return now. queue/sequence name that lost
    // work (both 0 when nothing was outstanding or an uncertain publication
    // never returned a sequence). An uncertain publication's quarantine is
    // resolved by the same proof. The private IB backing (GTT) is kept.
    memory::Status retire_after_reset(uint32_t& queue, uint32_t& sequence) noexcept;
    // After a failed rebind_after_reset whose new VM the caller destroyed.
    void abandon_rebind() noexcept;
    // Only after UserQueues removed this exact queue, or the process VM is gone.
    // Neither a timeout nor an absent receipt authorizes releasing resources.
    memory::Status retire_removed_queue(uint32_t queue) noexcept;
    memory::Status close_after_idle() noexcept;
    bool empty() const noexcept { return !ib_live_ && !source_held_ && !destination_held_; }

private:
    memory::Status submit(uint32_t queue, const ProcessTransfer& transfer,
                          bool fill, uint32_t& sequence) noexcept;
    memory::Status release_buffers() noexcept;
    memory::Status map_private_ib() noexcept;
    ProcessSubmissionConfig config_{};
    memory::Handle ib_{};
    uint8_t* cpu_ = nullptr;
    uint32_t queue_ = 0;
    uint32_t sequence_ = 0;
    uint64_t source_ = 0;
    uint64_t destination_ = 0;
    bool source_held_ = false;
    bool destination_held_ = false;
    bool ib_live_ = false;
    bool ib_retained_ = false;
    bool initialized_ = false;
    bool pending_ = false;
    bool quarantined_ = false;
};

} // namespace rx7900::reactos

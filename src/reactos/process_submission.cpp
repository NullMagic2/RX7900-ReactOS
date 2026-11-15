/*
Module: rx7900.reactos.process_submission
Purpose: Lease authorized buffers around driver-built SDMA and real queue fences.
Created: 2026-10-22
Architecture: Retained GTT IB is mapped read-only into its process VM; uncertain
publication retains all ownership until authoritative queue/VM retirement.
*/
#include "process_submission.hpp"

namespace rx7900::reactos {
namespace {
using memory::Status;
using hardware::UserQueueStatus;

bool span(const ProcessBufferInfo& buffer, uint64_t offset, uint64_t bytes) noexcept {
    return buffer.mapped && !buffer.quarantined && bytes != 0 && offset <= buffer.bytes &&
        bytes <= buffer.bytes - offset;
}

bool uncertain(hardware::GpuVmStatus status) noexcept {
    return status == hardware::GpuVmStatus::quarantined ||
        status == hardware::GpuVmStatus::mmio_failure;
}
}

memory::Status ProcessSubmissions::initialize(ProcessSubmissionConfig config) noexcept {
    if (initialized_ || !empty()) return Status::busy;
    if (config.queues == nullptr || config.buffers == nullptr || config.gtt == nullptr ||
        config.pasid == 0 || config.dma_bus == nullptr ||
        config.queues->vm_report(config.pasid).pasid != config.pasid) return Status::invalid_argument;
    config_ = config;
    auto status = config_.gtt->allocate(memory::page_bytes, memory::page_bytes,
        memory::Access::read_write, true, ib_);
    ib_live_ = status == Status::success;
    if (!ib_live_) return status;
    status = config_.gtt->retain(ib_, memory::Access::read_write);
    ib_retained_ = status == Status::success;
    if (!ib_retained_) return status;
    status = map_private_ib();
    if (status != Status::success) return status;
    initialized_ = true;
    quarantined_ = false;
    return Status::success;
}

memory::Status ProcessSubmissions::map_private_ib() noexcept {
    memory::AllocationInfo info{};
    uint64_t bus = 0;
    if (config_.gtt->describe(ib_, info) != Status::success || info.retiring ||
        info.backing.cpu_bytes == nullptr || info.backing.cpu_byte_count < memory::page_bytes ||
        info.gpu.byte_length < memory::page_bytes || !config_.dma_bus(config_.context, ib_, bus) ||
        (bus & (memory::page_bytes - 1)) != 0 || bus > UINT64_MAX - memory::page_bytes) {
        return Status::backend_failure;
    }
    uint8_t page[8]{};
    for (uint32_t byte = 0; byte < sizeof(page); ++byte)
        page[byte] = static_cast<uint8_t>(bus >> (byte * 8));
    const hardware::GpuVmMapFlags flags{memory::Access::read, false,
        hardware::GpuVmMtype::nc, false};
    const auto mapped = config_.queues->map_system(config_.pasid, process_submission_va,
        ByteView(page, sizeof(page)), false, flags);
    if (mapped != hardware::GpuVmStatus::mapped) {
        quarantined_ = uncertain(mapped);
        return quarantined_ ? Status::quarantined : Status::backend_failure;
    }
    if (config_.queues->flush_tlb(config_.pasid, 0) != hardware::GpuVmStatus::flushed) {
        quarantined_ = true;
        return Status::quarantined;
    }
    cpu_ = info.backing.cpu_bytes;
    return Status::success;
}

memory::Status ProcessSubmissions::rebind_after_reset(ProcessSubmissionConfig config) noexcept {
    if (!initialized_ || pending_ || quarantined_ || !ib_live_ || !ib_retained_ ||
        source_held_ || destination_held_ || config.queues == nullptr || config.pasid == 0 ||
        config.buffers != config_.buffers || config.gtt != config_.gtt ||
        config.context != config_.context || config.dma_bus != config_.dma_bus ||
        config.queues->vm_report(config.pasid).pasid != config.pasid) return Status::unavailable;
    config_ = config;
    const auto status = map_private_ib();
    if (status != Status::success) quarantined_ = true;
    queue_ = 0;
    sequence_ = 0;
    return status;
}

memory::Status ProcessSubmissions::retire_after_reset(uint32_t& queue, uint32_t& sequence) noexcept {
    queue = 0;
    sequence = 0;
    if (!initialized_) return empty() ? Status::success : Status::unavailable;
    if (pending_) {
        queue = queue_;
        sequence = sequence_;
    }
    const auto released = release_buffers();
    if (released != Status::success) return released;
    quarantined_ = false;
    queue_ = 0;
    sequence_ = 0;
    config_.pasid = 0;
    return Status::success;
}

void ProcessSubmissions::abandon_rebind() noexcept {
    // The caller destroyed the VM the failed rebind mapped into.
    if (initialized_ && !pending_ && !source_held_ && !destination_held_) {
        quarantined_ = false;
        config_.pasid = 0;
    }
}

bool ProcessSubmissions::ready_for_reset_rebind() const noexcept {
    return initialized_ && !pending_ && !quarantined_ && ib_live_ && ib_retained_ &&
        !source_held_ && !destination_held_;
}

memory::Status ProcessSubmissions::release_buffers() noexcept {
    if (source_held_) {
        if (config_.buffers->unretain(source_) != Status::success) {
            quarantined_ = true;
            return Status::quarantined;
        }
        source_held_ = false;
    }
    if (destination_held_) {
        if (config_.buffers->unretain(destination_) != Status::success) {
            quarantined_ = true;
            return Status::quarantined;
        }
        destination_held_ = false;
    }
    pending_ = false;
    return Status::success;
}

memory::Status ProcessSubmissions::poll() noexcept {
    if (!initialized_) return Status::unavailable;
    if (quarantined_) return Status::quarantined;
    if (!pending_) return Status::success;
    const auto state = config_.queues->query(queue_, sequence_);
    if (state == UserQueueStatus::signalled) return release_buffers();
    if (state == UserQueueStatus::pending) return Status::busy;
    // A missing/replaced queue is not a receipt. Only the native removal owner
    // may invoke retire_removed_queue after its acknowledged REMOVE_QUEUE.
    quarantined_ = true;
    return Status::quarantined;
}

memory::Status ProcessSubmissions::query(uint32_t queue, uint32_t sequence) noexcept {
    if (!initialized_) return Status::unavailable;
    if (queue == 0 || sequence == 0 || queue != queue_ || sequence != sequence_)
        return Status::invalid_argument;
    const auto report = config_.queues->queue_report(queue);
    if (report.queue_id != queue || report.pasid != config_.pasid)
        return Status::invalid_argument;
    return poll();
}

memory::Status ProcessSubmissions::submit(uint32_t queue, const ProcessTransfer& transfer,
                                         bool fill, uint32_t& sequence) noexcept {
    sequence = 0;
    const auto previous = poll();
    if (previous != Status::success) return previous;
    const auto report = config_.queues->queue_report(queue);
    if (queue == 0 || report.queue_id != queue || report.pasid != config_.pasid ||
        report.type != hardware::Mes11QueueType::sdma ||
        report.state != hardware::UserQueueState::active) return Status::invalid_argument;
    ProcessBufferInfo source{};
    ProcessBufferInfo destination{};
    auto status = config_.buffers->describe(transfer.destination, destination);
    if (status != Status::success) return status;
    if (!span(destination, transfer.destination_offset, transfer.bytes)) return Status::invalid_argument;
    if (fill) {
        if (transfer.source != 0 || transfer.source_offset != 0) return Status::invalid_argument;
    } else {
        if (transfer.value != 0) return Status::invalid_argument;
        status = config_.buffers->describe(transfer.source, source);
        if (status != Status::success) return status;
        if (!span(source, transfer.source_offset, transfer.bytes)) return Status::invalid_argument;
    }
    const sdma::GpuRange destination_range{{destination.va, destination.bytes},
        transfer.destination_offset};
    const sdma::GpuRange source_range{{source.va, source.bytes}, transfer.source_offset};
    sdma::EncodeResult encoded{};
    const auto result = fill ? sdma::encode_fill({destination_range, transfer.bytes, transfer.value},
        {cpu_, memory::page_bytes}, encoded) :
        sdma::encode_copy({source_range, destination_range, transfer.bytes},
            {cpu_, memory::page_bytes}, encoded);
    if (result != sdma::EncodeStatus::success) return Status::invalid_argument;
    // SDMA IBs are eight-dword aligned. NOP opcode zero owns all padding; the
    // driver never copies user command bytes into executable queue memory.
    const size_t padded = (encoded.bytes_written + 31u) & ~size_t{31};
    if (padded > memory::page_bytes) return Status::invalid_argument;
    const sdma::CommandBuffer output(cpu_, memory::page_bytes);
    for (size_t offset = encoded.bytes_written; offset < padded; offset += 4) {
        if (!output.write_u32(offset, 0)) return Status::backend_failure;
    }
    source_ = fill ? 0 : transfer.source;
    destination_ = transfer.destination;
    const bool shared = !fill && source_ == destination_;
    if (!fill) {
        status = config_.buffers->retain(source_, shared ? memory::Access::read_write : memory::Access::read);
        if (status != Status::success) return status;
        source_held_ = true;
    }
    if (!shared) {
        status = config_.buffers->retain(destination_, memory::Access::write);
        if (status != Status::success) {
            const auto released = release_buffers();
            return released == Status::success ? status : released;
        }
        destination_held_ = true;
    }
    queue_ = queue;
    sequence_ = 0;
    const auto submitted = config_.queues->submit(queue,
        {process_submission_va, static_cast<uint32_t>(padded / 4)}, sequence_);
    if (submitted == UserQueueStatus::submitted && sequence_ != 0) {
        pending_ = true;
        sequence = sequence_;
        return Status::success;
    }
    if (submitted == UserQueueStatus::hung || submitted == UserQueueStatus::quarantined) {
        // WPTR/doorbell failure can occur after publication, with output
        // sequence still zero. Keep every referenced buffer and the private IB.
        pending_ = true;
        quarantined_ = true;
        return Status::quarantined;
    }
    const auto released = release_buffers();
    if (released != Status::success) return released;
    return submitted == UserQueueStatus::ring_full || submitted == UserQueueStatus::busy ||
        submitted == UserQueueStatus::suspended ? Status::busy : Status::unavailable;
}

memory::Status ProcessSubmissions::copy(uint32_t queue, const ProcessTransfer& transfer,
                                       uint32_t& sequence) noexcept {
    return submit(queue, transfer, false, sequence);
}

memory::Status ProcessSubmissions::fill(uint32_t queue, const ProcessTransfer& transfer,
                                       uint32_t& sequence) noexcept {
    return submit(queue, transfer, true, sequence);
}

memory::Status ProcessSubmissions::retire_removed_queue(uint32_t queue) noexcept {
    if (!initialized_) return Status::unavailable;
    if (queue != queue_ || (!pending_ && !quarantined_)) return Status::success;
    if (config_.queues->queue_report(queue).queue_id != 0) return Status::busy;
    const auto released = release_buffers();
    if (released == Status::success) quarantined_ = false;
    return released;
}

memory::Status ProcessSubmissions::close_after_idle() noexcept {
    if (empty()) {
        initialized_ = false;
        config_ = {};
        return Status::success;
    }
    if (config_.queues == nullptr || config_.queues->vm_report(config_.pasid).pasid != 0)
        return Status::busy;
    const auto released = release_buffers();
    if (released != Status::success) return released;
    if (ib_retained_) {
        if (config_.gtt->unretain(ib_) != Status::success) return Status::quarantined;
        ib_retained_ = false;
    }
    if (ib_live_) {
        if (config_.gtt->release(ib_) != Status::success) return Status::quarantined;
        ib_live_ = false;
        ib_ = {};
    }
    cpu_ = nullptr;
    initialized_ = false;
    quarantined_ = false;
    pending_ = false;
    queue_ = 0;
    sequence_ = 0;
    config_ = {};
    return Status::success;
}

} // namespace rx7900::reactos

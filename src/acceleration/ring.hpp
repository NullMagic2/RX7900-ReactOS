/*
Module: rx7900.acceleration.ring
Purpose: Commit bounded SDMA work and retain buffers until actual GPU completion.
Created: 2026-10-21
Architecture: Device-owned queue; native backend provides real ordered HW access.
*/
#pragma once

#include "manager.hpp"

namespace rx7900 {
namespace sdma {

constexpr size_t pending_capacity = 64;
constexpr size_t ring_alignment_bytes = 16 * sizeof(uint32_t);

enum class RingStatus {
    success,
    submitted,
    pending,
    completed,
    invalidated,
    unavailable,
    invalid_argument,
    invalid_memory,
    encoder_failure,
    busy,
    sequence_exhausted,
    hardware_fault,
    timeout,
    stopped
};

struct Ticket final {
    uint64_t generation;
    uint32_t sequence;
};

struct HardwareSnapshot final {
    uint64_t read_pointer_bytes;
    uint64_t write_pointer_bytes;
    uint32_t completed_sequence;
    uint32_t ip_version;
    uint32_t vmid;
    uint32_t instance;
    bool ready;
    bool faulted;
};

enum class PublishResult {
    published,
    not_published,
    uncertain
};

struct RingBackend final {
    void* context;
    // Acquire coherent RPTR/fence reads and latched fault/firmware/VM state.
    // Values come from the bound hardware queue, never software predictions.
    bool (*query)(void*, HardwareSnapshot&) noexcept;
    // Flush CPU ring stores/HDP as required, issue a release barrier, update the
    // WPTR shadow and commit the 64-bit BYTE WPTR doorbell/registers. published
    // means accepted for execution, not completed. not_published guarantees no
    // new DMA access; uncertain preserves every lease until confirmed quiesce.
    PublishResult (*publish)(void*, uint64_t write_pointer_bytes) noexcept;
    // Disable RB/IB, halt the engine and confirm DMA cannot touch allocations.
    // This is not a promise to attempt reset. False must preserve all resources.
    bool (*quiesce)(void*) noexcept;
};

struct RingConfiguration final {
    memory::Handle ring_allocation;
    memory::Handle fence_allocation;
    uint32_t ring_byte_count;
    uint8_t* scratch_bytes;
    size_t scratch_byte_count;
    uint64_t timeout_nanoseconds;
    uint32_t instance;
    bool interrupts;
};

// All calls are externally serialized by the device queue owner at <= APC_LEVEL;
// completion from an ISR/DPC is queued to a worker before entering this class.
// Manager/native context, scratch, ring and fence storage outlive an active ring.
// Scratch must not alias ring/fence/backing bytes or this object. No automatic
// destructor frees DMA resources: stop() must succeed before device teardown.
// Direct packets require previously initialized VMID0 mappings, SDMA6.0.0
// firmware, ring registers, RPTR writeback, visibility and optional trap routing.
class Ring final {
public:
    Ring() noexcept = default;
    Ring(const Ring&) = delete;
    Ring& operator=(const Ring&) = delete;

    // Backend must bind a freshly reset queue with zero RPTR/WPTR/fence and
    // attest actual ready state. Does not initialize firmware or registers.
    RingStatus initialize(memory::Manager& manager, RingConfiguration configuration,
                          RingBackend backend) noexcept;

    RingStatus submit_copy(memory::Slice source, memory::Slice destination,
                           uint64_t byte_count, uint64_t now_nanoseconds,
                           Ticket& ticket) noexcept;
    RingStatus submit_fill(memory::Slice destination, uint64_t byte_count,
                           uint32_t value, uint64_t now_nanoseconds,
                           Ticket& ticket) noexcept;
    // Reclaims leases only after an actual observed completion fence. Trap IRQs
    // are hints to call poll, never evidence that a submitted packet completed.
    RingStatus poll(uint64_t now_nanoseconds) noexcept;
    RingStatus ticket_status(Ticket ticket) const noexcept;
    void notify_fault() noexcept;
    // Also serves recovery. All outstanding tickets become invalidated after
    // confirmed quiescence, then leases are released. A failed stop pins them.
    // Restart requires native queue reinitialization and a new initialize call.
    RingStatus stop() noexcept;

private:
    struct Pending final {
        uint32_t sequence = 0;
        uint64_t deadline = 0;
        memory::Handle source{};
        memory::Handle destination{};
        bool copy = false;
    };

    RingStatus submit(memory::Slice source, memory::Slice destination,
                      uint64_t count, uint32_t value, bool copy, uint64_t now,
                      Ticket& ticket) noexcept;
    bool release_pending(Pending& pending) noexcept;
    bool reserved(memory::Handle handle) const noexcept;

    memory::Manager* manager_ = nullptr;
    RingConfiguration configuration_{};
    RingBackend backend_{};
    memory::AllocationInfo ring_{};
    memory::AllocationInfo fence_{};
    Pending pending_[pending_capacity]{};
    size_t pending_begin_ = 0;
    size_t pending_count_ = 0;
    uint64_t generation_ = 0;
    uint64_t write_pointer_ = 0;
    uint64_t read_pointer_ = 0;
    uint64_t last_time_ = 0;
    uint32_t submitted_sequence_ = 0;
    uint32_t completed_sequence_ = 0;
    bool active_ = false;
    bool faulted_ = false;
};

} // namespace sdma
} // namespace rx7900

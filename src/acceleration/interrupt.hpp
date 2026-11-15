/*
Module: rx7900.acceleration.interrupt
Purpose: Decode bounded IH6 snapshots and route legacy SDMA queue events.
Created: 2026-10-21
Architecture: Native IH ownership/acknowledgement stays outside the decoder.
*/
#pragma once

#include "byte_view.hpp"
#include "ring.hpp"

namespace rx7900 {
namespace sdma {

constexpr size_t interrupt_vector_bytes = 32;

struct InterruptVector final {
    uint32_t client;
    uint32_t source;
    uint32_t ring;
    uint32_t vmid;
    uint32_t pasid;
    uint32_t node;
    uint64_t timestamp;
    uint32_t source_data[4];
    bool vmid_source;
    bool timestamp_source;
};

enum class InterruptAction {
    ignore,
    poll_fence,
    queue_fault,
    malformed
};

struct InterruptRoute final {
    InterruptAction action;
    uint32_t instance;
};

// Input is a caller-owned contiguous, coherent 32-byte IH entry snapshot after
// native WPTR ownership/overflow checks and acquire barrier. No unowned ring bytes
// are read; output is cleared on truncation/overflow. ISR use is allocation-free.
// This never acknowledges an interrupt or advances IH RPTR. Native overflow must
// latch queue failure and require recovery, never silently skip to a fresh RPTR.
bool decode_interrupt(ByteView snapshot, size_t offset, InterruptVector& vector) noexcept;
InterruptRoute classify_interrupt(const InterruptVector& vector) noexcept;

// Worker-only (same serialization/IRQL contract as Ring); never call from ISR/DPC.
// A trap prompts real fence observation through poll. It does not itself complete
// a ticket. Route must match the explicitly owned queue instance.
RingStatus service_interrupt(const InterruptVector& vector, uint32_t queue_instance,
                              Ring& queue, uint64_t now_nanoseconds) noexcept;

} // namespace sdma
} // namespace rx7900

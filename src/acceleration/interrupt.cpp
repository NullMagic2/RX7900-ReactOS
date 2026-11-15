/*
Module: rx7900.acceleration.interrupt
Purpose: Adapt IH6 vector fields and SDMA source/client IDs without struct overlays.
Created: 2026-10-21
Architecture: CPU-side bounded routing; native IH hardware setup is separate.
Copyright 2014, 2018, 2020, 2021 Advanced Micro Devices, Inc.
Adapted from Linux v6.12 amdgpu_ih.c, sdma_v6_0.c, soc15_ih_clientid.h and
irqsrcs_gfx_11_0_0.h; permission notice in licenses/AMDGPU-SDMA.txt.
*/
#include "interrupt.hpp"

namespace rx7900 {
namespace sdma {

bool decode_interrupt(ByteView snapshot, size_t offset, InterruptVector& vector) noexcept {
    vector = {};

    if (!snapshot.contains(offset, interrupt_vector_bytes)) {
        return false;
    }

    uint32_t words[8]{};

    for (size_t index = 0; index < 8; ++index) {
        snapshot.read_u32(offset + index * sizeof(uint32_t), words[index]);
    }

    vector.client = words[0] & 0xff;
    vector.source = (words[0] >> 8) & 0xff;
    vector.ring = (words[0] >> 16) & 0xff;
    vector.vmid = (words[0] >> 24) & 0xf;
    vector.vmid_source = (words[0] >> 31) != 0;
    vector.timestamp = words[1] | (static_cast<uint64_t>(words[2] & 0xffff) << 32);
    vector.timestamp_source = (words[2] >> 31) != 0;
    vector.pasid = words[3] & 0xffff;
    vector.node = (words[3] >> 16) & 0xff;

    for (size_t index = 0; index < 4; ++index) {
        vector.source_data[index] = words[index + 4];
    }

    return true;
}

InterruptRoute classify_interrupt(const InterruptVector& vector) noexcept {
    constexpr uint32_t gfx_client = 0x0a;
    constexpr uint32_t sdma_trap = 49;
    constexpr uint32_t sdma_protected_write = 50;
    constexpr uint32_t first_sdma_error = 54;
    constexpr uint32_t last_sdma_error = 66;

    if (vector.client != gfx_client || vector.vmid != 0 || vector.pasid != 0) {
        return {InterruptAction::ignore, 0};
    }

    const bool trap = vector.source == sdma_trap;
    const bool error = vector.source == sdma_protected_write ||
                       (vector.source >= first_sdma_error && vector.source <= last_sdma_error);

    if (!trap && !error) {
        return {InterruptAction::ignore, 0};
    }

    const uint32_t queue = vector.ring & 0xf;
    const uint32_t instance = vector.ring >> 4;

    if (vector.ring > 0xff || instance > 1) {
        return {InterruptAction::malformed, 0};
    }

    // This encoder emits only queue0/VMID0/context-zero legacy traps. MES or
    // other process events cannot be misattributed to this queue's fence.
    if (queue != 0 || (trap && vector.source_data[0] != 0)) {
        return {InterruptAction::ignore, instance};
    }

    return {trap ? InterruptAction::poll_fence : InterruptAction::queue_fault, instance};
}

RingStatus service_interrupt(const InterruptVector& vector, uint32_t queue_instance,
                              Ring& queue, uint64_t now_nanoseconds) noexcept {
    const auto route = classify_interrupt(vector);

    if (queue_instance > 1 || route.action == InterruptAction::malformed) {
        return RingStatus::invalid_argument;
    }

    if (route.action == InterruptAction::ignore || route.instance != queue_instance) {
        return RingStatus::unavailable;
    }

    if (route.action == InterruptAction::queue_fault) {
        queue.notify_fault();
        return RingStatus::hardware_fault;
    }

    return queue.poll(now_nanoseconds);
}

} // namespace sdma
} // namespace rx7900

/*
Module: rx7900.reactos.dma
Purpose: Own real VideoPort DMA adapters and bounded common-buffer allocations.
Created: 2026-10-21
Architecture: Device-owned native backing; GPUVM installation stays separate.
*/
#pragma once

#include "hardware.hpp"
#include "manager.hpp"

namespace rx7900::reactos {

struct DmaHandle final {
    uint32_t slot = UINT32_MAX;
    uint64_t generation = 0;
};

struct DmaBusAddress final { uint64_t value; };

// Borrowed native CPU/DMA views, never GPU virtual or PCI BAR addresses. The
// owner must retain the handle throughout CPU access and any GPU/firmware use.
struct DmaSpan final {
    uint8_t* cpu_bytes;
    ULONG byte_count;
    DmaBusAddress bus;
};

// Construct once in aligned device-owned storage. Calls are externally serialized
// at PASSIVE_LEVEL. All buffers have one explicit owner: this device pool. Call
// release only after GPUVM unmap/TLB confirmation and hardware retirement; leases
// also protect CPU readers. shutdown refuses live allocations rather than freeing
// DMA-accessible memory. Native removal must quiesce queues before releasing them.
// No destructor guesses that DMA has stopped; the device owner must call shutdown.
class DmaPool final {
public:
    DmaPool() noexcept = default;
    DmaPool(const DmaPool&) = delete;
    DmaPool& operator=(const DmaPool&) = delete;

    NTSTATUS initialize(PVOID extension, const HardwareState& hardware) noexcept;
    // Alignment applies to the DMA bus address. CPU storage is byte-addressed;
    // high DMA alignment does not imply equal CPU virtual-address alignment.
    NTSTATUS allocate(ULONG byte_count, ULONG alignment, DmaHandle& output) noexcept;
    NTSTATUS describe(DmaHandle handle, DmaSpan& output) const noexcept;
    NTSTATUS retain(DmaHandle handle) noexcept;
    NTSTATUS unretain(DmaHandle handle) noexcept;
    NTSTATUS release(DmaHandle handle) noexcept;
    NTSTATUS shutdown() noexcept;

private:
    struct Slot final {
        PVOID allocation = nullptr;
        PHYSICAL_ADDRESS logical{};
        ULONG allocation_bytes = 0;
        DmaSpan span{};
        uint64_t generation = 0;
        ULONG leases = 0;
    };
    Slot* find(DmaHandle handle) noexcept;
    const Slot* find(DmaHandle handle) const noexcept;

    PVOID extension_ = nullptr;
    PVP_DMA_ADAPTER adapter_ = nullptr;
    uint64_t generation_ = 0;
    Slot slots_[memory::allocation_capacity]{};
};

} // namespace rx7900::reactos

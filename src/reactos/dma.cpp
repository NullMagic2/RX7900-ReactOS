/*
Module: rx7900.reactos.dma
Purpose: Allocate, retain and release true DMA common buffers through VideoPort.
Created: 2026-10-21
Architecture: Native DMA bus translation; no physical-address identity assumption.
*/
#include "dma.hpp"

namespace rx7900::reactos {
namespace {

constexpr ULONG maximum_dma_bytes = 32u * 1024u * 1024u;
constexpr ULONG maximum_dma_alignment = 1024u * 1024u;

bool passive() noexcept { return KeGetCurrentIrql() == PASSIVE_LEVEL; }

} // namespace

NTSTATUS DmaPool::initialize(PVOID extension, const HardwareState& hardware) noexcept {
    if (!passive() || extension == nullptr || !hardware.acquired || adapter_ != nullptr) {
        return STATUS_INVALID_DEVICE_STATE;
    }

    VP_DEVICE_DESCRIPTION description{};
    description.ScatterGather = FALSE;
    description.Dma32BitAddresses = FALSE;
    description.Dma64BitAddresses = TRUE;
    description.MaximumLength = maximum_dma_bytes;
    PVP_DMA_ADAPTER adapter = VideoPortGetDmaAdapter(extension, &description);

    if (adapter == nullptr) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    extension_ = extension;
    adapter_ = adapter;
    return STATUS_SUCCESS;
}

const DmaPool::Slot* DmaPool::find(DmaHandle handle) const noexcept {
    if (handle.slot >= memory::allocation_capacity || handle.generation == 0) {
        return nullptr;
    }

    const auto& slot = slots_[handle.slot];
    return slot.allocation != nullptr && slot.generation == handle.generation ? &slot : nullptr;
}

DmaPool::Slot* DmaPool::find(DmaHandle handle) noexcept {
    return const_cast<Slot*>(static_cast<const DmaPool&>(*this).find(handle));
}

NTSTATUS DmaPool::allocate(ULONG byte_count, ULONG alignment, DmaHandle& output) noexcept {
    if (!passive() || adapter_ == nullptr) {
        return STATUS_INVALID_DEVICE_STATE;
    }
    if (byte_count == 0 || alignment < memory::page_bytes ||
        alignment > maximum_dma_alignment || (alignment & (alignment - 1u)) != 0 ||
        byte_count > maximum_dma_bytes - (alignment - 1u) || generation_ == UINT64_MAX) {
        return STATUS_INVALID_PARAMETER;
    }

    for (uint32_t index = 0; index < memory::allocation_capacity; ++index) {
        auto& slot = slots_[index];

        if (slot.allocation != nullptr) {
            continue;
        }

        const ULONG allocation_bytes = byte_count + alignment - 1u;
        PHYSICAL_ADDRESS logical{};
        PVOID allocation = VideoPortAllocateCommonBuffer(extension_, adapter_,
            allocation_bytes, &logical, FALSE, nullptr);

        if (allocation == nullptr) {
            return STATUS_INSUFFICIENT_RESOURCES;
        }

        const uint64_t bus = static_cast<uint64_t>(logical.QuadPart);
        const uint64_t delta = (0u - bus) & (alignment - 1u);

        if (logical.QuadPart < 0 || bus > sdma::gpu_virtual_address_max ||
            allocation_bytes - 1u > sdma::gpu_virtual_address_max - bus ||
            delta > allocation_bytes || byte_count > allocation_bytes - delta) {
            VideoPortReleaseCommonBuffer(extension_, adapter_, allocation_bytes,
                logical, allocation, FALSE);
            return STATUS_CONFLICTING_ADDRESSES;
        }

        // Common-buffer logical DMA addresses are supplied by the OS DMA/IOMMU
        // adapter, not MmGetPhysicalAddress or a truncated kernel pointer. Offset
        // arithmetic stays within the validated contiguous common-buffer extent.
        auto* cpu = static_cast<uint8_t*>(allocation) + static_cast<size_t>(delta);
        RtlZeroMemory(cpu, byte_count);
        slot.allocation = allocation;
        slot.logical = logical;
        slot.allocation_bytes = allocation_bytes;
        slot.span = {cpu, byte_count, {bus + delta}};
        slot.generation = ++generation_;
        slot.leases = 0;
        output = {index, slot.generation};
        return STATUS_SUCCESS;
    }

    return STATUS_INSUFFICIENT_RESOURCES;
}

NTSTATUS DmaPool::describe(DmaHandle handle, DmaSpan& output) const noexcept {
    if (!passive() || adapter_ == nullptr) {
        return STATUS_INVALID_DEVICE_STATE;
    }

    const Slot* slot = find(handle);

    if (slot == nullptr) {
        return STATUS_INVALID_HANDLE;
    }

    output = slot->span;
    return STATUS_SUCCESS;
}

NTSTATUS DmaPool::retain(DmaHandle handle) noexcept {
    if (!passive()) {
        return STATUS_INVALID_DEVICE_STATE;
    }

    Slot* slot = find(handle);

    if (slot == nullptr) {
        return STATUS_INVALID_HANDLE;
    }
    if (slot->leases == MAXULONG) {
        return STATUS_INTEGER_OVERFLOW;
    }

    ++slot->leases;
    return STATUS_SUCCESS;
}

NTSTATUS DmaPool::unretain(DmaHandle handle) noexcept {
    if (!passive()) {
        return STATUS_INVALID_DEVICE_STATE;
    }

    Slot* slot = find(handle);

    if (slot == nullptr || slot->leases == 0) {
        return STATUS_INVALID_HANDLE;
    }

    --slot->leases;
    return STATUS_SUCCESS;
}

NTSTATUS DmaPool::release(DmaHandle handle) noexcept {
    if (!passive() || adapter_ == nullptr) {
        return STATUS_INVALID_DEVICE_STATE;
    }

    Slot* slot = find(handle);

    if (slot == nullptr) {
        return STATUS_INVALID_HANDLE;
    }
    if (slot->leases != 0) {
        return STATUS_DEVICE_BUSY;
    }

    VideoPortReleaseCommonBuffer(extension_, adapter_, slot->allocation_bytes,
        slot->logical, slot->allocation, FALSE);
    *slot = {};
    return STATUS_SUCCESS;
}

NTSTATUS DmaPool::shutdown() noexcept {
    if (!passive()) {
        return STATUS_INVALID_DEVICE_STATE;
    }

    for (const auto& slot : slots_) {
        if (slot.allocation != nullptr) {
            return STATUS_DEVICE_BUSY;
        }
    }

    if (adapter_ != nullptr) {
        VideoPortPutDmaAdapter(extension_, adapter_);
    }

    adapter_ = nullptr;
    extension_ = nullptr;
    return STATUS_SUCCESS;
}

} // namespace rx7900::reactos

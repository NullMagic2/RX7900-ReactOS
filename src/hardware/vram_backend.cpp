/*
Module: rx7900.hardware.vram_backend
Purpose: Carve placement-class arenas and serve Manager create/destroy callbacks.
Created: 2026-10-22
Architecture: No OS or MMIO access; CPU views come only through VramCpuMapper.
*/
#include "vram_backend.hpp"

namespace rx7900::hardware {

VramStatus VramBackend::initialize(VramAllocator& allocator, uint64_t mc_base, VramPool pool,
                                   uint64_t arena_bytes, uint64_t arena_alignment,
                                   VramCpuMapper mapper) noexcept {
    if (initialized_) {
        return VramStatus::busy;
    }

    if (!allocator.initialized()) {
        return VramStatus::uninitialized;
    }

    if (arena_bytes == 0 || (arena_bytes % vram_chunk_bytes) != 0 ||
        mc_base > sdma::gpu_virtual_address_max ||
        allocator.vram_bytes() - 1 > sdma::gpu_virtual_address_max - mc_base ||
        (pool == VramPool::cpu_visible && (mapper.map == nullptr || mapper.unmap == nullptr))) {
        return VramStatus::invalid_argument;
    }

    // amdgpu_bo_create_kernel(..., cpu_addr ? &ptr : NULL) for the whole pool.
    VramAllocationHandle carve{};
    const VramStatus status = allocator.allocate(
        {arena_bytes, arena_alignment, vram_kernel_bo_flags(pool == VramPool::cpu_visible)}, carve);

    if (status != VramStatus::success) {
        return status;
    }

    VramAllocationInfo info{};

    if (allocator.describe(carve, info) != VramStatus::success || !info.contiguous ||
        info.requested_bytes != arena_bytes ||
        (pool == VramPool::cpu_visible && info.visible_bytes != arena_bytes)) {
        allocator.release(carve);
        return VramStatus::no_space;
    }

    for (auto& record : records_) {
        record = Record{};
    }

    allocator_ = &allocator;
    mapper_ = pool == VramPool::cpu_visible ? mapper : VramCpuMapper{};
    carve_ = carve;
    mc_base_ = mc_base;
    arena_offset_ = info.first_byte;
    arena_bytes_ = arena_bytes;
    pool_ = pool;
    initialized_ = true;
    return VramStatus::success;
}

VramStatus VramBackend::shutdown() noexcept {
    if (!initialized_) {
        return VramStatus::uninitialized;
    }

    for (const auto& record : records_) {
        if (record.used) {
            return VramStatus::busy;
        }
    }

    const VramStatus status = allocator_->release(carve_);

    if (status != VramStatus::success) {
        return status;
    }

    allocator_ = nullptr;
    mapper_ = {};
    carve_ = {};
    mc_base_ = 0;
    arena_offset_ = 0;
    arena_bytes_ = 0;
    initialized_ = false;
    return VramStatus::success;
}

memory::CreateResult VramBackend::create(void* context, sdma::GpuAllocation gpu, memory::Access,
                                         bool cpu_visible, memory::Backing& backing) noexcept {
    auto& self = *static_cast<VramBackend*>(context);
    backing = {};
    const uint64_t arena_start = self.mc_base_ + self.arena_offset_;

    if (!self.initialized_ || gpu.byte_length == 0 || gpu.virtual_address < arena_start ||
        gpu.virtual_address - arena_start > self.arena_bytes_ ||
        gpu.byte_length > self.arena_bytes_ - (gpu.virtual_address - arena_start) ||
        (cpu_visible && (self.pool_ != VramPool::cpu_visible || gpu.byte_length > SIZE_MAX))) {
        return memory::CreateResult::no_resources;
    }

    Record* record = nullptr;

    for (auto& candidate : self.records_) {
        if (!candidate.used) {
            record = &candidate;
            break;
        }
    }

    if (record == nullptr) {
        return memory::CreateResult::no_resources;
    }

    uint8_t* cpu = nullptr;
    void* mapping = nullptr;

    if (cpu_visible) {
        // The arena lies wholly in visible VRAM (checked at initialize).
        const uint64_t vram_offset = gpu.virtual_address - self.mc_base_;

        if (!self.mapper_.map(self.mapper_.context, vram_offset, gpu.byte_length, cpu, mapping) ||
            cpu == nullptr || mapping == nullptr) {
            return memory::CreateResult::no_resources;
        }
    }

    *record = {gpu, mapping, true};
    backing = {record, cpu, cpu_visible ? static_cast<size_t>(gpu.byte_length) : 0};
    return memory::CreateResult::created;
}

bool VramBackend::destroy(void* context, const memory::Backing& backing,
                          sdma::GpuAllocation gpu) noexcept {
    auto& self = *static_cast<VramBackend*>(context);

    for (auto& record : self.records_) {
        if (&record != backing.token || !record.used || record.gpu.virtual_address != gpu.virtual_address ||
            record.gpu.byte_length != gpu.byte_length) {
            continue;
        }

        if (record.mapping != nullptr && !self.mapper_.unmap(self.mapper_.context, record.mapping)) {
            return false;
        }

        record = Record{};
        return true;
    }

    return false;
}

} // namespace rx7900::hardware

/*
Module: rx7900.reactos.vram_window
Purpose: Own bounded uncached BAR0 views of CPU-visible VRAM allocations.
Created: 2026-10-22
Architecture: Thin VideoPort adapter over map_vram_temporary; placement policy
stays in src/hardware/vram_allocator.
*/
#include "vram_window.hpp"

namespace rx7900::reactos {

VP_STATUS VramAperture::initialize(HardwareState& hardware, PVOID extension, uint64_t vram_bytes,
                                   uint64_t budget_bytes) noexcept {
    if (hardware_ != nullptr) {
        return ERROR_BUSY;
    }

    if (!hardware.acquired || extension == nullptr || VideoPortGetCurrentIrql() != PASSIVE_LEVEL ||
        hardware.vram_range_bytes == 0 || vram_bytes == 0 || budget_bytes == 0) {
        return ERROR_INVALID_PARAMETER;
    }

    const uint64_t aperture = hardware.vram_range_bytes;
    const uint64_t visible = (aperture < vram_bytes ? aperture : vram_bytes) &
                             ~(memory::page_bytes - 1);

    if (visible == 0) {
        return ERROR_INVALID_PARAMETER;
    }

    for (auto& mapping : mappings_) {
        mapping = Mapping{};
    }

    hardware_ = &hardware;
    extension_ = extension;
    visible_bytes_ = visible;
    budget_bytes_ = budget_bytes;
    mapped_bytes_ = 0;
    return NO_ERROR;
}

VP_STATUS VramAperture::map(uint64_t vram_offset, uint64_t bytes, PUCHAR& cpu, void*& token) noexcept {
    cpu = nullptr;
    token = nullptr;

    if (hardware_ == nullptr || VideoPortGetCurrentIrql() != PASSIVE_LEVEL || bytes == 0 ||
        bytes > MAXULONG || vram_offset >= visible_bytes_ || bytes > visible_bytes_ - vram_offset) {
        return ERROR_INVALID_PARAMETER;
    }

    if (bytes > budget_bytes_ - mapped_bytes_) {
        return ERROR_NOT_ENOUGH_MEMORY;
    }

    Mapping* slot = nullptr;

    for (auto& mapping : mappings_) {
        if (!mapping.used) {
            slot = &mapping;
            break;
        }
    }

    if (slot == nullptr) {
        return ERROR_NOT_ENOUGH_MEMORY;
    }

    // BAR0 offset equals VRAM offset; the helper bounds-checks the assigned BAR0
    // again and requests an uncached (non-P6CACHE) mapping.
    PUCHAR view = nullptr;
    const VP_STATUS status = map_vram_temporary(*hardware_, extension_, vram_offset,
                                                static_cast<ULONG>(bytes), view);

    if (status != NO_ERROR) {
        return status;
    }

    *slot = {view, vram_offset, bytes, true};
    mapped_bytes_ += bytes;
    cpu = view;
    token = slot;
    return NO_ERROR;
}

VP_STATUS VramAperture::map_allocation(const hardware::VramAllocator& allocator,
                                       hardware::VramAllocationHandle handle, PUCHAR& cpu,
                                       void*& token) noexcept {
    cpu = nullptr;
    token = nullptr;
    hardware::VramAllocationInfo info{};

    if (allocator.describe(handle, info) != hardware::VramStatus::success || info.no_cpu_access ||
        !info.contiguous) {
        return ERROR_INVALID_PARAMETER;
    }

    // map refuses the range unless every requested byte is inside visible VRAM.
    return map(info.first_byte, info.requested_bytes, cpu, token);
}

bool VramAperture::unmap(void* token) noexcept {
    if (token == nullptr || VideoPortGetCurrentIrql() != PASSIVE_LEVEL) {
        return false;
    }

    for (auto& mapping : mappings_) {
        if (&mapping == token && mapping.used) {
            unmap_vram_temporary(extension_, mapping.cpu);
            mapped_bytes_ -= mapping.bytes;
            mapping = Mapping{};
            return true;
        }
    }

    return false;
}

void VramAperture::release() noexcept {
    if (hardware_ == nullptr) {
        return;
    }

    for (auto& mapping : mappings_) {
        if (mapping.used) {
            unmap_vram_temporary(extension_, mapping.cpu);
            mapping = Mapping{};
        }
    }

    hardware_ = nullptr;
    extension_ = nullptr;
    visible_bytes_ = 0;
    budget_bytes_ = 0;
    mapped_bytes_ = 0;
}

bool VramAperture::map_callback(void* context, uint64_t vram_offset, uint64_t bytes, uint8_t*& cpu,
                                void*& mapping) noexcept {
    PUCHAR view = nullptr;
    const VP_STATUS status = static_cast<VramAperture*>(context)->map(vram_offset, bytes, view, mapping);
    cpu = view;
    return status == NO_ERROR;
}

bool VramAperture::unmap_callback(void* context, void* mapping) noexcept {
    return static_cast<VramAperture*>(context)->unmap(mapping);
}

} // namespace rx7900::reactos

/*
Module: rx7900.reactos.process_memory
Purpose: Bind session buffer authorization to actual pinned backing and VM ACKs.
Created: 2026-10-22
Architecture: Reuses Manager, UserQueues and GPUVM; never encodes its own PTEs.
*/
#include "process_memory.hpp"

namespace rx7900::reactos {
namespace {

bool valid_access(memory::Access access) noexcept {
    const auto bits = static_cast<uint32_t>(access);
    return bits != 0 && (bits & ~3u) == 0;
}

bool permits(memory::Access granted, memory::Access requested) noexcept {
    return valid_access(requested) &&
        (static_cast<uint32_t>(granted) & static_cast<uint32_t>(requested)) ==
            static_cast<uint32_t>(requested);
}

bool cpu_span(const memory::AllocationInfo& info, uint64_t bytes) noexcept {
    return !info.retiring && info.backing.cpu_bytes != nullptr && bytes <= SIZE_MAX &&
        info.backing.cpu_byte_count >= bytes && info.gpu.byte_length >= bytes;
}

bool uncertain_vm(hardware::GpuVmStatus status) noexcept {
    return status == hardware::GpuVmStatus::quarantined ||
        status == hardware::GpuVmStatus::mmio_failure;
}

memory::Status vm_error(hardware::GpuVmStatus status) noexcept {
    switch (status) {
    case hardware::GpuVmStatus::overlap:
    case hardware::GpuVmStatus::invalid_argument:
    case hardware::GpuVmStatus::invalid_memory:
        return memory::Status::invalid_argument;
    case hardware::GpuVmStatus::out_of_tables:
    case hardware::GpuVmStatus::out_of_mappings:
    case hardware::GpuVmStatus::exhausted:
        return memory::Status::out_of_memory;
    case hardware::GpuVmStatus::busy:
        return memory::Status::busy;
    case hardware::GpuVmStatus::unavailable:
        return memory::Status::unavailable;
    default:
        return memory::Status::quarantined;
    }
}

// The owner has already checked the entire mapped span. Volatile byte access
// also preserves uncached local-VRAM reads/writes at this native boundary.
void zero_bytes(uint8_t* output, size_t bytes) noexcept {
    volatile uint8_t* destination = output;

    for (size_t index = 0; index < bytes; ++index) {
        destination[index] = 0;
    }
}

void copy_bytes(uint8_t* output, const uint8_t* input, size_t bytes) noexcept {
    volatile uint8_t* destination = output;
    const volatile uint8_t* source = input;

    for (size_t index = 0; index < bytes; ++index) {
        destination[index] = source[index];
    }
}

} // namespace

bool ProcessMemory::owns_vram(uint64_t mc_address, uint64_t bytes) const noexcept {
    if (!initialized_ || bytes == 0 || mc_address > UINT64_MAX - bytes) {
        return false;
    }

    for (const auto& slot : slots_) {
        memory::AllocationInfo info{};

        if (slot.handle == 0 || !slot.backing_live || slot.info.domain != ProcessMemoryDomain::vram ||
            manager(slot).describe(slot.backing, info) != memory::Status::success) {
            continue;
        }

        const uint64_t start = info.gpu.virtual_address;
        const uint64_t length = info.gpu.byte_length;

        if (start < mc_address + bytes && mc_address < start + length) {
            return true;
        }
    }

    return false;
}

bool ProcessMemory::empty() const noexcept {
    for (const auto& slot : slots_) {
        if (slot.handle != 0) {
            return false;
        }
    }

    return true;
}

bool ProcessMemory::uncertain() const noexcept {
    for (const auto& slot : slots_) {
        if (slot.handle != 0 && slot.info.quarantined) {
            return true;
        }
    }

    return false;
}

bool ProcessMemory::ready_for_reset_rebind() const noexcept {
    if (!initialized_ || !suspended_ || uncertain()) return false;
    for (const auto& slot : slots_) {
        if (slot.handle != 0 && slot.info.leases != 0) return false;
    }
    return true;
}

memory::Status ProcessMemory::initialize(ProcessMemoryConfig config) noexcept {
    if (initialized_ || !empty()) {
        return memory::Status::busy;
    }
    if (config.queues == nullptr || config.pasid == 0 || config.vram == nullptr ||
        config.gtt == nullptr || config.dma_bus == nullptr ||
        config.queues->vm_report(config.pasid).pasid != config.pasid) {
        return memory::Status::invalid_argument;
    }

    config_ = config;
    initialized_ = true;
    suspended_ = false;
    return memory::Status::success;
}

const ProcessMemory::Slot* ProcessMemory::find(uint64_t handle) const noexcept {
    if (handle != 0) {
        for (const auto& slot : slots_) {
            if (slot.handle != handle) {
                continue;
            }

            // A tombstone never resolves; the first one a request names is
            // recorded so its owner can answer that buffer lost exactly once.
            if (slot.lost) {
                if (lost_touch_ == 0) {
                    lost_touch_ = handle;
                }

                return nullptr;
            }

            return &slot;
        }
    }

    return nullptr;
}

ProcessMemory::Slot* ProcessMemory::find(uint64_t handle) noexcept {
    return const_cast<Slot*>(static_cast<const ProcessMemory&>(*this).find(handle));
}

memory::Manager& ProcessMemory::manager(const Slot& slot) const noexcept {
    return slot.info.domain == ProcessMemoryDomain::system ? *config_.gtt : *config_.vram;
}

bool ProcessMemory::backing(Slot& slot, memory::AllocationInfo& output) noexcept {
    return slot.backing_live && manager(slot).describe(slot.backing, output) == memory::Status::success &&
        cpu_span(output, slot.info.bytes);
}

memory::Status ProcessMemory::allocate(uint64_t bytes, ProcessMemoryDomain domain,
                                      memory::Access access, uint64_t& handle) noexcept {
    handle = 0;

    if (!initialized_) {
        return memory::Status::unavailable;
    }
    if (uncertain()) {
        return memory::Status::quarantined;
    }
    if (suspended_) {
        return memory::Status::busy;
    }
    if (bytes == 0 || bytes > process_memory_max_bytes ||
        (bytes & (memory::page_bytes - 1u)) != 0 || !valid_access(access) ||
        (domain != ProcessMemoryDomain::system && domain != ProcessMemoryDomain::vram)) {
        return memory::Status::invalid_argument;
    }
    if (generation_ == UINT64_MAX) {
        return memory::Status::exhausted;
    }
    const uint64_t charge = domain == ProcessMemoryDomain::vram ? 2u * bytes : bytes;
    if (committed_bytes() > process_memory_session_bytes ||
        charge > process_memory_session_bytes - committed_bytes()) {
        return memory::Status::out_of_memory;
    }

    Slot* selected = nullptr;

    for (auto& slot : slots_) {
        if (slot.handle == 0) {
            selected = &slot;
            break;
        }
    }

    if (selected == nullptr) {
        return memory::Status::out_of_memory;
    }

    auto& slot = *selected;
    slot.handle = ++generation_;
    slot.info.bytes = bytes;
    slot.info.domain = domain;
    slot.info.access = access;
    // Real backing is read/write internally to zero, copy and preserve it. The
    // client grant is independently enforced on copies and process PTE flags.
    auto& owner = manager(slot);
    auto status = owner.allocate(bytes, memory::page_bytes, memory::Access::read_write, true, slot.backing);
    slot.backing_live = status == memory::Status::success;

    if (status == memory::Status::success) {
        status = owner.retain(slot.backing, memory::Access::read_write);
        slot.backing_retained = status == memory::Status::success;
    }

    memory::AllocationInfo info{};

    if (status == memory::Status::success && !backing(slot, info)) {
        status = memory::Status::backend_failure;
    }
    if (status == memory::Status::success) {
        zero_bytes(info.backing.cpu_bytes, static_cast<size_t>(bytes));
    }
    if (status == memory::Status::success && domain == ProcessMemoryDomain::vram) {
        status = config_.gtt->allocate(bytes, memory::page_bytes, memory::Access::read_write,
                                       true, slot.backup);
        slot.backup_live = status == memory::Status::success;

        if (status == memory::Status::success) {
            status = config_.gtt->retain(slot.backup, memory::Access::read_write);
            slot.backup_retained = status == memory::Status::success;
        }
    }
    if (status != memory::Status::success) {
        const auto cleanup = release_slot(slot);
        return cleanup == memory::Status::success ? status : cleanup;
    }

    handle = slot.handle;
    return memory::Status::success;
}

uint64_t ProcessMemory::committed_bytes() const noexcept {
    uint64_t total = 0;

    for (const auto& slot : slots_) {
        total += (slot.backing_live ? slot.info.bytes : 0u) + (slot.backup_live ? slot.info.bytes : 0u);
    }

    return total;
}

memory::Status ProcessMemory::describe(uint64_t handle, ProcessBufferInfo& output) const noexcept {
    output = {};
    const auto* slot = find(handle);

    if (!initialized_ || slot == nullptr) {
        return memory::Status::stale_handle;
    }

    output = slot->info;
    return slot->info.quarantined ? memory::Status::quarantined : memory::Status::success;
}

memory::Status ProcessMemory::map(uint64_t handle, uint64_t va, memory::Access access) noexcept {
    auto* slot = find(handle);

    if (!initialized_ || slot == nullptr) {
        return memory::Status::stale_handle;
    }
    if (uncertain()) {
        return memory::Status::quarantined;
    }
    if (suspended_ || slot->info.leases != 0 || slot->info.mapped) {
        return memory::Status::busy;
    }
    if (!permits(slot->info.access, access)) {
        return memory::Status::access_denied;
    }
    if (va < memory::page_bytes || (va & (memory::page_bytes - 1u)) != 0 ||
        va >= (hardware::gpuvm_max_pfn << 12) ||
        slot->info.bytes > (hardware::gpuvm_max_pfn << 12) - va) {
        return memory::Status::invalid_argument;
    }

    return map_slot(*slot, va, access, false);
}

// The owner validated the grant and the address range (map) or recorded them
// from an earlier successful map (rebind). A rebind failure only reports: the
// caller destroys that new VM, so its partial state is retired with it.
memory::Status ProcessMemory::map_slot(Slot& slot, uint64_t va, memory::Access access,
                                       bool rebinding) noexcept {
    memory::AllocationInfo info{};

    if (!backing(slot, info)) {
        slot.info.quarantined = true;
        return memory::Status::quarantined;
    }

    const hardware::GpuVmMapFlags flags{access, false, hardware::GpuVmMtype::nc, false};
    hardware::GpuVmStatus mapped = hardware::GpuVmStatus::invalid_memory;

    if (slot.info.domain == ProcessMemoryDomain::vram) {
        mapped = config_.queues->map_vram(config_.pasid, va, info.gpu.virtual_address,
                                          slot.info.bytes, flags);
    } else {
        uint64_t bus = 0;

        if (!config_.dma_bus(config_.context, slot.backing, bus) ||
            (bus & (memory::page_bytes - 1u)) != 0 ||
            bus > sdma::gpu_virtual_address_max ||
            slot.info.bytes - 1u > sdma::gpu_virtual_address_max - bus) {
            return memory::Status::backend_failure;
        }

        const size_t count = static_cast<size_t>(slot.info.bytes / memory::page_bytes);

        for (size_t page = 0; page < count; ++page) {
            const uint64_t address = bus + page * memory::page_bytes;

            for (size_t byte = 0; byte < sizeof(address); ++byte) {
                pages_[page * sizeof(address) + byte] = static_cast<uint8_t>(address >> (byte * 8u));
            }
        }

        // Native VideoPort common buffers are allocated uncached (FALSE in
        // VideoPortAllocateCommonBuffer), so their process mapping is not snooped.
        mapped = config_.queues->map_system(config_.pasid, va,
            {pages_, count * sizeof(uint64_t)}, false, flags);
    }

    if (mapped != hardware::GpuVmStatus::mapped) {
        if (!rebinding) {
            slot.info.quarantined = uncertain_vm(mapped);
        }

        return vm_error(mapped);
    }

    // Keep the address reservation and all backing if the invalidation cannot
    // be acknowledged; no unmap/retry may recycle that hardware-visible span.
    slot.info.va = va;
    slot.info.mapped = true;
    slot.info.mapped_access = access;

    if (config_.queues->flush_tlb(config_.pasid, 0) != hardware::GpuVmStatus::flushed) {
        if (!rebinding) {
            slot.info.quarantined = true;
        }

        return memory::Status::quarantined;
    }

    return memory::Status::success;
}

memory::Status ProcessMemory::unmap(uint64_t handle) noexcept {
    auto* slot = find(handle);

    if (!initialized_ || slot == nullptr) {
        return memory::Status::stale_handle;
    }
    if (uncertain()) {
        return memory::Status::quarantined;
    }
    if (suspended_ || slot->info.leases != 0) {
        return memory::Status::busy;
    }
    if (!slot->info.mapped) {
        return memory::Status::invalid_argument;
    }

    const auto unmapped = config_.queues->unmap(config_.pasid, slot->info.va);

    if (unmapped != hardware::GpuVmStatus::unmapped ||
        config_.queues->flush_tlb(config_.pasid, 2) != hardware::GpuVmStatus::flushed) {
        slot->info.quarantined = true;
        return memory::Status::quarantined;
    }

    slot->info.mapped = false;
    slot->info.va = 0;
    return memory::Status::success;
}

memory::Status ProcessMemory::release_slot(Slot& slot) noexcept {
    auto release_backing = [](memory::Manager& owner, memory::Handle allocation,
                               bool& live, bool& retained) noexcept {
        if (!live) {
            return memory::Status::success;
        }

        auto status = owner.release(allocation);

        if (status == memory::Status::deferred && retained) {
            retained = false;
            status = owner.unretain(allocation);
        }
        if (status == memory::Status::success) {
            live = false;
        }

        return status;
    };

    if (release_backing(*config_.gtt, slot.backup, slot.backup_live,
                         slot.backup_retained) != memory::Status::success ||
        release_backing(manager(slot), slot.backing, slot.backing_live,
                         slot.backing_retained) != memory::Status::success) {
        slot.info.quarantined = true;
        return memory::Status::quarantined;
    }

    slot = {};
    return memory::Status::success;
}

memory::Status ProcessMemory::release(uint64_t handle) noexcept {
    auto* slot = find(handle);

    if (!initialized_ || slot == nullptr) {
        return memory::Status::stale_handle;
    }
    if (slot->info.quarantined) {
        return memory::Status::quarantined;
    }
    if (suspended_ || slot->info.leases != 0) {
        return memory::Status::busy;
    }
    if (slot->info.mapped) {
        const auto status = unmap(handle);

        if (status != memory::Status::success) {
            return status;
        }
    }

    return release_slot(*slot);
}

memory::Status ProcessMemory::read(uint64_t handle, uint64_t offset, uint8_t* destination,
                                  uint32_t bytes) noexcept {
    auto* slot = find(handle);

    if (!initialized_ || slot == nullptr) {
        return memory::Status::stale_handle;
    }
    if (uncertain()) {
        return memory::Status::quarantined;
    }
    if (suspended_ || slot->info.leases != 0) {
        return memory::Status::busy;
    }
    if (!permits(slot->info.access, memory::Access::read)) {
        return memory::Status::access_denied;
    }
    if (destination == nullptr || bytes == 0 || bytes > process_memory_copy_bytes ||
        offset > slot->info.bytes || bytes > slot->info.bytes - offset) {
        return memory::Status::invalid_argument;
    }

    memory::AllocationInfo info{};

    if (!backing(*slot, info)) {
        slot->info.quarantined = true;
        return memory::Status::quarantined;
    }

    copy_bytes(destination, info.backing.cpu_bytes + static_cast<size_t>(offset), bytes);
    return memory::Status::success;
}

memory::Status ProcessMemory::write(uint64_t handle, uint64_t offset, ByteView source) noexcept {
    auto* slot = find(handle);

    if (!initialized_ || slot == nullptr) {
        return memory::Status::stale_handle;
    }
    if (uncertain()) {
        return memory::Status::quarantined;
    }
    if (suspended_ || slot->info.leases != 0) {
        return memory::Status::busy;
    }
    if (!permits(slot->info.access, memory::Access::write)) {
        return memory::Status::access_denied;
    }
    if (source.size() == 0 || source.size() > process_memory_copy_bytes ||
        offset > slot->info.bytes || source.size() > slot->info.bytes - offset) {
        return memory::Status::invalid_argument;
    }

    memory::AllocationInfo info{};

    if (!backing(*slot, info)) {
        slot->info.quarantined = true;
        return memory::Status::quarantined;
    }

    volatile uint8_t* destination = info.backing.cpu_bytes + static_cast<size_t>(offset);

    for (size_t index = 0; index < source.size(); ++index) {
        uint8_t value = 0;
        source.read_u8(index, value);
        destination[index] = value;
    }

    return memory::Status::success;
}

memory::Status ProcessMemory::retain(uint64_t handle, memory::Access access) noexcept {
    auto* slot = find(handle);

    if (!initialized_ || slot == nullptr) {
        return memory::Status::stale_handle;
    }
    if (uncertain()) {
        return memory::Status::quarantined;
    }
    if (suspended_ || !slot->info.mapped) {
        return memory::Status::busy;
    }
    if (!permits(slot->info.mapped_access, access)) {
        return memory::Status::access_denied;
    }
    if (slot->info.leases == UINT32_MAX) {
        return memory::Status::exhausted;
    }

    ++slot->info.leases;
    return memory::Status::success;
}

memory::Status ProcessMemory::unretain(uint64_t handle) noexcept {
    auto* slot = find(handle);

    if (!initialized_ || slot == nullptr) {
        return memory::Status::stale_handle;
    }
    if (slot->info.leases == 0) {
        return memory::Status::invalid_argument;
    }

    --slot->info.leases;
    return memory::Status::success;
}

memory::Status ProcessMemory::save_for_recovery() noexcept {
    if (!initialized_ || suspended_) {
        return memory::Status::unavailable;
    }
    if (uncertain()) {
        return memory::Status::quarantined;
    }

    // Validate every owner before copying anything or changing admission.
    for (auto& slot : slots_) {
        if (slot.handle == 0 || slot.lost) {
            continue;
        }
        if (slot.info.leases != 0) {
            return memory::Status::busy;
        }

        memory::AllocationInfo info{};
        memory::AllocationInfo backup{};

        if (!backing(slot, info) || (slot.info.domain == ProcessMemoryDomain::vram &&
            (!slot.backup_live || config_.gtt->describe(slot.backup, backup) != memory::Status::success ||
             !cpu_span(backup, slot.info.bytes)))) {
            slot.info.quarantined = true;
            return memory::Status::quarantined;
        }
    }

    suspended_ = true;

    for (auto& slot : slots_) {
        if (slot.handle == 0 || slot.lost || slot.info.domain != ProcessMemoryDomain::vram) {
            continue;
        }

        memory::AllocationInfo info{};
        memory::AllocationInfo backup{};
        backing(slot, info);
        config_.gtt->describe(slot.backup, backup);
        copy_bytes(backup.backing.cpu_bytes, info.backing.cpu_bytes,
                    static_cast<size_t>(slot.info.bytes));
    }

    saved_ = true;
    return memory::Status::success;
}

memory::Status ProcessMemory::restore_after_recovery() noexcept {
    if (!initialized_ || !suspended_) {
        return memory::Status::unavailable;
    }

    for (auto& slot : slots_) {
        if (slot.handle == 0 || slot.lost || slot.info.domain != ProcessMemoryDomain::vram) {
            continue;
        }

        memory::AllocationInfo info{};
        memory::AllocationInfo backup{};

        if (!backing(slot, info) || config_.gtt->describe(slot.backup, backup) != memory::Status::success ||
            !cpu_span(backup, slot.info.bytes)) {
            slot.info.quarantined = true;
            return memory::Status::quarantined;
        }

        copy_bytes(info.backing.cpu_bytes, backup.backing.cpu_bytes,
                    static_cast<size_t>(slot.info.bytes));
    }

    suspended_ = false;
    saved_ = false;
    return memory::Status::success;
}

memory::Status ProcessMemory::retire_after_reset() noexcept {
    if (!initialized_) {
        return empty() ? memory::Status::success : memory::Status::unavailable;
    }

    // Every command owner returned its leases first (the reset proof retired
    // its work); a lease here would be a buffer still named by pending work.
    for (const auto& slot : slots_) {
        if (slot.handle != 0 && slot.info.leases != 0) {
            return memory::Status::busy;
        }
    }

    auto result = memory::Status::success;

    for (auto& slot : slots_) {
        if (slot.handle == 0 || slot.lost) {
            continue;
        }

        // The old VM is gone; the record survives for the rebind.
        if (slot.info.mapped) {
            slot.reset_mapped = true;
            slot.reset_va = slot.info.va;
            slot.reset_access = slot.info.mapped_access;
            slot.info.mapped = false;
            slot.info.va = 0;
        }

        memory::AllocationInfo info{};
        const bool described = backing(slot, info);
        const bool vram_lost = slot.info.domain == ProcessMemoryDomain::vram && !saved_;

        if (described && !vram_lost) {
            // The reset proof resolved every VM-side uncertainty of this slot.
            slot.info.quarantined = false;
            continue;
        }

        const uint64_t handle = slot.handle;

        if (release_slot(slot) != memory::Status::success) {
            result = memory::Status::quarantined;
            continue;
        }

        // Tombstone: no allocation, the handle stays reserved until answered.
        slot.handle = handle;
        slot.lost = true;
    }

    // Every VM of the old incarnation is gone: no PASID proof refers to it.
    config_.pasid = 0;
    suspended_ = true;
    return result;
}

memory::Status ProcessMemory::rebind_after_reset(ProcessMemoryConfig config) noexcept {
    if (!initialized_ || !suspended_ || uncertain() || config.queues == nullptr ||
        config.pasid == 0 || config.vram != config_.vram || config.gtt != config_.gtt ||
        config.context != config_.context || config.dma_bus != config_.dma_bus ||
        config.queues->vm_report(config.pasid).pasid != config.pasid) {
        return memory::Status::unavailable;
    }
    for (const auto& slot : slots_) {
        if (slot.handle != 0 && slot.info.leases != 0) return memory::Status::busy;
    }
    for (auto& slot : slots_) {
        if (slot.handle == 0 || slot.lost) continue;
        memory::AllocationInfo info{};
        memory::AllocationInfo backup{};
        if (!backing(slot, info) || (saved_ && slot.info.domain == ProcessMemoryDomain::vram &&
            (!slot.backup_live || config_.gtt->describe(slot.backup, backup) != memory::Status::success ||
             !cpu_span(backup, slot.info.bytes)))) {
            slot.info.quarantined = true;
            return memory::Status::quarantined;
        }
    }

    config_ = config;
    for (auto& slot : slots_) {
        if (slot.handle == 0 || slot.lost) continue;
        // A caller that skipped retire_after_reset (suspend-saved state only)
        // still has its live records here; take them over the same way.
        if (slot.info.mapped && !slot.reset_mapped) {
            slot.reset_mapped = true;
            slot.reset_va = slot.info.va;
            slot.reset_access = slot.info.mapped_access;
            slot.info.mapped = false;
            slot.info.va = 0;
        }
    }
    for (auto& slot : slots_) {
        if (slot.handle == 0 || slot.lost || !slot.reset_mapped || slot.info.mapped) continue;
        const auto status = map_slot(slot, slot.reset_va, slot.reset_access, true);
        if (status != memory::Status::success) return status;
    }
    for (auto& slot : slots_) {
        slot.reset_mapped = false;
        slot.reset_va = 0;
    }
    if (saved_) return restore_after_recovery();
    suspended_ = false;
    return memory::Status::success;
}

void ProcessMemory::abandon_rebind() noexcept {
    config_.pasid = 0;
    for (auto& slot : slots_) {
        if (slot.handle != 0 && !slot.lost && slot.reset_mapped) {
            slot.info.mapped = false;
            slot.info.va = 0;
        }
    }
}

uint64_t ProcessMemory::take_lost_touch() noexcept {
    const uint64_t handle = lost_touch_;
    lost_touch_ = 0;
    return handle;
}

bool ProcessMemory::retire_lost(uint64_t handle) noexcept {
    for (auto& slot : slots_) {
        if (handle != 0 && slot.handle == handle && slot.lost) {
            slot = {};
            return true;
        }
    }

    return false;
}

uint32_t ProcessMemory::lost_buffers() const noexcept {
    uint32_t count = 0;

    for (const auto& slot : slots_) {
        count += slot.handle != 0 && slot.lost ? 1u : 0u;
    }

    return count;
}

memory::Status ProcessMemory::unmap_all() noexcept {
    if (!initialized_) {
        return empty() ? memory::Status::success : memory::Status::unavailable;
    }
    if (uncertain()) {
        return memory::Status::quarantined;
    }

    for (const auto& slot : slots_) {
        if (slot.handle != 0 && slot.info.leases != 0) {
            return memory::Status::busy;
        }
    }

    uint32_t unmapped = 0;
    static_assert(process_memory_capacity <= 32, "unmap mask holds every slot");

    for (uint32_t index = 0; index < process_memory_capacity; ++index) {
        auto& slot = slots_[index];

        if (slot.handle == 0 || slot.lost || !slot.info.mapped) {
            continue;
        }

        if (config_.queues->unmap(config_.pasid, slot.info.va) != hardware::GpuVmStatus::unmapped) {
            slot.info.quarantined = true;
            return memory::Status::quarantined;
        }

        slot.info.mapped = false;
        slot.info.va = 0;
        unmapped |= 1u << index;
    }

    // amdgpu_vm_bo_unmap + amdgpu_vm_clear_freed, then the heavyweight flush
    // (KFD type 2): no backing returns before the invalidation is acknowledged.
    if (unmapped != 0 &&
        config_.queues->flush_tlb(config_.pasid, 2) != hardware::GpuVmStatus::flushed) {
        for (uint32_t index = 0; index < process_memory_capacity; ++index) {
            if ((unmapped & (1u << index)) != 0) {
                slots_[index].info.quarantined = true;
            }
        }

        return memory::Status::quarantined;
    }

    return memory::Status::success;
}

memory::Status ProcessMemory::close_after_idle() noexcept {
    if (!initialized_) {
        return empty() ? memory::Status::success : memory::Status::unavailable;
    }
    // Do not trust a caller's claim that only the latest fence was signalled.
    // A vanished process VM is an independently observable retirement proof.
    if (config_.queues->vm_report(config_.pasid).pasid != 0) {
        return memory::Status::busy;
    }

    auto result = memory::Status::success;

    for (auto& slot : slots_) {
        if (slot.handle != 0 && release_slot(slot) != memory::Status::success) {
            result = memory::Status::quarantined;
        }
    }

    if (result == memory::Status::success) {
        config_ = {};
        initialized_ = false;
        suspended_ = false;
    }

    return result;
}

} // namespace rx7900::reactos

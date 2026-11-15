/*
Module: rx7900.hardware.gpuvm
Purpose: Encode and maintain GC 11 process page tables, PASIDs and PASID-based TLB invalidation.
Created: 2026-10-22
Architecture: Port of the amdgpu_vm_ptes_update cursor walk over fixed table
slots; checked CommandBuffer stores into the retained pool, Nbio HDP flushes,
Gmc11 for every hub access and RegisterIo reads of the IH_VMID LUT.
Copyright 2008 Advanced Micro Devices, Inc., Copyright 2008 Red Hat Inc.,
Copyright 2009 Jerome Glisse; Copyright 2021 Advanced Micro Devices, Inc.
Adapted Linux v6.12 amdgpu_vm.c, amdgpu_vm_pt.c, amdgpu_vm_cpu.c, amdgpu_vm.h,
amdgpu_ids.c, amdgpu_ttm.c, amdgpu_gmc.c/.h, gmc_v11_0.c, amdgpu_mes.c and
osssys_6_0_0_offset.h. Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#include "gpuvm.hpp"

namespace rx7900::hardware {
namespace {

// amdgpu_vm.h PTE/PDE bits (pre-GFX12).
constexpr uint64_t pte_valid = 1ULL << 0;
constexpr uint64_t pte_system = 1ULL << 1;
constexpr uint64_t pte_snooped = 1ULL << 2;
constexpr uint64_t pte_executable = 1ULL << 4;
constexpr uint64_t pte_readable = 1ULL << 5;
constexpr uint64_t pte_writeable = 1ULL << 6;
constexpr uint32_t pte_frag_shift = 7;
constexpr uint64_t pte_frag_mask = 0x1fULL;
constexpr uint64_t pte_prt = 1ULL << 51;
constexpr uint64_t pde_pte = 1ULL << 54;
constexpr uint64_t pte_tf = 1ULL << 56;
constexpr uint32_t pte_mtype_shift = 48;
constexpr uint64_t pte_mtype_mask = 7ULL << pte_mtype_shift;
// AMDGPU_VM_NORETRY_FLAGS and gmc_v11_0_early_init noretry_flags (NORETRY_FLAGS_TF).
constexpr uint64_t noretry_flags = pte_executable | pde_pte | pte_tf;
constexpr uint64_t noretry_flags_tf = pte_valid | pte_system | pte_prt;
// amdgpu_gmc_set_pte_pde address mask and the gmc_v11_0_get_vm_pde BUG_ON mask.
constexpr uint64_t pte_address_mask = 0x0000fffffffff000ULL;
constexpr uint64_t pde_address_reject = 0xffff00000000003fULL;
// amdgpu_vm_pte_fragment max_frag for asic_type >= CHIP_VEGA10.
constexpr uint32_t max_fragment = 31;
constexpr uint32_t ptb_level = static_cast<uint32_t>(GpuVmLevel::ptb);
// osssys_6_0_0_offset.h regIH_VMID_0_LUT and regIH_VMID_0_LUT_MM (BASE_IDX 0).
constexpr uint32_t ih_vmid_0_lut = 0x0000;
constexpr uint32_t ih_vmid_0_lut_mm = 0x0010;
constexpr uint32_t ih_vmid_lut_pasid_mask = 0xffffu;
constexpr uint32_t max_pasid = (1u << gpuvm_pasid_bits) - 1;

// amdgpu_vm_pt_level_shift with block_size 9; ~0 for the level above the root.
uint32_t level_shift(uint32_t level) noexcept {
    switch (level) {
    case 0: return 27;
    case 1: return 18;
    case 2: return 9;
    case 3: return 0;
    default: return ~0u;
    }
}

// amdgpu_vm_pt_entries_mask: the root keeps every bit, directories 9 bits.
uint32_t entries_mask(uint32_t level) noexcept {
    return level == 0 ? 0xffffffffu : gpuvm_table_entries - 1;
}

// ffs() on the int Linux passes (so only the low 32 pfn bits count) minus one,
// and fls64() minus one, both wrapping to ~0 for zero as the kernel does.
uint32_t ffs32_minus_one(uint64_t value) noexcept {
    const uint32_t low = static_cast<uint32_t>(value);

    for (uint32_t bit = 0; bit < 32; ++bit) {
        if (((low >> bit) & 1u) != 0) {
            return bit;
        }
    }

    return ~0u;
}

uint32_t fls64_minus_one(uint64_t value) noexcept {
    for (uint32_t bit = 64; bit > 0; --bit) {
        if (((value >> (bit - 1)) & 1u) != 0) {
            return bit - 1;
        }
    }

    return ~0u;
}

uint32_t trailing_zeros(uint64_t value) noexcept {
    for (uint32_t bit = 0; bit < 64; ++bit) {
        if (((value >> bit) & 1u) != 0) {
            return bit;
        }
    }

    return 64;
}

bool page_at(ByteView pages, uint64_t index, uint64_t& address) noexcept {
    uint32_t low = 0;
    uint32_t high = 0;

    if (index > (SIZE_MAX - 4) / 8 || !pages.read_u32(static_cast<size_t>(index * 8), low) ||
        !pages.read_u32(static_cast<size_t>(index * 8 + 4), high)) {
        return false;
    }

    address = (static_cast<uint64_t>(high) << 32) | low;
    return true;
}

// amdgpu_vm_pte_fragment; max_frag is 31 except where VA/PA congruence limits it.
void fragment(bool pages_addr, uint64_t start, uint64_t end, uint32_t max_frag, uint32_t& frag,
              uint64_t& frag_end) noexcept {
    if (pages_addr) {
        frag = 0;
        frag_end = end;
        return;
    }

    const uint32_t a = ffs32_minus_one(start);
    const uint32_t b = fls64_minus_one(end - start);
    frag = a < b ? a : b;

    if (frag >= max_frag) {
        frag = max_frag;
        frag_end = end & ~((1ULL << max_frag) - 1);
    } else {
        frag_end = start + (1ULL << frag);
    }
}

// Fragment limit for a valid contiguous run starting at pfn with destination dst:
// every fragment of 2^f pages needs dst_pfn congruent to pfn modulo 2^f.
uint32_t congruent_fragment(uint64_t pfn, uint64_t dst) noexcept {
    const uint32_t zeros = trailing_zeros((dst >> 12) - pfn);
    return zeros < max_fragment ? zeros : max_fragment;
}

bool access_valid(memory::Access access) noexcept {
    const uint32_t bits = static_cast<uint32_t>(access);
    return bits != 0 && (bits & ~3u) == 0;
}

// gmc_v11_0_get_vm_pte on the mapping flags: EXECUTABLE and MTYPE from the
// mapping, UC for uncached buffers; READABLE/WRITEABLE filtered by the mapping.
uint64_t mapping_flags(uint64_t base, const GpuVmMapFlags& flags) noexcept {
    const uint32_t access = static_cast<uint32_t>(flags.access);
    uint64_t result = base & ~(pte_readable | pte_writeable | pte_executable | pte_mtype_mask);
    result |= (access & 1u) != 0 ? pte_readable : 0;
    result |= (access & 2u) != 0 ? pte_writeable : 0;
    result |= flags.executable ? pte_executable : 0;
    const uint64_t mtype = flags.uncached ? static_cast<uint64_t>(GpuVmMtype::uc) :
                                            static_cast<uint64_t>(flags.mtype) & 7u;
    return result | (mtype << pte_mtype_shift);
}

GpuVmStatus from_gmc(GmcStatus status) noexcept {
    switch (status) {
    case GmcStatus::flushed: return GpuVmStatus::flushed;
    case GmcStatus::invalid_argument: return GpuVmStatus::invalid_argument;
    case GmcStatus::mmio_failure: return GpuVmStatus::mmio_failure;
    case GmcStatus::unavailable: return GpuVmStatus::unavailable;
    default: return GpuVmStatus::quarantined;
    }
}

} // namespace

GpuVmStatus PasidAllocator::allocate(uint32_t bits, uint32_t& pasid) noexcept {
    if (bits == 0 || bits > gpuvm_pasid_bits) {
        return GpuVmStatus::invalid_argument;
    }

    for (uint32_t width = bits; width > 0; --width) {
        const uint32_t first = 1u << (width - 1);
        const uint32_t last = (1u << width) - 1;

        // ida_alloc_range: the lowest free id; full words are skipped whole.
        for (uint32_t id = first; id <= last;) {
            uint64_t& word = used_[id / 64];

            if (id % 64 == 0 && word == ~0ULL && last - id >= 63) {
                id += 64;
                continue;
            }

            const uint64_t bit = 1ULL << (id % 64);

            if ((word & bit) == 0) {
                word |= bit;
                pasid = id;
                return GpuVmStatus::ready;
            }

            ++id;
        }
    }

    return GpuVmStatus::exhausted;
}

bool PasidAllocator::allocated(uint32_t pasid) const noexcept {
    return pasid != 0 && pasid <= max_pasid && (used_[pasid / 64] & (1ULL << (pasid % 64))) != 0;
}

GpuVmStatus PasidAllocator::release(uint32_t pasid) noexcept {
    if (!allocated(pasid)) {
        return GpuVmStatus::invalid_argument;
    }

    used_[pasid / 64] &= ~(1ULL << (pasid % 64));
    return GpuVmStatus::released;
}

bool gpuvm_pasid_mapping_register(const Ih6Registers& osssys, VmHub hub, uint32_t vmid,
                                  uint32_t& reg) noexcept {
    if (!osssys.valid() || vmid > 15) {
        return false;
    }

    reg = osssys.segment0_dword_base() + (hub == VmHub::gfxhub ? ih_vmid_0_lut : ih_vmid_0_lut_mm) +
          vmid;
    return true;
}

GpuVmStatus GpuVmInvalidator::initialize(Gmc11& gmc, Ih6Registers osssys, RegisterIo io,
                                         GmcFirmwareInvalidation firmware,
                                         bool gfxoff_allowed) noexcept {
    initialized_ = false;

    if (!osssys.valid() || !register_io_valid(io) || io.aperture_bytes != osssys.aperture_bytes()) {
        return GpuVmStatus::invalid_argument;
    }

    gmc_ = &gmc;
    osssys_ = osssys;
    io_ = io;
    firmware_ = firmware;
    gfxoff_allowed_ = gfxoff_allowed;
    initialized_ = true;
    return GpuVmStatus::ready;
}

GpuVmStatus GpuVmInvalidator::flush_hub(VmHub hub, uint32_t vmid, uint32_t flush_type) noexcept {
    // gmc_v11_0_flush_gpu_tlb: MES once it is ready, else the register path.
    if (firmware_.submit != nullptr) {
        return from_gmc(gmc_->flush_tlb_firmware(hub, vmid, flush_type, firmware_));
    }

    // RREG32_RLC_NO_KIQ on a powered-down GC is not safe; Linux never takes the
    // register path once GFXOFF can engage because MES is running by then.
    if (hub == VmHub::gfxhub && gfxoff_allowed_) {
        return GpuVmStatus::unavailable;
    }

    return from_gmc(gmc_->flush_tlb(hub, vmid, flush_type));
}

GpuVmStatus GpuVmInvalidator::flush_pasid(uint32_t pasid, uint32_t flush_type, bool all_hub,
                                          uint32_t& flushed_vmids) noexcept {
    flushed_vmids = 0;

    if (!initialized_) {
        return GpuVmStatus::unavailable;
    }

    if (pasid == 0 || pasid > max_pasid || flush_type > gmc_max_flush_type) {
        return GpuVmStatus::invalid_argument;
    }

    for (uint32_t vmid = 1; vmid < 16; ++vmid) {
        uint32_t value = 0;

        // gmc_v11_0_get_vmid_pasid_mapping_info.
        if (!read_ip_register(io_, osssys_.segment0_dword_base(), ih_vmid_0_lut + vmid, value) ||
            value == UINT32_MAX) {
            return GpuVmStatus::mmio_failure;
        }

        const uint32_t queried = value & ih_vmid_lut_pasid_mask;

        if (queried == 0 || queried != pasid) {
            continue;
        }

        // for_each_set_bit(vmhubs_mask): AMDGPU_GFXHUB(0) = 0 before AMDGPU_MMHUB0(0) = 8.
        GpuVmStatus status = flush_hub(VmHub::gfxhub, vmid, flush_type);

        if (status == GpuVmStatus::flushed && all_hub) {
            status = flush_hub(VmHub::mmhub, vmid, flush_type);
        }

        if (status != GpuVmStatus::flushed) {
            return status;
        }

        flushed_vmids |= 1u << vmid;
    }

    return GpuVmStatus::flushed;
}

GpuVmStatus GpuVm::finish(GpuVmStatus status) noexcept {
    report_.status = status;
    return status;
}

uint64_t GpuVm::physical(uint64_t mc) const noexcept {
    // amdgpu_gmc_vram_mc2pa: mc - vram_start + vram_base_offset (MMHUB FB_OFFSET).
    return mc - vram_.mc_start + vram_.physical_offset;
}

uint64_t GpuVm::table_mc(uint16_t table) const noexcept {
    return pool_mc_ + static_cast<uint64_t>(table) * gpuvm_table_bytes;
}

uint16_t GpuVm::find_child(uint16_t parent, uint32_t index) const noexcept {
    for (uint32_t slot = 1; slot < table_count_; ++slot) {
        const Table& table = tables_[slot];

        if ((table.state == TableState::live || table.state == TableState::freeing) &&
            table.parent == parent && table.parent_entry == index) {
            return static_cast<uint16_t>(slot);
        }
    }

    return no_table;
}

uint16_t GpuVm::entry_table(const Cursor& cursor) const noexcept {
    return cursor.parent == no_table ? 0 : find_child(cursor.parent, cursor.index);
}

// amdgpu_vm_pt_descendant.
bool GpuVm::descendant(Cursor& cursor) const noexcept {
    const uint16_t table = entry_table(cursor);

    if (cursor.level == ptb_level || table == no_table) {
        return false;
    }

    const uint64_t index = (cursor.pfn >> level_shift(cursor.level)) & entries_mask(cursor.level);

    if (index >= gpuvm_table_entries) {
        return false;
    }

    ++cursor.level;
    cursor.parent = table;
    cursor.index = static_cast<uint32_t>(index);
    return true;
}

// amdgpu_vm_pt_sibling: every level has 512 entries on GC 11.
bool GpuVm::sibling(Cursor& cursor) const noexcept {
    if (cursor.parent == no_table || cursor.index == gpuvm_table_entries - 1) {
        return false;
    }

    const uint32_t shift = level_shift(cursor.level - 1);
    cursor.pfn += 1ULL << shift;
    cursor.pfn &= ~((1ULL << shift) - 1);
    ++cursor.index;
    return true;
}

// amdgpu_vm_pt_ancestor.
bool GpuVm::ancestor(Cursor& cursor) const noexcept {
    if (cursor.parent == no_table) {
        return false;
    }

    const Table& parent = tables_[cursor.parent];
    --cursor.level;
    cursor.index = parent.parent_entry;
    cursor.parent = cursor.parent == 0 ? no_table : parent.parent;
    return true;
}

// amdgpu_vm_pt_next.
void GpuVm::next(Cursor& cursor) const noexcept {
    if (descendant(cursor)) {
        return;
    }

    while (!sibling(cursor)) {
        if (!ancestor(cursor)) {
            cursor.pfn = ~0ULL;
            return;
        }
    }
}

bool GpuVm::write_entry(uint16_t table, uint32_t index, uint64_t value) noexcept {
    if (table >= table_count_ || index >= gpuvm_table_entries) {
        return false;
    }

    if (dry_run_) {
        return true;
    }

    // The pool holds table_count_ tables; table and index were bounded above.
    const sdma::CommandBuffer pool(pool_cpu_, static_cast<size_t>(table_count_) * gpuvm_table_bytes);
    const size_t offset = static_cast<size_t>(table) * gpuvm_table_bytes + index * 8u;
    return pool.write_u32(offset, static_cast<uint32_t>(value)) &&
           pool.write_u32(offset + 4, static_cast<uint32_t>(value >> 32));
}

// amdgpu_vm_pt_alloc + amdgpu_vm_pt_create + amdgpu_vm_pt_clear: PD entries
// AMDGPU_PDE_PTE (get_vm_pde leaves value 0), PTB entries AMDGPU_PTE_EXECUTABLE
// (GMC9 fault-priority workaround). The new table becomes relocated.
bool GpuVm::pt_alloc(const Cursor& cursor) noexcept {
    if (entry_table(cursor) != no_table) {
        return true;
    }

    for (uint32_t slot = 1; slot < table_count_; ++slot) {
        Table& table = tables_[slot];

        if (table.state != TableState::free) {
            continue;
        }

        table = {cursor.parent, static_cast<uint16_t>(cursor.index), static_cast<uint8_t>(cursor.level),
                 TableState::live, true, ++order_};
        const uint64_t value = cursor.level == ptb_level ? pte_executable : pde_pte;

        for (uint32_t index = 0; index < gpuvm_table_entries; ++index) {
            if (!write_entry(static_cast<uint16_t>(slot), index, value)) {
                return false;
            }
        }

        return true;
    }

    return false;
}

// amdgpu_vm_pt_add_list: the entry's table and its whole subtree move to the
// TLB-flush wait list (no longer relocated); they stay walkable until the update ends.
void GpuVm::add_list(const Cursor& cursor) noexcept {
    const uint16_t start = entry_table(cursor);

    if (start == no_table) {
        return;
    }

    for (uint32_t slot = 1; slot < table_count_; ++slot) {
        uint16_t walk = static_cast<uint16_t>(slot);

        for (uint32_t depth = 0; depth < gpuvm_levels && walk != no_table && walk != 0; ++depth) {
            if (walk == start) {
                Table& table = tables_[slot];

                if (table.state == TableState::live || table.state == TableState::freeing) {
                    table.state = TableState::freeing;
                    table.relocated = false;
                }

                break;
            }

            walk = tables_[walk].parent;
        }
    }
}

// amdgpu_vm_pte_update_flags + amdgpu_vm_cpu_update + amdgpu_gmc_set_pte_pde.
bool GpuVm::update(Update& params, uint16_t table, uint32_t level, uint32_t index, uint64_t addr,
                   uint32_t count, uint64_t increment, uint64_t flags) noexcept {
    if (level != ptb_level) {
        // AMDGPU_PDE_PTE_FLAG; gmc_v11_0_get_vm_pde then keeps addr and flags.
        flags |= pde_pte;

        if ((addr & pde_address_reject) != 0) {
            return false;
        }
    } else if ((flags & pte_valid) == 0 && (flags & pte_prt) == 0) {
        flags |= pte_executable;
    }

    // amdgpu_vm_pte_update_noretry_flags at the PTB level.
    if (level == ptb_level && (flags & noretry_flags) == noretry_flags) {
        flags = (flags & ~noretry_flags) | noretry_flags_tf;
    }

    if (index >= gpuvm_table_entries || count > gpuvm_table_entries - index) {
        return false;
    }

    for (uint32_t entry = 0; entry < count; ++entry) {
        uint64_t value = addr;

        // amdgpu_vm_map_gart: the DMA address of page addr >> PAGE_SHIFT.
        if (params.pages.size() != 0 && !page_at(params.pages, addr >> 12, value)) {
            return false;
        }

        if (!write_entry(table, index + entry, (value & pte_address_mask) | flags)) {
            return false;
        }

        addr += increment;
    }

    if (!dry_run_) {
        report_.entries_written += count;
    }

    return true;
}

// amdgpu_vm_ptes_update (locked, not immediate-unlocked) over pfn [start, end).
bool GpuVm::ptes_update(Update& params, uint64_t start, uint64_t end, uint64_t dst, uint64_t flags,
                        uint32_t max_frag) noexcept {
    const bool pages_addr = params.pages.size() != 0;
    uint64_t frag_start = start;
    uint64_t frag_end = 0;
    uint32_t frag = 0;
    fragment(pages_addr, frag_start, end, max_frag, frag, frag_end);
    Cursor cursor{start, 0, no_table, 0};

    while (cursor.pfn < end) {
        if (!pt_alloc(cursor)) {
            return false;
        }

        uint32_t shift = level_shift(cursor.level);
        const uint32_t parent_shift = level_shift(cursor.level - 1);

        if (frag < shift) {
            if (descendant(cursor)) {
                continue;
            }
        } else if (frag >= parent_shift) {
            if (!ancestor(cursor)) {
                return false;
            }

            continue;
        }

        uint16_t pt = entry_table(cursor);

        if (pt == no_table) {
            if ((flags & pte_valid) != 0 || !ancestor(cursor)) {
                return false;
            }

            pt = entry_table(cursor);
            shift = parent_shift;
            const uint64_t aligned = (frag_start + 1 + (1ULL << shift) - 1) & ~((1ULL << shift) - 1);
            frag_end = frag_end > aligned ? frag_end : aligned;
        }

        const uint64_t increment = gpuvm_page_bytes << shift;
        const uint32_t mask = entries_mask(cursor.level);
        uint64_t index = (cursor.pfn >> shift) & mask;
        uint64_t entry_end = (static_cast<uint64_t>(mask) + 1) << shift;
        entry_end += cursor.pfn & ~(entry_end - 1);
        entry_end = entry_end < end ? entry_end : end;

        do {
            const uint64_t upd_end = entry_end < frag_end ? entry_end : frag_end;
            uint64_t nptes = upd_end >= frag_start ? (upd_end - frag_start) >> shift : UINT64_MAX;
            nptes = nptes == 0 ? 1 : nptes;
            const uint64_t upd_flags = flags | ((frag & pte_frag_mask) << pte_frag_shift);

            if (index >= gpuvm_table_entries || nptes > gpuvm_table_entries - index ||
                !update(params, pt, cursor.level, static_cast<uint32_t>(index), dst,
                        static_cast<uint32_t>(nptes), increment, upd_flags)) {
                return false;
            }

            index += nptes;
            dst += nptes * increment;
            frag_start = upd_end;

            if (frag_start >= frag_end) {
                fragment(pages_addr, frag_start, end, max_frag, frag, frag_end);

                if (frag < shift) {
                    break;
                }
            }
        } while (frag_start < entry_end);

        if (descendant(cursor)) {
            // Free every child table now covered by the entries just written.
            while (cursor.pfn < frag_start) {
                if (entry_table(cursor) != no_table) {
                    params.needs_flush = true;
                    add_list(cursor);
                }

                next(cursor);
            }
        } else if (frag >= shift) {
            next(cursor);
        }
    }

    return true;
}

// amdgpu_vm_update_range: contiguity runs over dma pages, or one VRAM/clear run.
bool GpuVm::update_range(Update& params, uint64_t start, uint64_t pages, uint64_t flags,
                         uint64_t vram_address) noexcept {
    const ByteView dma_pages = params.pages;
    uint64_t consumed = 0;

    while (consumed < pages) {
        uint64_t entries = pages - consumed;
        uint64_t addr = 0;
        uint32_t max_frag = max_fragment;
        Update run{ByteView(nullptr, 0), params.needs_flush};

        if (dma_pages.size() != 0) {
            bool contiguous = true;

            if (entries > 1) {
                uint64_t first = 0;
                uint64_t second = 0;

                if (!page_at(dma_pages, consumed, first) || !page_at(dma_pages, consumed + 1, second)) {
                    return false;
                }

                contiguous = second == first + gpuvm_page_bytes;
                uint64_t count = 2;

                for (; count < entries; ++count) {
                    uint64_t previous = 0;
                    uint64_t current = 0;

                    if (!page_at(dma_pages, consumed + count - 1, previous) ||
                        !page_at(dma_pages, consumed + count, current)) {
                        return false;
                    }

                    if (contiguous != (current == previous + gpuvm_page_bytes)) {
                        break;
                    }
                }

                if (!contiguous) {
                    --count;
                }

                entries = count;
            }

            if (!contiguous) {
                addr = consumed * gpuvm_page_bytes;
                run.pages = dma_pages;
            } else {
                if (!page_at(dma_pages, consumed, addr)) {
                    return false;
                }

                max_frag = congruent_fragment(start, addr);
            }
        } else if ((flags & pte_valid) != 0) {
            addr = vram_address + consumed * gpuvm_page_bytes;
            max_frag = congruent_fragment(start, addr);
        }

        if (!ptes_update(run, start, start + entries, addr, flags, max_frag)) {
            return false;
        }

        params.needs_flush = params.needs_flush || run.needs_flush;
        consumed += entries;
        start += entries;
    }

    return true;
}

// amdgpu_vm_cpu_commit: tlb_seq advances when the update needs a flush; mb and HDP flush.
bool GpuVm::commit(bool needs_flush) noexcept {
    if (needs_flush) {
        ++report_.tlb_seq;
    }

    return nbio_->flush_hdp();
}

// amdgpu_vm_update_pdes: the relocated list is LIFO (list_move to its head), so
// the newest table's parent entry is written first. PDE = amdgpu_gmc_get_pde_for_bo
// (VRAM MC, VALID) through gmc_v11_0_get_vm_pde (amdgpu_gmc_vram_mc2pa).
bool GpuVm::update_pdes() noexcept {
    bool any = false;

    for (;;) {
        uint32_t newest = 0;
        uint32_t newest_order = 0;

        for (uint32_t slot = 1; slot < table_count_; ++slot) {
            if (tables_[slot].relocated && tables_[slot].order > newest_order) {
                newest = slot;
                newest_order = tables_[slot].order;
            }
        }

        if (newest == 0) {
            break;
        }

        Table& table = tables_[newest];
        const uint64_t pde = physical(table_mc(static_cast<uint16_t>(newest)));

        if ((pde & pde_address_reject) != 0 ||
            !write_entry(table.parent, table.parent_entry, (pde & pte_address_mask) | pte_valid)) {
            return false;
        }

        ++report_.entries_written;
        table.relocated = false;
        any = true;
    }

    return !any || commit(false);
}

void GpuVm::count_tables() noexcept {
    report_.tables_live = 0;
    report_.tables_retired = 0;
    report_.mappings = 0;

    for (uint32_t slot = 0; slot < table_count_; ++slot) {
        report_.tables_live += tables_[slot].state == TableState::live ? 1u : 0u;
        report_.tables_retired += tables_[slot].state == TableState::retired ? 1u : 0u;
    }

    for (const auto& mapping : mappings_) {
        report_.mappings += mapping.used ? 1u : 0u;
    }
}

GpuVmStatus GpuVm::apply(uint64_t start, uint64_t pages, uint64_t flags, uint64_t vram_address,
                         ByteView dma_pages, bool flush) noexcept {
    // Store-free rehearsal on the real metadata, then restore it: proves table
    // capacity and every bound before the first hardware-visible store.
    for (uint32_t slot = 0; slot < table_count_; ++slot) {
        shadow_[slot] = tables_[slot];
    }

    const uint32_t order = order_;
    Update rehearsal{dma_pages, flush};
    dry_run_ = true;
    const bool admitted = update_range(rehearsal, start, pages, flags, vram_address);
    dry_run_ = false;
    uint32_t free_slots = 0;

    for (uint32_t slot = 0; slot < table_count_; ++slot) {
        free_slots += tables_[slot].state == TableState::free ? 1u : 0u;
        tables_[slot] = shadow_[slot];
    }

    order_ = order;

    if (!admitted) {
        return free_slots == 0 ? GpuVmStatus::out_of_tables : GpuVmStatus::invalid_argument;
    }

    Update params{dma_pages, flush};

    if (!update_range(params, start, pages, flags, vram_address)) {
        // Unreachable after the rehearsal; stores may have happened.
        faulted_ = true;
        return GpuVmStatus::quarantined;
    }

    if (!commit(params.needs_flush)) {
        faulted_ = true;
        return GpuVmStatus::mmio_failure;
    }

    // amdgpu_vm_pt_free_list: detached; reusable only after a TLB flush.
    for (uint32_t slot = 1; slot < table_count_; ++slot) {
        if (tables_[slot].state == TableState::freeing) {
            tables_[slot] = {no_table, 0, tables_[slot].level, TableState::retired, false, 0};
        }
    }

    if (!update_pdes()) {
        faulted_ = true;
        return GpuVmStatus::mmio_failure;
    }

    return GpuVmStatus::mapped;
}

GpuVmStatus GpuVm::init(memory::Manager& manager, const GpuVmConfig& config, Nbio& nbio) noexcept {
    if (initialized_) {
        return finish(GpuVmStatus::busy);
    }

    report_ = {};

    if (config.table_count == 0 || config.table_count > gpuvm_max_tables ||
        config.recovery.confirm_idle == nullptr || config.vram.bytes == 0 ||
        config.vram.mc_end < config.vram.mc_start) {
        return finish(GpuVmStatus::invalid_argument);
    }

    memory::AllocationInfo info{};
    const uint64_t bytes = static_cast<uint64_t>(config.table_count) * gpuvm_table_bytes;

    if (!retain_local_vram(manager, config.tables, config.vram, bytes, gpuvm_table_bytes, true, info)) {
        return finish(GpuVmStatus::invalid_memory);
    }

    vram_ = config.vram;
    const uint64_t first = physical(info.gpu.virtual_address);
    const uint64_t last = first + bytes - gpuvm_table_bytes;

    if (((first | last) & pde_address_reject) != 0) {
        manager.unretain(config.tables);
        return finish(GpuVmStatus::invalid_memory);
    }

    manager_ = &manager;
    pool_ = config.tables;
    pool_cpu_ = info.backing.cpu_bytes;
    pool_mc_ = info.gpu.virtual_address;
    table_count_ = config.table_count;
    nbio_ = &nbio;
    recovery_ = config.recovery;
    order_ = 0;
    faulted_ = false;

    for (auto& table : tables_) {
        table = {};
    }

    for (auto& mapping : mappings_) {
        mapping = {};
    }

    tables_[0] = {no_table, 0, 0, TableState::live, false, 0};
    report_.memory_retained = true;
    report_.root_mc = pool_mc_;
    report_.page_table_base_addr = first;
    report_.pd_address = first | pte_valid;

    // amdgpu_vm_pt_clear of the root and its commit.
    bool ok = true;

    for (uint32_t index = 0; ok && index < gpuvm_table_entries; ++index) {
        ok = write_entry(0, index, pde_pte);
    }

    initialized_ = true;
    count_tables();

    if (!ok || !commit(false)) {
        faulted_ = true;
        return finish(GpuVmStatus::mmio_failure);
    }

    return finish(GpuVmStatus::ready);
}

GpuVmStatus GpuVm::set_pasid(uint32_t pasid) noexcept {
    if (!initialized_) {
        return finish(GpuVmStatus::unavailable);
    }

    if (pasid > max_pasid) {
        return finish(GpuVmStatus::invalid_argument);
    }

    report_.pasid = pasid;
    return finish(GpuVmStatus::ready);
}

// amdgpu_vm_verify_parameters plus the amdgpu_vm_bo_map interval conflict check.
GpuVmStatus GpuVm::admit(uint64_t va, uint64_t bytes, uint32_t& slot) const noexcept {
    if ((va & (gpuvm_page_bytes - 1)) != 0 || (bytes & (gpuvm_page_bytes - 1)) != 0 || bytes == 0 ||
        va > UINT64_MAX - bytes || ((va + bytes - 1) >> 12) >= gpuvm_max_pfn) {
        return GpuVmStatus::invalid_argument;
    }

    const uint64_t first = va >> 12;
    const uint64_t last = (va + bytes - 1) >> 12;
    slot = gpuvm_max_mappings;

    for (uint32_t index = 0; index < gpuvm_max_mappings; ++index) {
        const Mapping& mapping = mappings_[index];

        if (!mapping.used) {
            slot = slot == gpuvm_max_mappings ? index : slot;
            continue;
        }

        if (first <= mapping.last_pfn && mapping.start_pfn <= last) {
            return GpuVmStatus::overlap;
        }
    }

    return slot == gpuvm_max_mappings ? GpuVmStatus::out_of_mappings : GpuVmStatus::ready;
}

GpuVmStatus GpuVm::map_vram(uint64_t va, uint64_t vram_mc, uint64_t bytes,
                            GpuVmMapFlags flags) noexcept {
    if (!initialized_ || faulted_) {
        return finish(faulted_ ? GpuVmStatus::quarantined : GpuVmStatus::unavailable);
    }

    uint32_t slot = 0;
    const GpuVmStatus admitted = admit(va, bytes, slot);

    if (admitted != GpuVmStatus::ready) {
        return finish(admitted);
    }

    if (!access_valid(flags.access) || (vram_mc & (gpuvm_page_bytes - 1)) != 0 ||
        vram_mc < vram_.mc_start || vram_mc > vram_.mc_end || bytes - 1 > vram_.mc_end - vram_mc ||
        ((physical(vram_mc) + bytes - 1) & ~0x0000ffffffffffffULL) != 0) {
        return finish(GpuVmStatus::invalid_argument);
    }

    // amdgpu_ttm_tt_pte_flags for VRAM (write-combined bus caching: no SNOOPED)
    // with gart_pte_flags MTYPE_UC | EXECUTABLE, then the mapping flags.
    const uint64_t base = pte_valid | (static_cast<uint64_t>(GpuVmMtype::uc) << pte_mtype_shift) |
                          pte_executable | pte_readable | pte_writeable;
    const GpuVmStatus status = apply(va >> 12, bytes >> 12, mapping_flags(base, flags),
                                     physical(vram_mc), ByteView(nullptr, 0), false);

    if (status == GpuVmStatus::mapped) {
        mappings_[slot] = {va >> 12, (va + bytes - 1) >> 12, true};
    }

    count_tables();
    return finish(status);
}

GpuVmStatus GpuVm::map_system(uint64_t va, ByteView dma_pages, bool cpu_cached,
                              GpuVmMapFlags flags) noexcept {
    if (!initialized_ || faulted_) {
        return finish(faulted_ ? GpuVmStatus::quarantined : GpuVmStatus::unavailable);
    }

    const uint64_t pages = dma_pages.size() / 8;
    uint32_t slot = 0;

    if (dma_pages.size() == 0 || dma_pages.size() % 8 != 0 || pages > gpuvm_max_pfn) {
        return finish(GpuVmStatus::invalid_argument);
    }

    const GpuVmStatus admitted = admit(va, pages * gpuvm_page_bytes, slot);

    if (admitted != GpuVmStatus::ready) {
        return finish(admitted);
    }

    // amdgpu_ttm_tt_pte_flags for TT: VALID | SYSTEM | SNOOPED (ttm_cached) |
    // gart_pte_flags | READABLE | WRITEABLE. The shared encoder validates every
    // page (aligned, 48-bit) and supplies the valid/system/snooped/UC/access bits.
    uint64_t encoded = 0;

    for (uint64_t page = 0; page < pages; ++page) {
        uint64_t address = 0;

        if (!access_valid(flags.access) || !page_at(dma_pages, page, address) ||
            memory::prepare_system_pte(address, flags.access, encoded) != memory::PteStatus::prepared) {
            return finish(GpuVmStatus::invalid_argument);
        }
    }

    uint64_t base = (encoded & ~pte_address_mask) | pte_executable;
    base = cpu_cached ? (base | pte_snooped) : (base & ~pte_snooped);
    const GpuVmStatus status = apply(va >> 12, pages, mapping_flags(base, flags), 0, dma_pages, false);

    if (status == GpuVmStatus::mapped) {
        mappings_[slot] = {va >> 12, (va >> 12) + pages - 1, true};
    }

    count_tables();
    return finish(status);
}

GpuVmStatus GpuVm::unmap(uint64_t va) noexcept {
    if (!initialized_ || faulted_) {
        return finish(faulted_ ? GpuVmStatus::quarantined : GpuVmStatus::unavailable);
    }

    for (auto& mapping : mappings_) {
        if (!mapping.used || mapping.start_pfn != va >> 12 || (va & (gpuvm_page_bytes - 1)) != 0) {
            continue;
        }

        // amdgpu_vm_clear_freed: flags 0, addr 0, flush_tlb true.
        const GpuVmStatus status = apply(mapping.start_pfn, mapping.last_pfn - mapping.start_pfn + 1,
                                         0, 0, ByteView(nullptr, 0), true);

        if (status == GpuVmStatus::mapped) {
            mapping = {};
        }

        count_tables();
        return finish(status == GpuVmStatus::mapped ? GpuVmStatus::unmapped : status);
    }

    return finish(GpuVmStatus::not_mapped);
}

GpuVmStatus GpuVm::flush_tlb(GpuVmInvalidator& invalidator, uint32_t flush_type) noexcept {
    if (!initialized_ || faulted_) {
        return finish(faulted_ ? GpuVmStatus::quarantined : GpuVmStatus::unavailable);
    }

    if (flush_type > gmc_max_flush_type || report_.pasid == 0) {
        return finish(GpuVmStatus::invalid_argument);
    }

    // atomic64_xchg(&vm->kfd_last_flushed_seq, tlb_seq) == tlb_seq.
    if (!tlb_flush_needed()) {
        return finish(GpuVmStatus::flushed);
    }

    const uint64_t sequence = report_.tlb_seq;
    uint32_t vmids = 0;
    const GpuVmStatus status = invalidator.flush_pasid(report_.pasid, flush_type, false, vmids);

    if (status != GpuVmStatus::flushed) {
        return finish(status);
    }

    report_.flushed_seq = sequence;

    for (uint32_t slot = 1; slot < table_count_; ++slot) {
        if (tables_[slot].state == TableState::retired) {
            tables_[slot] = {};
        }
    }

    count_tables();
    return finish(GpuVmStatus::flushed);
}

GpuVmStatus GpuVm::walk(uint64_t va, GpuVmWalk& output) const noexcept {
    output = {};

    if (!initialized_) {
        return GpuVmStatus::unavailable;
    }

    const uint64_t pfn = va >> 12;

    if (pfn >= gpuvm_max_pfn) {
        return GpuVmStatus::invalid_argument;
    }

    const ByteView pool(pool_cpu_, static_cast<size_t>(table_count_) * gpuvm_table_bytes);
    uint32_t table = 0;

    for (uint32_t level = 0; level < gpuvm_levels; ++level) {
        const uint32_t index = static_cast<uint32_t>((pfn >> level_shift(level)) & (gpuvm_table_entries - 1));
        uint32_t low = 0;
        uint32_t high = 0;
        const size_t offset = static_cast<size_t>(table) * gpuvm_table_bytes + index * 8u;

        if (!pool.read_u32(offset, low) || !pool.read_u32(offset + 4, high)) {
            return GpuVmStatus::invalid_memory;
        }

        const uint64_t value = (static_cast<uint64_t>(high) << 32) | low;
        output.entries[output.count++] = {static_cast<GpuVmLevel>(level), index, value};

        if (level == ptb_level || (value & pte_valid) == 0 || (value & pde_pte) != 0) {
            break;
        }

        const uint64_t address = value & pte_address_mask;
        const uint64_t pool_physical = physical(pool_mc_);

        if (address < pool_physical ||
            address - pool_physical >= static_cast<uint64_t>(table_count_) * gpuvm_table_bytes) {
            return GpuVmStatus::invalid_memory;
        }

        const uint32_t child = static_cast<uint32_t>((address - pool_physical) / gpuvm_table_bytes);

        if (tables_[child].state != TableState::live || tables_[child].level != level + 1) {
            return GpuVmStatus::invalid_memory;
        }

        table = child;
    }

    return GpuVmStatus::ready;
}

GpuVmStatus GpuVm::save_tables(sdma::CommandBuffer backup) noexcept {
    if (!initialized_ || faulted_ || recovery_.confirm_idle == nullptr ||
        !recovery_.confirm_idle(recovery_.context)) {
        return GpuVmStatus::retained;
    }

    const size_t bytes = static_cast<size_t>(recovery_bytes());
    if (backup.size() != bytes) {
        return GpuVmStatus::invalid_memory;
    }

    const ByteView source(pool_cpu_, bytes);
    for (size_t offset = 0; offset < bytes; offset += 4) {
        uint32_t value = 0;
        if (!source.read_u32(offset, value) || !backup.write_u32(offset, value)) {
            return GpuVmStatus::invalid_memory;
        }
    }
    return GpuVmStatus::ready;
}

GpuVmStatus GpuVm::restore_tables(ByteView backup) noexcept {
    if (!initialized_ || faulted_ || recovery_.confirm_idle == nullptr ||
        !recovery_.confirm_idle(recovery_.context)) {
        return GpuVmStatus::retained;
    }

    const size_t bytes = static_cast<size_t>(recovery_bytes());
    if (backup.size() != bytes || report_.tlb_seq == UINT64_MAX) {
        return GpuVmStatus::invalid_memory;
    }

    const sdma::CommandBuffer destination(pool_cpu_, bytes);
    for (size_t offset = 0; offset < bytes; offset += 4) {
        uint32_t value = 0;
        if (!backup.read_u32(offset, value) || !destination.write_u32(offset, value)) {
            faulted_ = true;
            return finish(GpuVmStatus::quarantined);
        }
    }
    if (!commit(true)) {
        faulted_ = true;
        return finish(GpuVmStatus::quarantined);
    }
    return finish(GpuVmStatus::ready);
}

GpuVmStatus GpuVm::fini() noexcept {
    if (!initialized_) {
        return finish(GpuVmStatus::unavailable);
    }

    if (!recovery_.confirm_idle(recovery_.context)) {
        return finish(GpuVmStatus::retained);
    }

    // amdgpu_vm_set_pasid(vm, 0), amdgpu_vm_pt_free_root.
    manager_->unretain(pool_);
    manager_ = nullptr;
    pool_ = {};
    pool_cpu_ = nullptr;
    pool_mc_ = 0;
    table_count_ = 0;
    initialized_ = false;
    faulted_ = false;

    for (auto& table : tables_) {
        table = {};
    }

    for (auto& mapping : mappings_) {
        mapping = {};
    }

    const uint64_t written = report_.entries_written;
    report_ = {};
    report_.entries_written = written;
    return finish(GpuVmStatus::released);
}

} // namespace rx7900::hardware

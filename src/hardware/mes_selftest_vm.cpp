/*
Module: rx7900.hardware.mes_selftest_vm
Purpose: Encode the MES self-test process page tables and sequence amdgpu_mes_self_test.
Created: 2026-10-21
Architecture: Checked CommandBuffer stores into retained local-VRAM tables, the
shared memory::prepare_system_pte encoder for system-page PTE address bits, Nbio
HDP flushes and Mes11::self_test for every MES and queue access.
Copyright 2008 Advanced Micro Devices, Inc., Copyright 2008 Red Hat Inc.,
Copyright 2009 Jerome Glisse; Copyright 2019 Advanced Micro Devices, Inc.
Adapted Linux v6.12 amdgpu_mes.c, amdgpu_vm.c, amdgpu_vm_pt.c, amdgpu_vm_cpu.c,
amdgpu_vm.h, amdgpu_gmc.h/.c, gmc_v11_0.c, amdgpu_ttm.c, amdgpu_vram_mgr.c and
amdgpu_ids.c. Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#include "mes_selftest_vm.hpp"
#include "page_table.hpp"

namespace rx7900::hardware {
namespace {

// amdgpu_vm.h PTE/PDE bits.
constexpr uint64_t pte_valid = 1ULL << 0;
constexpr uint64_t pte_executable = 1ULL << 4;
constexpr uint64_t pde_pte = 1ULL << 54; // AMDGPU_PDE_PTE (pre-GFX12)
constexpr uint64_t pte_mtype_nv10_mask = 7ULL << 48;
constexpr uint32_t pte_frag_shift = 7;   // AMDGPU_PTE_FRAG(x) = (x & 0x1f) << 7
constexpr uint64_t pte_frag_mask = 0x1fULL;
// amdgpu_gmc_set_pte_pde address mask.
constexpr uint64_t pte_address_mask = 0x0000fffffffff000ULL;
// gmc_v11_0_get_vm_pde: BUG_ON(*addr & 0xFFFF00000000003FULL).
constexpr uint64_t pde_address_reject = 0xffff00000000003fULL;
constexpr uint64_t gpu_page_bytes = 4096;
// amdgpu_vm_pt_level_shift with block_size 9 for PDB2/PDB1/PDB0/PTB.
constexpr uint32_t level_shift[mes_selftest_vm_levels]{27, 18, 9, 0};
// AMDGPU_VA_RESERVED_BOTTOM in GPU pages.
constexpr uint64_t metadata_pfn = mes11_self_test_metadata_va / gpu_page_bytes;

// The PTE flags amdgpu_vm_bo_update computes for the GTT metadata BO and
// amdgpu_mes_ctx_map_meta_data's READABLE | WRITEABLE | EXECUTABLE mapping:
// amdgpu_ttm_tt_pte_flags gives VALID | SYSTEM | SNOOPED (ttm_cached kernel BO)
// | gart_pte_flags (MTYPE_UC, EXECUTABLE) | READABLE | WRITEABLE, then
// gmc_v11_0_get_vm_pte keeps EXECUTABLE from the mapping and replaces the MTYPE
// with the mapping's (none: MTYPE_NC 0). The shared encoder supplies the valid,
// system, snooped, read/write and address bits; the mapping's MTYPE and
// EXECUTABLE are applied here.
bool metadata_pte(uint64_t dma_page, uint64_t& pte) noexcept {
    uint64_t encoded = 0;

    if (memory::prepare_system_pte(dma_page, memory::Access::read_write, encoded) !=
        memory::PteStatus::prepared) {
        return false;
    }

    pte = (encoded & ~pte_mtype_nv10_mask) | pte_executable;
    return true;
}

bool page_at(ByteView pages, uint64_t index, uint64_t& address) noexcept {
    uint32_t low = 0;
    uint32_t high = 0;

    if (!pages.read_u32(static_cast<size_t>(index) * 8, low) ||
        !pages.read_u32(static_cast<size_t>(index) * 8 + 4, high)) {
        return false;
    }

    address = (static_cast<uint64_t>(high) << 32) | low;
    return true;
}

// ffs(value) - 1 and fls64(value) - 1 with Linux's unsigned wrap for zero.
uint32_t first_set(uint64_t value) noexcept {
    for (uint32_t bit = 0; bit < 64; ++bit) {
        if ((value >> bit) & 1u) {
            return bit;
        }
    }

    return UINT32_MAX;
}

uint32_t last_set(uint64_t value) noexcept {
    for (uint32_t bit = 64; bit > 0; --bit) {
        if ((value >> (bit - 1)) & 1u) {
            return bit - 1;
        }
    }

    return UINT32_MAX;
}

// amdgpu_vm_pte_fragment (asic_type >= VEGA10: max_frag 31).
void fragment(bool pages_addr, uint64_t start, uint64_t end, uint32_t& frag,
              uint64_t& frag_end) noexcept {
    constexpr uint32_t max_frag = 31;

    if (pages_addr) {
        frag = 0;
        frag_end = end;
        return;
    }

    const uint32_t a = first_set(start);
    const uint32_t b = last_set(end - start);
    frag = a < b ? a : b;

    if (frag >= max_frag) {
        frag = max_frag;
        frag_end = end & ~((1ULL << max_frag) - 1);
    } else {
        frag_end = start + (1ULL << frag);
    }
}

bool overlaps(uint64_t a, uint64_t a_bytes, uint64_t b, uint64_t b_bytes) noexcept {
    return a < b + b_bytes && b < a + a_bytes;
}

} // namespace

MesSelfTestVmStatus MesSelfTestVm::finish(MesSelfTestVmStatus status) noexcept {
    report_.status = status;
    return status;
}

uint64_t MesSelfTestVm::physical(uint64_t mc) const noexcept {
    // amdgpu_gmc_vram_mc2pa: mc - vram_start + vram_base_offset (FB_OFFSET).
    return mc - vram_.mc_start + vram_.physical_offset;
}

bool MesSelfTestVm::write_entry(MesSelfTestVmLevel level, uint32_t index, uint64_t value) noexcept {
    const uint32_t table = static_cast<uint32_t>(level);
    const sdma::CommandBuffer out(table_cpu_[table], mes_selftest_vm_table_bytes);
    return index < mes_selftest_vm_entries &&
           out.write_u32(index * 8, static_cast<uint32_t>(value)) &&
           out.write_u32(index * 8 + 4, static_cast<uint32_t>(value >> 32));
}

// amdgpu_vm_pt_clear: every PD entry 0 | AMDGPU_PDE_PTE (gmc_v11_0_get_vm_pde
// leaves it unchanged), every PTB entry AMDGPU_PTE_EXECUTABLE (the GMC9
// fault-priority workaround), written over the whole BO.
bool MesSelfTestVm::clear_table(MesSelfTestVmLevel level) noexcept {
    const uint64_t value = level == MesSelfTestVmLevel::ptb ? pte_executable : pde_pte;

    for (uint32_t index = 0; index < mes_selftest_vm_entries; ++index) {
        if (!write_entry(level, index, value)) {
            return false;
        }
    }

    return true;
}

// amdgpu_vm_ptes_update at the PTB level for [start, end) (pfn), dst the
// physical start (contiguous) or the BO byte offset (pages != empty: per-page
// amdgpu_vm_map_gart). Fragments at or above the PDB0 shift would make Linux
// write huge PDB0 entries; they cannot occur for this mapping and are refused.
bool MesSelfTestVm::update_ptes(uint64_t start, uint64_t end, uint64_t dst, uint64_t flags,
                                ByteView pages) noexcept {
    const bool pages_addr = pages.size() != 0;
    const uint64_t entry_end_limit = (start | (mes_selftest_vm_entries - 1)) + 1;
    const uint64_t entry_end = end < entry_end_limit ? end : entry_end_limit;
    uint64_t frag_start = start;
    uint64_t frag_end = 0;
    uint32_t frag = 0;

    if (start >= end || end > mes_selftest_vm_entries) {
        return false;
    }

    fragment(pages_addr, frag_start, end, frag, frag_end);

    do {
        if (frag >= level_shift[static_cast<uint32_t>(MesSelfTestVmLevel::pdb0)]) {
            return false;
        }

        const uint64_t upd_end = entry_end < frag_end ? entry_end : frag_end;
        uint64_t nptes = upd_end - frag_start;
        nptes = nptes == 0 ? 1 : nptes;
        uint64_t upd_flags = flags | ((frag & pte_frag_mask) << pte_frag_shift);

        // amdgpu_vm_pte_update_flags at AMDGPU_VM_PTB.
        if ((upd_flags & pte_valid) == 0) {
            upd_flags |= pte_executable;
        }

        for (uint64_t index = 0; index < nptes; ++index) {
            uint64_t value = dst + index * gpu_page_bytes;

            if (pages_addr && !page_at(pages, value / gpu_page_bytes, value)) {
                return false;
            }

            if (!write_entry(MesSelfTestVmLevel::ptb, static_cast<uint32_t>(frag_start + index),
                             (value & pte_address_mask) | upd_flags)) {
                return false;
            }
        }

        report_.ptes_written += static_cast<uint32_t>(nptes);
        dst += nptes * gpu_page_bytes;
        frag_start = upd_end;

        if (frag_start >= frag_end) {
            fragment(pages_addr, frag_start, end, frag, frag_end);
        }
    } while (frag_start < entry_end);

    return true;
}

// amdgpu_vm_update_range over the metadata mapping (start pfn 16, 52 pages).
// dma_pages empty: the amdgpu_vm_clear_freed unmap (flags 0, no resource, addr 0).
bool MesSelfTestVm::update_range(ByteView dma_pages, uint64_t flags) noexcept {
    uint64_t start = metadata_pfn;
    uint64_t cursor = 0;
    uint64_t remaining = mes_selftest_metadata_pages;

    while (remaining != 0) {
        uint64_t entries = remaining;
        uint64_t addr = 0;
        ByteView pages(nullptr, 0);

        if (dma_pages.size() != 0) {
            bool contiguous = true;

            if (entries > 1) {
                uint64_t first = 0;
                uint64_t second = 0;

                if (!page_at(dma_pages, cursor, first) || !page_at(dma_pages, cursor + 1, second)) {
                    return false;
                }

                contiguous = second == first + gpu_page_bytes;
                uint64_t count = 2;

                for (; count < entries; ++count) {
                    uint64_t previous = 0;
                    uint64_t current = 0;

                    if (!page_at(dma_pages, cursor + count - 1, previous) ||
                        !page_at(dma_pages, cursor + count, current)) {
                        return false;
                    }

                    if (contiguous != (current == previous + gpu_page_bytes)) {
                        break;
                    }
                }

                if (!contiguous) {
                    --count;
                }

                entries = count;
            }

            if (!contiguous) {
                addr = cursor * gpu_page_bytes;
                pages = dma_pages;
            } else if (!page_at(dma_pages, cursor, addr)) {
                return false;
            }
        }

        if (!update_ptes(start, start + entries, addr, flags, pages)) {
            return false;
        }

        ++report_.mapping_runs;
        cursor += entries;
        start += entries;
        remaining -= entries;
    }

    return true;
}

bool MesSelfTestVm::retain(memory::Manager& vram_manager, const MesSelfTestVmMemory& tables,
                           memory::Manager& gtt_manager, const MesSelfTestVmInput& input) noexcept {
    memory::AllocationInfo info[mes_selftest_vm_levels]{};
    uint32_t retained = 0;
    bool ok = true;

    for (; ok && retained < mes_selftest_vm_levels; ++retained) {
        ok = retain_local_vram(vram_manager, tables.tables[retained], input.vram,
                               mes_selftest_vm_table_bytes, mes_selftest_vm_table_bytes, true,
                               info[retained]);
    }

    if (!ok) {
        --retained;
    }

    for (uint32_t a = 0; ok && a < mes_selftest_vm_levels; ++a) {
        for (uint32_t b = a + 1; ok && b < mes_selftest_vm_levels; ++b) {
            ok = !overlaps(info[a].gpu.virtual_address, mes_selftest_vm_table_bytes,
                           info[b].gpu.virtual_address, mes_selftest_vm_table_bytes);
        }
    }

    memory::AllocationInfo metadata{};
    sdma::GpuRange range{};
    const uint64_t bytes = mes11_ctx::bytes;
    ok = ok && gtt_manager.describe(input.metadata, metadata) == memory::Status::success &&
         gtt_manager.resolve({input.metadata, 0}, bytes, memory::Access::read_write, range) ==
             memory::Status::success;
    const uint64_t start = metadata.gpu.virtual_address;
    ok = ok && (start & (gpu_page_bytes - 1)) == 0 && metadata.gpu.byte_length >= bytes &&
         start >= input.gart.start && start <= input.gart.end &&
         bytes - 1 <= input.gart.end - start && metadata.backing.cpu_bytes != nullptr &&
         metadata.backing.cpu_byte_count >= bytes &&
         gtt_manager.retain(input.metadata, memory::Access::read_write) == memory::Status::success;

    if (!ok) {
        while (retained > 0) {
            --retained;
            vram_manager.unretain(tables.tables[retained]);
        }

        return false;
    }

    vram_manager_ = &vram_manager;
    gtt_manager_ = &gtt_manager;
    tables_ = tables;
    metadata_ = input.metadata;
    metadata_cpu_ = metadata.backing.cpu_bytes;
    report_.metadata_gart = start;
    report_.memory_retained = true;

    for (uint32_t level = 0; level < mes_selftest_vm_levels; ++level) {
        table_cpu_[level] = info[level].backing.cpu_bytes;
        report_.table_mc[level] = info[level].gpu.virtual_address;
    }

    return true;
}

void MesSelfTestVm::release_leases() noexcept {
    if (report_.memory_retained) {
        for (const auto& table : tables_.tables) {
            vram_manager_->unretain(table);
        }

        gtt_manager_->unretain(metadata_);
    }

    vram_manager_ = nullptr;
    gtt_manager_ = nullptr;
    tables_ = {};
    metadata_ = {};
    metadata_cpu_ = nullptr;

    for (auto& table : table_cpu_) {
        table = nullptr;
    }

    report_.memory_retained = false;
}

MesSelfTestVmStatus MesSelfTestVm::map(memory::Manager& vram_manager,
                                       const MesSelfTestVmMemory& tables,
                                       memory::Manager& gtt_manager,
                                       const MesSelfTestVmInput& input, Nbio& nbio) noexcept {
    if (phase_ != Phase::idle) {
        return phase_ == Phase::retained ? MesSelfTestVmStatus::quarantined :
                                           MesSelfTestVmStatus::busy;
    }

    report_ = MesSelfTestVmReport();

    // Every page must encode (aligned, inside the 48-bit space) before any store.
    bool pages_ok = input.metadata_dma_pages.size() ==
                    static_cast<size_t>(mes_selftest_metadata_pages) * 8;

    for (uint32_t page = 0; pages_ok && page < mes_selftest_metadata_pages; ++page) {
        uint64_t address = 0;
        uint64_t pte = 0;
        pages_ok = page_at(input.metadata_dma_pages, page, address) && metadata_pte(address, pte);
    }

    if (!pages_ok || input.vram.bytes == 0 || input.vram.mc_end < input.vram.mc_start ||
        input.gart.end <= input.gart.start || input.recovery.confirm_reset_quiescence == nullptr) {
        return finish(MesSelfTestVmStatus::invalid_argument);
    }

    vram_ = input.vram;

    if (!retain(vram_manager, tables, gtt_manager, input)) {
        return finish(MesSelfTestVmStatus::invalid_memory);
    }

    nbio_ = &nbio;
    recovery_ = input.recovery;
    const uint64_t root = report_.table_mc[0];
    report_.page_table_base_addr = physical(root);

    // PDE values: amdgpu_gmc_get_pde_for_bo for a VRAM BO is its MC address with
    // amdgpu_ttm_tt_pde_flags VALID (write-combined dGPU VRAM: no SNOOPED), then
    // gmc_v11_0_get_vm_pde converts the address with amdgpu_gmc_vram_mc2pa.
    uint64_t pde[mes_selftest_vm_levels]{};

    for (uint32_t level = 1; level < mes_selftest_vm_levels; ++level) {
        pde[level] = physical(report_.table_mc[level]);

        if ((pde[level] & pde_address_reject) != 0) {
            release_leases();
            return finish(MesSelfTestVmStatus::invalid_memory);
        }

        pde[level] |= pte_valid;
    }

    uint64_t flags = 0;
    uint64_t first = 0;
    page_at(input.metadata_dma_pages, 0, first);
    metadata_pte(first, flags);
    flags &= ~pte_address_mask;
    const sdma::CommandBuffer metadata(metadata_cpu_, mes11_ctx::bytes);
    bool ok = clear_table(MesSelfTestVmLevel::pdb2);

    // amdgpu_mes_ctx_alloc_meta_data: memset of the whole structure.
    for (uint32_t offset = 0; ok && offset < mes11_ctx::bytes; offset += 4) {
        ok = metadata.write_u32(offset, 0);
    }

    // amdgpu_vm_ptes_update walk from the root: amdgpu_vm_pt_alloc creates and
    // clears PDB1, PDB0 and the PTB, then the PTEs; amdgpu_vm_update_pdes writes
    // the relocated entries in allocation order (root[0], PDB1[0], PDB0[0]).
    ok = ok && clear_table(MesSelfTestVmLevel::pdb1) && clear_table(MesSelfTestVmLevel::pdb0) &&
         clear_table(MesSelfTestVmLevel::ptb) && update_range(input.metadata_dma_pages, flags) &&
         write_entry(MesSelfTestVmLevel::pdb2, 0, pde[1]) &&
         write_entry(MesSelfTestVmLevel::pdb1, 0, pde[2]) &&
         write_entry(MesSelfTestVmLevel::pdb0, 0, pde[3]);

    if (!ok) {
        // CPU stores only: nothing references these buffers yet.
        release_leases();
        return finish(MesSelfTestVmStatus::invalid_memory);
    }

    if (!nbio_->flush_hdp()) {
        release_leases();
        return finish(MesSelfTestVmStatus::mmio_failure);
    }

    phase_ = Phase::mapped;
    return finish(MesSelfTestVmStatus::mapped);
}

Mes11ProcessVm MesSelfTestVm::process_vm() const noexcept {
    Mes11ProcessVm vm;

    if (phase_ == Phase::mapped) {
        vm.page_table_base_addr_ = report_.page_table_base_addr;
        vm.metadata_va_ = mes11_self_test_metadata_va;
        vm.metadata_gart_ = report_.metadata_gart;
        vm.metadata_cpu_ = metadata_cpu_;
    }

    return vm;
}

MesSelfTestVmStatus MesSelfTestVm::teardown(const Mes11SelfTestReport& test) noexcept {
    if (phase_ != Phase::mapped) {
        return phase_ == Phase::retained ? MesSelfTestVmStatus::quarantined :
                                           MesSelfTestVmStatus::unavailable;
    }

    // A queue MES may still run, or an engine that did not finish as expected,
    // may still walk these tables and touch the metadata.
    if (test.memory_quarantined || (test.ran && test.memory_retained)) {
        phase_ = Phase::retained;
        return finish(MesSelfTestVmStatus::retained);
    }

    // amdgpu_mes_ctx_unmap_meta_data -> amdgpu_vm_clear_freed.
    if (!update_range(ByteView(nullptr, 0), 0)) {
        phase_ = Phase::retained;
        return finish(MesSelfTestVmStatus::invalid_memory);
    }

    report_.unmapped = true;

    if (!nbio_->flush_hdp()) {
        phase_ = Phase::retained;
        return finish(MesSelfTestVmStatus::mmio_failure);
    }

    // amdgpu_vm_fini and amdgpu_mes_ctx_free_meta_data.
    release_leases();
    phase_ = Phase::idle;
    return finish(MesSelfTestVmStatus::released);
}

MesSelfTestVmStatus MesSelfTestVm::release_after_reset() noexcept {
    if (phase_ == Phase::idle) {
        return MesSelfTestVmStatus::busy;
    }

    if (recovery_.confirm_reset_quiescence == nullptr ||
        !recovery_.confirm_reset_quiescence(recovery_.context)) {
        return MesSelfTestVmStatus::quarantined;
    }

    release_leases();
    phase_ = Phase::idle;
    return finish(MesSelfTestVmStatus::released);
}

MesSelfTestOutcome run_mes_self_test(Mes11& mes, MesSelfTestVm& vm,
                                     const MesSelfTestInputs& inputs) noexcept {
    MesSelfTestOutcome outcome{MesSelfTestVmStatus::unavailable, Mes11Status::unavailable,
                               MesSelfTestVmStatus::unavailable};

    if (inputs.vram_manager == nullptr || inputs.gtt_manager == nullptr || inputs.nbio == nullptr) {
        outcome.map_status = MesSelfTestVmStatus::invalid_argument;
        return outcome;
    }

    outcome.map_status = vm.map(*inputs.vram_manager, inputs.tables, *inputs.gtt_manager,
                                inputs.vm, *inputs.nbio);

    if (outcome.map_status != MesSelfTestVmStatus::mapped) {
        return outcome;
    }

    outcome.test_status = mes.self_test(*inputs.gtt_manager, inputs.queues, vm.process_vm(),
                                        inputs.config);
    outcome.teardown_status = vm.teardown(mes.self_test_report());
    return outcome;
}

} // namespace rx7900::hardware

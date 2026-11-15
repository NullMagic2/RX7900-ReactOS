/*
Module: rx7900.hardware.mes_selftest_vm
Purpose: Build and tear down the per-process GPUVM of the MES self-test and run amdgpu_mes_self_test.
Created: 2026-10-21
Architecture: Four local-VRAM page tables (retain_local_vram) for a 4-level GC 11
GPUVM, the CPU-visible GTT metadata buffer mapped through system PTEs from the
shared memory::prepare_system_pte encoder, HDP publication through Nbio; the
queue part runs in Mes11::self_test through the Mes11ProcessVm token.
Copyright 2008 Advanced Micro Devices, Inc., Copyright 2008 Red Hat Inc.,
Copyright 2009 Jerome Glisse; Copyright 2019 Advanced Micro Devices, Inc.
Adapted Linux v6.12 amdgpu_mes.c (amdgpu_mes_self_test, ctx alloc/map/unmap),
amdgpu_vm.c (amdgpu_vm_init, amdgpu_vm_adjust_size, amdgpu_vm_bo_update,
amdgpu_vm_update_range, amdgpu_vm_clear_freed, amdgpu_vm_map_gart),
amdgpu_vm_pt.c (level shifts, entries, amdgpu_vm_pt_clear, amdgpu_vm_pt_alloc,
amdgpu_vm_pde_update, amdgpu_vm_pte_update_flags, amdgpu_vm_pte_fragment,
amdgpu_vm_ptes_update), amdgpu_vm_cpu.c, amdgpu_vm.h, amdgpu_gmc.h/.c
(amdgpu_gmc_get_pde_for_bo, amdgpu_gmc_vram_mc2pa), gmc_v11_0.c (get_vm_pde,
get_vm_pte, gart_pte_flags), amdgpu_ttm.c (tt_pde/pte_flags), amdgpu_vram_mgr.c
and amdgpu_ids.c. Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#pragma once

#include "mes11.hpp"

namespace rx7900::hardware {

// amdgpu_vm_adjust_size(256 TiB, fragment 9, max_level 3, 48 bits) on GC 11:
// max_pfn 2^36, num_level 3 so root_level AMDGPU_VM_PDB2, block_size 9. The walk
// has four tables, PDB2 (root), PDB1, PDB0 and PTB, with pfn shifts 27/18/9/0;
// every table has 512 entries (the root: max_pfn >> 27) and is one 4 KiB page.
// gmc_v11_0 never sets translate_further, so no PDE gets BFS or TF bits.
constexpr uint32_t mes_selftest_vm_levels = 4;
constexpr uint32_t mes_selftest_vm_table_bytes = 4096;
constexpr uint32_t mes_selftest_vm_entries = 512;
// amdgpu_mes_ctx_meta_data pages mapped at mes11_self_test_metadata_va (PTB
// entries 16..67 of the first page table).
constexpr uint32_t mes_selftest_metadata_pages = mes11_ctx::bytes / 4096;

enum class MesSelfTestVmStatus {
    ready,
    mapped,
    released,
    // Teardown kept every lease: the self-test left uncertain hardware state.
    retained,
    unavailable,
    invalid_argument,
    invalid_memory,
    mmio_failure,
    busy,
    quarantined
};

// Index order of the four page tables.
enum class MesSelfTestVmLevel : uint32_t { pdb2 = 0, pdb1 = 1, pdb0 = 2, ptb = 3 };

// Four separate Manager allocations in CPU-visible local VRAM, 4 KiB each and
// aligned, as amdgpu_vm_pt_create places them (AMDGPU_GEM_DOMAIN_VRAM on a dGPU,
// VRAM_CONTIGUOUS). The root must be VRAM: ADD_QUEUE page_table_base_addr is
// vram_base_offset + root MC - vram_start, a VRAM physical address.
struct MesSelfTestVmMemory final {
    memory::Handle tables[mes_selftest_vm_levels];
};

struct MesSelfTestVmRecovery final {
    void* context;
    // True only after a reset proves no engine walks these tables or touches the
    // metadata any more. Never inferred from a failure.
    bool (*confirm_reset_quiescence)(void*) noexcept;
};

// metadata: a CPU-visible GTT allocation of at least mes11_ctx::bytes on the GTT
// manager, 4 KiB aligned, its GPU address inside gart (amdgpu_bo_create_kernel
// GTT: ctx_data->meta_data_mc_addr). metadata_dma_pages: the
// mes_selftest_metadata_pages little-endian 64-bit DMA bus addresses of exactly
// the pinned system pages the GART maps for it, in order (the TTM dma_address[]
// amdgpu_vm_bo_update maps); immutable and caller-owned for the call.
struct MesSelfTestVmInput final {
    VramLocation vram;
    GartLayout gart;
    memory::Handle metadata;
    ByteView metadata_dma_pages;
    MesSelfTestVmRecovery recovery;
};

struct MesSelfTestVmReport final {
    MesSelfTestVmStatus status = MesSelfTestVmStatus::unavailable;
    uint64_t table_mc[mes_selftest_vm_levels]{};
    uint64_t page_table_base_addr = 0;
    uint64_t metadata_gart = 0;
    // amdgpu_vm_update_range loop iterations (contiguity runs) of the mapping and
    // the number of PTB entries written by it.
    uint32_t mapping_runs = 0;
    uint32_t ptes_written = 0;
    bool memory_retained = false;
    bool unmapped = false;
};

// Serialized PASSIVE_LEVEL use by the exclusive device owner, around one
// Mes11::self_test. map: amdgpu_vm_init (root PD create + amdgpu_vm_pt_clear),
// amdgpu_mes_ctx_alloc_meta_data's memset and amdgpu_mes_ctx_map_meta_data
// (PDB1/PDB0/PTB allocated and cleared while walking, the PTEs with
// amdgpu_vm_update_range's contiguity and fragment rules, then the PDEs), then one
// HDP flush. Linux updates these gfx-VM tables with SDMA jobs (vm_update_mode 0
// on a dGPU) and waits for them; here the CPU writes the identical entries
// through the uncached VRAM mapping, as amdgpu_vm_cpu_update would, and
// amdgpu_vm_cpu_commit's HDP flush publishes them. The GPU references nothing
// until Mes11::self_test's first ADD_QUEUE.
//
// teardown after Mes11::self_test: amdgpu_mes_ctx_unmap_meta_data
// (amdgpu_vm_clear_freed rewrites the 52 PTEs with the unmapped value), an HDP
// flush, amdgpu_vm_fini and amdgpu_mes_ctx_free_meta_data (every lease returned).
// When the self-test reports quarantined memory, nothing is rewritten and every
// lease stays until release_after_reset. No TLB invalidation is issued (Linux
// has none here either: tlb_seq only increments for later jobs).
class MesSelfTestVm final {
public:
    MesSelfTestVm() noexcept = default;
    MesSelfTestVm(const MesSelfTestVm&) = delete;
    MesSelfTestVm& operator=(const MesSelfTestVm&) = delete;

    MesSelfTestVmStatus map(memory::Manager& vram_manager, const MesSelfTestVmMemory& tables,
                            memory::Manager& gtt_manager, const MesSelfTestVmInput& input,
                            Nbio& nbio) noexcept;
    // Valid only between a successful map and teardown/release.
    Mes11ProcessVm process_vm() const noexcept;
    MesSelfTestVmStatus teardown(const Mes11SelfTestReport& report) noexcept;
    MesSelfTestVmStatus release_after_reset() noexcept;

    MesSelfTestVmReport report() const noexcept { return report_; }

private:
    enum class Phase { idle, mapped, retained };

    bool retain(memory::Manager& vram_manager, const MesSelfTestVmMemory& tables,
                memory::Manager& gtt_manager, const MesSelfTestVmInput& input) noexcept;
    void release_leases() noexcept;
    bool write_entry(MesSelfTestVmLevel level, uint32_t index, uint64_t value) noexcept;
    bool clear_table(MesSelfTestVmLevel level) noexcept;
    bool update_range(ByteView dma_pages, uint64_t flags) noexcept;
    bool update_ptes(uint64_t start, uint64_t end, uint64_t dst, uint64_t flags,
                     ByteView pages) noexcept;
    uint64_t physical(uint64_t mc) const noexcept;
    MesSelfTestVmStatus finish(MesSelfTestVmStatus status) noexcept;

    MesSelfTestVmReport report_{};
    Phase phase_ = Phase::idle;
    memory::Manager* vram_manager_ = nullptr;
    memory::Manager* gtt_manager_ = nullptr;
    MesSelfTestVmMemory tables_{};
    memory::Handle metadata_{};
    uint8_t* table_cpu_[mes_selftest_vm_levels]{};
    uint8_t* metadata_cpu_ = nullptr;
    Nbio* nbio_ = nullptr;
    VramLocation vram_{};
    MesSelfTestVmRecovery recovery_{};
};

// Everything amdgpu_mes_self_test needs. The two managers may be the same object.
struct MesSelfTestInputs final {
    memory::Manager* vram_manager;
    MesSelfTestVmMemory tables;
    memory::Manager* gtt_manager;
    MesSelfTestVmInput vm;
    Mes11SelfTestMemory queues;
    Mes11SelfTestConfig config;
    Nbio* nbio;
};

struct MesSelfTestOutcome final {
    MesSelfTestVmStatus map_status;
    Mes11Status test_status;
    MesSelfTestVmStatus teardown_status;
};

// amdgpu_mes_self_test as mes_v11_0_late_init calls it on GC 11.0.0 bare metal
// (first initialization only, not on reset, suspend or s0ix resume): map the VM,
// Mes11::self_test, tear down. A failed map skips the test exactly as Linux's
// "failed to map ctx meta data" path. The integrator logs the outcome and
// continues bring-up regardless (Linux returns 0); it runs in late init after the
// SMU, display, GFX, SDMA and multimedia late_init and before
// amdgpu_device_set_cg_state/set_pg_state(GATE).
MesSelfTestOutcome run_mes_self_test(Mes11& mes, MesSelfTestVm& vm,
                                     const MesSelfTestInputs& inputs) noexcept;

} // namespace rx7900::hardware

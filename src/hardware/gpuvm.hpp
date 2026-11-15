/*
Module: rx7900.hardware.gpuvm
Purpose: Own per-process GC 11 GPU virtual memory: page tables, mappings, PASIDs and VM TLB invalidation.
Created: 2026-10-22
Architecture: A fixed pool of 4 KiB local-VRAM tables (one retained Manager
allocation) updated by the CPU as amdgpu_vm_cpu does, HDP publication through
Nbio, system-page PTE bits from the shared memory::prepare_system_pte encoder,
and invalidation through Gmc11 (register or MES path) after a PASID-to-VMID
lookup in the OSSSYS IH_VMID LUT (Ih6Registers token).
Copyright 2008 Advanced Micro Devices, Inc., Copyright 2008 Red Hat Inc.,
Copyright 2009 Jerome Glisse; Copyright 2021 Advanced Micro Devices, Inc.
Adapted Linux v6.12 amdgpu_vm.c (init, fini, set_pasid, verify_parameters,
update_range, update_pdes, bo_update flags, clear_freed, flush_compute_tlb,
map_gart), amdgpu_vm_pt.c (cursor walk, pt_alloc/clear/free, pte_fragment,
pte_update_flags, ptes_update, pde_update), amdgpu_vm_cpu.c, amdgpu_vm.h,
amdgpu_ids.c (amdgpu_pasid_alloc/free; amdgpu_vmid_grab is not used by MES
queues), amdgpu_ttm.c (tt_pde/pte_flags), amdgpu_gmc.c/.h (get_pde_for_bo,
pd_addr, vram_mc2pa, flush_gpu_tlb_pasid), gmc_v11_0.c (get_vm_pde, get_vm_pte,
map_mtype, flush_gpu_tlb_pasid, emit_pasid_mapping, sw_init VMID split),
amdgpu_mes.c (vmid masks, ADD_QUEUE page_table_base_addr) and
osssys_6_0_0_offset.h. Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#pragma once

#include "gmc11.hpp"
#include "ih6.hpp"

namespace rx7900::hardware {

// amdgpu_vm_adjust_size(256 TiB, 9, 3, 48) on GC 11: root_level AMDGPU_VM_PDB2,
// block_size 9, max_pfn 2^36. Four levels PDB2/PDB1/PDB0/PTB with pfn shifts
// 27/18/9/0, every table 512 entries in one 4 KiB page (the root: max_pfn >> 27).
// gmc_v11_0 never sets translate_further: no PDE gets BFS or TF bits.
constexpr uint32_t gpuvm_levels = 4;
constexpr uint32_t gpuvm_table_bytes = 4096;
constexpr uint32_t gpuvm_table_entries = 512;
constexpr uint64_t gpuvm_page_bytes = 4096;
constexpr uint64_t gpuvm_max_pfn = 1ULL << 36;
// Fixed capacities of one VM object (pool tables and live mappings).
constexpr uint32_t gpuvm_max_tables = 256;
constexpr uint32_t gpuvm_max_mappings = 64;

// VMID ownership on GC 11 with MES (gmc_v11_0_sw_init, amdgpu_mes_init): VMID 0
// is the system/GART context, 1..7 belong to amdgpu_vmid_grab for kernel-ring
// jobs (id_mgr num_ids = first_kfd_vmid = 8), and MES assigns 8..15 to user
// queues itself (SET_HW_RESOURCES vmid_mask_gfxhub/mmhub 0xffffff00). The driver
// never grabs a VMID for a MES queue (amdgpu_ib_schedule skips the VMID check for
// is_mes_queue) and never programs its page-table base: MES does both and
// updates IH_VMID_n_LUT. A process VM therefore has no driver VMID; its TLB is
// invalidated by PASID through that LUT.
constexpr uint32_t gpuvm_first_mes_vmid = 8;
constexpr uint32_t gpuvm_mes_vmid_mask = 0xffffff00u;
// amdgpu_pasid_alloc(16): the IH/MES PASID field is 16 bits wide.
constexpr uint32_t gpuvm_pasid_bits = 16;

enum class GpuVmStatus {
    ready,
    mapped,
    unmapped,
    flushed,
    released,
    // fini refused: an engine may still walk the tables; every lease is kept.
    retained,
    unavailable,
    invalid_argument,
    invalid_memory,
    overlap,
    not_mapped,
    out_of_tables,
    out_of_mappings,
    exhausted,
    mmio_failure,
    busy,
    quarantined
};

enum class GpuVmLevel : uint32_t { pdb2 = 0, pdb1 = 1, pdb0 = 2, ptb = 3 };

// gmc_v11_0_map_mtype: AMDGPU_VM_MTYPE_DEFAULT and NC give MTYPE_NC; values are
// the navi10_enum MTYPE codes placed in PTE bits 50:48.
enum class GpuVmMtype : uint32_t { nc = 0, wc = 1, cc = 2, uc = 3 };

// The mapping flags amdgpu_gem_va_map_flags would build (READABLE, WRITEABLE,
// EXECUTABLE, MTYPE). uncached: an AMDGPU_GEM_CREATE_UNCACHED/COHERENT/
// EXT_COHERENT buffer, which gmc_v11_0_get_vm_pte forces to MTYPE_UC.
struct GpuVmMapFlags final {
    memory::Access access;
    bool executable;
    GpuVmMtype mtype;
    bool uncached;
};

// amdgpu_pasid_alloc semantics over a bounded 16-bit id space: for bits =
// requested down to 1, the lowest free id of [2^(bits-1), 2^bits - 1]
// (ida_alloc_range), so the first PASID is 0x8000 and 0 is never returned.
// bits above gpuvm_pasid_bits are refused (Linux clamps to 31; amdgpu always
// asks for 16). Device-owned, serialized use; no delayed free: the caller frees
// only after GpuVm::fini succeeded (amdgpu_pasid_free_delayed waits for the
// root PD fences the same way).
class PasidAllocator final {
public:
    PasidAllocator() noexcept = default;
    GpuVmStatus allocate(uint32_t bits, uint32_t& pasid) noexcept;
    GpuVmStatus release(uint32_t pasid) noexcept;
    bool allocated(uint32_t pasid) const noexcept;

private:
    static constexpr uint32_t words = (1u << gpuvm_pasid_bits) / 64u;
    uint64_t used_[words]{};
};

// gmc_v11_0_emit_pasid_mapping for a non-MES kernel ring: IH_VMID_0_LUT (GFXHUB)
// or IH_VMID_0_LUT_MM (MMHUB) plus vmid, as an absolute dword offset. MES queues
// skip it (MES firmware maintains the LUT). False for vmid above 15.
bool gpuvm_pasid_mapping_register(const Ih6Registers& osssys, VmHub hub, uint32_t vmid,
                                  uint32_t& reg) noexcept;

// amdgpu_gmc_flush_gpu_tlb_pasid as it runs on GC 11 once MES is up: the KIQ ring
// is no longer schedulable (mes_v11_0_hw_init clears its sched.ready), so the
// call falls through to gmc_v11_0_flush_gpu_tlb_pasid; GC 11 has neither extra
// type-2 nor type-0 flushes. Serialized PASSIVE_LEVEL use; Gmc11 and the RegisterIo
// mapping outlive this object.
class GpuVmInvalidator final {
public:
    GpuVmInvalidator() noexcept = default;

    // firmware: the MES submission path (submit may be null before MES runs).
    // gfxoff_allowed: GFXOFF may power GC down (clock_gating enabled it); then a
    // GFXHUB register flush is refused and only the MES path is used, as Linux
    // does ("necessary ... for GFXOFF to function properly under bare metal").
    GpuVmStatus initialize(Gmc11& gmc, Ih6Registers osssys, RegisterIo io,
                           GmcFirmwareInvalidation firmware, bool gfxoff_allowed) noexcept;

    // gmc_v11_0_flush_gpu_tlb_pasid: for vmid 1..15 read IH_VMID_<vmid>_LUT
    // (PASID bits 15:0, zero is no mapping); on a match flush GFXHUB, or every hub
    // of vmhubs_mask (GFXHUB(0) then MMHUB0(0)) when all_hub, through
    // gmc_v11_0_flush_gpu_tlb (MES path when submit is set, else the register
    // path). No match is success (Linux returns 0). flushed_vmids gets the
    // matched VMID bit mask. Stops at the first failed read or flush.
    GpuVmStatus flush_pasid(uint32_t pasid, uint32_t flush_type, bool all_hub,
                            uint32_t& flushed_vmids) noexcept;

private:
    GpuVmStatus flush_hub(VmHub hub, uint32_t vmid, uint32_t flush_type) noexcept;

    Gmc11* gmc_ = nullptr;
    Ih6Registers osssys_{};
    RegisterIo io_{};
    GmcFirmwareInvalidation firmware_{};
    bool gfxoff_allowed_ = false;
    bool initialized_ = false;
};

struct GpuVmRecovery final {
    void* context;
    // True only when no engine can walk these tables any more: every MES queue of
    // this PASID was removed and invalidated, or a confirmed reset. Never
    // inferred from a failure.
    bool (*confirm_idle)(void*) noexcept;
};

// tables: one CPU-visible local-VRAM Manager allocation of table_count * 4 KiB,
// 4 KiB aligned, holding every page table of this VM (Linux allocates one
// VRAM_CONTIGUOUS BO per table; the PDE/PTE values are identical). table 0 is
// the root. table_count 1..gpuvm_max_tables.
struct GpuVmConfig final {
    VramLocation vram;
    memory::Handle tables;
    uint32_t table_count;
    GpuVmRecovery recovery;
};

struct GpuVmReport final {
    GpuVmStatus status = GpuVmStatus::unavailable;
    uint64_t root_mc = 0;
    // MESAPI ADD_QUEUE page_table_base_addr: vram_base_offset + root MC - vram_start.
    uint64_t page_table_base_addr = 0;
    // amdgpu_gmc_pd_addr: the same VRAM "PA" with AMDGPU_PTE_VALID.
    uint64_t pd_address = 0;
    uint32_t pasid = 0;
    uint32_t tables_live = 0;
    // Tables detached by a huge-page or unmap update, reusable after a TLB flush.
    uint32_t tables_retired = 0;
    uint32_t mappings = 0;
    // amdgpu_vm tlb_seq and kfd_last_flushed_seq.
    uint64_t tlb_seq = 0;
    uint64_t flushed_seq = 0;
    uint64_t entries_written = 0;
    bool memory_retained = false;
};

// One hardware-visible walk step: the entry index and value read at a level.
struct GpuVmWalkEntry final {
    GpuVmLevel level;
    uint32_t index;
    uint64_t value;
};

struct GpuVmWalk final {
    GpuVmWalkEntry entries[gpuvm_levels]{};
    uint32_t count = 0;
};

// One process VM (amdgpu_vm with use_cpu_for_update). Serialized PASSIVE_LEVEL
// use by the exclusive device owner; the Manager, Nbio and the pool mapping
// outlive it. Updates follow amdgpu_vm_update_range + amdgpu_vm_update_pdes:
// PTE/huge-PDE stores, HDP flush (amdgpu_vm_cpu_commit), then the PDEs of newly
// allocated tables, HDP flush. Every update is first run without stores to prove
// table capacity, so a refusal leaves the tables unchanged. A failed HDP flush
// quarantines. Mapped memory (VRAM ranges and DMA pages) stays owned and pinned
// by the caller until unmap plus a successful flush_tlb, or a confirmed idle fini.
class GpuVm final {
public:
    GpuVm() noexcept = default;
    GpuVm(const GpuVm&) = delete;
    GpuVm& operator=(const GpuVm&) = delete;

    // amdgpu_vm_init: retain the pool, clear the root (amdgpu_vm_pt_clear: every
    // PD entry AMDGPU_PDE_PTE, value 0x0040000000000000) and publish it.
    GpuVmStatus init(memory::Manager& manager, const GpuVmConfig& config, Nbio& nbio) noexcept;

    // amdgpu_vm_set_pasid: 0 clears. The PASID must come from PasidAllocator and is
    // what MES ADD_QUEUE carries and what flush_tlb looks up.
    GpuVmStatus set_pasid(uint32_t pasid) noexcept;

    // amdgpu_vm_bo_map + amdgpu_vm_bo_update of a VRAM buffer at MC address
    // vram_mc: PTE address vram_base_offset + (mc - vram_start), flags VALID |
    // READABLE/WRITEABLE per access | EXECUTABLE per flags | MTYPE (UC when
    // uncached), fragments from amdgpu_vm_pte_fragment and huge PDB0/PDB1 entries
    // (AMDGPU_PDE_PTE) when the range allows. Fragments are additionally limited
    // to the alignment shared by VA and physical address (Linux relies on its
    // buddy allocator for that). va, vram_mc and bytes are 4 KiB multiples; the
    // last pfn must be below max_pfn; overlapping an existing mapping is refused.
    GpuVmStatus map_vram(uint64_t va, uint64_t vram_mc, uint64_t bytes, GpuVmMapFlags flags) noexcept;

    // The same for a GTT buffer: dma_pages holds one little-endian 64-bit DMA bus
    // address per 4 KiB page (the TTM dma_address[] amdgpu_vm_bo_update maps).
    // Flags VALID | SYSTEM | SNOOPED (cpu_cached) | access | EXECUTABLE | MTYPE;
    // contiguous runs use fragments, other pages are written one by one
    // (amdgpu_vm_map_gart). dma_pages is caller-owned and only read here.
    GpuVmStatus map_system(uint64_t va, ByteView dma_pages, bool cpu_cached,
                           GpuVmMapFlags flags) noexcept;

    // amdgpu_vm_bo_unmap + amdgpu_vm_clear_freed of the mapping starting at va:
    // flags 0 with flush_tlb set, so tlb_seq advances. Tables covered by a whole
    // 2 MiB/1 GiB cleared entry are detached and retired until the next flush.
    GpuVmStatus unmap(uint64_t va) noexcept;

    // amdgpu_vm_flush_compute_tlb: nothing to do when tlb_seq equals the last
    // flushed sequence; otherwise flush_pasid(pasid, flush_type, all_hub false: GC
    // 11 is neither AMDGPU_FAMILY_AI nor RV). Success frees retired tables.
    // Linux KFD passes TLB_FLUSH_LEGACY (0) after map and TLB_FLUSH_HEAVYWEIGHT (2)
    // after unmap.
    bool tlb_flush_needed() const noexcept { return report_.tlb_seq != report_.flushed_seq; }
    GpuVmStatus flush_tlb(GpuVmInvalidator& invalidator, uint32_t flush_type) noexcept;

    // Read-only walk of the hardware-visible tables for va (diagnostics/tests):
    // stops at an invalid entry, a huge (PDE_PTE) entry or the PTB; refuses a PDE
    // that points outside this VM's live tables (invalid_memory).
    GpuVmStatus walk(uint64_t va, GpuVmWalk& output) const noexcept;

    // amdgpu_vm_fini: confirm_idle, pasid cleared, every table returned and the
    // pool lease released. Refused (retained) while an engine may walk the tables.
    GpuVmStatus fini() noexcept;

    // Serialized owner-only suspend image. All MES queues must already be
    // removed (confirm_idle); storage stays pinned and private until recovery.
    // Restore copies the complete retained table pool, publishes through HDP,
    // and forces a new PASID invalidation before any queue may be reattached.
    GpuVmStatus save_tables(sdma::CommandBuffer backup) noexcept;
    GpuVmStatus restore_tables(ByteView backup) noexcept;
    uint64_t recovery_bytes() const noexcept { return uint64_t{table_count_} * gpuvm_table_bytes; }

    uint64_t page_table_base_addr() const noexcept { return report_.page_table_base_addr; }
    uint64_t pd_address() const noexcept { return report_.pd_address; }
    GpuVmReport report() const noexcept { return report_; }

private:
    static constexpr uint16_t no_table = 0xffff;
    enum class TableState : uint8_t { free, live, freeing, retired };

    struct Table final {
        uint16_t parent = no_table;
        uint16_t parent_entry = 0;
        uint8_t level = 0;
        TableState state = TableState::free;
        bool relocated = false;
        uint32_t order = 0;
    };

    // amdgpu_vm_pt_cursor: entry is slot index of table parent, or the root
    // when parent is no_table.
    struct Cursor final {
        uint64_t pfn;
        uint32_t level;
        uint16_t parent;
        uint32_t index;
    };

    struct Mapping final {
        uint64_t start_pfn = 0;
        uint64_t last_pfn = 0;
        bool used = false;
    };

    // amdgpu_vm_update_params subset.
    struct Update final {
        ByteView pages;
        bool needs_flush;
    };

    GpuVmStatus finish(GpuVmStatus status) noexcept;
    uint64_t physical(uint64_t mc) const noexcept;
    uint64_t table_mc(uint16_t table) const noexcept;
    uint16_t find_child(uint16_t parent, uint32_t index) const noexcept;
    uint16_t entry_table(const Cursor& cursor) const noexcept;
    bool descendant(Cursor& cursor) const noexcept;
    bool sibling(Cursor& cursor) const noexcept;
    bool ancestor(Cursor& cursor) const noexcept;
    void next(Cursor& cursor) const noexcept;
    bool pt_alloc(const Cursor& cursor) noexcept;
    void add_list(const Cursor& cursor) noexcept;
    bool write_entry(uint16_t table, uint32_t index, uint64_t value) noexcept;
    bool update(Update& params, uint16_t table, uint32_t level, uint32_t index, uint64_t addr,
                uint32_t count, uint64_t increment, uint64_t flags) noexcept;
    bool ptes_update(Update& params, uint64_t start, uint64_t end, uint64_t dst, uint64_t flags,
                     uint32_t max_frag) noexcept;
    bool update_range(Update& params, uint64_t start, uint64_t pages, uint64_t flags,
                      uint64_t vram_address) noexcept;
    GpuVmStatus apply(uint64_t start, uint64_t pages, uint64_t flags, uint64_t vram_address,
                      ByteView dma_pages, bool flush) noexcept;
    bool commit(bool needs_flush) noexcept;
    bool update_pdes() noexcept;
    GpuVmStatus admit(uint64_t va, uint64_t bytes, uint32_t& slot) const noexcept;
    void count_tables() noexcept;

    GpuVmReport report_{};
    memory::Manager* manager_ = nullptr;
    memory::Handle pool_{};
    uint8_t* pool_cpu_ = nullptr;
    uint64_t pool_mc_ = 0;
    uint32_t table_count_ = 0;
    Nbio* nbio_ = nullptr;
    VramLocation vram_{};
    GpuVmRecovery recovery_{};
    Table tables_[gpuvm_max_tables]{};
    Table shadow_[gpuvm_max_tables]{};
    Mapping mappings_[gpuvm_max_mappings]{};
    uint32_t order_ = 0;
    bool dry_run_ = false;
    bool initialized_ = false;
    bool faulted_ = false;
};

} // namespace rx7900::hardware

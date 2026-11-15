/*
Module: rx7900.hardware.mes11
Purpose: Bring up the GC 11.0.0 MES KIQ and scheduler pipes, submit MES API frames, manage MES process queues and run the MES self-test queues.
Created: 2026-10-21
Architecture: Discovery-validated GC/MMHUB/OSSSYS token, RegisterIo for GC MMIO,
DoorbellIo for ring write pointers, Nbio for HDP publication, Manager leases on
local-VRAM rings, MQDs, EOP buffers and writeback; PM4 headers from pm4.hpp;
self-test user queues in GTT through the gfx11_queues builders and the
Mes11ProcessVm token of mes_selftest_vm; the process-queue API (doorbell page,
queue ids, MQD init, ADD/REMOVE_QUEUE, SUSPEND/RESUME) serves user_queue.
Copyright 2019 Advanced Micro Devices, Inc.; Copyright 2014 Advanced Micro Devices, Inc.;
Copyright 2008 Advanced Micro Devices, Inc., Copyright 2008 Red Hat Inc.,
Copyright 2009 Jerome Glisse.
Adapted Linux v6.12 mes_v11_0.c, amdgpu_mes.c, amdgpu_mes.h, mes_v11_api_def.h,
v11_structs.h, gfx_v11_0.c (KIQ PM4 functions), nvd.h, amdgpu_ring.c,
amdgpu_ring.h, amdgpu_fence.c, amdgpu_doorbell.h, amdgpu_doorbell_mgr.c, soc21.c,
amdgpu_gfx.c and gc_11_0_0 register headers; for amdgpu_mes_self_test also
amdgpu_mes_ctx.h (with v10_structs.h), amdgpu_ib.c (amdgpu_ib_schedule),
sdma_v6_0.c (mqd_init, ring/IB tests, emit functions), sdma_v6_0_0_pkt_open.h,
amdgpu_ids.c (amdgpu_pasid_alloc), amdgpu_gfx.h and gfxhub_v3_0.c (vm_cntx_cntl).
Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#pragma once

#include "gmc11.hpp"
#include "nbio.hpp"

namespace rx7900::hardware {

// soc15_hw_ip.h OSSSYS_HWID (also named by ih6.hpp, which this module does not include).
constexpr uint16_t mes11_osssys_hardware_id = 40;

// soc21_init_doorbell_index: AMDGPU_NAVI10_DOORBELL_MES_RING0/1 (0x00b/0x00c) in
// 64-bit units, shifted to dword doorbell indices as mes_v11_0_ring_init and
// mes_v11_0_kiq_ring_init do.
constexpr uint32_t mes11_scheduler_doorbell_index = 0x00bu << 1;
constexpr uint32_t mes11_kiq_doorbell_index = 0x00cu << 1;

// amdgpu_ring_init: MES rings use sched_hw_submission 8, KIQ rings at least 256;
// both pass max_dw 1024, so ring_size = roundup_pow_of_two(1024 * 4 * n).
constexpr uint32_t mes11_scheduler_ring_bytes = 1024u * 4u * 8u;
constexpr uint32_t mes11_kiq_ring_bytes = 1024u * 4u * 256u;
// mes_v11_0.c MES_EOP_SIZE and sizeof(struct v11_compute_mqd) (measured: 2048).
constexpr uint32_t mes11_eop_bytes = 2048;
constexpr uint32_t mes11_mqd_bytes = 2048;
// One 4 KiB page of 256-bit amdgpu_device_wb_get slots (see Mes11Memory).
constexpr uint32_t mes11_writeback_bytes = 4096;
constexpr uint32_t mes11_page_alignment = 4096;
// mes_v11_api_def.h API_FRAME_SIZE_IN_DWORDS.
constexpr uint32_t mes11_api_frame_bytes = 64u * 4u;
// mes_v11_0_kiq_hw_init: scheduler firmware version (bits 11:0) from which
// adev->mes.enable_legacy_queue_map is set and hw_init runs inside kiq_hw_init.
constexpr uint32_t mes11_legacy_queue_map_version = 0x47;

enum class Mes11Status {
    ready,
    kiq_ready,
    scheduler_ready,
    completed,
    disabled,
    released,
    unavailable,
    unsupported_ip,
    unsupported_firmware,
    invalid_argument,
    invalid_memory,
    mmio_failure,
    doorbell_failure,
    ring_test_failed,
    timeout,
    busy,
    quarantined
};

// Only validate_mes11_discovery creates a valid token: unique GC instance0
// 11.0.0 with at least the two segments the MES/HQD registers use (BASE_IDX 0
// and 1), every used register inside BAR5, plus unique MMHUB 3.0.0 and OSSSYS
// 6.0.0. For MESAPI_SET_HW_RESOURCES the actual discovered base counts (up to
// five, as Linux copies) are probed; absent indices are zero.
class Mes11Registers final {
public:
    static constexpr size_t hw_resource_bases = 5;

    Mes11Registers() noexcept = default;
    bool valid() const noexcept { return gc_.valid() && aperture_bytes_ != 0; }
    uint32_t gc_segment(size_t index) const noexcept { return gc_.segment(index); }
    uint32_t gc_base(size_t index) const noexcept { return pick(gc_bases_, index); }
    uint32_t mmhub_base(size_t index) const noexcept { return pick(mmhub_bases_, index); }
    uint32_t osssys_base(size_t index) const noexcept { return pick(osssys_bases_, index); }
    uint32_t aperture_bytes() const noexcept { return aperture_bytes_; }

private:
    friend DiscoveryStatus validate_mes11_discovery(ByteView, uint32_t, Mes11Registers&) noexcept;
    static uint32_t pick(const uint32_t (&bases)[hw_resource_bases], size_t index) noexcept {
        return index < hw_resource_bases ? bases[index] : 0;
    }

    DiscoveredIp gc_{};
    uint32_t gc_bases_[hw_resource_bases]{};
    uint32_t mmhub_bases_[hw_resource_bases]{};
    uint32_t osssys_bases_[hw_resource_bases]{};
    uint32_t aperture_bytes_ = 0;
};

DiscoveryStatus validate_mes11_discovery(ByteView binary, uint32_t register_aperture_bytes,
                                         Mes11Registers& output) noexcept;

// amdgpu_mes_init_microcode: mes_firmware_header_v1_0 mes_uc_start_addr_lo/hi
// (byte offsets 56/60 after the 32-byte common header) of a MES file whose
// header version is 1.x. The value is a byte address; mes_v11_0_enable programs
// it shifted right by two. Output unchanged on failure.
bool mes11_uc_start_address(ByteView mes_file, uint64_t& address) noexcept;

struct Mes11Config final {
    // mes_uc_start_addr of the scheduler (gc_11_0_0_mes_2.bin or mes.bin) and
    // KIQ (gc_11_0_0_mes1.bin) images that PSP loaded.
    uint64_t scheduler_uc_start;
    uint64_t kiq_uc_start;
    // adev->sdma.num_instances for amdgpu_mes_init sdma_hqd_mask; Navi31 has 2.
    uint32_t sdma_instances;
};

struct Mes11Policy final {
    // adev->usec_timeout (Linux 100000) for 1 us HQD dequeue and ring-test polls.
    uint32_t poll_iterations;
    // mes_v11_0_submit_pkt_and_poll_completion timeout in microseconds, polled
    // in 2 us steps (Linux 2100000 on bare metal).
    uint32_t api_timeout_microseconds;
};

struct Mes11Recovery final {
    void* context;
    // True only after a reset proves MES/KIQ/CP no longer access any retained
    // buffer. Never inferred from a failed or timed-out access.
    bool (*confirm_reset_quiescence)(void*) noexcept;
};

// Seven separate Manager allocations, all CPU-visible local VRAM, 4 KiB aligned:
// rings of mes11_kiq_ring_bytes and mes11_scheduler_ring_bytes, MQDs of
// mes11_mqd_bytes, EOP buffers of mes11_eop_bytes, one writeback page. The
// writeback page holds 32-byte slots: 0-4 KIQ rptr/wptr/fence/trail/cond_exec,
// 5-9 the same for the scheduler ring, 10/11 sch_ctx[pipe], 12/13
// query_status_fence[pipe], 14 read_val and 15 the API completion status.
struct Mes11Memory final {
    memory::Handle kiq_ring;
    memory::Handle scheduler_ring;
    memory::Handle kiq_mqd;
    memory::Handle scheduler_mqd;
    memory::Handle kiq_eop;
    memory::Handle scheduler_eop;
    memory::Handle writeback;
};

enum class Mes11QueueType : uint32_t { gfx = 0, compute = 1, sdma = 2 };

// amdgpu_mes_map_legacy_queue input for a kernel ring owned by the caller.
// mqd_address and wptr_address are GPU addresses in the VMID0 space (MC for
// VRAM, GART for system memory); doorbell_index is the ring's dword doorbell.
struct Mes11LegacyQueue final {
    Mes11QueueType type;
    uint32_t doorbell_index;
    uint32_t pipe;
    uint32_t queue;
    uint64_t mqd_address;
    uint64_t wptr_address;
};

enum class Mes11UnmapAction { unmap, preempt_no_unmap };

// amdgpu_mes_unmap_legacy_queue input. trail_fence_* are used only for
// preempt_no_unmap (PREEMPT_QUEUES_NO_UNMAP) and only the low 32 data bits.
struct Mes11LegacyUnmap final {
    Mes11LegacyQueue queue;
    Mes11UnmapAction action;
    uint64_t trail_fence_address;
    uint64_t trail_fence_data;
};

struct Mes11Report final {
    Mes11Status status = Mes11Status::unavailable;
    uint32_t scheduler_version = 0;
    uint32_t kiq_version = 0;
    bool legacy_queue_map = false;
    bool kiq_ring_ready = false;
    bool scheduler_ready = false;
    bool mes_enabled = false;
    bool memory_retained = false;
    uint64_t kiq_ring_mc = 0;
    uint64_t scheduler_ring_mc = 0;
    uint64_t kiq_mqd_mc = 0;
    uint64_t scheduler_mqd_mc = 0;
    uint64_t writeback_mc = 0;
    uint64_t kiq_wptr = 0;
    uint64_t scheduler_wptr = 0;
    uint32_t fence_sequence = 0;
    uint32_t api_submissions = 0;
    uint32_t last_api_opcode = 0;
    uint32_t aggregated_doorbell_base = 0;
    bool dequeue_timeout = false;
    // Process queues added through add_hw_queue and not yet removed, MES
    // doorbells and queue ids currently allocated (self-test included).
    uint32_t process_queues = 0;
    uint32_t doorbells_allocated = 0;
    uint32_t queue_ids_allocated = 0;
    // suspend_all/resume_all: firmware supports them (version >= 0x63) and the
    // last SUSPEND(suspend_all_gangs) completed without a later RESUME.
    bool suspend_resume_supported = false;
    bool gangs_suspended = false;
};

// amdgpu_mes_self_test (Linux v6.12, called from mes_v11_0_late_init on bare
// metal for every GC 11 except 11.0.3; never on reset, suspend or s0ix resume).
//
// amdgpu_pasid_alloc(16) takes the lowest free PASID of [0x8000, 0xffff]; at
// late_init no other PASID exists, so the process id is 0x8000.
constexpr uint32_t mes11_self_test_pasid = 0x8000;
// AMDGPU_MES_PROC_CTX_SIZE and AMDGPU_MES_GANG_CTX_SIZE (one page each).
constexpr uint32_t mes11_process_context_bytes = 0x1000;
constexpr uint32_t mes11_gang_context_bytes = 0x1000;
// adev->mqds[].mqd_size: sizeof v11_gfx_mqd and v11_compute_mqd (2048, as
// mes11_mqd_bytes) and v11_sdma_mqd (512), measured from the pinned v11_structs.h.
constexpr uint32_t mes11_sdma_mqd_bytes = 512;
// queue_types[] of amdgpu_mes_self_test: one gfx, one compute and (GC 11 is not
// below 11.0.0, so not skipped) one SDMA queue, each in its own gang.
constexpr uint32_t mes11_self_test_queues = 3;
// amdgpu_mes_test_queues: amdgpu_ring_test_ib(ring, 1000 * 10) passes 10000
// jiffies: 10 s with CONFIG_HZ=1000 (the default here), 40 s with HZ=250 (the
// largest budget accepted).
constexpr uint32_t mes11_self_test_ib_timeout_us = 10000000;
constexpr uint32_t mes11_self_test_ib_timeout_max_us = 40000000;
// amdgpu_mes_ctx_map_meta_data at AMDGPU_VA_RESERVED_BOTTOM (1 << 16).
constexpr uint64_t mes11_self_test_metadata_va = 1ULL << 16;
// amdgpu_vm_adjust_size(256 TiB, 9, 3, 48): max_pfn 2^36, so ADD_QUEUE
// process_va_end = (max_pfn - 1) << AMDGPU_GPU_PAGE_SHIFT.
constexpr uint64_t mes11_self_test_vm_max_pfn = 1ULL << 36;

// struct amdgpu_mes_ctx_meta_data (amdgpu_mes_ctx.h with v10_structs.h, PAGE_SIZE
// 4096) byte offsets, measured by compiling the pinned headers with offsetof.
// Only index 0 of each engine is used (amdgpu_mes_add_ring idx j = 0).
namespace mes11_ctx {
constexpr uint32_t bytes = 212992; // 52 pages
constexpr uint32_t gfx_ring = 0;
constexpr uint32_t gfx_slots = 86016;
constexpr uint32_t gfx_ib = 86272;
constexpr uint32_t gfx_padding = 87296;
constexpr uint32_t compute_ring = 90112;
constexpr uint32_t compute_mec_hpd = 106496;
constexpr uint32_t compute_slots = 108544;
constexpr uint32_t compute_ib = 108800;
constexpr uint32_t compute_padding = 109824;
constexpr uint32_t sdma_ring = 172032;
constexpr uint32_t sdma_slots = 188480;
constexpr uint32_t sdma_ib = 188672;
constexpr uint32_t sdma_padding = 189696;
// AMDGPU_MES_CTX_RPTR/WPTR/FENCE/COND_EXE/TRAIL_FENCE_OFFS of struct amdgpu_wb_slot.
constexpr uint32_t slot_bytes = 32;
constexpr uint32_t slot_rptr = 0;
constexpr uint32_t slot_wptr = 1;
constexpr uint32_t slot_fence = 2;
constexpr uint32_t slot_cond_exe = 3;
// ring[PAGE_SIZE * 4]: amdgpu_ring_init(max_dw 1024) with amdgpu_sched_hw_submission
// 2 gives 8192-byte rings for all three engines; ib[256] dwords per engine.
constexpr uint32_t ring_bytes = 8192;
constexpr uint32_t ib_bytes = 1024;
} // namespace mes11_ctx

// MES process queues (amdgpu_mes_add_hw_queue and friends, shared by the
// self-test and the user_queue module).
//
// amdgpu_mes_doorbell_init: the MES doorbell page holds num_mes_dbs = PAGE_SIZE /
// AMDGPU_ONE_DOORBELL_SIZE 8-byte doorbells; the first AMDGPU_MES_PRIORITY_NUM_LEVELS
// bits are the aggregated doorbells. Bit b is dword index db_start_dw_offset + 2b.
constexpr uint32_t mes11_doorbell_slots = 4096 / 8;
constexpr uint32_t mes11_priority_levels = 5;
// MES queue ids (adev->mes.queue_id_idr, idr_alloc(.., 1, 0)): lowest free id
// from 1. They travel in RELEASE_MEM/TRAP as hw_queue_id | AMDGPU_FENCE_MES_QUEUE_FLAG
// and are bounded here to the doorbell count (one doorbell per queue).
constexpr uint32_t mes11_max_queue_ids = mes11_doorbell_slots;
// amdgpu_mes_suspend_resume_all_supported: GC 11 with scheduler firmware
// (sched_version & AMDGPU_MES_VERSION_MASK) >= 0x63; below it amdgpu_mes_suspend
// and amdgpu_mes_resume return 0 without a frame.
constexpr uint32_t mes11_suspend_resume_version = 0x63;
// amdgpu_mes.h AMDGPU_MES_PRIORITY_LEVEL_NORMAL.
constexpr uint32_t mes11_priority_normal = 1;

// mes_add_queue_input as amdgpu_mes_add_hw_queue fills it (is_kfd_process 0, no
// gws/trap/AQL, paging false, queue_size 0 so gds_size 0). Addresses: the
// context and MQD buffers are VMID0 GART addresses (AMDGPU_GEM_DOMAIN_GTT kernel
// BOs, page aligned); wptr_va is the process VA of the wptr shadow and wptr_mc
// its VMID0 address (ADD_QUEUE carries wptr_mc from MES API version 2 on).
// doorbell_index must have come from allocate_doorbell. vm_context_cntl is read
// by add_hw_queue itself (see self_test).
struct Mes11AddQueue final {
    Mes11QueueType type;
    uint32_t pasid;
    uint64_t page_table_base_addr;
    uint64_t process_va_end;
    uint64_t process_quantum;
    uint64_t process_context_address;
    uint64_t gang_quantum;
    uint64_t gang_context_address;
    uint32_t inprocess_gang_priority;
    uint32_t gang_global_priority_level;
    uint32_t doorbell_index;
    uint64_t mqd_address;
    uint64_t wptr_va;
    uint64_t wptr_mc;
};

// mes_remove_queue_input of amdgpu_mes_remove_hw_queue.
struct Mes11RemoveQueue final {
    uint32_t doorbell_index;
    uint64_t gang_context_address;
};

struct Gfx11MqdProperties;
struct Gfx11RingWriter;

// sdma_v6_0_mqd_init into a mes11_sdma_mqd_bytes image (the BO memset first):
// RB_CNTL from queue_bytes with RPTR writeback and WPTR polling, ring base,
// rptr/wptr-poll addresses, ib_control (SDMA0_QUEUE0_IB_CNTL as read by the
// caller), doorbell offset/enable and the AQL/DUMMY register defaults. Only
// hqd_base_address, rptr_address, wptr_address, queue_bytes and doorbell_index
// are used. False on a store/bounds failure.
bool mes11_sdma_mqd_init(uint32_t ib_control, const Gfx11MqdProperties& properties,
                         sdma::CommandBuffer mqd) noexcept;

// One amdgpu_ib_schedule(ring, 1, &ib, NULL, &f) frame on a MES SDMA queue
// (sdma_v6_0 ring funcs, no burst NOPs): COND_EXE on cond_exec_address (count
// patched after the fence), POLL_REGMEM HDP flush on the NBIO DONE/REQ dword
// registers with ref_and_mask_sdma0, NOPs so the 6-dword INDIRECT ends on an
// 8-dword boundary, INDIRECT vmid 0 (ib_address & ~0x1f, ib_dwords), the 32-bit
// FENCE of fence_sequence and the TRAP carrying hw_queue_id |
// AMDGPU_FENCE_MES_QUEUE_FLAG. No amdgpu_ring_commit padding.
struct Mes11SdmaIbFrame final {
    uint64_t cond_exec_address;
    uint32_t hdp_flush_req;
    uint32_t hdp_flush_done;
    uint64_t ib_address;
    uint32_t ib_dwords;
    uint64_t fence_address;
    uint32_t fence_sequence;
    uint32_t hw_queue_id;
};

bool mes11_sdma_emit_ib_frame(Gfx11RingWriter& writer, const Mes11SdmaIbFrame& frame) noexcept;
// amdgpu_ring_commit padding for SDMA: plain NOPs up to align_mask 0xf + 1.
bool mes11_sdma_ring_pad(Gfx11RingWriter& writer) noexcept;

class MesSelfTestVm;

// The process GPUVM the self-test queues run in. Only MesSelfTestVm::map
// creates a valid token, after it built the page tables mapping the CPU-visible
// GTT metadata buffer at mes11_self_test_metadata_va. The metadata pointer is a
// non-owning view: it stays valid only while that MesSelfTestVm holds its lease.
class Mes11ProcessVm final {
public:
    Mes11ProcessVm() noexcept = default;
    bool valid() const noexcept { return metadata_cpu_ != nullptr; }
    // ADD_QUEUE page_table_base_addr: vram_base_offset + root PD MC - vram_start.
    uint64_t page_table_base_addr() const noexcept { return page_table_base_addr_; }
    uint64_t metadata_va() const noexcept { return metadata_va_; }
    // VMID0 GART address of the metadata buffer (ctx_data->meta_data_mc_addr).
    uint64_t metadata_gart() const noexcept { return metadata_gart_; }
    uint8_t* metadata_cpu() const noexcept { return metadata_cpu_; }

private:
    friend class MesSelfTestVm;
    uint64_t page_table_base_addr_ = 0;
    uint64_t metadata_va_ = 0;
    uint64_t metadata_gart_ = 0;
    uint8_t* metadata_cpu_ = nullptr;
};

// Separate CPU-visible GTT allocations (GPU addresses inside config.gart, 4 KiB
// aligned), as amdgpu_bo_create_kernel(AMDGPU_GEM_DOMAIN_GTT) gives Linux: the
// process context (mes11_process_context_bytes), one gang context per queue
// (mes11_gang_context_bytes) and one MQD per queue (gfx and compute
// mes11_mqd_bytes, SDMA mes11_sdma_mqd_bytes). Index order: gfx, compute, SDMA.
struct Mes11SelfTestMemory final {
    memory::Handle process_context;
    memory::Handle gang_context[mes11_self_test_queues];
    memory::Handle mqd[mes11_self_test_queues];
};

// nbio: the Nbio token, for the BIF_BX_PF0_GPU_HDP_FLUSH_REQ/DONE registers the
// gfx/compute WAIT_REG_MEM and SDMA POLL_REGMEM HDP flushes use. gart: the VMID0
// GART aperture (Gmc11::gart()). ib_timeout_microseconds: 1 us fence polls per IB
// test, 1..mes11_self_test_ib_timeout_max_us (Linux: mes11_self_test_ib_timeout_us).
struct Mes11SelfTestConfig final {
    NbioRegisters nbio;
    GartLayout gart;
    uint32_t ib_timeout_microseconds;
};

enum class Mes11SelfTestResult : uint32_t { not_run, passed, timeout, failed };

struct Mes11SelfTestQueue final {
    Mes11QueueType type = Mes11QueueType::gfx;
    // amdgpu_mes_add_gang/add_hw_queue idr ids and the MES doorbell dword index
    // (db_start_dw_offset + 2 * bit of the MES doorbell page).
    uint32_t gang_id = 0;
    uint32_t queue_id = 0;
    uint32_t doorbell_index = 0;
    bool added = false;
    Mes11Status add_status = Mes11Status::unavailable;
    uint64_t gang_context_address = 0;
    uint64_t mqd_address = 0;
    // ADD_QUEUE wptr_addr (VMID0 GART address with MES API >= 2, else the VA).
    uint64_t frame_wptr_address = 0;
    uint64_t ring_va = 0;
    uint64_t fence_va = 0;
    // Driver write pointer in dwords; SDMA publishes wptr << 2 bytes.
    uint64_t wptr = 0;
    uint32_t fence_seq = 0;
    uint32_t fence_value = 0;
    Mes11SelfTestResult ring_test = Mes11SelfTestResult::not_run;
    Mes11SelfTestResult ib_test = Mes11SelfTestResult::not_run;
    uint32_t ib_polls = 0;
    bool removed = false;
    Mes11Status remove_status = Mes11Status::unavailable;
};

// status: completed (every ring and IB test passed, every queue removed),
// ring_test_failed (a test failed or timed out; testing stopped there as Linux
// returns), or the refusal/fault reason. A diagnostic only: mes_v11_0_late_init
// ignores amdgpu_mes_self_test's result and bring-up continues either way.
// memory_quarantined: an add/remove frame did not complete, a test failed or an
// MMIO access failed, so the self-test GTT buffers stay retained here (and the
// VM/metadata in MesSelfTestVm) until release_after_reset.
struct Mes11SelfTestReport final {
    Mes11Status status = Mes11Status::unavailable;
    bool ran = false;
    uint32_t pasid = 0;
    uint64_t page_table_base_addr = 0;
    uint64_t process_va_end = 0;
    uint64_t process_context_address = 0;
    // GCVM_CONTEXT15_CNTL as read before the first ADD_QUEUE (see self_test).
    uint32_t vm_context_cntl = 0;
    // (scheduler_version & AMDGPU_MES_API_VERSION_MASK) >> 12.
    uint32_t api_version = 0;
    uint32_t queues_added = 0;
    uint32_t rings_passed = 0;
    uint32_t ibs_passed = 0;
    // First queue index whose ring or IB test failed, or UINT32_MAX.
    uint32_t failed_queue = UINT32_MAX;
    bool memory_retained = false;
    bool memory_quarantined = false;
    Mes11SelfTestQueue queues[mes11_self_test_queues]{};
};

// Serialized PASSIVE_LEVEL use by the exclusive device owner. GRBM_GFX_CNTL
// (soc21_grbm_select, Linux srbm_mutex) is shared GC state: no other module may
// select a ME/pipe/queue while any call here runs, and every call restores the
// broadcast selection 0. API submissions busy-wait up to the policy timeout.
//
// Prerequisites (gfx_v11_0_hw_init before gfx_v11_0_cp_resume): PSP loaded the
// MES scheduler and KIQ firmware (CP_MES/MES_STACK, CP_MES_KIQ/MES_KIQ_STACK),
// RLC autoload completed, GFXHUB/GART enabled (VMID0 reaches local VRAM), the
// HDP remap and doorbell aperture enabled (Nbio::program_common), the GC
// doorbell S2A routing (Nbio::gc_doorbell_init) and gfx_v11_0_cp_set_doorbell_range
// done by the GFX core. No doorbell aperture means refusal: Linux has no
// non-doorbell MES path (mes_v11_0_ring_set_wptr calls BUG()).
//
// Linux allocates the rings, EOP buffers and adev->wb in GTT and the MQDs in
// VRAM or GTT. Here everything is CPU-mapped uncached local VRAM addressed by MC
// address through VMID0, so CPU stores are published by an HDP flush before
// every doorbell (Linux needs only a CPU barrier for coherent GTT). HDP6 has no
// read invalidate; device-written fences are read through the same uncached
// mapping. After the first MES register write every lease stays retained until
// kiq_hw_fini halted MES (then release) or a confirmed reset (release_after_reset).
class Mes11 final {
public:
    Mes11() noexcept = default;
    Mes11(const Mes11&) = delete;
    Mes11& operator=(const Mes11&) = delete;

    // DoorbellIo must cover both MES doorbells; the doorbell BAR size also fixes
    // the aggregated doorbell indices (amdgpu_doorbell_create_kernel_doorbells).
    Mes11Status initialize(Mes11Registers registers, RegisterIo io, DoorbellIo doorbells,
                           Nbio& nbio, VramLocation vram, Mes11Config config,
                           Mes11Policy policy, Mes11Recovery recovery) noexcept;

    // mes_v11_0_kiq_hw_init (PSP load type): retains and prepares memory as
    // mes_v11_0_sw_init/amdgpu_mes_init/amdgpu_ring_init would, then
    // mes_v11_0_enable, get_fw_version, kiq_setting and the register-programmed
    // KIQ queue. With scheduler firmware >= 0x47 it continues with hw_init and
    // returns scheduler_ready; otherwise kiq_ready and the MES IP hw_init must
    // be called later. Any failure after the first register write quarantines.
    Mes11Status kiq_hw_init(memory::Manager& manager, const Mes11Memory& memory) noexcept;

    // mes_v11_0_hw_init with enable_mes_kiq: scheduler MQD, KIQ MAP_QUEUES plus
    // KIQ ring test, MESAPI SET_HW_RESOURCES and QUERY_SCHEDULER_STATUS. A
    // second call after success returns scheduler_ready without hardware access.
    Mes11Status hw_init() noexcept;
    // amdgpu_mes_reg_write_reg_wait and the other MESAPI_MISC users: submit one
    // caller-encoded MISC frame (header opcode MES_SCH_API_MISC) while the
    // scheduler runs; api_status_offset is where the frame carries api_status.
    // Any other opcode is refused (no general frame injection).
    Mes11Status misc_op(ByteView frame, uint32_t api_status_offset) noexcept;

    // mes_v11_0_query_sched_status.
    Mes11Status query_sched_status() noexcept;

    // mes_v11_0_map_legacy_queue (amdgpu_gfx_mes_enable_kcq / enable_kgq).
    // Refused (unsupported_firmware) when the scheduler version is below 0x47:
    // Linux then maps kernel queues through the KIQ instead.
    Mes11Status map_legacy_queue(const Mes11LegacyQueue& queue) noexcept;
    // mes_v11_0_unmap_legacy_queue (amdgpu_gfx_disable_kcq / kgq with MES).
    Mes11Status unmap_legacy_queue(const Mes11LegacyUnmap& request) noexcept;

    // amdgpu_gfx_mes_enable_kcq first step: HDP flush, gfx11_kiq_set_resources
    // on the KIQ ring and its ring test. Linux only logs a failure here and
    // continues; the caller receives the status and decides.
    Mes11Status kiq_set_resources(uint64_t queue_mask) noexcept;

    // mes_v11_0_kiq_hw_fini on bare metal: dequeue the scheduler HQD when it
    // was ready, then mes_v11_0_enable(false) (halt, reset both pipes).
    Mes11Status kiq_hw_fini() noexcept;
    // mes_v11_0_hw_fini: frees only the SR-IOV resource_1 buffer, so it has no
    // bare-metal effect; it returns the current state.
    Mes11Status hw_fini() noexcept;

    // amdgpu_mes_self_test's MES part, from scheduler_ready, once per bring-up,
    // with the process VM already built (amdgpu_vm_init and
    // amdgpu_mes_ctx_map_meta_data, see MesSelfTestVm) and before MesSelfTestVm
    // tears it down. In Linux order: amdgpu_mes_create_process (zeroed process
    // context; v6.12 MES11 has no add-process API, the process fields travel in
    // every ADD_QUEUE), then for gfx, compute and SDMA: amdgpu_mes_add_gang
    // (zeroed gang context), amdgpu_mes_add_ring (ring in the metadata cleared
    // with the engine NOP, cond_exec 1, fence 0) and amdgpu_mes_add_hw_queue
    // (zeroed MQD, queue id, MES doorbell, the engine's mqd_init with
    // amdgpu_mqd_prop, MESAPI ADD_QUEUE); then amdgpu_mes_test_queues (ring
    // test and IB test of each added queue, stopping at the first failure), then
    // REMOVE_QUEUE of every added queue, amdgpu_mes_remove_gang and
    // amdgpu_mes_destroy_process (no further frames). A failed add continues with
    // the next engine as Linux does.
    //
    // ADD_QUEUE carries pasid 0x8000, vm.page_table_base_addr(), VA range
    // 0..process_va_end, quanta 0 (default_process/gang_quantum are never set in
    // v6.12), normal priorities, the process/gang context GART addresses and
    // vm_context_cntl. Linux sends its cached hub->vm_cntx_cntl (the last
    // GCVM_CONTEXT1..15_CNTL value gfxhub_v3_0_setup_vmid_config wrote); this
    // reads GCVM_CONTEXT15_CNTL instead and refuses before any memory use unless
    // ENABLE_CONTEXT=1, PAGE_TABLE_DEPTH=3 and PAGE_TABLE_BLOCK_SIZE=0.
    //
    // No VM TLB invalidation is issued: Linux has none for this VM either (no job
    // runs amdgpu_vm_flush; MES assigns and invalidates the VMID). Fences are
    // polled instead of signalled by the EOP/trap interrupt. User-queue commits
    // store the wptr shadow then ring the doorbell without an HDP flush (GTT,
    // Linux mb()). Returns the report status; never a reason to fail bring-up.
    Mes11Status self_test(memory::Manager& gtt_manager, const Mes11SelfTestMemory& memory,
                          const Mes11ProcessVm& vm, const Mes11SelfTestConfig& config) noexcept;
    Mes11SelfTestReport self_test_report() const noexcept { return self_test_; }

    // amdgpu_mes_kernel_doorbell_get: the lowest free MES doorbell bit at or above
    // 0 (gfx/compute) or adev->doorbell_index.sdma_engine[0] (SDMA); the result is
    // the absolute dword index aggregated_doorbell_base + 2 * bit. doorbell_failure
    // when none is free (Linux -ENOSPC). Needs initialize; no hardware access.
    Mes11Status allocate_doorbell(Mes11QueueType type, uint32_t& doorbell_index) noexcept;
    // amdgpu_mes_kernel_doorbell_free; invalid_argument for an unallocated index.
    Mes11Status free_doorbell(uint32_t doorbell_index) noexcept;
    bool doorbell_allocated(uint32_t doorbell_index) const noexcept;
    // queue_id_idr: lowest free id from 1 (busy when exhausted) and its removal.
    Mes11Status allocate_queue_id(uint32_t& queue_id) noexcept;
    // Reserve a retained public queue identity after confirmed power recovery.
    // Does not overwrite another owner; id zero and occupied ids are refused.
    Mes11Status reserve_queue_id(uint32_t queue_id) noexcept;
    Mes11Status free_queue_id(uint32_t queue_id) noexcept;

    // (scheduler_version & AMDGPU_MES_API_VERSION_MASK) >> 12.
    uint32_t api_version() const noexcept;

    // amdgpu_mes_queue_init_mqd: adev->mqds[type].init_mqd into mqd (gfx/compute
    // gfx11_mqd_bytes, SDMA mes11_sdma_mqd_bytes) with the properties MES queues
    // use (use_doorbell, hqd_active false). Gfx and compute run under
    // amdgpu_gfx_select_me_pipe_q(ring me, pipe, 0, 0, 0) of gfx_ring[0] (ME0
    // pipe 0) and compute_ring[0] (MEC1 = ME1 pipe 0), then selection 0; SDMA
    // reads SDMA0_QUEUE0_IB_CNTL. Scheduler phase only; an MMIO failure
    // quarantines, a short buffer is invalid_argument.
    Mes11Status init_queue_mqd(Mes11QueueType type, const Gfx11MqdProperties& properties,
                               sdma::CommandBuffer mqd) noexcept;

    // mes_v11_0_add_hw_queue: MESAPI ADD_QUEUE with the process and gang fields,
    // vm_context_cntl from GCVM_CONTEXT15_CNTL (validated as self_test does:
    // unsupported_ip without a frame), wptr_addr per the MES API version, gds_size
    // 0, flags 0. completed means MES accepted the queue (report process_queues
    // counts it); a frame timeout quarantines as every API submission does.
    Mes11Status add_hw_queue(const Mes11AddQueue& queue) noexcept;
    // mes_v11_0_remove_hw_queue: REMOVE_QUEUE with the doorbell and gang
    // context only. Only for a queue add_hw_queue accepted.
    Mes11Status remove_hw_queue(const Mes11RemoveQueue& queue) noexcept;

    // amdgpu_mes_suspend / amdgpu_mes_resume (mes_v11_0_suspend before hw_fini,
    // mes_v11_0_resume after hw_init): MESAPI SUSPEND with suspend_all_gangs = 1
    // (gang context, fence address/value 0) and RESUME with resume_all_gangs = 1.
    // With firmware below mes11_suspend_resume_version both return completed
    // without a frame (Linux returns 0); report suspend_resume_supported tells
    // which. Scheduler phase only. A timeout quarantines (Linux only logs it).
    Mes11Status suspend_all() noexcept;
    Mes11Status resume_all() noexcept;

    // amdgpu_ring_fini/mes_v11_0_sw_fini after a successful kiq_hw_fini.
    // Refused (quarantined) while quarantined self-test buffers are retained,
    // busy while process queues are still registered (remove them first).
    Mes11Status release() noexcept;
    // After confirmed reset quiescence from any state; also returns the
    // self-test buffers and forgets every process queue, doorbell and queue id
    // (the reset removed them from MES; their owners release their own leases).
    Mes11Status release_after_reset() noexcept;

    Mes11Report report() const noexcept { return report_; }

private:
    enum class Phase { uninitialized, ready, kiq, scheduler, halted, faulted };

    // A self-test user queue in the metadata buffer.
    struct UserQueue final {
        uint8_t* mqd_cpu;
        uint32_t ring_offset;
        uint32_t slots_offset;
        uint32_t ib_offset;
        uint32_t padding_offset;
    };

    struct Ring final {
        uint8_t* cpu;
        uint64_t mc;
        uint32_t bytes;
        uint32_t nop;
        uint32_t align_mask;
        uint32_t doorbell;
        uint32_t pipe;
        uint32_t slot_base;
        uint64_t wptr;
        uint64_t mqd_mc;
        uint8_t* mqd_cpu;
        uint64_t eop_mc;
        uint8_t* eop_cpu;
    };

    bool read(uint32_t segment, uint32_t reg, uint32_t& value) noexcept;
    bool write(uint32_t segment, uint32_t reg, uint32_t value) noexcept;
    bool grbm_select(uint32_t me, uint32_t pipe) noexcept;
    bool enable(bool on) noexcept;
    bool read_versions() noexcept;
    bool kiq_setting() noexcept;
    bool mqd_init(Ring& ring) noexcept;
    bool queue_init_register(const Ring& ring) noexcept;
    bool kiq_dequeue(const Ring& ring) noexcept;
    bool retain_memory(memory::Manager& manager, const Mes11Memory& memory) noexcept;
    bool prepare_memory() noexcept;
    void release_leases() noexcept;
    bool ring_write(Ring& ring, uint32_t value) noexcept;
    bool ring_commit(Ring& ring) noexcept;
    Mes11Status kiq_ring_test() noexcept;
    Mes11Status submit(uint8_t (&frame)[mes11_api_frame_bytes], uint32_t api_status_offset) noexcept;
    bool write_slot64(uint32_t slot, uint64_t value) noexcept;
    bool read_slot64(uint32_t slot, uint64_t& value) const noexcept;
    uint64_t slot_mc(uint32_t slot) const noexcept;
    bool wait_fence(uint32_t sequence, uint32_t& remaining) noexcept;
    Mes11Status query_sched_status_frame() noexcept;
    Mes11Status validate_queue(const Mes11LegacyQueue& queue) const noexcept;
    Mes11Status quarantine(Mes11Status reason) noexcept;
    Mes11Status finish(Mes11Status status) noexcept;
    bool retain_self_test(memory::Manager& manager, const Mes11SelfTestMemory& memory,
                          const Mes11ProcessVm& vm, const GartLayout& gart) noexcept;
    void release_self_test() noexcept;
    bool self_test_add(uint32_t index, const Mes11ProcessVm& vm) noexcept;
    bool self_test_mqd(uint32_t index, const Mes11ProcessVm& vm) noexcept;
    bool queue_mqd(Mes11QueueType type, const Gfx11MqdProperties& properties,
                   sdma::CommandBuffer mqd) noexcept;
    bool read_vm_context_cntl(uint32_t& value) noexcept;
    bool encode_add_queue(const Mes11AddQueue& queue, uint32_t vm_context_cntl,
                          uint8_t (&frame)[mes11_api_frame_bytes]) const noexcept;
    Mes11Status submit_remove_queue(const Mes11RemoveQueue& queue) noexcept;
    Mes11Status submit_gang_frame(uint32_t opcode, uint32_t api_status_offset) noexcept;
    void forget_allocations() noexcept;
    bool user_write(uint32_t index, uint32_t value) noexcept;
    bool user_commit(uint32_t index) noexcept;
    Mes11SelfTestResult self_test_ring(uint32_t index, const Mes11ProcessVm& vm) noexcept;
    Mes11SelfTestResult self_test_ib(uint32_t index, const Mes11ProcessVm& vm,
                                     const Mes11SelfTestConfig& config) noexcept;
    Mes11SelfTestResult poll_user_dword(uint32_t offset, uint32_t expected,
                                        uint32_t polls) noexcept;

    Mes11Registers registers_{};
    RegisterIo io_{};
    DoorbellIo doorbells_{};
    Nbio* nbio_ = nullptr;
    VramLocation vram_{};
    Mes11Config config_{};
    Mes11Policy policy_{};
    Mes11Recovery recovery_{};
    Mes11Report report_{};
    Phase phase_ = Phase::uninitialized;
    memory::Manager* manager_ = nullptr;
    Mes11Memory handles_{};
    Ring kiq_{};
    Ring scheduler_{};
    uint8_t* writeback_cpu_ = nullptr;
    uint32_t aggregated_doorbells_[5]{};
    Mes11SelfTestReport self_test_{};
    memory::Manager* self_test_manager_ = nullptr;
    Mes11SelfTestMemory self_test_handles_{};
    uint8_t* metadata_cpu_ = nullptr;
    UserQueue user_queues_[mes11_self_test_queues]{};
    bool self_test_done_ = false;
    // amdgpu_mes doorbell_bitmap and queue_id_idr (bit n: doorbell bit n / id n).
    uint64_t doorbell_bits_[mes11_doorbell_slots / 64]{};
    uint64_t queue_id_bits_[mes11_max_queue_ids / 64]{};
};

} // namespace rx7900::hardware

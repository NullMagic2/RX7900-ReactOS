/*
Module: rx7900.hardware.gfx11_queues
Purpose: Bring up, ring-test, IB-test and unmap the GC 11.0.0 kernel gfx (KGQ) and compute (KCQ) rings.
Created: 2026-10-21
Architecture: Shared GC discovery token (memory::validate_gc_discovery), RegisterIo for
GC MMIO under GRBM selection, DoorbellIo for ring write pointers, Nbio for HDP
publication, Manager leases on local-VRAM rings, MQDs, the MEC EOP buffer and
writeback; queue mapping and unmapping through Mes11 legacy-queue API frames;
PM4 headers from pm4.hpp and clear-state data from gfx11_clear_state.hpp. The
MQD, ring-write and IB-frame builders are shared with Mes11's self-test queues.
Copyright 2021 Advanced Micro Devices, Inc.; Copyright 2014 Advanced Micro Devices, Inc.;
Copyright 2008 Advanced Micro Devices, Inc., Copyright 2008 Red Hat Inc.,
Copyright 2009 Jerome Glisse.
Adapted Linux v6.12 gfx_v11_0.c (sw_init ring setup, mec_init, gfx/compute_ring_init,
gfx_mqd_init, compute_mqd_init, kgq/kcq_init_queue, kcq_resume,
cp_async_gfx_ring_resume, cp_gfx_start, cp_resume ring tests, ring_test_ring,
ring_test_ib, emit_mem_sync, ring_emit_init_cond_exec, ring_emit_hdp_flush,
ring_emit_ib_gfx/compute, ring_emit_fence, check_fw_cp_gfx_shadow,
ring_set_wptr_gfx/compute, insert_nop, ring funcs, hw_fini), amdgpu_ib.c
(amdgpu_ib_get, amdgpu_ib_schedule, amdgpu_ib_pool_init, amdgpu_ib_ring_tests),
amdgpu_fence.c (amdgpu_fence_emit, amdgpu_fence_process), amdgpu_device.c
(delayed_init_work_handler, flush/invalidate_hdp), nbio_v4_3.c (hdp_flush_reg,
get_hdp_flush_req/done_offset), amdgpu_gfx.c
(compute/graphics_queue_acquire, priority policy, enable/disable_kcq/kgq,
mes_enable_kcq, get_num_kcq), amdgpu_gfx.h, amdgpu_ring.c (ring_init sizes,
ring_to_mqd_prop, alloc, commit), amdgpu_ring.h, amdgpu_mes.c (map/unmap legacy
queue), amdgpu_drv.c, soc15.h, soc21.c (grbm_select), nvd.h, v11_structs.h and
gc_11_0_0 offset/sh_mask/default headers. Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#pragma once

#include "fence.hpp"
#include "gfx11_init.hpp"
#include "mes11.hpp"

namespace rx7900::hardware {

// gfx_v11_0_sw_init for GC 11.0.0: one ME with one pipe and one queue
// (GFX11_NUM_GFX_RINGS 1), two MECs with four pipes of four queues each.
constexpr uint32_t gfx11_gfx_rings = 1;
// amdgpu_gfx_get_num_kcq: amdgpu_num_kcq -1 (default) and the upper bound are 8.
constexpr uint32_t gfx11_max_compute_rings = 8;
constexpr uint32_t gfx11_default_compute_rings = 8;
// Ring index convention of this module: 0 is the gfx ring, 1 + i compute ring i.
constexpr uint32_t gfx11_max_rings = gfx11_gfx_rings + gfx11_max_compute_rings;

// amdgpu_ring_init(max_dw 1024) with amdgpu_sched_hw_submission 2. Neither
// ring type raises max_dw: emit_frame_size + max_ibs * emit_ib_size, rounded to
// align_mask 0xff, is 223 + 192 * 4 -> 1024 (gfx) and 129 + 125 * 7 -> 1024
// (compute) with SOC15_FLUSH_GPU_TLB_NUM_WREG 6 and NUM_REG_WAIT 3. So
// ring_size = roundup_pow_of_two(1024 * 4 * 2) for both; extra_dw is 0.
constexpr uint32_t gfx11_ring_bytes = 1024u * 4u * 2u;
// GFX11_MEC_HPD_SIZE: one EOP area per compute ring inside one MEC HPD buffer.
constexpr uint32_t gfx11_mec_hpd_bytes = 2048;
// amdgpu_gfx_mqd_sw_init(sizeof(struct v11_compute_mqd)) for KGQ and KCQ; the
// pinned v11_structs.h gives 2048 for both v11_compute_mqd and v11_gfx_mqd.
constexpr uint32_t gfx11_mqd_bytes = 2048;
// Linux BOs are PAGE_SIZE aligned.
constexpr uint32_t gfx11_queue_alignment = 4096;

// Writeback layout (Linux adev->wb, amdgpu_device_wb_get 256-bit slots): ring r
// (0 gfx, 1 + i compute i) owns slots 5r..5r+4 in the order amdgpu_ring_init
// requests them: rptr, wptr, fence, trail_fence, cond_exec. The CP writes rptr;
// the CPU writes the 64-bit wptr shadow polled by the CP; fence starts at 0 and
// cond_exec at 1 as amdgpu_ring_init/amdgpu_fence_driver_start_ring leave them.
constexpr uint32_t gfx11_writeback_slot_bytes = 32;
constexpr uint32_t gfx11_writeback_slots_per_ring = 5;
constexpr uint32_t gfx11_writeback_rptr_slot = 0;
constexpr uint32_t gfx11_writeback_wptr_slot = 1;
constexpr uint32_t gfx11_writeback_fence_slot = 2;
constexpr uint32_t gfx11_writeback_trail_fence_slot = 3;
constexpr uint32_t gfx11_writeback_cond_exec_slot = 4;
constexpr uint32_t gfx11_writeback_bytes = 4096;

// The module ring index is the FenceManager ring index (fence_ring_gfx,
// fence_ring_compute0 + i).
static_assert(gfx11_gfx_rings == fence_gfx_rings && gfx11_max_compute_rings == fence_compute_rings &&
              gfx11_max_rings == fence_ring_sdma0, "Gfx11Queues and FenceManager ring indices agree");

// amdgpu_ib_schedule -> amdgpu_ring_alloc(emit_frame_size + emit_ib_size),
// rounded up to align_mask 0xff: gfx 223 + 4 and compute 129 + 7 (with
// SOC15_FLUSH_GPU_TLB_NUM_WREG 6 and NUM_REG_WAIT 3) both reserve 256 dwords.
constexpr uint32_t gfx11_submit_alloc_dwords = 256;
// INDIRECT_BUFFER IB_SIZE is 20 bits.
constexpr uint32_t gfx11_max_ib_dwords = 0x000fffffu;

// VMIDs amdgpu_vmid_grab hands to kernel-ring jobs on GC 11 with MES
// (gmc_v11_0_sw_init first_kfd_vmid 8): 1..7 of the GFXHUB. MES owns 8..15.
constexpr uint32_t gfx11_first_job_vmid = 1;
constexpr uint32_t gfx11_job_vmids = 7;
// amdgpu_gmc_allocate_vm_inv_eng: bitmap 0x1FFF3 minus engine 5 (MES) on every
// hub, rings in adev->rings order; the gfx ring is the first GFXHUB ring.
constexpr uint32_t gfx11_gfx_vm_invalidate_engine = 0;

// gfx_v11_0_ring_test_ring and adev->usec_timeout on bare metal.
constexpr uint32_t gfx11_ring_test_seed = 0xCAFEDEADu;
constexpr uint32_t gfx11_ring_test_value = 0xDEADBEEFu;
constexpr uint32_t gfx11_usec_timeout = 100000;

// amdgpu_ib.c AMDGPU_IB_TEST_TIMEOUT (msecs_to_jiffies(1000)): the bare-metal
// tmo_gfx that amdgpu_ib_ring_tests passes to gfx, compute and SDMA rings.
// AMDGPU_IB_TEST_GFX_XGMI_TIMEOUT (2000 ms) applies only inside an XGMI hive and
// is the largest budget accepted here. SR-IOV multipliers do not apply.
constexpr uint32_t gfx11_ib_test_timeout_us = 1000000;
constexpr uint32_t gfx11_ib_test_xgmi_timeout_us = 2000000;
// amdgpu_ib_pool_init suballocates IBs with 256-byte alignment; the gfx IB test
// requests 20 bytes (five dwords). The caller-provided IB buffer holds one
// 256-byte IB slot per module ring index (ring r at byte r * 256).
constexpr uint32_t gfx11_ib_test_slot_bytes = 256;
constexpr uint32_t gfx11_ib_test_buffer_bytes = gfx11_max_rings * gfx11_ib_test_slot_bytes;
constexpr uint32_t gfx11_ib_test_alignment = 256;
constexpr uint32_t gfx11_ib_test_dwords = 5;
// gfx_v11_0_ring_test_ib takes a fresh amdgpu_device_wb_get slot for the
// WRITE_DATA destination. Here ring r uses writeback slot
// gfx11_writeback_ib_test_slot_base + r of the retained writeback page (slots
// 45..53, bytes 1440..1727), seeded with gfx11_ring_test_seed before each test.
constexpr uint32_t gfx11_writeback_ib_test_slot_base =
    gfx11_max_rings * gfx11_writeback_slots_per_ring;

enum class Gfx11QueuesStatus {
    ready,
    running,
    stopped,
    released,
    unavailable,
    unsupported_ip,
    unsupported_firmware,
    invalid_argument,
    invalid_memory,
    mmio_failure,
    doorbell_failure,
    mes_failure,
    ring_test_failed,
    busy,
    quarantined,
    // test_ib: at least one ring's IB test timed out or wrote a wrong value;
    // those rings are disabled and quarantined, the others stay usable.
    ib_test_failed,
    // submit: four fences outstanding or not enough ring space; retry after a
    // fence of this ring signalled. Nothing was written or published.
    ring_full,
    // submit/disable_submission: the FenceManager quarantined the ring (job
    // timeout or fault); only an explicit mode-1 reset returns it.
    ring_hung
};

struct Gfx11QueuesConfig final {
    // adev->gfx.num_compute_rings (amdgpu_num_kcq), 1..gfx11_max_compute_rings.
    // Linux also accepts 0; this module refuses it.
    uint32_t compute_rings;
};

struct Gfx11QueuesPolicy final {
    // adev->usec_timeout ring-test polls of SCRATCH_REG0, 1 us apart.
    uint32_t poll_iterations;
};

struct Gfx11QueuesRecovery final {
    void* context;
    // True only after a reset proves CP/MES no longer access any ring, MQD,
    // EOP or writeback byte retained here. Never inferred from a failure.
    bool (*confirm_reset_quiescence)(void*) noexcept;
};

// Separate Manager allocations, all CPU-visible local VRAM addressed by MC
// address through VMID0 and 4 KiB aligned: the gfx ring and one ring per compute
// ring (gfx11_ring_bytes each), one MQD each (gfx11_mqd_bytes), one MEC HPD/EOP
// buffer of compute_rings * gfx11_mec_hpd_bytes (ring i uses offset i * 2048,
// as gfx_v11_0_compute_ring_init) and one writeback page laid out as above.
// Entries of compute_ring/compute_mqd at or beyond config.compute_rings are ignored.
struct Gfx11QueueMemory final {
    memory::Handle gfx_ring;
    memory::Handle gfx_mqd;
    memory::Handle compute_ring[gfx11_max_compute_rings];
    memory::Handle compute_mqd[gfx11_max_compute_rings];
    memory::Handle compute_eop;
    memory::Handle writeback;
};

enum class Gfx11RingTest { not_run, passed, timeout, failed };

struct Gfx11RingReport final {
    Mes11QueueType type = Mes11QueueType::gfx;
    // Linux ring->me (0 gfx, 1 = MEC1 compute), pipe and queue.
    uint32_t me = 0;
    uint32_t pipe = 0;
    uint32_t queue = 0;
    // Dword doorbell index (soc21 64-bit index << 1); 64-bit WDOORBELL64.
    uint32_t doorbell_index = 0;
    bool high_priority = false;
    uint64_t ring_mc = 0;
    uint64_t mqd_mc = 0;
    uint64_t eop_mc = 0;
    uint64_t rptr_mc = 0;
    uint64_t wptr_mc = 0;
    // Driver write pointer in dwords (64-bit, never masked: support_64bit_ptrs).
    uint64_t wptr = 0;
    bool mapped = false;
    Gfx11RingTest ring_test = Gfx11RingTest::not_run;
    // test_ib result (Linux amdgpu_ring_test_ib): passed, timeout (fence not
    // signalled in time, Linux -ETIMEDOUT) or failed (fence signalled but the
    // destination lacks 0xDEADBEEF, Linux -EINVAL).
    Gfx11RingTest ib_test = Gfx11RingTest::not_run;
    // Linux ring->sched.ready = false after an IB test failure. A disabled ring
    // is quarantined: the CP may still fetch its frame, so no later test_ib
    // submits to it and its writeback and the IB buffer stay leased until reset.
    bool ib_disabled = false;
    // Last fence sequence emitted (ring->fence_drv.sync_seq; 0 before the first
    // fence) and the last value read back from the fence writeback slot.
    uint32_t fence_seq = 0;
    uint32_t fence_value = 0;
    uint64_t fence_mc = 0;
    uint64_t ib_mc = 0;
    uint64_t ib_test_mc = 0;
    // 1 us fence polls the last IB test used.
    uint32_t ib_polls = 0;
    // Kernel-ring submission (enable_submission): attached to the FenceManager,
    // IB leases held for unsignalled fences, submissions, the ring position the
    // engine provably consumed at the last submit and the free dwords then.
    bool submit_enabled = false;
    uint32_t ib_leases = 0;
    uint64_t submissions = 0;
    uint64_t consumed_position = 0;
    uint32_t free_dwords = 0;
};

struct Gfx11QueuesReport final {
    Gfx11QueuesStatus status = Gfx11QueuesStatus::unavailable;
    uint32_t compute_rings = 0;
    // Last Mes11 result seen by resume/fini and the KIQ SET_RESOURCES result.
    Mes11Status mes_status = Mes11Status::unavailable;
    Mes11Status set_resources_status = Mes11Status::unavailable;
    // queue_mask passed to gfx11_kiq_set_resources (amdgpu_gfx_mes_enable_kcq: ~0).
    uint64_t set_resources_mask = 0;
    // First ring (module index) whose ring test failed, or UINT32_MAX.
    uint32_t failed_ring = UINT32_MAX;
    uint32_t queues_mapped = 0;
    bool clear_state_emitted = false;
    bool memory_retained = false;
    uint64_t compute_eop_mc = 0;
    uint64_t writeback_mc = 0;
    // test_ib: first ring (module index) whose IB test failed, or UINT32_MAX;
    // rings that passed in the last call; true when the gfx ring failed
    // (Linux clears accel_working and returns before testing later rings,
    // including both SDMA rings); the retained IB buffer address.
    uint32_t ib_failed_ring = UINT32_MAX;
    uint32_t ib_rings_passed = 0;
    bool ib_primary_failed = false;
    bool ib_buffer_retained = false;
    uint64_t ib_buffer_mc = 0;
    bool submission_enabled = false;
    Gfx11RingReport rings[gfx11_max_rings]{};
};

// Process-VM jobs on the gfx ring (enabled with the GMC/OSSSYS tokens):
// accepted submissions, PASID LUT writes, refusals because every VMID still
// had unsignalled work (amdgpu_vmid_grab_idle found none), and the PASID each
// job VMID (index vmid - gfx11_first_job_vmid) was last bound to.
struct Gfx11VmReport final {
    bool enabled = false;
    uint64_t submissions = 0;
    uint64_t pasid_mappings = 0;
    uint32_t vmid_busy = 0;
    uint32_t vmid_pasid[gfx11_job_vmids]{};
};

// enable_submission inputs: the NBIO token (HDP flush WAIT_REG_MEM addresses,
// as test_ib) and the atomic WPTR shadow store. gmc and osssys enable process-VM
// jobs on the gfx ring (the GFXHUB CONTEXTn/invalidation registers of
// gmc_v11_0_emit_flush_gpu_tlb and the IH_VMID LUT of emit_pasid_mapping);
// without both tokens only VMID 0 submissions are accepted.
struct Gfx11SubmitConfig final {
    NbioRegisters nbio;
    RingPointerStore wptr_store;
    GmcRegisters gmc{};
    Ih6Registers osssys{};
};

// A kernel gfx-ring job in a process GPUVM (amdgpu_job with vm: amdgpu_cs on
// Linux). pd_address is amdgpu_gmc_pd_addr of the VM root (GpuVmReport
// pd_address), pasid the VM's PASID, ib_va the IB's address in that VM. The
// caller guarantees ib_va maps exactly the ib_dwords dwords that
// Gfx11Submission::ib holds at ib_offset, read-only, for as long as the fence is
// pending; this module only leases that backing allocation.
struct Gfx11VmJob final {
    uint32_t pasid = 0;
    uint64_t pd_address = 0;
    uint64_t ib_va = 0;
};

// One amdgpu_ib_schedule(ring, 1, &ib, NULL, &f) without a job (VMID 0): ib is
// an allocation of the resume Manager (GTT or VRAM, VMID0-addressed), owned by
// the caller; ib_dwords dwords from byte ib_offset, 4-byte aligned, outside
// every ring, MQD, EOP and writeback buffer of this module. With vm (gfx ring
// only), the same allocation range is fetched through vm->ib_va in a VMID this
// module grabs for the job (amdgpu_ib_schedule with a job).
struct Gfx11Submission final {
    uint32_t ring;
    memory::Handle ib;
    uint64_t ib_offset;
    uint32_t ib_dwords;
    const Gfx11VmJob* vm = nullptr;
};

// Inputs of test_ib. nbio is the token Nbio was initialized with: the HDP flush
// WAIT_REG_MEM addresses BIF_BX_PF0_GPU_HDP_FLUSH_REQ/DONE (NBIF segment 2,
// 0x106/0x107), as nbio_v4_3_get_hdp_flush_req/done_offset. timeout_microseconds
// is the fence poll budget, 1..gfx11_ib_test_xgmi_timeout_us; Linux bare metal
// uses gfx11_ib_test_timeout_us.
struct Gfx11IbTestConfig final {
    NbioRegisters nbio;
    uint32_t timeout_microseconds;
};

// Shared GC 11 queue builders. Gfx11Queues uses them for the kernel rings and
// Mes11::self_test for the MES-mapped user queues of amdgpu_mes_self_test, so
// both emit one implementation of the Linux MQD, ring and IB-frame contracts.

// gfx_v11_0_ring_funcs_gfx/compute nop (PACKET3(NOP, 0x3FFF)) and align_mask.
constexpr uint32_t gfx11_ring_nop = 0xffff1000u;
constexpr uint32_t gfx11_ring_align_mask = 0xff;
// amdgpu_mes_ctx.h AMDGPU_FENCE_MES_QUEUE_FLAG: MES user-queue fences carry
// hw_queue_id | this flag as interrupt context (RELEASE_MEM data, SDMA TRAP).
constexpr uint32_t gfx11_fence_mes_queue_flag = 0x01000000u;
// BIF_BX_PF0_GPU_HDP_FLUSH_REQ/DONE (nbio_4_3_0_offset.h, NBIF BASE_IDX 2), the
// registers gfx_v11_0_ring_emit_hdp_flush and sdma_v6_0_ring_emit_hdp_flush poll.
constexpr uint32_t gfx11_hdp_flush_nbif_segment = 2;
constexpr uint32_t gfx11_hdp_flush_req_register = 0x0106;
constexpr uint32_t gfx11_hdp_flush_done_register = 0x0107;

// GC register access for MQD construction: one RegisterIo and the GC BASE_IDX 0
// and 1 segment bases a discovery token provided. GRBM_GFX_CNTL selection is
// shared GC state; the caller serializes it (Linux srbm_mutex).
struct Gfx11GcAccess final {
    RegisterIo io;
    uint32_t segment0;
    uint32_t segment1;
};

// soc21_grbm_select(me, pipe, queue, 0) target of an MQD build.
struct Gfx11QueueSelect final {
    uint32_t me;
    uint32_t pipe;
    uint32_t queue;
};

// The struct amdgpu_mqd_prop fields gfx_v11_0_gfx_mqd_init and
// gfx_v11_0_compute_mqd_init consume (use_doorbell is always true: GC 11 has no
// MMIO wptr path for these queues). Addresses are GPU addresses in the queue's
// address space: VMID0 MC/GART for kernel rings, the process GPUVM for MES user
// queues; mqd_address is always the VMID0 address MES/CP reads the MQD from.
// queue_bytes is a power of two of at least 1024; eop_address, allow_tunneling and
// hqd_active are used by the compute MQD only (gfx sets CP_GFX_HQD_ACTIVE 1).
struct Gfx11MqdProperties final {
    uint64_t mqd_address;
    uint64_t hqd_base_address;
    uint64_t rptr_address;
    uint64_t wptr_address;
    uint64_t eop_address;
    uint32_t queue_bytes;
    uint32_t doorbell_index;
    uint32_t hqd_pipe_priority;
    uint32_t hqd_queue_priority;
    bool allow_tunneling;
    bool hqd_active;
};

// gfx_v11_0_compute_mqd_init / gfx_v11_0_gfx_mqd_init into a gfx11_mqd_bytes
// image: zero the image (the BO memset), soc21_grbm_select(select), read the
// HQD registers in Linux order, select 0, then write every field Linux writes.
// False on a CPU store, bounds or MMIO failure; the selection may then still be
// non-zero and the caller restores it before reporting.
bool gfx11_compute_mqd_init(const Gfx11GcAccess& access, Gfx11QueueSelect select,
                            const Gfx11MqdProperties& properties,
                            sdma::CommandBuffer mqd) noexcept;
bool gfx11_gfx_mqd_init(const Gfx11GcAccess& access, Gfx11QueueSelect select,
                        const Gfx11MqdProperties& properties, sdma::CommandBuffer mqd) noexcept;

// A CPU-visible GC 11 ring of a power-of-two dword count and its 64-bit driver
// write pointer in dwords (support_64bit_ptrs: never masked, only indexed).
struct Gfx11RingWriter final {
    sdma::CommandBuffer ring;
    uint64_t wptr;
};

// amdgpu_ring_write.
bool gfx11_ring_write(Gfx11RingWriter& writer, uint32_t value) noexcept;
// amdgpu_ring_commit padding: gfx_v11_ring_insert_nop up to align_mask + 1.
bool gfx11_ring_pad(Gfx11RingWriter& writer) noexcept;
// gfx_v11_0_ring_test_ring body for gfx/compute rings: SET_UCONFIG_REG of the
// absolute SCRATCH_REG0 dword index with 0xDEADBEEF. False when the register is
// outside the SET_UCONFIG_REG window.
bool gfx11_emit_ring_test(Gfx11RingWriter& writer, uint32_t scratch_register) noexcept;
// gfx_v11_0_ring_test_ib IB body: WRITE_DATA DST_SEL(5) WR_CONFIRM of 0xDEADBEEF
// to destination (gfx11_ib_test_dwords dwords).
bool gfx11_write_test_ib(sdma::CommandBuffer ib, uint64_t destination) noexcept;

// One amdgpu_ib_schedule(ring, 1, &ib, NULL, &f) frame. mem_sync is
// AMDGPU_IB_FLAG_EMIT_MEM_SYNC (set by amdgpu_ib_get; MES-queue IB tests skip
// amdgpu_ib_get and leave it clear). mes_queue selects the user-queue variants:
// the INDIRECT_BUFFER "inherit vmid from mqd" bit and RELEASE_MEM's last dword
// hw_queue_id | AMDGPU_FENCE_MES_QUEUE_FLAG. pipe selects the HDP flush
// ref_and_mask (gfx ME0: cp0 << pipe; compute MEC1: cp2 << pipe).
// The amdgpu_vm_flush part of a gfx-ring job in VMID vmid (1..15):
// pipeline_sync waits (PFP, memory) until the ring's fence writeback at
// sync_address equals sync_sequence (gfx_v11_0_ring_emit_pipeline_sync with
// fence_drv.sync_seq); flush is gmc11_emit_flush_gpu_tlb for vmid (always
// emitted, then PFP_SYNC_ME); pasid_mapping writes pasid to the absolute
// pasid_register (gpuvm_pasid_mapping_register, gmc_v11_0_emit_pasid_mapping);
// context_switch selects the AMDGPU_HAVE_CTX_SWITCH CONTEXT_CONTROL bits.
struct Gfx11VmFrame final {
    uint32_t vmid = 0;
    bool pipeline_sync = false;
    uint64_t sync_address = 0;
    uint32_t sync_sequence = 0;
    GmcRingFlush flush{};
    bool pasid_mapping = false;
    uint32_t pasid_register = 0;
    uint32_t pasid = 0;
    bool context_switch = false;
};

struct Gfx11IbFrame final {
    bool gfx;
    bool mem_sync;
    bool mes_queue;
    uint32_t hw_queue_id;
    uint32_t pipe;
    uint64_t cond_exec_address;
    uint32_t hdp_flush_req;
    uint32_t hdp_flush_done;
    uint64_t ib_address;
    uint32_t ib_dwords;
    uint64_t fence_address;
    uint32_t fence_sequence;
    // A job in a process VM (gfx ring only, not with mes_queue); null: VMID 0.
    const Gfx11VmFrame* vm = nullptr;
};

// The ring stream of that frame (without amdgpu_ring_commit padding):
// [ACQUIRE_MEM if mem_sync], [gfx: COND_EXEC], WAIT_REG_MEM HDP flush,
// INDIRECT_BUFFER (vmid 0), [gfx: second unpatched COND_EXEC], RELEASE_MEM fence
// with INT_SEL 2, then the gfx COND_EXEC count patch.
// With vm, amdgpu_ib_schedule's job order on the gfx ring: [ACQUIRE_MEM],
// amdgpu_vm_flush (COND_EXEC, [pipeline sync], VM flush ops, PFP_SYNC_ME,
// [pasid mapping], count patch), COND_EXEC, HDP flush, CONTEXT_CONTROL,
// FRAME_CONTROL start, INDIRECT_BUFFER (vmid << 24), FRAME_CONTROL end,
// second COND_EXEC, RELEASE_MEM, count patch. Deviations: the extra fence
// amdgpu_vm_flush emits for id->last_flush/pasid_mapping is not emitted (the
// job fence after the IB covers both: a VMID changes owner only after it);
// update_spm_vmid (SPM perf counters) and the GDS switch (jobs here use no
// GDS/GWS/OA; the per-VMID registers keep their zero reset values) are omitted;
// cp_gfx_shadow is false on bare metal, so no SET_Q_PREEMPTION_MODE packets.
bool gfx11_emit_ib_frame(Gfx11RingWriter& writer, const Gfx11IbFrame& frame) noexcept;

// Serialized PASSIVE_LEVEL use by the exclusive device owner. GRBM_GFX_CNTL is
// shared GC selection state (Linux srbm_mutex): no other module may select a
// ME/pipe/queue while a call here runs, and every MQD build restores the
// broadcast selection 0. Mes11 calls made from here busy-wait up to its policy.
//
// Prerequisites, in gfx_v11_0_hw_init order: Gfx11Init through cp_start (which
// already did cp_set_doorbell_range, cp_compute_enable and cp_gfx_enable), then
// Mes11::kiq_hw_init returning scheduler_ready with legacy_queue_map (firmware
// >= 0x47); the Linux KIQ map path for older firmware is not implemented.
//
// Placement: Linux allocates rings, the MEC HPD buffer and adev->wb in GTT and
// MQDs in VRAM|GTT. Buffers come from the caller's Manager inside the vram window
// it passes (AsicInit: GART-mapped system pages and the GART window), addressed
// through VMID0; CPU stores are published with Nbio::flush_hdp before every
// doorbell and before MES reads an MQD (Linux needs only mb() for coherent GTT). After the first register access every lease stays
// retained until fini unmapped every queue (then release) or a confirmed reset
// (release_after_reset). Any uncertain hardware result quarantines.
class Gfx11Queues final {
public:
    Gfx11Queues() noexcept = default;
    Gfx11Queues(const Gfx11Queues&) = delete;
    Gfx11Queues& operator=(const Gfx11Queues&) = delete;

    // Validates the token (registers used, SCRATCH_REG0 inside the
    // SET_UCONFIG_REG window), the mandatory doorbell aperture covering every
    // ring doorbell, the config and the policy. No hardware access.
    Gfx11QueuesStatus initialize(memory::GcRegisterBase registers, RegisterIo io,
                                 DoorbellIo doorbells, Nbio& nbio, VramLocation vram,
                                 Gfx11QueuesConfig config, Gfx11QueuesPolicy policy,
                                 Gfx11QueuesRecovery recovery) noexcept;

    // gfx_v11_0_cp_resume after amdgpu_mes_kiq_hw_init: sw_init-time buffer
    // state, gfx_v11_0_kcq_resume (compute MQDs, MES enable_kcq: KIQ
    // SET_RESOURCES then ADD_QUEUE per ring), gfx_v11_0_cp_async_gfx_ring_resume
    // (gfx MQD, enable_kgq ADD_QUEUE, cp_gfx_start clear state), then ring tests
    // of the gfx ring and every compute ring, stopping at the first failure.
    // Returns running on success; failures before the first register access
    // leave nothing retained, later failures quarantine.
    Gfx11QueuesStatus resume(memory::Manager& manager, const Gfx11QueueMemory& memory,
                             Mes11& mes) noexcept;

    // gfx_v11_0_hw_fini queue part: amdgpu_gfx_disable_kgq (MES REMOVE_QUEUE
    // for PREEMPT_QUEUES, i.e. unmap_legacy_queue with the queue type) then
    // amdgpu_gfx_disable_kcq (RESET_QUEUES, same frame) for every mapped queue.
    // Linux ignores these results; here any failure quarantines. Call before
    // Mes11::kiq_hw_fini. Returns stopped when every queue was unmapped.
    Gfx11QueuesStatus fini(Mes11& mes) noexcept;

    // gfx_v11_0_cp_resume on resume from suspend (adev->in_suspend) after fini and
    // a new MES KIQ start: gfx_v11_0_kcq_init_queue / kgq_init_queue take the
    // in_suspend branch, so every MQD is restored from the backup resume took
    // right after its first mqd_init (mqd_backup), ring->wptr and the wptr shadow
    // go to 0 and the ring is refilled with NOPs (amdgpu_ring_clear_ring); then
    // KIQ SET_RESOURCES, every KCQ map, the KGQ map, cp_gfx_start and the ring
    // tests exactly as resume. Fence slots and fence sequences continue (the
    // fence driver keeps sync_seq), so a later test_ib/enable_submission resumes
    // the same FenceManager sequence. Requires stopped, submission disabled, no
    // IB-test quarantine and the retained memory; failures quarantine as resume.
    Gfx11QueuesStatus resume_from_suspend(Mes11& mes) noexcept;

    // amdgpu_ib_ring_tests for this module's rings, as the bare-metal
    // amdgpu_device_delayed_init_work_handler runs it AMDGPU_RESUME_MS (2 s)
    // after amdgpu_device_ip_late_init (clock/power gating): the gfx ring,
    // then compute rings 0..compute_rings-1, each with gfx_v11_0_ring_test_ib.
    // The MES scheduler and MES KIQ rings are skipped exactly as Linux skips them
    // (no test_ib for MES; KIQ skipped under enable_mes). Call from running only.
    //
    // ib_buffer: one Manager allocation from the same manager as resume, CPU
    // visible, GPU readable, gfx11_ib_test_alignment aligned and at least
    // gfx11_ib_test_buffer_bytes long, not overlapping any ring, MQD, EOP or
    // writeback buffer. Its GPU address may be local VRAM or GART-mapped system
    // memory: it is used through VMID0 and is not required to lie in the VRAM
    // aperture. One lease is held during the call; it is returned when every
    // tested ring passed and kept (ib_buffer_retained) when any ring failed,
    // until release_after_reset.
    //
    // Per ring: seed the destination slot 0xCAFEDEAD, write the five-dword
    // WRITE_DATA IB, emit the amdgpu_ib_schedule frame (ACQUIRE_MEM; COND_EXEC
    // on gfx; ring HDP flush; INDIRECT_BUFFER vmid 0; second unpatched COND_EXEC
    // on gfx; RELEASE_MEM fence of ++fence_seq with INT_SEL 2), pad to 256
    // dwords, flush HDP from the CPU and ring the doorbell, then poll the fence
    // writeback once per microsecond up to the timeout and compare the
    // destination with 0xDEADBEEF.
    //
    // Deviation: Linux waits in dma_fence_wait_timeout for a fence signalled
    // by amdgpu_fence_process from the CP EOP interrupt (plus its fallback
    // timer). The IB tests run before IrqService::enable_sources sets
    // CP_INT_CNTL TIME_STAMP_INT_ENABLE, so the fence writeback is polled here;
    // the RELEASE_MEM still requests the EOP interrupt as Linux does.
    // Interrupt-driven completion is the submit path through FenceManager.
    //
    // Returns running when every tested ring passed and ib_test_failed when a
    // ring failed (stopping at a gfx ring failure, as Linux returns). A failing
    // ring is disabled and quarantined; the protocol stays running for the other
    // rings and fini, but only release_after_reset can then return the leases.
    // invalid_argument/invalid_memory refuse before any access; a failed CPU,
    // MMIO, HDP or doorbell store quarantines the whole protocol.
    Gfx11QueuesStatus test_ib(memory::Manager& manager, memory::Handle ib_buffer,
                              const Gfx11IbTestConfig& config) noexcept;

    // Attaches every ring that passed its ring and IB test to fences
    // (amdgpu_fence_driver_start_ring with the IB test's sync_seq; the fence
    // writeback must already hold it) and enables submit. Requires running, the
    // IB tests completed with the IB buffer returned, and no prior enable.
    // fences outlives this object's submission state. test_ib is refused
    // afterwards (both would emit fences on the same rings).
    Gfx11QueuesStatus enable_submission(FenceManager& fences,
                                        const Gfx11SubmitConfig& config) noexcept;

    // amdgpu_ib_schedule for one IB on one kernel ring, PASSIVE_LEVEL, one
    // submitter per module (the device owner serializes it with every other
    // method here). Reclaims IB leases of signalled fences, validates the IB,
    // reserves the fence, checks ring space (amdgpu_ring_alloc: 256 dwords
    // must fit below the consumed position, which is the end of the newest
    // signalled frame or the CP RPTR writeback when that is further), retains
    // one IB lease until the fence signals, writes the frame at the 64-bit
    // WPTR (wrapping in the ring), publishes the fence to the manager, then the
    // WPTR shadow (atomic store), HDP flush and doorbell. sequence is the fence
    // value (wait/query it on the FenceManager with the same ring index).
    // ring_full and ring_hung change nothing; invalid_argument/invalid_memory
    // refuse before any write; a failed ring store or publication quarantines
    // the protocol and marks the ring hung.
    //
    // A submission with vm (gfx ring 0 only; needs the GMC/OSSSYS tokens) is an
    // amdgpu_job: amdgpu_vmid_grab picks one of the job VMIDs (refused with
    // ring_full and nothing written while none is idle, i.e. every one still
    // has an unsignalled fence; else one already bound to this PASID and page
    // directory, else the least recently used idle one), then the frame of
    // gfx11_emit_ib_frame with its vm part: a pipeline sync on the previous
    // fence and the VM flush on every job (use_cpu_for_update VMs always need
    // it), the PASID mapping when the VMID changed PASID or since enable/reset,
    // and the full-context CONTEXT_CONTROL when the previous job on the ring was
    // another PASID or VMID 0 work.
    Gfx11QueuesStatus submit(const Gfx11Submission& submission, uint32_t& sequence) noexcept;
    // Returns IB leases whose fences signalled (PASSIVE_LEVEL).
    Gfx11QueuesStatus retire() noexcept;
    // Detaches every submission ring once all its fences signalled and returns
    // the IB leases. busy while fences are outstanding, ring_hung when a ring
    // is quarantined (then only release_after_reset). Allowed after fini.
    Gfx11QueuesStatus disable_submission() noexcept;

    // Returns every lease after a successful fini. Refused (quarantined) while
    // an IB test failure keeps rings or the IB buffer quarantined, busy while
    // submission is enabled.
    Gfx11QueuesStatus release() noexcept;
    // After confirmed reset quiescence from any retained or faulted state; also
    // detaches the submission rings from the FenceManager and returns IB leases.
    Gfx11QueuesStatus release_after_reset() noexcept;

    Gfx11QueuesReport report() const noexcept { return report_; }
    Gfx11VmReport vm_report() const noexcept { return vm_report_; }

private:
    enum class Phase { uninitialized, ready, running, stopped, faulted };

    struct Ring final {
        uint8_t* cpu;
        uint64_t mc;
        uint8_t* mqd_cpu;
        uint64_t mqd_mc;
        uint8_t* eop_cpu;
    };

    // An IB lease held until its fence signals.
    struct IbLease final {
        bool used;
        uint32_t sequence;
        memory::Handle ib;
    };

    // amdgpu_vmid for the GFXHUB job VMIDs: owner (PASID and page-directory
    // address), the gfx-ring fence of its last job (0: never used since the
    // last enable or reset, so its PASID mapping is due) and the LRU stamp.
    struct JobVmid final {
        uint32_t pasid;
        uint64_t pd_address;
        uint32_t last_sequence;
        uint64_t lru;
    };

    bool read(uint32_t segment, uint32_t reg, uint32_t& value) noexcept;
    bool write(uint32_t segment, uint32_t reg, uint32_t value) noexcept;
    bool grbm_select(uint32_t me, uint32_t pipe, uint32_t queue) noexcept;
    Gfx11MqdProperties mqd_properties(uint32_t ring) const noexcept;
    bool retain_memory(memory::Manager& manager, const Gfx11QueueMemory& memory) noexcept;
    void release_leases() noexcept;
    bool prepare_memory() noexcept;
    uint64_t slot_mc(uint32_t ring, uint32_t slot) const noexcept;
    bool write_slot64(uint32_t ring, uint32_t slot, uint64_t value) noexcept;
    bool compute_mqd_init(uint32_t ring) noexcept;
    bool gfx_mqd_init() noexcept;
    void backup_mqd(uint32_t ring) noexcept;
    bool restore_mqd(uint32_t ring) noexcept;
    bool clear_ring(uint32_t ring) noexcept;
    Mes11LegacyQueue legacy_queue(uint32_t ring) const noexcept;
    Gfx11QueuesStatus map_queue(Mes11& mes, uint32_t ring) noexcept;
    bool ring_write(uint32_t ring, uint32_t value) noexcept;
    Gfx11QueuesStatus ring_commit(uint32_t ring) noexcept;
    Gfx11QueuesStatus cp_gfx_start() noexcept;
    Gfx11QueuesStatus ring_test(uint32_t ring) noexcept;
    bool retain_ib(memory::Manager& manager, memory::Handle ib_buffer) noexcept;
    void release_ib() noexcept;
    bool emit_ib_frame(uint32_t ring) noexcept;
    Gfx11QueuesStatus ib_test(uint32_t ring, uint32_t timeout) noexcept;
    Gfx11QueuesStatus quarantine(Gfx11QueuesStatus reason) noexcept;
    Gfx11QueuesStatus finish(Gfx11QueuesStatus status) noexcept;
    bool submit_ib_address(const Gfx11Submission& submission, uint64_t& address) noexcept;
    bool vm_job_valid(const Gfx11Submission& submission) const noexcept;
    bool grab_vmid(const Gfx11VmJob& job, uint32_t& index, bool& pasid_mapping) noexcept;
    void reset_vmids() noexcept;
    uint64_t consumed_position(uint32_t ring, uint64_t retired) const noexcept;
    void reclaim(uint32_t ring, bool all) noexcept;
    bool attach_fence_ring(uint32_t ring) noexcept;

    memory::GcRegisterBase registers_{};
    RegisterIo io_{};
    DoorbellIo doorbells_{};
    Nbio* nbio_ = nullptr;
    VramLocation vram_{};
    Gfx11QueuesConfig config_{};
    Gfx11QueuesPolicy policy_{};
    Gfx11QueuesRecovery recovery_{};
    Gfx11QueuesReport report_{};
    Phase phase_ = Phase::uninitialized;
    memory::Manager* manager_ = nullptr;
    Gfx11QueueMemory handles_{};
    Ring rings_[gfx11_max_rings]{};
    uint8_t* writeback_cpu_ = nullptr;
    memory::Handle ib_handle_{};
    uint8_t* ib_cpu_ = nullptr;
    uint32_t hdp_flush_req_ = 0;
    uint32_t hdp_flush_done_ = 0;
    FenceManager* fences_ = nullptr;
    RingPointerStore wptr_store_{};
    IbLease leases_[gfx11_max_rings][fence_slots]{};
    // Process-VM jobs: tokens from enable_submission, the job VMIDs, the LRU
    // clock and the PASID of the last job on the gfx ring (ring->current_ctx;
    // 0 for VMID 0 work, so the next job loads a full context).
    GmcRegisters vm_gmc_{};
    Ih6Registers vm_osssys_{};
    JobVmid vmids_[gfx11_job_vmids]{};
    uint64_t vmid_clock_ = 0;
    uint32_t ring_context_pasid_ = 0;
    Gfx11VmReport vm_report_{};
    // adev->gfx.me/mec.mqd_backup: each MQD as first initialized, for resume.
    uint8_t mqd_backup_[gfx11_max_rings][gfx11_mqd_bytes]{};
    bool mqd_backed_up_[gfx11_max_rings]{};
};

} // namespace rx7900::hardware

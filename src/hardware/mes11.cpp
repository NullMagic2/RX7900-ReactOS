/*
Module: rx7900.hardware.mes11
Purpose: Program the MES KIQ/scheduler pipes, their MQDs, the MES API ring, process-queue frames and the self-test user queues.
Created: 2026-10-21
Architecture: Bounded GC register sequence and checked byte writers; the native
owner supplies MMIO, doorbell stores, delays, local-VRAM and GTT allocations.
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
#include "mes11.hpp"
#include "gfx11_queues.hpp"
#include "pm4.hpp"

namespace rx7900::hardware {
namespace {

// gc_11_0_0_offset.h, BASE_IDX 0.
constexpr uint32_t reg_cp_mqd_base_addr = 0x1fa9;
constexpr uint32_t reg_cp_mqd_base_addr_hi = 0x1faa;
constexpr uint32_t reg_cp_hqd_active = 0x1fab;
constexpr uint32_t reg_cp_hqd_vmid = 0x1fac;
constexpr uint32_t reg_cp_hqd_persistent_state = 0x1fad;
constexpr uint32_t reg_cp_hqd_pq_base = 0x1fb1;
constexpr uint32_t reg_cp_hqd_pq_base_hi = 0x1fb2;
constexpr uint32_t reg_cp_hqd_pq_rptr = 0x1fb3;
constexpr uint32_t reg_cp_hqd_pq_rptr_report_addr = 0x1fb4;
constexpr uint32_t reg_cp_hqd_pq_rptr_report_addr_hi = 0x1fb5;
constexpr uint32_t reg_cp_hqd_pq_wptr_poll_addr = 0x1fb6;
constexpr uint32_t reg_cp_hqd_pq_wptr_poll_addr_hi = 0x1fb7;
constexpr uint32_t reg_cp_hqd_pq_doorbell_control = 0x1fb8;
constexpr uint32_t reg_cp_hqd_pq_control = 0x1fba;
constexpr uint32_t reg_cp_hqd_dequeue_request = 0x1fc1;
constexpr uint32_t reg_cp_mqd_control = 0x1fcb;
constexpr uint32_t reg_cp_hqd_pq_wptr_lo = 0x1fdf;
constexpr uint32_t reg_cp_hqd_pq_wptr_hi = 0x1fe0;
constexpr uint32_t highest_segment0_register = reg_cp_hqd_pq_wptr_hi;
// gc_11_0_0_offset.h, BASE_IDX 1.
constexpr uint32_t reg_grbm_gfx_cntl = 0x0900;
constexpr uint32_t reg_rlc_cp_schedulers = 0x098a;
constexpr uint32_t reg_scratch_reg0 = 0x2040;
constexpr uint32_t reg_cp_mes_prgrm_cntr_start = 0x2800;
constexpr uint32_t reg_cp_mes_cntl = 0x2807;
constexpr uint32_t reg_cp_mes_gp3_lo = 0x2849;
constexpr uint32_t reg_cp_mes_prgrm_cntr_start_hi = 0x289d;
constexpr uint32_t highest_segment1_register = reg_cp_mes_prgrm_cntr_start_hi;

// gc_11_0_0_sh_mask.h fields.
constexpr uint32_t grbm_gfx_cntl_meid_shift = 2;
constexpr uint32_t mes_cntl_invalidate_icache = 0x00000010u;
constexpr uint32_t mes_cntl_pipe0_reset = 0x00010000u;
constexpr uint32_t mes_cntl_pipe1_reset = 0x00020000u;
constexpr uint32_t mes_cntl_pipe0_active = 0x04000000u;
constexpr uint32_t mes_cntl_pipe1_active = 0x08000000u;
constexpr uint32_t mes_cntl_halt = 0x40000000u;
constexpr uint32_t hqd_vmid_mask = 0x0000000fu;
constexpr uint32_t mqd_control_vmid_mask = 0x0000000fu;
constexpr uint32_t doorbell_offset_mask = 0x0ffffffcu;
constexpr uint32_t doorbell_en = 0x40000000u;
constexpr uint32_t doorbell_hit = 0x80000000u;
constexpr uint32_t eop_size_mask = 0x0000003fu;
constexpr uint32_t pq_queue_size_mask = 0x0000003fu;
constexpr uint32_t pq_rptr_block_size_mask = 0x00003f00u;
constexpr uint32_t pq_rptr_block_size_shift = 8;
constexpr uint32_t pq_no_update_rptr = 0x08000000u;
constexpr uint32_t pq_unord_dispatch = 0x10000000u;
constexpr uint32_t pq_tunnel_dispatch = 0x20000000u;
constexpr uint32_t pq_priv_state = 0x40000000u;
constexpr uint32_t pq_kmd_queue = 0x80000000u;
constexpr uint32_t persistent_preload_size_mask = 0x0003ff00u;
constexpr uint32_t persistent_preload_size_shift = 8;
constexpr uint32_t rlc_cp_schedulers_scheduler0_mask = 0x000000ffu;

// gc_11_0_0_default.h.
constexpr uint32_t cp_hqd_eop_control_default = 0x00000006u;
constexpr uint32_t cp_mqd_control_default = 0x00000100u;
constexpr uint32_t cp_hqd_pq_control_default = 0x00308509u;
constexpr uint32_t cp_hqd_persistent_state_default = 0x0be05501u;
constexpr uint32_t cp_hqd_ib_control_default = 0x00300000u;
constexpr uint32_t cp_hqd_iq_timer_default = 0x00000000u;
constexpr uint32_t cp_hqd_quantum_default = 0x00000000u;

// struct v11_compute_mqd byte offsets, measured by compiling the pinned
// v11_structs.h with offsetof (sizeof is 2048).
namespace mqd {
constexpr uint32_t header = 0;
constexpr uint32_t compute_pipelinestat_enable = 44;
constexpr uint32_t static_thread_mgmt_se0 = 92;
constexpr uint32_t static_thread_mgmt_se1 = 96;
constexpr uint32_t static_thread_mgmt_se2 = 104;
constexpr uint32_t static_thread_mgmt_se3 = 108;
constexpr uint32_t compute_misc_reserved = 128;
constexpr uint32_t cp_mqd_base_addr_lo = 512;
constexpr uint32_t cp_mqd_base_addr_hi = 516;
constexpr uint32_t cp_hqd_active = 520;
constexpr uint32_t cp_hqd_vmid = 524;
constexpr uint32_t cp_hqd_persistent_state = 528;
constexpr uint32_t cp_hqd_quantum = 540;
constexpr uint32_t cp_hqd_pq_base_lo = 544;
constexpr uint32_t cp_hqd_pq_base_hi = 548;
constexpr uint32_t cp_hqd_pq_rptr = 552;
constexpr uint32_t cp_hqd_pq_rptr_report_addr_lo = 556;
constexpr uint32_t cp_hqd_pq_rptr_report_addr_hi = 560;
constexpr uint32_t cp_hqd_pq_wptr_poll_addr_lo = 564;
constexpr uint32_t cp_hqd_pq_wptr_poll_addr_hi = 568;
constexpr uint32_t cp_hqd_pq_doorbell_control = 572;
constexpr uint32_t cp_hqd_pq_control = 580;
constexpr uint32_t cp_hqd_ib_control = 596;
constexpr uint32_t cp_hqd_iq_timer = 600;
constexpr uint32_t cp_mqd_control = 648;
constexpr uint32_t cp_hqd_eop_base_addr_lo = 660;
constexpr uint32_t cp_hqd_eop_base_addr_hi = 664;
constexpr uint32_t cp_hqd_eop_control = 668;
constexpr uint32_t cp_hqd_pq_wptr_lo = 728;
constexpr uint32_t cp_hqd_pq_wptr_hi = 732;
} // namespace mqd

// mes_v11_api_def.h (pragma pack 4) byte offsets, measured with offsetof; every
// API union is API_FRAME_SIZE_IN_DWORDS (256 bytes).
namespace api {
constexpr uint32_t type_scheduler = 1;
constexpr uint32_t op_set_hw_rsrc = 0;
constexpr uint32_t op_add_queue = 2;
constexpr uint32_t op_remove_queue = 3;
constexpr uint32_t op_suspend = 6;
constexpr uint32_t op_resume = 7;
constexpr uint32_t op_query_scheduler_status = 11;
// MES_API_HEADER: type 3:0, opcode 11:4, dwsize 19:12.
constexpr uint32_t header(uint32_t opcode) noexcept {
    return type_scheduler | (opcode << 4) | ((mes11_api_frame_bytes / 4) << 12);
}
// MES_API_STATUS: api_completion_fence_addr then _value, both 64-bit.
constexpr uint32_t status_value = 8;

namespace hw {
constexpr uint32_t vmid_mask_mmhub = 4;
constexpr uint32_t vmid_mask_gfxhub = 8;
constexpr uint32_t gds_size = 12;
constexpr uint32_t paging_vmid = 16;
constexpr uint32_t compute_hqd_mask = 20; // [MAX_COMPUTE_PIPES = 8]
constexpr uint32_t gfx_hqd_mask = 52;     // [MAX_GFX_PIPES = 2]
constexpr uint32_t sdma_hqd_mask = 60;    // [MAX_SDMA_PIPES = 2]
constexpr uint32_t aggregated_doorbells = 68; // [AMD_PRIORITY_NUM_LEVELS = 5]
constexpr uint32_t g_sch_ctx_gpu_mc_ptr = 88;
constexpr uint32_t query_status_fence_gpu_mc_ptr = 96;
constexpr uint32_t gc_base = 104;    // [MES_MAX_HWIP_SEGMENT = 8]
constexpr uint32_t mmhub_base = 136; // [8]
constexpr uint32_t osssys_base = 168; // [8]
constexpr uint32_t api_status = 200;
constexpr uint32_t flags = 216;
constexpr uint32_t oversubscription_timer = 220;
// disable_reset bit0, use_different_vmid_compute bit1, disable_mes_log bit2,
// enable_level_process_quantum_check bit6, enable_reg_active_poll bit10.
constexpr uint32_t flag_value = 0x00000447u;
} // namespace hw

namespace add {
constexpr uint32_t process_id = 4;
constexpr uint32_t page_table_base_addr = 8;
constexpr uint32_t process_va_start = 16;
constexpr uint32_t process_va_end = 24;
constexpr uint32_t process_quantum = 32;
constexpr uint32_t process_context_addr = 40;
constexpr uint32_t gang_quantum = 48;
constexpr uint32_t gang_context_addr = 56;
constexpr uint32_t inprocess_gang_priority = 64;
constexpr uint32_t gang_global_priority_level = 68;
constexpr uint32_t gds_size = 116;
constexpr uint32_t vm_context_cntl = 140;
constexpr uint32_t doorbell_offset = 72;
constexpr uint32_t mqd_addr = 76;
constexpr uint32_t wptr_addr = 84;
constexpr uint32_t queue_type = 108;
constexpr uint32_t flags = 144;
constexpr uint32_t map_legacy_kq = 1u << 13;
constexpr uint32_t api_status = 148;
constexpr uint32_t pipe_id = 192;
constexpr uint32_t queue_id = 196;
} // namespace add

namespace remove {
constexpr uint32_t doorbell_offset = 4;
constexpr uint32_t gang_context_addr = 8;
constexpr uint32_t flags = 16;
constexpr uint32_t preempt_legacy_gfx_queue = 1u << 2;
constexpr uint32_t unmap_legacy_queue = 1u << 3;
constexpr uint32_t api_status = 20;
constexpr uint32_t pipe_id = 36;
constexpr uint32_t queue_id = 40;
constexpr uint32_t tf_addr = 44;
constexpr uint32_t tf_data = 52;
constexpr uint32_t queue_type = 56;
} // namespace remove

namespace query {
constexpr uint32_t api_status = 8; // after header and bool mes_healthy
} // namespace query

// MESAPI__SUSPEND / MESAPI__RESUME: the suspend_all_gangs / resume_all_gangs bit
// is bit 0 of dword 1; gang_context_addr at 8 (zero for "all").
namespace gang {
constexpr uint32_t all_flags = 4;
constexpr uint32_t all_gangs = 1;
constexpr uint32_t suspend_api_status = 28;
constexpr uint32_t resume_api_status = 16;
} // namespace gang
} // namespace api

// nvd.h values not in pm4.hpp (candidates to move there).
constexpr uint32_t pm4_nop = 0x10;
constexpr uint32_t map_queues_queue_shift = 13;
constexpr uint32_t map_queues_pipe_shift = 16;
constexpr uint32_t map_queues_me_shift = 18;
constexpr uint32_t map_queues_engine_sel_shift = 26;
constexpr uint32_t map_queues_num_queues_shift = 29;
constexpr uint32_t map_queues_doorbell_offset_shift = 2;
constexpr uint32_t set_resources_unmap_latency_shift = 16;
// gfx_v11_0_ring_emit_wreg for AMDGPU_RING_TYPE_KIQ: "no inc addr".
constexpr uint32_t write_data_kiq_control = 1u << 16;
// gfx11_kiq_map_queues for AMDGPU_RING_TYPE_MES.
constexpr uint32_t map_queues_mes_me = 2;
constexpr uint32_t map_queues_mes_engine_sel = 5;
// gfx11_kiq_set_resources: unmap_latency 0xa (about 1 s).
constexpr uint32_t set_resources_unmap_latency = 0xa;

// mes_v11_0_kiq_setting and soc21_grbm_select ME 3 (MES).
constexpr uint32_t mes_me = 3;
// mes_v11_0_enable: udelay(500) outside emulation.
constexpr uint32_t enable_settle_microseconds = 500;
// gfx_v11_0_ring_test_ring.
constexpr uint32_t ring_test_seed = 0xcafedeadu;
constexpr uint32_t ring_test_value = 0xdeadbeefu;
// amdgpu_fence_driver_init_ring: num_fences_mask = 2 * 8 - 1 for MES.
constexpr uint32_t mes_fences_mask = 2u * 8u - 1u;
// amdgpu_fence_wait_polling: udelay(2) per iteration.
constexpr uint32_t fence_poll_step = 2;
// gfx_v11_0_set_gds_init.
constexpr uint32_t gds_size = 0x1000;
// amdgpu_mes_init.
constexpr uint32_t vmid_mask = 0xffffff00u;
constexpr uint32_t compute_hqd_mask = 0xc;
constexpr uint32_t gfx_pipe0_hqd_mask = 0xfffffffeu;
constexpr uint32_t sdma6_hqd_mask = 0xfc;
// gfx_v11_0_sw_init GC 11.0.0: num_pipe_per_mec.
constexpr uint32_t mec_pipes = 4;
constexpr uint32_t oversubscription_timer = 50;
// amdgpu_doorbell_init/create_kernel_doorbells with soc21 max_assignment.
constexpr uint32_t doorbell_max_assignment = 0x190u << 1;
constexpr uint32_t doorbell_page_dwords = 4096 / 4;

// Writeback slots (amdgpu_device_wb_get returns 8-dword slots).
constexpr uint32_t slot_bytes = 32;
constexpr uint32_t slot_rptr = 0;
constexpr uint32_t slot_wptr = 1;
constexpr uint32_t slot_fence = 2;
constexpr uint32_t slot_cond_exec = 4;
constexpr uint32_t kiq_slots = 0;
constexpr uint32_t scheduler_slots = 5;
constexpr uint32_t slot_sch_ctx = 10;
constexpr uint32_t slot_query_status_fence = 12;
constexpr uint32_t slot_api_status = 15;

constexpr uint32_t maximum_polls = 1000000;
constexpr uint32_t maximum_api_timeout = 100000000;
constexpr uint64_t gpu_address_limit = 1ULL << 48;

// amdgpu_mes_self_test constants.
namespace self_test {
// AMDGPU_MES_PRIORITY_LEVEL_NORMAL (gang priority, in-process priority and
// global priority level, as amdgpu_mes_test_create_gang_and_queues sets them).
constexpr uint32_t priority_normal = 1;
// amdgpu_mes.h AMDGPU_MES_API_VERSION_MASK/SHIFT: ADD_QUEUE wptr_addr is the
// VMID0 MC address from API version 2 on, the process VA before.
constexpr uint32_t api_version_mask = 0x00fff000u;
constexpr uint32_t api_version_shift = 12;
constexpr uint32_t api_version_wptr_mc = 2;
// amdgpu_mes_doorbell_init: one doorbell page of 8-byte doorbells (512 bits),
// the first AMDGPU_MES_PRIORITY_NUM_LEVELS bits taken by aggregated doorbells.
constexpr uint32_t doorbell_bits = 4096 / 8;
constexpr uint32_t aggregated_levels = 5;
// amdgpu_ring_to_queue_props: AMDGPU_GFX_PIPE_PRIO_NORMAL (AMDGPU_RING_PRIO_1)
// and AMDGPU_GFX_QUEUE_PRIORITY_MINIMUM.
constexpr uint32_t pipe_priority_normal = 1;
constexpr uint32_t queue_priority_minimum = 0;
// gc_11_0_0_offset.h BASE_IDX 0: GCVM_CONTEXT15_CNTL (last context written by
// gfxhub_v3_0_setup_vmid_config) and SDMA0_QUEUE0_IB_CNTL.
constexpr uint32_t reg_gcvm_context15_cntl = 0x1697;
constexpr uint32_t reg_sdma0_queue0_ib_cntl = 0x008a;
// GCVM_CONTEXT1_CNTL ENABLE_CONTEXT | PAGE_TABLE_DEPTH | PAGE_TABLE_BLOCK_SIZE
// and the self-test VM's values (num_level 3, block_size 9 -> field 0).
constexpr uint32_t vm_structure_mask = 0x0000007fu;
constexpr uint32_t vm_structure = 0x00000007u;
// Engine data per queue index (gfx, compute, SDMA): AMDGPU_RING_TYPE_* equals
// MES_QUEUE_TYPE_* and AMDGPU_HW_IP_* here. me/pipe are adev->gfx.gfx_ring[0]
// (ME0 pipe0) and compute_ring[0] (MEC1 = ME1, pipe 0) as amdgpu_mes_add_ring
// copies them; the SDMA ring keeps me 0 from kzalloc.
struct Engine final {
    Mes11QueueType type;
    uint32_t me;
    uint32_t mqd_bytes;
    uint32_t ring;
    uint32_t slots;
    uint32_t ib;
    uint32_t padding;
};
constexpr Engine engines[mes11_self_test_queues]{
    {Mes11QueueType::gfx, 0, mes11_mqd_bytes, mes11_ctx::gfx_ring, mes11_ctx::gfx_slots,
     mes11_ctx::gfx_ib, mes11_ctx::gfx_padding},
    {Mes11QueueType::compute, 1, mes11_mqd_bytes, mes11_ctx::compute_ring,
     mes11_ctx::compute_slots, mes11_ctx::compute_ib, mes11_ctx::compute_padding},
    {Mes11QueueType::sdma, 0, mes11_sdma_mqd_bytes, mes11_ctx::sdma_ring, mes11_ctx::sdma_slots,
     mes11_ctx::sdma_ib, mes11_ctx::sdma_padding}};
} // namespace self_test

// sdma_v6_0.c, sdma_v6_0_0_pkt_open.h and gc_11_0_0 SDMA0_QUEUE0 fields.
namespace sdma6 {
constexpr uint32_t op_nop = 0;
constexpr uint32_t op_write = 2;
constexpr uint32_t op_indirect = 4;
constexpr uint32_t op_trap = 6;
constexpr uint32_t op_poll_regmem = 8;
constexpr uint32_t op_cond_exe = 9;
constexpr uint32_t subop_write_linear_shift = 8; // SDMA_SUBOP_WRITE_LINEAR is 0
constexpr uint32_t poll_regmem_hdp_flush = 1u << 26;
constexpr uint32_t poll_regmem_func_equal = 3u << 28;
// SDMA_PKT_POLL_REGMEM_DW5_RETRY_COUNT(0xfff) | DW5_INTERVAL(10).
constexpr uint32_t poll_regmem_retry_interval = (0xfffu << 16) | 10u;
constexpr uint32_t trap_context_mask = 0x0fffffffu;
// sdma_v6_0_ring_funcs: align_mask 0xf; MES rings have no amdgpu_sdma_instance,
// so sdma_v6_0_ring_insert_nop never uses burst NOPs for them.
constexpr uint32_t align_mask = 0xf;
// BIF_BX_PF_GPU_HDP_FLUSH_DONE__SDMA0_MASK (nbio_v4_3 ref_and_mask_sdma0) << me 0.
constexpr uint32_t hdp_flush_ref_sdma0 = 0x00000400u;
// sdma_v6_0_ring_test_ib: WRITE_LINEAR (5 dwords) and three NOPs.
constexpr uint32_t ib_test_dwords = 8;
// SDMA0_QUEUE0_RB_CNTL RB_SIZE (shift 1), F32_WPTR_POLL_ENABLE (11),
// RPTR_WRITEBACK_ENABLE (12), RPTR_WRITEBACK_TIMER (16, value 4).
constexpr uint32_t rb_size_shift = 1;
constexpr uint32_t rb_f32_wptr_poll_enable = 1u << 11;
constexpr uint32_t rb_rptr_writeback_enable = 1u << 12;
constexpr uint32_t rb_rptr_writeback_timer = 4u << 16;
// SDMA0_QUEUE0_DOORBELL ENABLE and SDMA0_QUEUE0_DOORBELL_OFFSET OFFSET shift.
constexpr uint32_t doorbell_enable = 1u << 28;
constexpr uint32_t doorbell_offset_shift = 2;
// gc_11_0_0_default.h regSDMA0_QUEUE0_RB_AQL_CNTL/DUMMY_REG defaults.
constexpr uint32_t rb_aql_cntl_default = 0x00004000u;
constexpr uint32_t dummy_reg_default = 0x0000000fu;
// struct v11_sdma_mqd byte offsets, measured with offsetof (sizeof 512).
namespace mqd {
constexpr uint32_t rb_cntl = 0;
constexpr uint32_t rb_base = 4;
constexpr uint32_t rb_base_hi = 8;
constexpr uint32_t rb_rptr_addr_hi = 28;
constexpr uint32_t rb_rptr_addr_lo = 32;
constexpr uint32_t ib_cntl = 36;
constexpr uint32_t skip_cntl = 60;
constexpr uint32_t context_status = 64;
constexpr uint32_t doorbell = 68;
constexpr uint32_t doorbell_log = 72;
constexpr uint32_t doorbell_offset = 76;
constexpr uint32_t dummy_reg = 100;
constexpr uint32_t rb_wptr_poll_addr_hi = 104;
constexpr uint32_t rb_wptr_poll_addr_lo = 108;
constexpr uint32_t rb_aql_cntl = 112;
} // namespace mqd
} // namespace sdma6

uint32_t low(uint64_t value) noexcept { return static_cast<uint32_t>(value); }
uint32_t high(uint64_t value) noexcept { return static_cast<uint32_t>(value >> 32); }

// order_base_2 for the exact powers of two used here.
uint32_t log2_exact(uint32_t value) noexcept {
    uint32_t order = 0;

    while ((1u << order) < value) {
        ++order;
    }

    return order;
}

bool put32(sdma::CommandBuffer bytes, uint32_t offset, uint32_t value) noexcept {
    return bytes.write_u32(offset, value);
}

bool put64(sdma::CommandBuffer bytes, uint32_t offset, uint64_t value) noexcept {
    return bytes.write_u32(offset, low(value)) && bytes.write_u32(offset + 4, high(value));
}

bool overlaps(uint64_t a, uint64_t a_bytes, uint64_t b, uint64_t b_bytes) noexcept {
    return a < b + b_bytes && b < a + a_bytes;
}

// Find the actual local base count by asking for six down to one segments.
DiscoveryStatus probe_bases(ByteView binary, uint16_t hardware_id, uint8_t major,
                            uint8_t minor, uint32_t (&bases)[Mes11Registers::hw_resource_bases]) noexcept {
    for (uint8_t count = discovery_max_segments; count > 0; --count) {
        DiscoveredIp ip;
        const DiscoveryStatus status =
            discover_ip(binary, {hardware_id, 0, major, minor, 0, count}, ip);

        if (status == DiscoveryStatus::validated) {
            for (size_t index = 0; index < Mes11Registers::hw_resource_bases; ++index) {
                bases[index] = ip.segment(index);
            }

            return status;
        }

        if (status != DiscoveryStatus::unsupported) {
            return status;
        }
    }

    return DiscoveryStatus::unsupported;
}

// amdgpu_ring_write of one SDMA dword.
bool sdma_write(Gfx11RingWriter& writer, uint32_t value) noexcept {
    return gfx11_ring_write(writer, value);
}

void set_bit(uint64_t* bits, uint32_t bit) noexcept {
    bits[bit / 64] |= 1ULL << (bit % 64);
}

void clear_bit(uint64_t* bits, uint32_t bit) noexcept {
    bits[bit / 64] &= ~(1ULL << (bit % 64));
}

bool test_bit(const uint64_t* bits, uint32_t bit) noexcept {
    return (bits[bit / 64] & (1ULL << (bit % 64))) != 0;
}

bool valid_priority(uint32_t level) noexcept { return level < mes11_priority_levels; }

bool valid_page_address(uint64_t address) noexcept {
    return address != 0 && (address & (mes11_page_alignment - 1)) == 0 &&
           address < gpu_address_limit;
}

bool valid_qword_address(uint64_t address) noexcept {
    return address != 0 && (address & 7u) == 0 && address < gpu_address_limit;
}

} // namespace

bool mes11_sdma_mqd_init(uint32_t ib_control, const Gfx11MqdProperties& properties,
                         sdma::CommandBuffer mqd) noexcept {
    bool ok = mqd.size() >= mes11_sdma_mqd_bytes && properties.queue_bytes >= 1024 &&
              (properties.queue_bytes & (properties.queue_bytes - 1)) == 0;

    for (uint32_t offset = 0; ok && offset < mes11_sdma_mqd_bytes; offset += 4) {
        ok = mqd.write_u32(offset, 0);
    }

    const uint32_t rb_cntl = (log2_exact(properties.queue_bytes / 4) << sdma6::rb_size_shift) |
                             sdma6::rb_rptr_writeback_enable | sdma6::rb_rptr_writeback_timer |
                             sdma6::rb_f32_wptr_poll_enable;
    const uint64_t ring = properties.hqd_base_address >> 8;
    return ok && put32(mqd, sdma6::mqd::rb_cntl, rb_cntl) &&
        put32(mqd, sdma6::mqd::rb_base, low(ring)) &&
        put32(mqd, sdma6::mqd::rb_base_hi, high(ring)) &&
        put32(mqd, sdma6::mqd::rb_wptr_poll_addr_lo, low(properties.wptr_address)) &&
        put32(mqd, sdma6::mqd::rb_wptr_poll_addr_hi, high(properties.wptr_address)) &&
        put32(mqd, sdma6::mqd::rb_rptr_addr_lo, low(properties.rptr_address)) &&
        put32(mqd, sdma6::mqd::rb_rptr_addr_hi, high(properties.rptr_address)) &&
        put32(mqd, sdma6::mqd::ib_cntl, ib_control) &&
        put32(mqd, sdma6::mqd::doorbell_offset,
              properties.doorbell_index << sdma6::doorbell_offset_shift) &&
        put32(mqd, sdma6::mqd::doorbell, sdma6::doorbell_enable) &&
        put32(mqd, sdma6::mqd::skip_cntl, 0) && put32(mqd, sdma6::mqd::context_status, 0) &&
        put32(mqd, sdma6::mqd::doorbell_log, 0) &&
        put32(mqd, sdma6::mqd::rb_aql_cntl, sdma6::rb_aql_cntl_default) &&
        put32(mqd, sdma6::mqd::dummy_reg, sdma6::dummy_reg_default);
}

bool mes11_sdma_ring_pad(Gfx11RingWriter& writer) noexcept {
    const uint32_t count = (sdma6::align_mask + 1 - (low(writer.wptr) & sdma6::align_mask)) &
                           sdma6::align_mask;
    bool ok = true;

    for (uint32_t pad = 0; ok && pad < count; ++pad) {
        ok = sdma_write(writer, sdma6::op_nop);
    }

    return ok;
}

bool mes11_sdma_emit_ib_frame(Gfx11RingWriter& writer, const Mes11SdmaIbFrame& frame) noexcept {
    const uint32_t ring_dwords = static_cast<uint32_t>(writer.ring.size() / 4);

    if (ring_dwords == 0 || (ring_dwords & (ring_dwords - 1)) != 0) {
        return false;
    }

    // sdma_v6_0_ring_init_cond_exec on cond_exe_gpu_addr; the count dword is
    // patched by amdgpu_ring_patch_cond_exec after the fence.
    bool ok = sdma_write(writer, sdma6::op_cond_exe) &&
              sdma_write(writer, low(frame.cond_exec_address)) &&
              sdma_write(writer, high(frame.cond_exec_address)) && sdma_write(writer, 1);
    const uint64_t cond_count = writer.wptr;
    ok = ok && sdma_write(writer, 0);
    // sdma_v6_0_ring_emit_hdp_flush: POLL_REGMEM HDP_FLUSH FUNC(3) on the
    // NBIO DONE/REQ byte addresses with ref_and_mask_sdma0 << me (0).
    ok = ok && sdma_write(writer, sdma6::op_poll_regmem | sdma6::poll_regmem_hdp_flush |
                                sdma6::poll_regmem_func_equal) &&
         sdma_write(writer, frame.hdp_flush_done << 2) &&
         sdma_write(writer, frame.hdp_flush_req << 2) &&
         sdma_write(writer, sdma6::hdp_flush_ref_sdma0) &&
         sdma_write(writer, sdma6::hdp_flush_ref_sdma0) &&
         sdma_write(writer, sdma6::poll_regmem_retry_interval);
    // sdma_v6_0_ring_emit_ib: NOPs so the 6-dword INDIRECT ends on 8 dwords,
    // vmid 0, base & ~0x1f, CSA 0 (vmid 0: no MCBP CSA).
    const uint32_t nops = (2u - low(writer.wptr)) & 7u;

    for (uint32_t pad = 0; ok && pad < nops; ++pad) {
        ok = sdma_write(writer, sdma6::op_nop);
    }

    ok = ok && sdma_write(writer, sdma6::op_indirect) &&
         sdma_write(writer, low(frame.ib_address) & 0xffffffe0u) &&
         sdma_write(writer, high(frame.ib_address)) && sdma_write(writer, frame.ib_dwords) &&
         sdma_write(writer, 0) && sdma_write(writer, 0);
    // sdma_v6_0_ring_emit_fence (32-bit, MTYPE UC) via the shared encoder, then
    // the AMDGPU_FENCE_FLAG_INT trap with the MES queue context.
    uint8_t fence[sdma::fence32_packet_bytes]{};
    sdma::EncodeResult encoded{};
    ok = ok && sdma::encode_fence({{{frame.fence_address, 4}, 0}, frame.fence_sequence,
                                   sdma::FenceWidth::bits32, false},
                                  sdma::CommandBuffer(fence, sizeof(fence)), encoded) ==
                   sdma::EncodeStatus::success;

    for (uint32_t offset = 0; ok && offset < sizeof(fence); offset += 4) {
        uint32_t word = 0;
        ok = ByteView(fence, sizeof(fence)).read_u32(offset, word) && sdma_write(writer, word);
    }

    ok = ok && sdma_write(writer, sdma6::op_trap) &&
         sdma_write(writer, (frame.hw_queue_id | gfx11_fence_mes_queue_flag) &
                                sdma6::trap_context_mask);
    return ok && writer.ring.write_u32(static_cast<size_t>(cond_count & (ring_dwords - 1)) * 4,
                                       static_cast<uint32_t>(writer.wptr - 1 - cond_count));
}

DiscoveryStatus validate_mes11_discovery(ByteView binary, uint32_t aperture,
                                         Mes11Registers& output) noexcept {
    output = {};

    if (aperture == 0 || (aperture & 3u) != 0) {
        return DiscoveryStatus::invalid_argument;
    }

    Mes11Registers result;
    DiscoveryStatus status =
        discover_ip(binary, {discovery_gc_hardware_id, 0, 11, 0, 0, 2}, result.gc_);

    if (status != DiscoveryStatus::validated) {
        return status;
    }

    uint32_t offset = 0;

    if (!discovery_register_offset(result.gc_.segment(0), highest_segment0_register, aperture,
                                   offset) ||
        !discovery_register_offset(result.gc_.segment(1), highest_segment1_register, aperture,
                                   offset) ||
        !discovery_register_offset(result.gc_.segment(1), reg_grbm_gfx_cntl, aperture, offset)) {
        return DiscoveryStatus::invalid_argument;
    }

    status = probe_bases(binary, discovery_gc_hardware_id, 11, 0, result.gc_bases_);

    if (status == DiscoveryStatus::validated) {
        status = probe_bases(binary, discovery_mmhub_hardware_id, 3, 0, result.mmhub_bases_);
    }

    if (status == DiscoveryStatus::validated) {
        status = probe_bases(binary, mes11_osssys_hardware_id, 6, 0, result.osssys_bases_);
    }

    if (status != DiscoveryStatus::validated) {
        return status;
    }

    result.aperture_bytes_ = aperture;
    output = result;
    return DiscoveryStatus::validated;
}

bool mes11_uc_start_address(ByteView file, uint64_t& address) noexcept {
    constexpr size_t header_bytes = 64;
    uint32_t header_size = 0;
    uint16_t major = 0;
    uint32_t start_low = 0;
    uint32_t start_high = 0;

    if (!file.read_u32(4, header_size) || !file.read_u16(8, major) ||
        !file.read_u32(56, start_low) || !file.read_u32(60, start_high) ||
        major != 1 || header_size < header_bytes || header_size > file.size()) {
        return false;
    }

    address = (static_cast<uint64_t>(start_high) << 32) | start_low;
    return true;
}

Mes11Status Mes11::finish(Mes11Status status) noexcept {
    report_.status = status;
    return status;
}

Mes11Status Mes11::quarantine(Mes11Status reason) noexcept {
    phase_ = Phase::faulted;
    report_.scheduler_ready = false;
    report_.kiq_ring_ready = false;
    return finish(reason);
}

bool Mes11::read(uint32_t segment, uint32_t reg, uint32_t& value) noexcept {
    return read_ip_register(io_, registers_.gc_segment(segment), reg, value) &&
           value != UINT32_MAX;
}

bool Mes11::write(uint32_t segment, uint32_t reg, uint32_t value) noexcept {
    return write_ip_register(io_, registers_.gc_segment(segment), reg, value);
}

// soc21_grbm_select(adev, me, pipe, 0, 0).
bool Mes11::grbm_select(uint32_t me, uint32_t pipe) noexcept {
    return write(1, reg_grbm_gfx_cntl, pipe | (me << grbm_gfx_cntl_meid_shift));
}

Mes11Status Mes11::initialize(Mes11Registers registers, RegisterIo io, DoorbellIo doorbells,
                              Nbio& nbio, VramLocation vram, Mes11Config config,
                              Mes11Policy policy, Mes11Recovery recovery) noexcept {
    if (phase_ != Phase::uninitialized && phase_ != Phase::ready) {
        return phase_ == Phase::faulted ? Mes11Status::quarantined : Mes11Status::busy;
    }

    if (report_.memory_retained || self_test_.memory_retained) {
        return Mes11Status::busy;
    }

    phase_ = Phase::uninitialized;
    report_ = {};
    self_test_ = Mes11SelfTestReport();
    self_test_done_ = false;

    if (!registers.valid()) {
        return finish(Mes11Status::unsupported_ip);
    }

    if (!register_io_valid(io) || io.aperture_bytes != registers.aperture_bytes() ||
        !doorbell_io_valid(doorbells) || recovery.confirm_reset_quiescence == nullptr) {
        return finish(Mes11Status::unavailable);
    }

    // amdgpu_doorbell_init/create_kernel_doorbells: kernel doorbells are
    // min(BAR dwords, max_assignment + 1) plus one page, page aligned; the MES
    // page follows them and holds the five aggregated doorbells.
    const uint32_t bar_dwords = doorbells.aperture_dwords;
    const uint32_t kernel = (bar_dwords < doorbell_max_assignment + 1 ? bar_dwords :
                             doorbell_max_assignment + 1) + doorbell_page_dwords;
    const uint32_t start = (kernel + doorbell_page_dwords - 1) / doorbell_page_dwords *
                           doorbell_page_dwords;

    if (bar_dwords <= mes11_kiq_doorbell_index + 1 || start > bar_dwords ||
        bar_dwords - start < doorbell_page_dwords) {
        return finish(Mes11Status::unavailable);
    }

    if (vram.bytes == 0 || vram.mc_end < vram.mc_start || config.sdma_instances == 0 ||
        config.sdma_instances > 2 || (config.scheduler_uc_start & 3u) != 0 ||
        (config.kiq_uc_start & 3u) != 0 ||
        policy.poll_iterations == 0 || policy.poll_iterations > maximum_polls ||
        policy.api_timeout_microseconds == 0 ||
        policy.api_timeout_microseconds > maximum_api_timeout) {
        return finish(Mes11Status::invalid_argument);
    }

    for (uint32_t level = 0; level < 5; ++level) {
        aggregated_doorbells_[level] = start + level * 2;
    }

    forget_allocations();

    registers_ = registers;
    io_ = io;
    doorbells_ = doorbells;
    nbio_ = &nbio;
    vram_ = vram;
    config_ = config;
    policy_ = policy;
    recovery_ = recovery;
    report_.aggregated_doorbell_base = start;
    phase_ = Phase::ready;
    return finish(Mes11Status::ready);
}

bool Mes11::retain_memory(memory::Manager& manager, const Mes11Memory& memory) noexcept {
    struct Need final {
        memory::Handle handle;
        uint32_t bytes;
    };

    const Need needs[7]{{memory.kiq_ring, mes11_kiq_ring_bytes},
        {memory.scheduler_ring, mes11_scheduler_ring_bytes}, {memory.kiq_mqd, mes11_mqd_bytes},
        {memory.scheduler_mqd, mes11_mqd_bytes}, {memory.kiq_eop, mes11_eop_bytes},
        {memory.scheduler_eop, mes11_eop_bytes}, {memory.writeback, mes11_writeback_bytes}};
    memory::AllocationInfo info[7]{};
    size_t retained = 0;
    bool ok = true;

    for (; retained < 7 && ok; ++retained) {
        ok = retain_local_vram(manager, needs[retained].handle, vram_, needs[retained].bytes,
                               mes11_page_alignment, true, info[retained]);
    }

    if (!ok) {
        --retained;
    }

    for (size_t a = 0; ok && a < 7; ++a) {
        for (size_t b = a + 1; ok && b < 7; ++b) {
            ok = !overlaps(info[a].gpu.virtual_address, needs[a].bytes,
                           info[b].gpu.virtual_address, needs[b].bytes);
        }
    }

    if (!ok) {
        while (retained > 0) {
            --retained;
            manager.unretain(needs[retained].handle);
        }

        return false;
    }

    manager_ = &manager;
    handles_ = memory;
    kiq_ = {info[0].backing.cpu_bytes, info[0].gpu.virtual_address, mes11_kiq_ring_bytes,
            pm4::packet3(pm4_nop, 0x3fff), 0xff, mes11_kiq_doorbell_index, 1, kiq_slots, 0,
            info[2].gpu.virtual_address, info[2].backing.cpu_bytes,
            info[4].gpu.virtual_address, info[4].backing.cpu_bytes};
    scheduler_ = {info[1].backing.cpu_bytes, info[1].gpu.virtual_address,
                  mes11_scheduler_ring_bytes, 0, 1, mes11_scheduler_doorbell_index, 0,
                  scheduler_slots, 0, info[3].gpu.virtual_address, info[3].backing.cpu_bytes,
                  info[5].gpu.virtual_address, info[5].backing.cpu_bytes};
    writeback_cpu_ = info[6].backing.cpu_bytes;
    report_.memory_retained = true;
    report_.kiq_ring_mc = kiq_.mc;
    report_.scheduler_ring_mc = scheduler_.mc;
    report_.kiq_mqd_mc = kiq_.mqd_mc;
    report_.scheduler_mqd_mc = scheduler_.mqd_mc;
    report_.writeback_mc = info[6].gpu.virtual_address;
    return true;
}

void Mes11::release_leases() noexcept {
    if (report_.memory_retained && manager_ != nullptr) {
        const memory::Handle handles[7]{handles_.kiq_ring, handles_.scheduler_ring,
            handles_.kiq_mqd, handles_.scheduler_mqd, handles_.kiq_eop, handles_.scheduler_eop,
            handles_.writeback};

        for (const auto& handle : handles) {
            manager_->unretain(handle);
        }
    }

    manager_ = nullptr;
    handles_ = {};
    kiq_ = {};
    scheduler_ = {};
    writeback_cpu_ = nullptr;
    report_.memory_retained = false;
    report_.kiq_ring_mc = 0;
    report_.scheduler_ring_mc = 0;
    report_.kiq_mqd_mc = 0;
    report_.scheduler_mqd_mc = 0;
    report_.writeback_mc = 0;
}

uint64_t Mes11::slot_mc(uint32_t slot) const noexcept {
    return report_.writeback_mc + static_cast<uint64_t>(slot) * slot_bytes;
}

bool Mes11::write_slot64(uint32_t slot, uint64_t value) noexcept {
    return put64(sdma::CommandBuffer(writeback_cpu_, mes11_writeback_bytes), slot * slot_bytes,
                 value);
}

bool Mes11::read_slot64(uint32_t slot, uint64_t& value) const noexcept {
    const ByteView bytes(writeback_cpu_, mes11_writeback_bytes);
    uint32_t lo = 0;
    uint32_t hi = 0;

    if (!bytes.read_u32(slot * slot_bytes, lo) || !bytes.read_u32(slot * slot_bytes + 4, hi)) {
        return false;
    }

    value = (static_cast<uint64_t>(hi) << 32) | lo;
    return true;
}

// The sw_init-time state: amdgpu_device_wb_init zeroes adev->wb, amdgpu_ring_init
// sets cond_exec to 1 and clears each ring with its NOP, the fence driver writes
// sequence 0, mes_v11_0_allocate_eop_buf and mes_v11_0_mqd_sw_init zero their BOs.
bool Mes11::prepare_memory() noexcept {
    const sdma::CommandBuffer writeback(writeback_cpu_, mes11_writeback_bytes);

    for (uint32_t offset = 0; offset < mes11_writeback_bytes; offset += 4) {
        if (!writeback.write_u32(offset, 0)) {
            return false;
        }
    }

    Ring* const rings[2]{&kiq_, &scheduler_};

    for (Ring* ring : rings) {
        const sdma::CommandBuffer ring_bytes(ring->cpu, ring->bytes);
        const sdma::CommandBuffer eop(ring->eop_cpu, mes11_eop_bytes);
        const sdma::CommandBuffer mqd_bytes(ring->mqd_cpu, mes11_mqd_bytes);

        if (!write_slot64(ring->slot_base + slot_cond_exec, 1)) {
            return false;
        }

        for (uint32_t offset = 0; offset < ring->bytes; offset += 4) {
            if (!ring_bytes.write_u32(offset, ring->nop)) {
                return false;
            }
        }

        for (uint32_t offset = 0; offset < mes11_eop_bytes; offset += 4) {
            if (!eop.write_u32(offset, 0) || !mqd_bytes.write_u32(offset, 0)) {
                return false;
            }
        }
    }

    return nbio_->flush_hdp();
}

// mes_v11_0_enable.
bool Mes11::enable(bool on) noexcept {
    uint32_t data = 0;

    if (!read(1, reg_cp_mes_cntl, data)) {
        return false;
    }

    if (!on) {
        data &= ~(mes_cntl_pipe0_active | mes_cntl_pipe1_active);
        data |= mes_cntl_invalidate_icache | mes_cntl_pipe0_reset | mes_cntl_pipe1_reset |
                mes_cntl_halt;
        report_.mes_enabled = false;
        return write(1, reg_cp_mes_cntl, data);
    }

    // enable_mes_kiq is true on GC 11.0.0, so both pipes are reset and started.
    if (!write(1, reg_cp_mes_cntl, data | mes_cntl_pipe0_reset | mes_cntl_pipe1_reset)) {
        return false;
    }

    const uint64_t starts[2]{config_.scheduler_uc_start >> 2, config_.kiq_uc_start >> 2};

    for (uint32_t pipe = 0; pipe < 2; ++pipe) {
        if (!grbm_select(mes_me, pipe) ||
            !write(1, reg_cp_mes_prgrm_cntr_start, low(starts[pipe])) ||
            !write(1, reg_cp_mes_prgrm_cntr_start_hi, high(starts[pipe]))) {
            return false;
        }
    }

    if (!grbm_select(0, 0) ||
        !write(1, reg_cp_mes_cntl, mes_cntl_pipe0_active | mes_cntl_pipe1_active)) {
        return false;
    }

    report_.mes_enabled = true;
    io_.delay_microseconds(io_.context, enable_settle_microseconds);
    return true;
}

// mes_v11_0_get_fw_version.
bool Mes11::read_versions() noexcept {
    return grbm_select(mes_me, 0) && read(1, reg_cp_mes_gp3_lo, report_.scheduler_version) &&
           grbm_select(mes_me, 1) && read(1, reg_cp_mes_gp3_lo, report_.kiq_version) &&
           grbm_select(0, 0);
}

// mes_v11_0_kiq_setting: tell RLC that ME3 pipe1 queue0 is the KIQ.
bool Mes11::kiq_setting() noexcept {
    uint32_t value = 0;

    if (!read(1, reg_rlc_cp_schedulers, value)) {
        return false;
    }

    value &= ~rlc_cp_schedulers_scheduler0_mask;
    value |= (mes_me << 5) | (kiq_.pipe << 3);
    return write(1, reg_rlc_cp_schedulers, value) &&
           write(1, reg_rlc_cp_schedulers, value | 0x80u);
}

// mes_v11_0_mqd_init, field by field at the measured v11_compute_mqd offsets.
bool Mes11::mqd_init(Ring& ring) noexcept {
    const sdma::CommandBuffer bytes(ring.mqd_cpu, mes11_mqd_bytes);

    for (uint32_t offset = 0; offset < mes11_mqd_bytes; offset += 4) {
        if (!bytes.write_u32(offset, 0)) {
            return false;
        }
    }

    const uint64_t eop = ring.eop_mc >> 8;
    const uint64_t pq = ring.mc >> 8;
    const uint64_t rptr = slot_mc(ring.slot_base + slot_rptr);
    const uint64_t wptr = slot_mc(ring.slot_base + slot_wptr);
    // Register value is 2^(EOP_SIZE+1) dwords.
    const uint32_t eop_control = (cp_hqd_eop_control_default & ~eop_size_mask) |
                                 ((log2_exact(mes11_eop_bytes / 4) - 1) & eop_size_mask);
    // Linux passes (order_base_2(1024) - 1) << 8 to REG_SET_FIELD, which shifts
    // by 8 again and masks, so RPTR_BLOCK_SIZE is written as zero; kept as is.
    const uint32_t block = (((log2_exact(4096 / 4) - 1) << 8) << pq_rptr_block_size_shift) &
                           pq_rptr_block_size_mask;
    const uint32_t pq_control = (cp_hqd_pq_control_default &
        ~(pq_queue_size_mask | pq_rptr_block_size_mask | pq_tunnel_dispatch)) |
        ((log2_exact(ring.bytes / 4) - 1) & pq_queue_size_mask) | block | pq_unord_dispatch |
        pq_priv_state | pq_kmd_queue | pq_no_update_rptr;
    const uint32_t doorbell = ((ring.doorbell << 2) & doorbell_offset_mask) | doorbell_en;
    const uint32_t persistent = (cp_hqd_persistent_state_default & ~persistent_preload_size_mask) |
                                (0x55u << persistent_preload_size_shift);

    ring.wptr = 0;
    return put32(bytes, mqd::header, 0xc0310800u) &&
        put32(bytes, mqd::compute_pipelinestat_enable, 1) &&
        put32(bytes, mqd::static_thread_mgmt_se0, 0xffffffffu) &&
        put32(bytes, mqd::static_thread_mgmt_se1, 0xffffffffu) &&
        put32(bytes, mqd::static_thread_mgmt_se2, 0xffffffffu) &&
        put32(bytes, mqd::static_thread_mgmt_se3, 0xffffffffu) &&
        put32(bytes, mqd::compute_misc_reserved, 7) &&
        put32(bytes, mqd::cp_hqd_eop_base_addr_lo, low(eop)) &&
        put32(bytes, mqd::cp_hqd_eop_base_addr_hi, high(eop)) &&
        put32(bytes, mqd::cp_hqd_eop_control, eop_control) &&
        put32(bytes, mqd::cp_hqd_pq_rptr, 0) && put32(bytes, mqd::cp_hqd_pq_wptr_lo, 0) &&
        put32(bytes, mqd::cp_hqd_pq_wptr_hi, 0) &&
        put32(bytes, mqd::cp_mqd_base_addr_lo, low(ring.mqd_mc) & 0xfffffffcu) &&
        put32(bytes, mqd::cp_mqd_base_addr_hi, high(ring.mqd_mc)) &&
        put32(bytes, mqd::cp_mqd_control, cp_mqd_control_default & ~mqd_control_vmid_mask) &&
        put32(bytes, mqd::cp_hqd_pq_base_lo, low(pq)) &&
        put32(bytes, mqd::cp_hqd_pq_base_hi, high(pq)) &&
        put32(bytes, mqd::cp_hqd_pq_rptr_report_addr_lo, low(rptr) & 0xfffffffcu) &&
        put32(bytes, mqd::cp_hqd_pq_rptr_report_addr_hi, high(rptr) & 0xffffu) &&
        put32(bytes, mqd::cp_hqd_pq_wptr_poll_addr_lo, low(wptr) & 0xfffffff8u) &&
        put32(bytes, mqd::cp_hqd_pq_wptr_poll_addr_hi, high(wptr) & 0xffffu) &&
        put32(bytes, mqd::cp_hqd_pq_control, pq_control) &&
        put32(bytes, mqd::cp_hqd_pq_doorbell_control, doorbell) &&
        put32(bytes, mqd::cp_hqd_vmid, 0) && put32(bytes, mqd::cp_hqd_active, 1) &&
        put32(bytes, mqd::cp_hqd_persistent_state, persistent) &&
        put32(bytes, mqd::cp_hqd_ib_control, cp_hqd_ib_control_default) &&
        put32(bytes, mqd::cp_hqd_iq_timer, cp_hqd_iq_timer_default) &&
        put32(bytes, mqd::cp_hqd_quantum, cp_hqd_quantum_default) && nbio_->flush_hdp();
}

// mes_v11_0_queue_init_register: program the KIQ HQD (ME3 pipe1) from its MQD.
bool Mes11::queue_init_register(const Ring& ring) noexcept {
    const ByteView mqd_bytes(ring.mqd_cpu, mes11_mqd_bytes);
    uint32_t field[11]{};
    const uint32_t offsets[11]{mqd::cp_mqd_base_addr_lo, mqd::cp_mqd_base_addr_hi,
        mqd::cp_hqd_pq_base_lo, mqd::cp_hqd_pq_base_hi, mqd::cp_hqd_pq_rptr_report_addr_lo,
        mqd::cp_hqd_pq_rptr_report_addr_hi, mqd::cp_hqd_pq_control,
        mqd::cp_hqd_pq_wptr_poll_addr_lo, mqd::cp_hqd_pq_wptr_poll_addr_hi,
        mqd::cp_hqd_pq_doorbell_control, mqd::cp_hqd_persistent_state};

    for (size_t index = 0; index < 11; ++index) {
        if (!mqd_bytes.read_u32(offsets[index], field[index])) {
            return false;
        }
    }

    uint32_t active = 0;
    uint32_t data = 0;

    if (!mqd_bytes.read_u32(mqd::cp_hqd_active, active) || !grbm_select(mes_me, ring.pipe) ||
        !read(0, reg_cp_hqd_vmid, data) || !write(0, reg_cp_hqd_vmid, data & ~hqd_vmid_mask) ||
        !read(0, reg_cp_hqd_pq_doorbell_control, data) ||
        !write(0, reg_cp_hqd_pq_doorbell_control, data & ~doorbell_en) ||
        !write(0, reg_cp_mqd_base_addr, field[0]) || !write(0, reg_cp_mqd_base_addr_hi, field[1])) {
        return false;
    }

    // Linux reads CP_MQD_CONTROL, clears VMID, then writes literal 0.
    return read(0, reg_cp_mqd_control, data) && write(0, reg_cp_mqd_control, 0) &&
        write(0, reg_cp_hqd_pq_base, field[2]) && write(0, reg_cp_hqd_pq_base_hi, field[3]) &&
        write(0, reg_cp_hqd_pq_rptr_report_addr, field[4]) &&
        write(0, reg_cp_hqd_pq_rptr_report_addr_hi, field[5]) &&
        write(0, reg_cp_hqd_pq_control, field[6]) &&
        write(0, reg_cp_hqd_pq_wptr_poll_addr, field[7]) &&
        write(0, reg_cp_hqd_pq_wptr_poll_addr_hi, field[8]) &&
        write(0, reg_cp_hqd_pq_doorbell_control, field[9]) &&
        write(0, reg_cp_hqd_persistent_state, field[10]) &&
        write(0, reg_cp_hqd_active, active) && grbm_select(0, 0);
}

// amdgpu_ring_write.
bool Mes11::ring_write(Ring& ring, uint32_t value) noexcept {
    const uint64_t index = ring.wptr & (ring.bytes / 4 - 1);

    if (!sdma::CommandBuffer(ring.cpu, ring.bytes).write_u32(static_cast<size_t>(index) * 4,
                                                              value)) {
        return false;
    }

    ++ring.wptr;
    return true;
}

// amdgpu_ring_commit plus mes_v11_0_ring_set_wptr / gfx_v11_0_ring_set_wptr_compute.
bool Mes11::ring_commit(Ring& ring) noexcept {
    const uint32_t pad = (ring.align_mask + 1 - (low(ring.wptr) & ring.align_mask)) &
                         ring.align_mask;

    for (uint32_t index = 0; index < pad; ++index) {
        if (!ring_write(ring, ring.nop)) {
            return false;
        }
    }

    // atomic64_set(wptr_cpu_addr); the HDP flush publishes the VRAM ring and
    // writeback before the doorbell (GTT in Linux needs only mb()).
    if (!write_slot64(ring.slot_base + slot_wptr, ring.wptr) || !nbio_->flush_hdp()) {
        return false;
    }

    if (&ring == &kiq_) {
        report_.kiq_wptr = ring.wptr;
    } else {
        report_.scheduler_wptr = ring.wptr;
    }

    return doorbells_.write64(doorbells_.context, ring.doorbell, ring.wptr);
}

// gfx_v11_0_ring_test_ring for the KIQ ring, committing whatever the caller
// already placed on the ring (amdgpu_ring_test_helper after kiq_map_queues).
Mes11Status Mes11::kiq_ring_test() noexcept {
    const uint32_t scratch = registers_.gc_segment(1) + reg_scratch_reg0;

    if (!write(1, reg_scratch_reg0, ring_test_seed)) {
        return Mes11Status::mmio_failure;
    }

    if (!ring_write(kiq_, pm4::packet3(pm4::write_data, 3)) ||
        !ring_write(kiq_, write_data_kiq_control) || !ring_write(kiq_, scratch) ||
        !ring_write(kiq_, 0) || !ring_write(kiq_, ring_test_value)) {
        return Mes11Status::invalid_memory;
    }

    if (!ring_commit(kiq_)) {
        return Mes11Status::doorbell_failure;
    }

    for (uint32_t poll = 0; poll < policy_.poll_iterations; ++poll) {
        uint32_t value = 0;

        if (!read(1, reg_scratch_reg0, value)) {
            return Mes11Status::mmio_failure;
        }

        if (value == ring_test_value) {
            report_.kiq_ring_ready = true;
            return Mes11Status::completed;
        }

        io_.delay_microseconds(io_.context, 1);
    }

    report_.kiq_ring_ready = false;
    return Mes11Status::ring_test_failed;
}

// amdgpu_fence_wait_polling on the MES ring fence (32-bit sequence compare).
bool Mes11::wait_fence(uint32_t sequence, uint32_t& remaining) noexcept {
    const ByteView bytes(writeback_cpu_, mes11_writeback_bytes);
    const uint32_t fence_offset = (scheduler_.slot_base + slot_fence) * slot_bytes;

    for (;;) {
        uint32_t value = 0;

        if (!bytes.read_u32(fence_offset, value)) {
            return false;
        }

        const uint32_t distance = sequence - value;

        if (distance == 0 || distance >= 0x80000000u || remaining == 0) {
            return remaining > 0;
        }

        io_.delay_microseconds(io_.context, fence_poll_step);
        remaining = remaining > fence_poll_step ? remaining - fence_poll_step : 0;
    }
}


// mes_v11_0_submit_pkt_and_poll_completion: the API frame, then a
// QUERY_SCHEDULER_STATUS frame that writes the ring fence, one commit. frame is
// a zero-initialized 256-byte union image built field by field by the caller.
Mes11Status Mes11::submit(uint8_t (&frame)[mes11_api_frame_bytes],
                          uint32_t api_status_offset) noexcept {
    uint8_t query[mes11_api_frame_bytes]{};
    const sdma::CommandBuffer frame_out(frame, sizeof(frame));
    const sdma::CommandBuffer query_out(query, sizeof(query));
    uint32_t header = 0;

    if (phase_ == Phase::faulted) {
        return Mes11Status::quarantined;
    }

    // Linux rejects header.opcode >= MES_SCH_API_MAX before touching the ring.
    if (!ByteView(frame, sizeof(frame)).read_u32(0, header) || ((header >> 4) & 0xffu) >= 0xffu ||
        api_status_offset > mes11_api_frame_bytes - 16 || !write_slot64(slot_api_status, 0)) {
        return finish(Mes11Status::invalid_argument);
    }

    const uint32_t sequence = report_.fence_sequence + 1;
    uint32_t remaining = policy_.api_timeout_microseconds;

    // Linux undoes the ring and returns the polling result here; a full ring is
    // impossible with synchronous completion, so a wait failure is a fault.
    if (!wait_fence(sequence - mes_fences_mask, remaining)) {
        return quarantine(Mes11Status::timeout);
    }

    report_.fence_sequence = sequence;
    report_.last_api_opcode = (header >> 4) & 0xffu;

    if (!put64(frame_out, api_status_offset, slot_mc(slot_api_status)) ||
        !put64(frame_out, api_status_offset + api::status_value, 1) ||
        !put32(query_out, 0, api::header(api::op_query_scheduler_status)) ||
        !put64(query_out, api::query::api_status, slot_mc(scheduler_.slot_base + slot_fence)) ||
        !put64(query_out, api::query::api_status + api::status_value, sequence)) {
        return quarantine(Mes11Status::invalid_memory);
    }

    // amdgpu_ring_write_multiple of both frames.
    const uint8_t* const sources[2]{frame, query};

    for (const uint8_t* source : sources) {
        const ByteView words(source, mes11_api_frame_bytes);

        for (uint32_t offset = 0; offset < mes11_api_frame_bytes; offset += 4) {
            uint32_t value = 0;

            if (!words.read_u32(offset, value) || !ring_write(scheduler_, value)) {
                return quarantine(Mes11Status::invalid_memory);
            }
        }
    }

    if (!ring_commit(scheduler_)) {
        return quarantine(Mes11Status::doorbell_failure);
    }

    ++report_.api_submissions;
    remaining = policy_.api_timeout_microseconds;
    uint64_t status = 0;

    // MES may still complete later and write the fence and status slots: keep
    // every lease (quarantine) instead of reusing them.
    if (!wait_fence(sequence, remaining) || !read_slot64(slot_api_status, status) ||
        status == 0) {
        return quarantine(Mes11Status::timeout);
    }

    return Mes11Status::completed;
}

Mes11Status Mes11::kiq_hw_init(memory::Manager& manager, const Mes11Memory& memory) noexcept {
    if (phase_ != Phase::ready) {
        return phase_ == Phase::faulted ? Mes11Status::quarantined :
               phase_ == Phase::uninitialized ? Mes11Status::unavailable : Mes11Status::busy;
    }

    if (!retain_memory(manager, memory)) {
        return finish(Mes11Status::invalid_memory);
    }

    // From here the HDP flush and MES registers may have changed device state.
    if (!prepare_memory() || !enable(true) || !read_versions() || !kiq_setting()) {
        return quarantine(Mes11Status::mmio_failure);
    }

    // mes_v11_0_queue_init(AMDGPU_MES_KIQ_PIPE): MQD, then direct HQD registers.
    if (!mqd_init(kiq_) || !queue_init_register(kiq_)) {
        return quarantine(Mes11Status::mmio_failure);
    }

    report_.legacy_queue_map = (report_.scheduler_version & 0xfffu) >=
                               mes11_legacy_queue_map_version;
    phase_ = Phase::kiq;

    if (!report_.legacy_queue_map) {
        return finish(Mes11Status::kiq_ready);
    }

    return hw_init();
}

Mes11Status Mes11::misc_op(ByteView frame, uint32_t api_status_offset) noexcept {
    constexpr uint32_t misc_opcode = 14; // MES_SCH_API_MISC
    uint32_t header = 0;

    if (phase_ != Phase::scheduler) {
        return phase_ == Phase::faulted ? Mes11Status::quarantined : Mes11Status::unavailable;
    }

    if (frame.size() != mes11_api_frame_bytes || !frame.read_u32(0, header) ||
        ((header >> 4) & 0xffu) != misc_opcode || api_status_offset + 16 > mes11_api_frame_bytes ||
        (api_status_offset & 7u) != 0) {
        return finish(Mes11Status::invalid_argument);
    }

    uint8_t copy[mes11_api_frame_bytes]{};

    for (uint32_t index = 0; index < mes11_api_frame_bytes; ++index) {
        frame.read_u8(index, copy[index]);
    }

    return submit(copy, api_status_offset);
}

Mes11Status Mes11::hw_init() noexcept {
    if (phase_ == Phase::scheduler) {
        return finish(Mes11Status::scheduler_ready);
    }

    if (phase_ != Phase::kiq) {
        return phase_ == Phase::faulted ? Mes11Status::quarantined : Mes11Status::unavailable;
    }

    // mes_v11_0_queue_init(AMDGPU_MES_SCHED_PIPE): MQD, then
    // mes_v11_0_kiq_enable_queue (gfx11_kiq_map_queues + amdgpu_ring_test_helper).
    if (!mqd_init(scheduler_)) {
        return quarantine(Mes11Status::mmio_failure);
    }

    const uint32_t select = (scheduler_.pipe << map_queues_pipe_shift) |
        (map_queues_mes_me << map_queues_me_shift) |
        (map_queues_mes_engine_sel << map_queues_engine_sel_shift) |
        (1u << map_queues_num_queues_shift);

    if (!ring_write(kiq_, pm4::packet3(pm4::map_queues, 5)) || !ring_write(kiq_, select) ||
        !ring_write(kiq_, scheduler_.doorbell << map_queues_doorbell_offset_shift) ||
        !ring_write(kiq_, low(scheduler_.mqd_mc)) || !ring_write(kiq_, high(scheduler_.mqd_mc)) ||
        !ring_write(kiq_, low(slot_mc(scheduler_.slot_base + slot_wptr))) ||
        !ring_write(kiq_, high(slot_mc(scheduler_.slot_base + slot_wptr)))) {
        return quarantine(Mes11Status::invalid_memory);
    }

    const Mes11Status test = kiq_ring_test();

    if (test != Mes11Status::completed) {
        return quarantine(test);
    }

    // mes_v11_0_set_hw_resources with the amdgpu_mes_init values.
    uint8_t frame[mes11_api_frame_bytes]{};
    const sdma::CommandBuffer out(frame, sizeof(frame));
    bool ok = put32(out, 0, api::header(api::op_set_hw_rsrc)) &&
        put32(out, api::hw::vmid_mask_mmhub, vmid_mask) &&
        put32(out, api::hw::vmid_mask_gfxhub, vmid_mask) &&
        put32(out, api::hw::gds_size, gds_size) && put32(out, api::hw::paging_vmid, 0);

    for (uint32_t pipe = 0; pipe < 8; ++pipe) {
        ok = ok && put32(out, api::hw::compute_hqd_mask + pipe * 4,
                         pipe < mec_pipes ? compute_hqd_mask : 0);
    }

    // SDMA 6.0.0 is not below IP 6.0.0: 0xfc per existing engine.
    ok = ok && put32(out, api::hw::gfx_hqd_mask, gfx_pipe0_hqd_mask) &&
        put32(out, api::hw::gfx_hqd_mask + 4, 0) &&
        put32(out, api::hw::sdma_hqd_mask, sdma6_hqd_mask) &&
        put32(out, api::hw::sdma_hqd_mask + 4, config_.sdma_instances == 1 ? 0 : sdma6_hqd_mask);

    for (uint32_t level = 0; level < 5; ++level) {
        ok = ok && put32(out, api::hw::aggregated_doorbells + level * 4,
                         aggregated_doorbells_[level]);
    }

    ok = ok && put64(out, api::hw::g_sch_ctx_gpu_mc_ptr, slot_mc(slot_sch_ctx)) &&
        put64(out, api::hw::query_status_fence_gpu_mc_ptr, slot_mc(slot_query_status_fence));

    for (uint32_t index = 0; index < Mes11Registers::hw_resource_bases; ++index) {
        ok = ok && put32(out, api::hw::gc_base + index * 4, registers_.gc_base(index)) &&
            put32(out, api::hw::mmhub_base + index * 4, registers_.mmhub_base(index)) &&
            put32(out, api::hw::osssys_base + index * 4, registers_.osssys_base(index));
    }

    ok = ok && put32(out, api::hw::flags, api::hw::flag_value) &&
        put32(out, api::hw::oversubscription_timer, oversubscription_timer);

    if (!ok) {
        return quarantine(Mes11Status::invalid_memory);
    }

    Mes11Status status = submit(frame, api::hw::api_status);

    if (status != Mes11Status::completed) {
        return status;
    }

    status = query_sched_status_frame();

    if (status != Mes11Status::completed) {
        return status;
    }

    // MES owns the KIQ ring from now on.
    report_.kiq_ring_ready = false;
    report_.scheduler_ready = true;
    phase_ = Phase::scheduler;
    return finish(Mes11Status::scheduler_ready);
}

// mes_v11_0_query_sched_status frame.
Mes11Status Mes11::query_sched_status_frame() noexcept {
    uint8_t frame[mes11_api_frame_bytes]{};

    if (!put32(sdma::CommandBuffer(frame, sizeof(frame)), 0,
               api::header(api::op_query_scheduler_status))) {
        return quarantine(Mes11Status::invalid_memory);
    }

    return submit(frame, api::query::api_status);
}

Mes11Status Mes11::query_sched_status() noexcept {
    if (phase_ != Phase::scheduler) {
        return phase_ == Phase::faulted ? Mes11Status::quarantined : Mes11Status::unavailable;
    }

    return finish(query_sched_status_frame());
}

Mes11Status Mes11::validate_queue(const Mes11LegacyQueue& queue) const noexcept {
    // mes_v11_api_def.h MAX_GFX_PIPES 2, MAX_COMPUTE_PIPES 8, MAX_SDMA_PIPES 2 and
    // eight HQDs per pipe (MAX_SDMA_HQD_PER_PIPE_11_0); GC 11.0.0 has 4 MEC pipes.
    const uint32_t pipes = queue.type == Mes11QueueType::compute ? mec_pipes : 2;

    if ((queue.type != Mes11QueueType::gfx && queue.type != Mes11QueueType::compute &&
         queue.type != Mes11QueueType::sdma) || queue.pipe >= pipes || queue.queue >= 8 ||
        queue.doorbell_index >= doorbells_.aperture_dwords || queue.mqd_address == 0 ||
        (queue.mqd_address & 3u) != 0 || queue.mqd_address >= gpu_address_limit ||
        queue.wptr_address == 0 || (queue.wptr_address & 7u) != 0 ||
        queue.wptr_address >= gpu_address_limit) {
        return Mes11Status::invalid_argument;
    }

    return Mes11Status::completed;
}

Mes11Status Mes11::map_legacy_queue(const Mes11LegacyQueue& queue) noexcept {
    if (phase_ != Phase::scheduler) {
        return phase_ == Phase::faulted ? Mes11Status::quarantined : Mes11Status::unavailable;
    }

    if (!report_.legacy_queue_map) {
        return Mes11Status::unsupported_firmware;
    }

    if (validate_queue(queue) != Mes11Status::completed) {
        return Mes11Status::invalid_argument;
    }

    uint8_t frame[mes11_api_frame_bytes]{};
    const sdma::CommandBuffer out(frame, sizeof(frame));

    if (!put32(out, 0, api::header(api::op_add_queue)) ||
        !put32(out, api::add::pipe_id, queue.pipe) ||
        !put32(out, api::add::queue_id, queue.queue) ||
        !put32(out, api::add::doorbell_offset, queue.doorbell_index) ||
        !put64(out, api::add::mqd_addr, queue.mqd_address) ||
        !put64(out, api::add::wptr_addr, queue.wptr_address) ||
        !put32(out, api::add::queue_type, static_cast<uint32_t>(queue.type)) ||
        !put32(out, api::add::flags, api::add::map_legacy_kq)) {
        return quarantine(Mes11Status::invalid_memory);
    }

    return finish(submit(frame, api::add::api_status));
}

Mes11Status Mes11::unmap_legacy_queue(const Mes11LegacyUnmap& request) noexcept {
    if (phase_ != Phase::scheduler) {
        return phase_ == Phase::faulted ? Mes11Status::quarantined : Mes11Status::unavailable;
    }

    const bool preempt = request.action == Mes11UnmapAction::preempt_no_unmap;

    if (validate_queue(request.queue) != Mes11Status::completed ||
        (!preempt && request.action != Mes11UnmapAction::unmap) ||
        (preempt && (request.trail_fence_address == 0 || (request.trail_fence_address & 3u) != 0 ||
                     request.trail_fence_address >= gpu_address_limit))) {
        return Mes11Status::invalid_argument;
    }

    uint8_t frame[mes11_api_frame_bytes]{};
    const sdma::CommandBuffer out(frame, sizeof(frame));
    bool ok = put32(out, 0, api::header(api::op_remove_queue)) &&
        put32(out, api::remove::doorbell_offset, request.queue.doorbell_index) &&
        put64(out, api::remove::gang_context_addr, 0) &&
        put32(out, api::remove::pipe_id, request.queue.pipe) &&
        put32(out, api::remove::queue_id, request.queue.queue);

    if (preempt) {
        ok = ok && put32(out, api::remove::flags, api::remove::preempt_legacy_gfx_queue) &&
            put64(out, api::remove::tf_addr, request.trail_fence_address) &&
            put32(out, api::remove::tf_data, low(request.trail_fence_data));
    } else {
        ok = ok && put32(out, api::remove::flags, api::remove::unmap_legacy_queue) &&
            put32(out, api::remove::queue_type, static_cast<uint32_t>(request.queue.type));
    }

    if (!ok) {
        return quarantine(Mes11Status::invalid_memory);
    }

    return finish(submit(frame, api::remove::api_status));
}

Mes11Status Mes11::kiq_set_resources(uint64_t queue_mask) noexcept {
    if (phase_ != Phase::kiq && phase_ != Phase::scheduler) {
        return phase_ == Phase::faulted ? Mes11Status::quarantined : Mes11Status::unavailable;
    }

    if (!nbio_->flush_hdp()) {
        return quarantine(Mes11Status::mmio_failure);
    }

    // gfx11_kiq_set_resources: vmid_mask 0, unmap latency 0xa, queue type 0 (KIQ),
    // then gws, oac and gds heap fields all zero.
    if (!ring_write(kiq_, pm4::packet3(pm4::set_resources, 6)) ||
        !ring_write(kiq_, set_resources_unmap_latency << set_resources_unmap_latency_shift) ||
        !ring_write(kiq_, low(queue_mask)) || !ring_write(kiq_, high(queue_mask)) ||
        !ring_write(kiq_, 0) || !ring_write(kiq_, 0) || !ring_write(kiq_, 0) ||
        !ring_write(kiq_, 0)) {
        return quarantine(Mes11Status::invalid_memory);
    }

    const Mes11Status test = kiq_ring_test();

    // MES owns the KIQ once the scheduler is ready; the helper flag is not kept.
    if (phase_ == Phase::scheduler) {
        report_.kiq_ring_ready = false;
    }

    return test == Mes11Status::completed ? finish(test) : quarantine(test);
}

// mes_v11_0_kiq_dequeue for ME3 pipe "ring.pipe". Linux continues after a
// dequeue timeout; the timeout is reported and quarantines afterwards.
bool Mes11::kiq_dequeue(const Ring& ring) noexcept {
    uint32_t active = 0;
    uint32_t data = 0;

    if (!grbm_select(mes_me, ring.pipe) || !read(0, reg_cp_hqd_active, active)) {
        return false;
    }

    if ((active & 1u) != 0) {
        if (!write(0, reg_cp_hqd_dequeue_request, 1)) {
            return false;
        }

        uint32_t poll = 0;

        for (; poll < policy_.poll_iterations; ++poll) {
            if (!read(0, reg_cp_hqd_active, active)) {
                return false;
            }

            if ((active & 1u) == 0) {
                break;
            }

            io_.delay_microseconds(io_.context, 1);
        }

        report_.dequeue_timeout = poll >= policy_.poll_iterations;
    }

    return read(0, reg_cp_hqd_pq_doorbell_control, data) &&
        write(0, reg_cp_hqd_pq_doorbell_control, (data & ~doorbell_en) | doorbell_hit) &&
        write(0, reg_cp_hqd_pq_doorbell_control, 0) && write(0, reg_cp_hqd_pq_wptr_lo, 0) &&
        write(0, reg_cp_hqd_pq_wptr_hi, 0) && write(0, reg_cp_hqd_pq_rptr, 0) &&
        grbm_select(0, 0);
}

Mes11Status Mes11::kiq_hw_fini() noexcept {
    if (report_.process_queues != 0) {
        return Mes11Status::busy;
    }
    if (phase_ != Phase::kiq && phase_ != Phase::scheduler && phase_ != Phase::faulted) {
        return phase_ == Phase::halted ? finish(Mes11Status::disabled) : Mes11Status::unavailable;
    }

    // A quarantined protocol may still halt MES, but its leases stay retained
    // until release_after_reset.
    const bool faulted = phase_ == Phase::faulted;
    const bool dequeue = phase_ == Phase::scheduler;
    report_.dequeue_timeout = false;

    if (dequeue && !kiq_dequeue(scheduler_)) {
        enable(false);
        return quarantine(Mes11Status::mmio_failure);
    }

    report_.scheduler_ready = false;
    report_.kiq_ring_ready = false;

    if (!enable(false)) {
        return quarantine(Mes11Status::mmio_failure);
    }

    if (faulted) {
        return quarantine(Mes11Status::quarantined);
    }

    if (report_.dequeue_timeout) {
        return quarantine(Mes11Status::timeout);
    }

    phase_ = Phase::halted;
    return finish(Mes11Status::disabled);
}

Mes11Status Mes11::hw_fini() noexcept {
    return phase_ == Phase::faulted ? Mes11Status::quarantined : report_.status;
}

Mes11Status Mes11::release() noexcept {
    if (phase_ != Phase::halted) {
        return phase_ == Phase::faulted ? Mes11Status::quarantined : Mes11Status::busy;
    }

    // Quarantined self-test buffers: only a confirmed reset proves MES/CP/SDMA
    // stopped using them.
    if (self_test_.memory_retained) {
        return Mes11Status::quarantined;
    }

    // A queue owner still holds MES doorbells or queue ids (amdgpu_mes_fini
    // expects every process destroyed first).
    if (report_.process_queues != 0 || report_.doorbells_allocated != 0 ||
        report_.queue_ids_allocated != 0) {
        return Mes11Status::busy;
    }

    release_leases();
    forget_allocations();
    phase_ = Phase::ready;
    self_test_done_ = false;
    return finish(Mes11Status::released);
}

Mes11Status Mes11::release_after_reset() noexcept {
    if (!report_.memory_retained && !self_test_.memory_retained && phase_ != Phase::faulted) {
        return Mes11Status::busy;
    }

    if (recovery_.confirm_reset_quiescence == nullptr ||
        !recovery_.confirm_reset_quiescence(recovery_.context)) {
        return Mes11Status::quarantined;
    }

    release_leases();
    release_self_test();
    forget_allocations();
    report_.gangs_suspended = false;
    report_.mes_enabled = false;
    report_.scheduler_ready = false;
    report_.kiq_ring_ready = false;
    phase_ = Phase::ready;
    self_test_done_ = false;
    return finish(Mes11Status::released);
}

// amdgpu_mes_create_process/add_gang/queue_alloc_mqd buffers: amdgpu_bo_create_kernel
// in AMDGPU_GEM_DOMAIN_GTT, so a VMID0 GART address, page aligned, CPU mapped.
// They must not overlap each other or the metadata buffer. Zeroed here (the
// memsets of create_process, add_gang and queue_alloc_mqd) before any MES use.
bool Mes11::retain_self_test(memory::Manager& manager, const Mes11SelfTestMemory& memory,
                             const Mes11ProcessVm& vm, const GartLayout& gart) noexcept {
    constexpr uint32_t count = 1 + 2 * mes11_self_test_queues;
    struct Need final {
        memory::Handle handle;
        uint32_t bytes;
    };

    const Need needs[count]{{memory.process_context, mes11_process_context_bytes},
        {memory.gang_context[0], mes11_gang_context_bytes},
        {memory.gang_context[1], mes11_gang_context_bytes},
        {memory.gang_context[2], mes11_gang_context_bytes},
        {memory.mqd[0], self_test::engines[0].mqd_bytes},
        {memory.mqd[1], self_test::engines[1].mqd_bytes},
        {memory.mqd[2], self_test::engines[2].mqd_bytes}};
    memory::AllocationInfo info[count]{};
    uint32_t retained = 0;
    bool ok = true;

    for (; ok && retained < count; ++retained) {
        sdma::GpuRange range{};
        memory::AllocationInfo& item = info[retained];
        const uint64_t bytes = needs[retained].bytes;
        ok = manager.describe(needs[retained].handle, item) == memory::Status::success &&
             manager.resolve({needs[retained].handle, 0}, bytes, memory::Access::read_write,
                             range) == memory::Status::success;
        const uint64_t start = item.gpu.virtual_address;
        ok = ok && (start & (mes11_page_alignment - 1)) == 0 && item.gpu.byte_length >= bytes &&
             start >= gart.start && start <= gart.end && bytes - 1 <= gart.end - start &&
             item.backing.cpu_bytes != nullptr && item.backing.cpu_byte_count >= bytes &&
             !overlaps(start, bytes, vm.metadata_gart(), mes11_ctx::bytes) &&
             manager.retain(needs[retained].handle, memory::Access::read_write) ==
                 memory::Status::success;
    }

    if (!ok) {
        --retained;
    }

    for (uint32_t a = 0; ok && a < count; ++a) {
        for (uint32_t b = a + 1; ok && b < count; ++b) {
            ok = !overlaps(info[a].gpu.virtual_address, needs[a].bytes,
                           info[b].gpu.virtual_address, needs[b].bytes);
        }
    }

    for (uint32_t index = 0; ok && index < count; ++index) {
        const sdma::CommandBuffer bytes(info[index].backing.cpu_bytes, needs[index].bytes);

        for (uint32_t offset = 0; ok && offset < needs[index].bytes; offset += 4) {
            ok = bytes.write_u32(offset, 0);
        }
    }

    if (!ok) {
        while (retained > 0) {
            --retained;
            manager.unretain(needs[retained].handle);
        }

        return false;
    }

    self_test_manager_ = &manager;
    self_test_handles_ = memory;
    metadata_cpu_ = vm.metadata_cpu();
    self_test_.memory_retained = true;
    self_test_.process_context_address = info[0].gpu.virtual_address;

    for (uint32_t index = 0; index < mes11_self_test_queues; ++index) {
        const self_test::Engine& engine = self_test::engines[index];
        Mes11SelfTestQueue& queue = self_test_.queues[index];
        queue.type = engine.type;
        queue.gang_context_address = info[1 + index].gpu.virtual_address;
        queue.mqd_address = info[1 + mes11_self_test_queues + index].gpu.virtual_address;
        queue.ring_va = vm.metadata_va() + engine.ring;
        queue.fence_va = vm.metadata_va() + engine.slots + mes11_ctx::slot_fence * mes11_ctx::slot_bytes;
        user_queues_[index] = {info[1 + mes11_self_test_queues + index].backing.cpu_bytes,
                               engine.ring, engine.slots, engine.ib, engine.padding};
    }

    return true;
}

void Mes11::release_self_test() noexcept {
    if (self_test_.memory_retained && self_test_manager_ != nullptr) {
        self_test_manager_->unretain(self_test_handles_.process_context);

        for (uint32_t index = 0; index < mes11_self_test_queues; ++index) {
            self_test_manager_->unretain(self_test_handles_.gang_context[index]);
            self_test_manager_->unretain(self_test_handles_.mqd[index]);
        }
    }

    self_test_manager_ = nullptr;
    self_test_handles_ = {};
    metadata_cpu_ = nullptr;

    for (auto& queue : user_queues_) {
        queue = {};
    }

    self_test_.memory_retained = false;
}

// amdgpu_mes_queue_init_mqd: the engine's mqd_init (adev->mqds[type].init_mqd).
// Gfx and compute run under amdgpu_gfx_select_me_pipe_q(me, 0, 0, 0, 0) of
// gfx_ring[0] / compute_ring[0]; the SDMA ring keeps me 0 and reads
// SDMA0_QUEUE0_IB_CNTL as is (no GRBM selection).
bool Mes11::queue_mqd(Mes11QueueType type, const Gfx11MqdProperties& properties,
                      sdma::CommandBuffer mqd) noexcept {
    if (type == Mes11QueueType::sdma) {
        uint32_t ib_control = 0;
        return read(0, self_test::reg_sdma0_queue0_ib_cntl, ib_control) &&
               mes11_sdma_mqd_init(ib_control, properties, mqd);
    }

    const bool compute = type == Mes11QueueType::compute;
    const uint32_t me = self_test::engines[compute ? 1 : 0].me;
    const Gfx11GcAccess access{io_, registers_.gc_segment(0), registers_.gc_segment(1)};
    return compute ? gfx11_compute_mqd_init(access, {me, 0, 0}, properties, mqd) :
                     gfx11_gfx_mqd_init(access, {me, 0, 0}, properties, mqd);
}

// The self-test properties (amdgpu_mes_queue_init_mqd of amdgpu_mes_add_ring):
// queue_size 8192, use_doorbell, the MES doorbell, PIPE_PRIO_NORMAL,
// QUEUE_PRIORITY_MINIMUM, hqd_active false; ring, rptr, wptr and the compute
// MEC HPD in the metadata at their process VAs.
bool Mes11::self_test_mqd(uint32_t index, const Mes11ProcessVm& vm) noexcept {
    const self_test::Engine& engine = self_test::engines[index];
    const Mes11SelfTestQueue& queue = self_test_.queues[index];
    const uint64_t slots = vm.metadata_va() + engine.slots;
    const uint64_t rptr = slots + mes11_ctx::slot_rptr * mes11_ctx::slot_bytes;
    const uint64_t wptr = slots + mes11_ctx::slot_wptr * mes11_ctx::slot_bytes;
    const bool compute = engine.type == Mes11QueueType::compute;
    const Gfx11MqdProperties properties{queue.mqd_address, queue.ring_va, rptr, wptr,
        compute ? vm.metadata_va() + mes11_ctx::compute_mec_hpd : 0, mes11_ctx::ring_bytes,
        queue.doorbell_index, self_test::pipe_priority_normal,
        self_test::queue_priority_minimum, false, false};
    return queue_mqd(engine.type, properties,
                     sdma::CommandBuffer(user_queues_[index].mqd_cpu, engine.mqd_bytes));
}

bool Mes11::read_vm_context_cntl(uint32_t& value) noexcept {
    return read(0, self_test::reg_gcvm_context15_cntl, value);
}

// mes_v11_0_add_hw_queue frame image (union MESAPI__ADD_QUEUE, zeroed first).
bool Mes11::encode_add_queue(const Mes11AddQueue& queue, uint32_t vm_context_cntl,
                             uint8_t (&frame)[mes11_api_frame_bytes]) const noexcept {
    const sdma::CommandBuffer out(frame, sizeof(frame));
    const uint64_t wptr = api_version() >= self_test::api_version_wptr_mc ? queue.wptr_mc :
                                                                           queue.wptr_va;

    for (uint32_t offset = 0; offset < mes11_api_frame_bytes; offset += 4) {
        if (!out.write_u32(offset, 0)) {
            return false;
        }
    }

    return put32(out, 0, api::header(api::op_add_queue)) &&
        put32(out, api::add::process_id, queue.pasid) &&
        put64(out, api::add::page_table_base_addr, queue.page_table_base_addr) &&
        put64(out, api::add::process_va_start, 0) &&
        put64(out, api::add::process_va_end, queue.process_va_end) &&
        put64(out, api::add::process_quantum, queue.process_quantum) &&
        put64(out, api::add::process_context_addr, queue.process_context_address) &&
        put64(out, api::add::gang_quantum, queue.gang_quantum) &&
        put64(out, api::add::gang_context_addr, queue.gang_context_address) &&
        put32(out, api::add::inprocess_gang_priority, queue.inprocess_gang_priority) &&
        put32(out, api::add::gang_global_priority_level, queue.gang_global_priority_level) &&
        put32(out, api::add::doorbell_offset, queue.doorbell_index) &&
        put64(out, api::add::mqd_addr, queue.mqd_address) &&
        put64(out, api::add::wptr_addr, wptr) &&
        put32(out, api::add::queue_type, static_cast<uint32_t>(queue.type)) &&
        put32(out, api::add::gds_size, 0) &&
        put32(out, api::add::vm_context_cntl, vm_context_cntl) &&
        put32(out, api::add::flags, 0);
}

// mes_v11_0_remove_hw_queue frame (union MESAPI__REMOVE_QUEUE, zeroed first).
Mes11Status Mes11::submit_remove_queue(const Mes11RemoveQueue& queue) noexcept {
    uint8_t frame[mes11_api_frame_bytes]{};
    const sdma::CommandBuffer out(frame, sizeof(frame));

    if (!put32(out, 0, api::header(api::op_remove_queue)) ||
        !put32(out, api::remove::doorbell_offset, queue.doorbell_index) ||
        !put64(out, api::remove::gang_context_addr, queue.gang_context_address) ||
        !put32(out, api::remove::flags, 0)) {
        return Mes11Status::invalid_memory;
    }

    return submit(frame, api::remove::api_status);
}

// amdgpu_mes_add_gang, amdgpu_mes_add_ring and amdgpu_mes_add_hw_queue for queue
// index (gfx, compute, SDMA). Gang ids are index + 1 (the self-test's own gangs);
// the queue id and the MES doorbell come from the shared queue_id_idr and
// doorbell bitmap as amdgpu_mes_add_hw_queue takes them. A failed add frees both.
bool Mes11::self_test_add(uint32_t index, const Mes11ProcessVm& vm) noexcept {
    const self_test::Engine& engine = self_test::engines[index];
    Mes11SelfTestQueue& queue = self_test_.queues[index];
    queue.gang_id = index + 1;

    if (phase_ == Phase::faulted) {
        queue.add_status = Mes11Status::quarantined;
        return false;
    }

    // amdgpu_ring_init for the MES queue: amdgpu_ring_clear_ring with the engine
    // NOP over ring_size, cond_exec 1, fence = last_seq 0 (the rest of the
    // metadata is still the amdgpu_mes_ctx_alloc_meta_data memset).
    const uint32_t nop = engine.type == Mes11QueueType::sdma ? sdma6::op_nop : gfx11_ring_nop;
    const sdma::CommandBuffer metadata(metadata_cpu_, mes11_ctx::bytes);
    bool ok = true;

    for (uint32_t offset = 0; ok && offset < mes11_ctx::ring_bytes; offset += 4) {
        ok = metadata.write_u32(engine.ring + offset, nop);
    }

    ok = ok && metadata.write_u32(engine.slots + mes11_ctx::slot_cond_exe * mes11_ctx::slot_bytes, 1) &&
         metadata.write_u32(engine.slots + mes11_ctx::slot_fence * mes11_ctx::slot_bytes, 0);

    if (!ok) {
        queue.add_status = Mes11Status::invalid_memory;
        return false;
    }

    if (allocate_queue_id(queue.queue_id) != Mes11Status::completed) {
        queue.add_status = Mes11Status::busy;
        return false;
    }

    uint32_t doorbell = 0;

    if (allocate_doorbell(engine.type, doorbell) != Mes11Status::completed) {
        free_queue_id(queue.queue_id);
        queue.add_status = Mes11Status::doorbell_failure;
        return false;
    }

    queue.doorbell_index = doorbell;
    queue.wptr = 0;
    queue.fence_seq = 0;

    if (!self_test_mqd(index, vm)) {
        grbm_select(0, 0);
        free_doorbell(doorbell);
        free_queue_id(queue.queue_id);
        queue.add_status = Mes11Status::mmio_failure;
        quarantine(Mes11Status::mmio_failure);
        return false;
    }

    // amdgpu_mes_add_hw_queue -> mes_v11_0_add_hw_queue.
    const uint64_t wptr_offset = engine.slots + mes11_ctx::slot_wptr * mes11_ctx::slot_bytes;
    const Mes11AddQueue add{engine.type, self_test_.pasid, self_test_.page_table_base_addr,
        self_test_.process_va_end, 0, self_test_.process_context_address, 0,
        queue.gang_context_address, self_test::priority_normal, self_test::priority_normal,
        queue.doorbell_index, queue.mqd_address, vm.metadata_va() + wptr_offset,
        vm.metadata_gart() + wptr_offset};
    queue.frame_wptr_address = self_test_.api_version >= self_test::api_version_wptr_mc ?
                               add.wptr_mc : add.wptr_va;
    uint8_t frame[mes11_api_frame_bytes]{};

    if (!encode_add_queue(add, self_test_.vm_context_cntl, frame)) {
        free_doorbell(doorbell);
        free_queue_id(queue.queue_id);
        queue.add_status = Mes11Status::invalid_memory;
        return false;
    }

    queue.add_status = submit(frame, api::add::api_status);
    queue.added = queue.add_status == Mes11Status::completed;

    if (queue.added) {
        ++self_test_.queues_added;
    } else {
        free_doorbell(doorbell);
        free_queue_id(queue.queue_id);
    }

    return queue.added;
}

// amdgpu_ring_write on a self-test ring (8192 bytes in the metadata).
bool Mes11::user_write(uint32_t index, uint32_t value) noexcept {
    Mes11SelfTestQueue& queue = self_test_.queues[index];
    Gfx11RingWriter writer{sdma::CommandBuffer(metadata_cpu_, mes11_ctx::bytes)
                               .subbuffer(user_queues_[index].ring_offset, mes11_ctx::ring_bytes),
                           queue.wptr};
    const bool ok = gfx11_ring_write(writer, value);
    queue.wptr = writer.wptr;
    return ok;
}

// amdgpu_ring_commit: the engine's insert_nop padding (gfx11 NOP packet; plain
// SDMA NOPs), then set_wptr: the 64-bit wptr shadow in the metadata slot (dwords
// for gfx/compute, bytes for SDMA) and WDOORBELL64 of the same value. The
// metadata is snooped GTT, so no HDP flush precedes the doorbell (Linux mb()).
bool Mes11::user_commit(uint32_t index) noexcept {
    Mes11SelfTestQueue& queue = self_test_.queues[index];
    const bool sdma = queue.type == Mes11QueueType::sdma;
    bool ok = true;

    {
        Gfx11RingWriter writer{sdma::CommandBuffer(metadata_cpu_, mes11_ctx::bytes)
                                   .subbuffer(user_queues_[index].ring_offset,
                                              mes11_ctx::ring_bytes),
                               queue.wptr};
        ok = sdma ? mes11_sdma_ring_pad(writer) : gfx11_ring_pad(writer);
        queue.wptr = writer.wptr;
    }

    const uint64_t value = sdma ? queue.wptr << 2 : queue.wptr;
    ok = ok && put64(sdma::CommandBuffer(metadata_cpu_, mes11_ctx::bytes),
                     user_queues_[index].slots_offset + mes11_ctx::slot_wptr * mes11_ctx::slot_bytes,
                     value);

    if (!ok) {
        quarantine(Mes11Status::invalid_memory);
        return false;
    }

    if (!doorbells_.write64(doorbells_.context, queue.doorbell_index, value)) {
        quarantine(Mes11Status::doorbell_failure);
        return false;
    }

    return true;
}

// Poll one metadata dword, 1 us apart, until it equals expected.
Mes11SelfTestResult Mes11::poll_user_dword(uint32_t offset, uint32_t expected,
                                           uint32_t polls) noexcept {
    const ByteView metadata(metadata_cpu_, mes11_ctx::bytes);

    for (uint32_t poll = 0; poll < polls; ++poll) {
        uint32_t value = 0;

        if (!metadata.read_u32(offset, value)) {
            return Mes11SelfTestResult::failed;
        }

        if (value == expected) {
            return Mes11SelfTestResult::passed;
        }

        io_.delay_microseconds(io_.context, 1);
    }

    return Mes11SelfTestResult::timeout;
}

// amdgpu_ring_test_helper -> gfx_v11_0_ring_test_ring (SCRATCH_REG0 seeded
// 0xCAFEDEAD, SET_UCONFIG_REG 0xDEADBEEF, adev->usec_timeout 1 us polls) or
// sdma_v6_0_ring_test_ring for a MES queue (padding seeded 0xCAFEDEAD,
// WRITE_LINEAR of 0xDEADBEEF to the padding VA, polled through the CPU mapping).
Mes11SelfTestResult Mes11::self_test_ring(uint32_t index, const Mes11ProcessVm& vm) noexcept {
    Mes11SelfTestQueue& queue = self_test_.queues[index];
    const UserQueue& user = user_queues_[index];

    if (queue.type == Mes11QueueType::sdma) {
        const uint64_t target = vm.metadata_va() + user.padding_offset;

        if (!put32(sdma::CommandBuffer(metadata_cpu_, mes11_ctx::bytes), user.padding_offset,
                   gfx11_ring_test_seed) ||
            !user_write(index, sdma6::op_write | (0u << sdma6::subop_write_linear_shift)) ||
            !user_write(index, low(target)) || !user_write(index, high(target)) ||
            !user_write(index, 0) || !user_write(index, gfx11_ring_test_value) ||
            !user_commit(index)) {
            return Mes11SelfTestResult::failed;
        }

        return poll_user_dword(user.padding_offset, gfx11_ring_test_value, policy_.poll_iterations);
    }

    if (!write(1, reg_scratch_reg0, gfx11_ring_test_seed)) {
        quarantine(Mes11Status::mmio_failure);
        return Mes11SelfTestResult::failed;
    }

    Gfx11RingWriter writer{sdma::CommandBuffer(metadata_cpu_, mes11_ctx::bytes)
                               .subbuffer(user.ring_offset, mes11_ctx::ring_bytes),
                           queue.wptr};
    const bool emitted = gfx11_emit_ring_test(writer, registers_.gc_segment(1) + reg_scratch_reg0);
    queue.wptr = writer.wptr;

    if (!emitted || !user_commit(index)) {
        if (!emitted) {
            quarantine(Mes11Status::invalid_memory);
        }

        return Mes11SelfTestResult::failed;
    }

    for (uint32_t poll = 0; poll < policy_.poll_iterations; ++poll) {
        uint32_t value = 0;

        if (!read(1, reg_scratch_reg0, value)) {
            quarantine(Mes11Status::mmio_failure);
            return Mes11SelfTestResult::failed;
        }

        if (value == gfx11_ring_test_value) {
            return Mes11SelfTestResult::passed;
        }

        io_.delay_microseconds(io_.context, 1);
    }

    return Mes11SelfTestResult::timeout;
}

// amdgpu_ring_test_ib -> gfx_v11_0_ring_test_ib / sdma_v6_0_ring_test_ib for a MES
// queue: the IB lives in the metadata ib[] slot, the destination is the padding
// dword (seeded 0xCAFEDEAD), ib.flags stays 0 (no amdgpu_ib_get, so no
// EMIT_MEM_SYNC), then amdgpu_ib_schedule without a job and a fence wait.
Mes11SelfTestResult Mes11::self_test_ib(uint32_t index, const Mes11ProcessVm& vm,
                                        const Mes11SelfTestConfig& config) noexcept {
    Mes11SelfTestQueue& queue = self_test_.queues[index];
    const UserQueue& user = user_queues_[index];
    const sdma::CommandBuffer metadata(metadata_cpu_, mes11_ctx::bytes);
    const sdma::CommandBuffer ib = metadata.subbuffer(user.ib_offset, mes11_ctx::ib_bytes);
    const uint64_t va = vm.metadata_va();
    const uint64_t padding = va + user.padding_offset;
    const uint64_t cond = va + user.slots_offset + mes11_ctx::slot_cond_exe * mes11_ctx::slot_bytes;
    const uint32_t nbif = config.nbio.nbif_base(gfx11_hdp_flush_nbif_segment);
    const uint32_t flush_req = nbif + gfx11_hdp_flush_req_register;
    const uint32_t flush_done = nbif + gfx11_hdp_flush_done_register;
    // amdgpu_fence_emit: seq = ++ring->fence_drv.sync_seq.
    ++queue.fence_seq;
    bool ok = put32(metadata, user.padding_offset, gfx11_ring_test_seed);

    if (queue.type != Mes11QueueType::sdma) {
        Gfx11RingWriter writer{metadata.subbuffer(user.ring_offset, mes11_ctx::ring_bytes),
                               queue.wptr};
        ok = ok && gfx11_write_test_ib(ib, padding) &&
             gfx11_emit_ib_frame(writer, {queue.type == Mes11QueueType::gfx, false, true,
                 queue.queue_id, 0, cond, flush_req, flush_done, va + user.ib_offset,
                 gfx11_ib_test_dwords, queue.fence_va, queue.fence_seq});
        queue.wptr = writer.wptr;
    } else {
        // IB: WRITE_LINEAR of 0xDEADBEEF to the padding, three NOPs (length 8).
        ok = ok && put32(ib, 0, sdma6::op_write) && put32(ib, 4, low(padding)) &&
             put32(ib, 8, high(padding)) && put32(ib, 12, 0) &&
             put32(ib, 16, gfx11_ring_test_value) && put32(ib, 20, sdma6::op_nop) &&
             put32(ib, 24, sdma6::op_nop) && put32(ib, 28, sdma6::op_nop);
        Gfx11RingWriter writer{metadata.subbuffer(user.ring_offset, mes11_ctx::ring_bytes),
                               queue.wptr};
        ok = ok && mes11_sdma_emit_ib_frame(writer, {cond, flush_req, flush_done,
            va + user.ib_offset, sdma6::ib_test_dwords, queue.fence_va, queue.fence_seq,
            queue.queue_id});
        queue.wptr = writer.wptr;
    }

    if (!ok) {
        quarantine(Mes11Status::invalid_memory);
        return Mes11SelfTestResult::failed;
    }

    if (!user_commit(index)) {
        return Mes11SelfTestResult::failed;
    }

    // dma_fence_wait_timeout(f, false, timeout), with the fence polled: signalled
    // once the written sequence reaches seq in 32-bit wrap-around order.
    const ByteView result(metadata_cpu_, mes11_ctx::bytes);
    const uint32_t fence_offset = user.slots_offset + mes11_ctx::slot_fence * mes11_ctx::slot_bytes;
    bool signalled = false;

    for (queue.ib_polls = 0; queue.ib_polls < config.ib_timeout_microseconds; ++queue.ib_polls) {
        if (!result.read_u32(fence_offset, queue.fence_value)) {
            return Mes11SelfTestResult::failed;
        }

        if (queue.fence_value - queue.fence_seq < 0x80000000u) {
            signalled = true;
            break;
        }

        io_.delay_microseconds(io_.context, 1);
    }

    uint32_t value = 0;

    if (!signalled) {
        return Mes11SelfTestResult::timeout;
    }

    return result.read_u32(user.padding_offset, value) && value == gfx11_ring_test_value ?
        Mes11SelfTestResult::passed : Mes11SelfTestResult::failed;
}

Mes11Status Mes11::self_test(memory::Manager& gtt_manager, const Mes11SelfTestMemory& memory,
                             const Mes11ProcessVm& vm, const Mes11SelfTestConfig& config) noexcept {
    if (self_test_done_ || self_test_.memory_retained) {
        return self_test_.memory_retained ? Mes11Status::quarantined : Mes11Status::busy;
    }

    self_test_ = Mes11SelfTestReport();

    if (phase_ != Phase::scheduler) {
        self_test_.status = phase_ == Phase::faulted ? Mes11Status::quarantined :
                                                       Mes11Status::unavailable;
        return self_test_.status;
    }

    uint32_t offset = 0;
    const uint32_t nbif = config.nbio.valid() ?
        config.nbio.nbif_base(gfx11_hdp_flush_nbif_segment) : 0;

    if (!config.nbio.valid() ||
        !discovery_register_offset(nbif, gfx11_hdp_flush_done_register,
                                   config.nbio.aperture_bytes(), offset) ||
        config.ib_timeout_microseconds == 0 ||
        config.ib_timeout_microseconds > mes11_self_test_ib_timeout_max_us ||
        config.gart.end <= config.gart.start || !vm.valid() ||
        vm.metadata_va() != mes11_self_test_metadata_va ||
        vm.metadata_gart() < config.gart.start || vm.metadata_gart() > config.gart.end ||
        mes11_ctx::bytes - 1 > config.gart.end - vm.metadata_gart() ||
        (vm.page_table_base_addr() & (mes11_page_alignment - 1)) != 0 ||
        vm.page_table_base_addr() >= gpu_address_limit) {
        self_test_.status = Mes11Status::invalid_argument;
        return self_test_.status;
    }

    // hub->vm_cntx_cntl for ADD_QUEUE vm_context_cntl (see the header).
    uint32_t vm_context_cntl = 0;

    if (!read_vm_context_cntl(vm_context_cntl)) {
        self_test_.status = quarantine(Mes11Status::mmio_failure);
        return self_test_.status;
    }

    if ((vm_context_cntl & self_test::vm_structure_mask) != self_test::vm_structure) {
        self_test_.vm_context_cntl = vm_context_cntl;
        self_test_.status = Mes11Status::unsupported_ip;
        return self_test_.status;
    }

    if (!retain_self_test(gtt_manager, memory, vm, config.gart)) {
        self_test_.status = Mes11Status::invalid_memory;
        return self_test_.status;
    }

    self_test_done_ = true;
    self_test_.ran = true;
    self_test_.pasid = mes11_self_test_pasid;
    self_test_.page_table_base_addr = vm.page_table_base_addr();
    self_test_.process_va_end = (mes11_self_test_vm_max_pfn - 1) << 12;
    self_test_.vm_context_cntl = vm_context_cntl;
    self_test_.api_version = (report_.scheduler_version & self_test::api_version_mask) >>
                             self_test::api_version_shift;

    // amdgpu_mes_test_create_gang_and_queues for gfx, compute and SDMA; a failed
    // add_ring breaks only its own loop and the next engine follows.
    for (uint32_t index = 0; index < mes11_self_test_queues; ++index) {
        self_test_add(index, vm);
    }

    // amdgpu_mes_test_queues over added_rings in order; the first failing ring
    // or IB test ends the testing (its return value is ignored by the caller).
    bool tests_failed = false;

    for (uint32_t index = 0; index < mes11_self_test_queues && !tests_failed; ++index) {
        Mes11SelfTestQueue& queue = self_test_.queues[index];

        if (!queue.added) {
            continue;
        }

        // A quarantined MES/MMIO state tests nothing more (not_run, still a failure).
        queue.ring_test = phase_ == Phase::faulted ? Mes11SelfTestResult::not_run :
                                                     self_test_ring(index, vm);

        if (queue.ring_test == Mes11SelfTestResult::passed) {
            ++self_test_.rings_passed;
            queue.ib_test = phase_ == Phase::faulted ? Mes11SelfTestResult::not_run :
                                                       self_test_ib(index, vm, config);
        }

        if (queue.ring_test != Mes11SelfTestResult::passed ||
            queue.ib_test != Mes11SelfTestResult::passed) {
            tests_failed = true;
            self_test_.failed_queue = index;
        } else {
            ++self_test_.ibs_passed;
        }
    }

    // amdgpu_mes_remove_ring -> amdgpu_mes_remove_hw_queue -> REMOVE_QUEUE with
    // the doorbell and gang context, for every added ring; then remove_gang and
    // destroy_process find no queues and send nothing.
    bool all_removed = true;

    for (uint32_t index = 0; index < mes11_self_test_queues; ++index) {
        Mes11SelfTestQueue& queue = self_test_.queues[index];

        if (!queue.added) {
            continue;
        }

        queue.remove_status = submit_remove_queue({queue.doorbell_index, queue.gang_context_address});
        queue.removed = queue.remove_status == Mes11Status::completed;
        all_removed = all_removed && queue.removed;

        // amdgpu_mes_remove_hw_queue frees the queue id and doorbell. A failed
        // REMOVE_QUEUE quarantined MES: both stay allocated until reset.
        if (queue.removed) {
            free_doorbell(queue.doorbell_index);
            free_queue_id(queue.queue_id);
        }
    }

    // Fail closed: a queue MES may still run, a test the engine did not finish
    // as expected or any uncertain MES/MMIO state keeps every self-test buffer.
    const bool faulted = phase_ == Phase::faulted;
    self_test_.memory_quarantined = faulted || tests_failed || !all_removed;

    if (!self_test_.memory_quarantined) {
        release_self_test();
    }

    bool all_added = true;

    for (const auto& queue : self_test_.queues) {
        all_added = all_added && queue.added;
    }

    self_test_.status = faulted ? report_.status :
                        tests_failed ? Mes11Status::ring_test_failed :
                        !all_added ? Mes11Status::invalid_memory : Mes11Status::completed;
    return self_test_.status;
}

void Mes11::forget_allocations() noexcept {
    for (auto& word : doorbell_bits_) {
        word = 0;
    }

    for (auto& word : queue_id_bits_) {
        word = 0;
    }

    // amdgpu_mes_doorbell_init reserves the aggregated doorbells; idr id 0 is
    // never handed out.
    for (uint32_t level = 0; level < mes11_priority_levels; ++level) {
        set_bit(doorbell_bits_, level);
    }

    set_bit(queue_id_bits_, 0);
    report_.process_queues = 0;
    report_.doorbells_allocated = 0;
    report_.queue_ids_allocated = 0;
}

Mes11Status Mes11::allocate_doorbell(Mes11QueueType type, uint32_t& doorbell_index) noexcept {
    doorbell_index = 0;

    if (phase_ == Phase::uninitialized) {
        return Mes11Status::unavailable;
    }

    if (type != Mes11QueueType::gfx && type != Mes11QueueType::compute &&
        type != Mes11QueueType::sdma) {
        return Mes11Status::invalid_argument;
    }

    const uint32_t first = type == Mes11QueueType::sdma ? soc21_doorbell::sdma_engine0 : 0;

    for (uint32_t bit = first; bit < mes11_doorbell_slots; ++bit) {
        if (!test_bit(doorbell_bits_, bit)) {
            set_bit(doorbell_bits_, bit);
            ++report_.doorbells_allocated;
            doorbell_index = report_.aggregated_doorbell_base + bit * 2;
            return Mes11Status::completed;
        }
    }

    return Mes11Status::doorbell_failure;
}

bool Mes11::doorbell_allocated(uint32_t doorbell_index) const noexcept {
    const uint32_t base = report_.aggregated_doorbell_base;

    if (phase_ == Phase::uninitialized || doorbell_index < base ||
        ((doorbell_index - base) & 1u) != 0) {
        return false;
    }

    const uint32_t bit = (doorbell_index - base) / 2;
    return bit >= mes11_priority_levels && bit < mes11_doorbell_slots &&
           test_bit(doorbell_bits_, bit);
}

Mes11Status Mes11::free_doorbell(uint32_t doorbell_index) noexcept {
    if (!doorbell_allocated(doorbell_index)) {
        return Mes11Status::invalid_argument;
    }

    clear_bit(doorbell_bits_, (doorbell_index - report_.aggregated_doorbell_base) / 2);
    --report_.doorbells_allocated;
    return Mes11Status::completed;
}

Mes11Status Mes11::allocate_queue_id(uint32_t& queue_id) noexcept {
    queue_id = 0;

    if (phase_ == Phase::uninitialized) {
        return Mes11Status::unavailable;
    }

    for (uint32_t id = 1; id < mes11_max_queue_ids; ++id) {
        if (!test_bit(queue_id_bits_, id)) {
            set_bit(queue_id_bits_, id);
            ++report_.queue_ids_allocated;
            queue_id = id;
            return Mes11Status::completed;
        }
    }

    return Mes11Status::busy;
}

Mes11Status Mes11::reserve_queue_id(uint32_t queue_id) noexcept {
    if (phase_ == Phase::uninitialized || queue_id == 0 || queue_id >= mes11_max_queue_ids) {
        return Mes11Status::invalid_argument;
    }
    if (test_bit(queue_id_bits_, queue_id)) {
        return Mes11Status::busy;
    }
    set_bit(queue_id_bits_, queue_id);
    ++report_.queue_ids_allocated;
    return Mes11Status::completed;
}

Mes11Status Mes11::free_queue_id(uint32_t queue_id) noexcept {
    if (queue_id == 0 || queue_id >= mes11_max_queue_ids || !test_bit(queue_id_bits_, queue_id)) {
        return Mes11Status::invalid_argument;
    }

    clear_bit(queue_id_bits_, queue_id);
    --report_.queue_ids_allocated;
    return Mes11Status::completed;
}

uint32_t Mes11::api_version() const noexcept {
    return (report_.scheduler_version & self_test::api_version_mask) >> self_test::api_version_shift;
}

Mes11Status Mes11::init_queue_mqd(Mes11QueueType type, const Gfx11MqdProperties& properties,
                                  sdma::CommandBuffer mqd) noexcept {
    if (phase_ != Phase::scheduler) {
        return phase_ == Phase::faulted ? Mes11Status::quarantined : Mes11Status::unavailable;
    }

    const uint32_t bytes = properties.queue_bytes;
    const bool sdma = type == Mes11QueueType::sdma;

    if ((type != Mes11QueueType::gfx && type != Mes11QueueType::compute && !sdma) ||
        mqd.size() < (sdma ? mes11_sdma_mqd_bytes : mes11_mqd_bytes) || bytes < 1024 ||
        (bytes & (bytes - 1)) != 0 || !doorbell_allocated(properties.doorbell_index)) {
        return Mes11Status::invalid_argument;
    }

    if (!queue_mqd(type, properties, mqd)) {
        if (!sdma) {
            grbm_select(0, 0);
        }

        return quarantine(Mes11Status::mmio_failure);
    }

    return Mes11Status::completed;
}

Mes11Status Mes11::add_hw_queue(const Mes11AddQueue& queue) noexcept {
    if (phase_ != Phase::scheduler) {
        return phase_ == Phase::faulted ? Mes11Status::quarantined : Mes11Status::unavailable;
    }

    if ((queue.type != Mes11QueueType::gfx && queue.type != Mes11QueueType::compute &&
         queue.type != Mes11QueueType::sdma) || queue.pasid == 0 ||
        queue.pasid >= (1u << 16) || !valid_page_address(queue.page_table_base_addr) ||
        queue.process_va_end == 0 || queue.process_va_end >= gpu_address_limit ||
        !valid_page_address(queue.process_context_address) ||
        !valid_page_address(queue.gang_context_address) ||
        !valid_priority(queue.inprocess_gang_priority) ||
        !valid_priority(queue.gang_global_priority_level) ||
        !doorbell_allocated(queue.doorbell_index) || !valid_page_address(queue.mqd_address) ||
        !valid_qword_address(queue.wptr_va) || !valid_qword_address(queue.wptr_mc)) {
        return Mes11Status::invalid_argument;
    }

    // hub->vm_cntx_cntl, as self_test reads and checks it.
    uint32_t vm_context_cntl = 0;

    if (!read_vm_context_cntl(vm_context_cntl)) {
        return quarantine(Mes11Status::mmio_failure);
    }

    if ((vm_context_cntl & self_test::vm_structure_mask) != self_test::vm_structure) {
        return finish(Mes11Status::unsupported_ip);
    }

    uint8_t frame[mes11_api_frame_bytes]{};

    if (!encode_add_queue(queue, vm_context_cntl, frame)) {
        return quarantine(Mes11Status::invalid_memory);
    }

    const Mes11Status status = submit(frame, api::add::api_status);

    if (status == Mes11Status::completed) {
        ++report_.process_queues;
    }

    return finish(status);
}

Mes11Status Mes11::remove_hw_queue(const Mes11RemoveQueue& queue) noexcept {
    if (phase_ != Phase::scheduler) {
        return phase_ == Phase::faulted ? Mes11Status::quarantined : Mes11Status::unavailable;
    }

    if (report_.process_queues == 0 || !doorbell_allocated(queue.doorbell_index) ||
        !valid_page_address(queue.gang_context_address)) {
        return Mes11Status::invalid_argument;
    }

    const Mes11Status status = submit_remove_queue(queue);

    if (status == Mes11Status::completed) {
        --report_.process_queues;
    }

    return finish(status);
}

// mes_v11_0_suspend_gang / mes_v11_0_resume_gang with the all-gangs bit.
Mes11Status Mes11::submit_gang_frame(uint32_t opcode, uint32_t api_status_offset) noexcept {
    if (phase_ != Phase::scheduler) {
        return phase_ == Phase::faulted ? Mes11Status::quarantined : Mes11Status::unavailable;
    }

    report_.suspend_resume_supported = (report_.scheduler_version & 0xfffu) >=
                                       mes11_suspend_resume_version;

    if (!report_.suspend_resume_supported) {
        return finish(Mes11Status::completed);
    }

    uint8_t frame[mes11_api_frame_bytes]{};
    const sdma::CommandBuffer out(frame, sizeof(frame));

    if (!put32(out, 0, api::header(opcode)) ||
        !put32(out, api::gang::all_flags, api::gang::all_gangs)) {
        return quarantine(Mes11Status::invalid_memory);
    }

    return finish(submit(frame, api_status_offset));
}

Mes11Status Mes11::suspend_all() noexcept {
    const Mes11Status status = submit_gang_frame(api::op_suspend, api::gang::suspend_api_status);

    if (status == Mes11Status::completed && report_.suspend_resume_supported) {
        report_.gangs_suspended = true;
    }

    return status;
}

Mes11Status Mes11::resume_all() noexcept {
    const Mes11Status status = submit_gang_frame(api::op_resume, api::gang::resume_api_status);

    if (status == Mes11Status::completed) {
        report_.gangs_suspended = false;
    }

    return status;
}

} // namespace rx7900::hardware

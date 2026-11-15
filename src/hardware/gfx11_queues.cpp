/*
Module: rx7900.hardware.gfx11_queues
Purpose: Build GC 11.0.0 KGQ/KCQ MQDs, map them through MES, emit clear state, ring- and IB-test them; share the MQD/ring builders.
Created: 2026-10-21
Architecture: Bounded GC register reads under GRBM selection and checked byte
writers over native-owned local-VRAM allocations; MES legacy-queue frames through
Mes11; the native owner supplies MMIO, doorbell stores, delays and allocations.
Copyright 2021 Advanced Micro Devices, Inc.; Copyright 2014 Advanced Micro Devices, Inc.;
Copyright 2008 Advanced Micro Devices, Inc., Copyright 2008 Red Hat Inc.,
Copyright 2009 Jerome Glisse.
Adapted Linux v6.12 gfx_v11_0.c, amdgpu_gfx.c, amdgpu_gfx.h, amdgpu_ring.c,
amdgpu_ring.h, amdgpu_mes.c, amdgpu_drv.c, amdgpu_ib.c, amdgpu_fence.c,
amdgpu_device.c, nbio_v4_3.c, soc15.h, soc21.c, nvd.h, v11_structs.h, soc21_enum
(CACHE_FLUSH_AND_INV_TS_EVENT) and gc_11_0_0/nbio_4_3_0 offset/sh_mask/default headers.
Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#include "gfx11_queues.hpp"
#include "gfx11_clear_state.hpp"
#include "gpuvm.hpp"
#include "pm4.hpp"

namespace rx7900::hardware {
namespace {

// gc_11_0_0_offset.h, BASE_IDX 0.
constexpr uint32_t reg_cp_device_id = 0x1deb;
constexpr uint32_t reg_cp_max_context = 0x1e4e;
constexpr uint32_t reg_cp_gfx_hqd_vmid = 0x1e81;
constexpr uint32_t reg_cp_gfx_hqd_queue_priority = 0x1e84;
constexpr uint32_t reg_cp_gfx_hqd_quantum = 0x1e85;
constexpr uint32_t reg_cp_gfx_hqd_rptr = 0x1e88;
constexpr uint32_t reg_cp_rb_doorbell_control = 0x1e8d;
constexpr uint32_t reg_cp_gfx_hqd_cntl = 0x1e8f;
constexpr uint32_t reg_cp_gfx_mqd_control = 0x1e9a;
constexpr uint32_t reg_cp_hqd_persistent_state = 0x1fad;
constexpr uint32_t reg_cp_hqd_pq_rptr = 0x1fb3;
constexpr uint32_t reg_cp_hqd_pq_doorbell_control = 0x1fb8;
constexpr uint32_t reg_cp_hqd_pq_control = 0x1fba;
constexpr uint32_t reg_cp_hqd_ib_control = 0x1fbe;
constexpr uint32_t reg_cp_mqd_control = 0x1fcb;
constexpr uint32_t reg_cp_hqd_eop_control = 0x1fd0;
constexpr uint32_t highest_segment0_register = reg_cp_hqd_eop_control;
// gc_11_0_0_offset.h, BASE_IDX 1.
constexpr uint32_t reg_grbm_gfx_cntl = 0x0900;
constexpr uint32_t reg_pa_sc_tile_steering_override = 0x00d7;
constexpr uint32_t reg_scratch_reg0 = 0x2040;
constexpr uint32_t highest_segment1_register = reg_scratch_reg0;

// gc_11_0_0_sh_mask.h fields (mask, shift).
struct Field final {
    uint32_t mask;
    uint32_t shift;
};

constexpr Field gfx_mqd_vmid{0x0000000fu, 0};
constexpr Field gfx_mqd_priv_state{0x00000100u, 8};
constexpr Field gfx_mqd_cache_policy{0x03000000u, 24};
constexpr Field gfx_hqd_vmid{0x0000000fu, 0};
constexpr Field gfx_priority_level{0x0000000fu, 0};
constexpr Field gfx_quantum_en{0x00000001u, 0};
constexpr Field gfx_rb_bufsz{0x0000003fu, 0};
constexpr Field gfx_rb_blksz{0x00003f00u, 8};
constexpr Field rb_doorbell_offset{0x0ffffffcu, 2};
constexpr Field rb_doorbell_en{0x40000000u, 30};
constexpr Field eop_size{0x0000003fu, 0};
constexpr Field pq_doorbell_offset{0x0ffffffcu, 2};
constexpr Field pq_doorbell_source{0x10000000u, 28};
constexpr Field pq_doorbell_en{0x40000000u, 30};
constexpr Field pq_doorbell_hit{0x80000000u, 31};
constexpr Field mqd_control_vmid{0x0000000fu, 0};
constexpr Field pq_queue_size{0x0000003fu, 0};
constexpr Field pq_rptr_block_size{0x00003f00u, 8};
constexpr Field pq_unord_dispatch{0x10000000u, 28};
constexpr Field pq_tunnel_dispatch{0x20000000u, 29};
constexpr Field pq_priv_state{0x40000000u, 30};
constexpr Field pq_kmd_queue{0x80000000u, 31};
constexpr Field persistent_preload_size{0x0003ff00u, 8};
constexpr Field ib_min_ib_avail_size{0x00300000u, 20};

// REG_SET_FIELD.
constexpr uint32_t set_field(uint32_t value, Field field, uint32_t data) noexcept {
    return (value & ~field.mask) | ((data << field.shift) & field.mask);
}

// struct v11_gfx_mqd byte offsets, measured by compiling the pinned
// v11_structs.h with offsetof (sizeof 2048).
namespace gfx_mqd {
constexpr uint32_t cp_mqd_base_addr = 512;
constexpr uint32_t cp_mqd_base_addr_hi = 516;
constexpr uint32_t cp_gfx_hqd_active = 520;
constexpr uint32_t cp_gfx_hqd_vmid = 524;
constexpr uint32_t cp_gfx_hqd_queue_priority = 536;
constexpr uint32_t cp_gfx_hqd_quantum = 540;
constexpr uint32_t cp_gfx_hqd_base = 544;
constexpr uint32_t cp_gfx_hqd_base_hi = 548;
constexpr uint32_t cp_gfx_hqd_rptr = 552;
constexpr uint32_t cp_gfx_hqd_rptr_addr = 556;
constexpr uint32_t cp_gfx_hqd_rptr_addr_hi = 560;
constexpr uint32_t cp_rb_wptr_poll_addr_lo = 564;
constexpr uint32_t cp_rb_wptr_poll_addr_hi = 568;
constexpr uint32_t cp_rb_doorbell_control = 572;
constexpr uint32_t cp_gfx_hqd_cntl = 580;
constexpr uint32_t cp_gfx_hqd_wptr = 596;
constexpr uint32_t cp_gfx_hqd_wptr_hi = 600;
constexpr uint32_t cp_gfx_mqd_control = 648;
} // namespace gfx_mqd

// struct v11_compute_mqd byte offsets, measured the same way (sizeof 2048).
namespace compute_mqd {
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
constexpr uint32_t cp_hqd_pipe_priority = 532;
constexpr uint32_t cp_hqd_queue_priority = 536;
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
constexpr uint32_t cp_hqd_dequeue_request = 608;
constexpr uint32_t cp_mqd_control = 648;
constexpr uint32_t cp_hqd_eop_base_addr_lo = 660;
constexpr uint32_t cp_hqd_eop_base_addr_hi = 664;
constexpr uint32_t cp_hqd_eop_control = 668;
constexpr uint32_t cp_hqd_pq_wptr_lo = 728;
constexpr uint32_t cp_hqd_pq_wptr_hi = 732;
} // namespace compute_mqd

// gfx_v11_0_compute_mqd_init literals.
constexpr uint32_t compute_mqd_header = 0xC0310800u;
constexpr uint32_t static_thread_mgmt_all = 0xffffffffu;
constexpr uint32_t compute_misc_reserved_value = 7;
constexpr uint32_t preload_size = 0x55;
constexpr uint32_t min_ib_avail_size = 3;
// AMDGPU_GPU_PAGE_SIZE.
constexpr uint32_t gpu_page_bytes = 4096;
// amdgpu_gfx.h / amdgpu_ring.h: AMDGPU_GFX_PIPE_PRIO_HIGH (AMDGPU_RING_PRIO_2)
// and AMDGPU_GFX_QUEUE_PRIORITY_MAXIMUM; amdgpu_ring_to_mqd_prop zeroes both
// for every other ring.
constexpr uint32_t pipe_priority_high = 2;
constexpr uint32_t queue_priority_maximum = 15;

// nvd.h values not in pm4.hpp.
constexpr uint32_t pm4_nop = 0x10;
constexpr uint32_t pm4_set_uconfig_reg = 0x79;
constexpr uint32_t set_uconfig_reg_start = 0x0000c000;
constexpr uint32_t set_uconfig_reg_end = 0x0000c400;
// gfx_v11_ring_insert_nop: at most 0x3ffe in the first NOP header count.
constexpr uint32_t nop_count_limit = 0x3ffe;

// gfx_v11_0_gpu_early_init GC 11.0.0: max_hw_contexts 8; CP_DEVICE_ID 1.
constexpr uint32_t cp_max_context_value = 8 - 1;
constexpr uint32_t cp_device_id_value = 1;
// gfx_v11_0_constants_init leaves pa_sc_tile_steering_override 0.
constexpr uint32_t pa_sc_tile_steering_override_value = 0;

// gfx_v11_0_sw_init GC 11.0.0 MEC topology.
constexpr uint32_t mec_pipes = 4;
constexpr uint32_t mec_queues_per_pipe = 4;
// amdgpu_gfx_mes_enable_kcq: queue_mask = ~0ULL for gfx11_kiq_set_resources.
constexpr uint64_t mes_kcq_queue_mask = ~0ULL;

// nvd.h opcodes and fields used by the IB test frame (amdgpu_ib_schedule).
constexpr uint32_t pm4_cond_exec = 0x22;
constexpr uint32_t pm4_wait_reg_mem = 0x3c;
constexpr uint32_t pm4_indirect_buffer = 0x3f;
constexpr uint32_t pm4_release_mem = 0x49;
constexpr uint32_t pm4_acquire_mem = 0x58;
constexpr uint32_t write_data_dst_sel_memory = 5u << 8; // WRITE_DATA_DST_SEL(5)
constexpr uint32_t write_data_wr_confirm = 1u << 20;   // WR_CONFIRM
constexpr uint32_t indirect_buffer_valid = 1u << 23;   // INDIRECT_BUFFER_VALID
// gfx_v11_0_wait_reg_mem(eng_sel, mem_space 0, opt 1, ...): FUNCTION(3) "equal",
// OPERATION(1) write-then-wait, ENGINE(1) PFP for gfx, 0 ME for compute; poll 0x20.
constexpr uint32_t wait_reg_mem_write_wait_equal = (1u << 6) | 3u;
constexpr uint32_t wait_reg_mem_engine_pfp = 1u << 8;
constexpr uint32_t wait_reg_mem_poll_interval = 0x20;
// nbio_v4_3_hdp_flush_reg: ref_and_mask_cp0 (gfx, << pipe) and cp2 (MEC1, << pipe).
constexpr uint32_t hdp_flush_ref_cp0 = 0x00000001u;
constexpr uint32_t hdp_flush_ref_cp2 = 0x00000004u;
// gfx_v11_0_emit_mem_sync: ACQUIRE_MEM GCR_CNTL GL2_INV|GL2_WB|GLM_INV|GLM_WB|
// GL1_INV|GLV_INV|GLK_INV|GLI_INV(1), CP_COHER_SIZE 0xffffffff/0xffffff, poll 0xA.
constexpr uint32_t acquire_mem_gcr_cntl = (1u << 14) | (1u << 15) | (1u << 5) | (1u << 4) |
                                          (1u << 9) | (1u << 8) | (1u << 7) | 1u;
constexpr uint32_t acquire_mem_size = 0xffffffffu;
constexpr uint32_t acquire_mem_size_hi = 0x00ffffffu;
constexpr uint32_t acquire_mem_poll_interval = 0x0000000au;
// gfx_v11_0_ring_emit_fence: GCR_SEQ|GCR_GL2_WB|GCR_GLM_INV|GCR_GLM_WB|
// CACHE_POLICY(3)|EVENT_TYPE(CACHE_FLUSH_AND_INV_TS_EVENT 0x14)|EVENT_INDEX(5),
// then DATA_SEL(1) (32-bit, amdgpu_fence_emit never sets FLAG_64BIT) and
// INT_SEL(2) (AMDGPU_FENCE_FLAG_INT), address, seq lo/hi and 0 (not a MES queue).
constexpr uint32_t release_mem_event = (1u << 22) | (1u << 21) | (1u << 13) | (1u << 12) |
                                       (3u << 25) | 0x14u | (5u << 8);
constexpr uint32_t release_mem_data = (1u << 29) | (2u << 24);
// MES user queues (ring->is_mes_queue): INDIRECT_BUFFER "inherit vmid from mqd"
// (gfx_v11_0_ring_emit_ib_gfx 0x400000, _compute 0x40000000).
constexpr uint32_t mes_gfx_ib_inherit_vmid = 0x00400000u;
constexpr uint32_t mes_compute_ib_inherit_vmid = 0x40000000u;
// Process-VM job packets (nvd.h, gfx_v11_0.c): PFP_SYNC_ME, FRAME_CONTROL with
// FRAME_CMD(start ? 0 : 1), WRITE_DATA ENGINE_SEL(1) | WR_CONFIRM to DST_SEL 0
// (gfx_v11_0_ring_emit_wreg on the gfx ring), WAIT_REG_MEM register wait
// (emit_reg_wait: ME, function equal) and memory wait on PFP (pipeline sync,
// poll interval 4), the INDIRECT_BUFFER VMID field and CONTEXT_CONTROL dw2
// (load_enable, plus load_global_config/uconfig, cs_sh_regs, per_context_state
// and gfx_sh_regs on AMDGPU_HAVE_CTX_SWITCH).
constexpr uint32_t pm4_pfp_sync_me = 0x42;
constexpr uint32_t pm4_frame_control = 0x90;
constexpr uint32_t frame_cmd_end = 1u << 28;
constexpr uint32_t write_data_register_gfx = (1u << 30) | write_data_wr_confirm;
constexpr uint32_t wait_reg_mem_register_equal = 3u;
constexpr uint32_t wait_reg_mem_memory_equal_pfp = (1u << 4) | 3u | wait_reg_mem_engine_pfp;
constexpr uint32_t pipeline_sync_poll_interval = 4;
constexpr uint32_t indirect_buffer_vmid_shift = 24;
constexpr uint32_t context_control_load_enable = 0x80000000u;
constexpr uint32_t context_control_switch = 0x8001u | 0x01000000u | 0x10002u;
// AMDGPU_PASID_BITS 16 (IH_VMID LUT field); 48-bit process VAs.
constexpr uint32_t job_pasid_limit = 1u << 16;
constexpr uint64_t job_va_limit = 1ULL << 48;
// IB control length field (INDIRECT_BUFFER IB_SIZE, 20 bits).
constexpr uint32_t ib_size_mask = 0x000fffffu;

constexpr uint32_t maximum_polls = 1000000;
constexpr uint32_t allocation_count = 4 + 2 * gfx11_max_compute_rings;

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

bool zero(uint8_t* bytes, uint32_t count) noexcept {
    const sdma::CommandBuffer out(bytes, count);

    for (uint32_t offset = 0; offset < count; offset += 4) {
        if (!out.write_u32(offset, 0)) {
            return false;
        }
    }

    return true;
}

bool overlaps(uint64_t a, uint64_t a_bytes, uint64_t b, uint64_t b_bytes) noexcept {
    return a < b + b_bytes && b < a + a_bytes;
}

bool gc_read(const Gfx11GcAccess& access, uint32_t segment, uint32_t reg, uint32_t& value) noexcept {
    return read_ip_register(access.io, segment == 0 ? access.segment0 : access.segment1, reg,
                            value) &&
           value != UINT32_MAX;
}

// soc21_grbm_select(adev, me, pipe, queue, 0): PIPEID[1:0], MEID[3:2], QUEUEID[10:8].
bool gc_select(const Gfx11GcAccess& access, Gfx11QueueSelect select) noexcept {
    return write_ip_register(access.io, access.segment1, reg_grbm_gfx_cntl,
                             (select.pipe & 0x3u) | ((select.me & 0x3u) << 2) |
                                 ((select.queue & 0x7u) << 8));
}

bool valid_queue_bytes(uint32_t bytes) noexcept {
    return bytes >= 1024 && (bytes & (bytes - 1)) == 0;
}

bool zero_mqd(sdma::CommandBuffer mqd) noexcept {
    if (mqd.size() < gfx11_mqd_bytes) {
        return false;
    }

    for (uint32_t offset = 0; offset < gfx11_mqd_bytes; offset += 4) {
        if (!mqd.write_u32(offset, 0)) {
            return false;
        }
    }

    return true;
}

} // namespace

bool gfx11_compute_mqd_init(const Gfx11GcAccess& access, Gfx11QueueSelect select,
                            const Gfx11MqdProperties& prop, sdma::CommandBuffer mqd) noexcept {
    if (!valid_queue_bytes(prop.queue_bytes) || !zero_mqd(mqd) || !gc_select(access, select)) {
        return false;
    }

    const uint64_t eop = prop.eop_address >> 8;
    const uint64_t pq = prop.hqd_base_address >> 8;
    uint32_t eop_control = 0;
    uint32_t doorbell_first = 0;
    uint32_t mqd_control = 0;
    uint32_t pq_control = 0;
    uint32_t doorbell = 0;
    uint32_t rptr = 0;
    uint32_t persistent = 0;
    uint32_t ib_control = 0;

    // Linux computes the doorbell control twice from the same register; the
    // first value is overwritten, but both reads are part of the sequence.
    if (!gc_read(access, 0, reg_cp_hqd_eop_control, eop_control) ||
        !gc_read(access, 0, reg_cp_hqd_pq_doorbell_control, doorbell_first) ||
        !gc_read(access, 0, reg_cp_mqd_control, mqd_control) ||
        !gc_read(access, 0, reg_cp_hqd_pq_control, pq_control) ||
        !gc_read(access, 0, reg_cp_hqd_pq_doorbell_control, doorbell) ||
        !gc_read(access, 0, reg_cp_hqd_pq_rptr, rptr) ||
        !gc_read(access, 0, reg_cp_hqd_persistent_state, persistent) ||
        !gc_read(access, 0, reg_cp_hqd_ib_control, ib_control) ||
        !gc_select(access, {0, 0, 0})) {
        return false;
    }

    eop_control = set_field(eop_control, eop_size, log2_exact(gfx11_mec_hpd_bytes / 4) - 1);
    doorbell = set_field(doorbell, pq_doorbell_offset, prop.doorbell_index);
    doorbell = set_field(doorbell, pq_doorbell_en, 1);
    doorbell = set_field(doorbell, pq_doorbell_source, 0);
    doorbell = set_field(doorbell, pq_doorbell_hit, 0);
    mqd_control = set_field(mqd_control, mqd_control_vmid, 0);
    pq_control = set_field(pq_control, pq_queue_size, log2_exact(prop.queue_bytes / 4) - 1);
    pq_control = set_field(pq_control, pq_rptr_block_size, log2_exact(gpu_page_bytes / 4) - 1);
    pq_control = set_field(pq_control, pq_unord_dispatch, 1);
    pq_control = set_field(pq_control, pq_tunnel_dispatch, prop.allow_tunneling ? 1 : 0);
    pq_control = set_field(pq_control, pq_priv_state, 1);
    pq_control = set_field(pq_control, pq_kmd_queue, 1);
    persistent = set_field(persistent, persistent_preload_size, preload_size);
    ib_control = set_field(ib_control, ib_min_ib_avail_size, min_ib_avail_size);

    return mqd.write_u32(compute_mqd::header, compute_mqd_header) &&
        mqd.write_u32(compute_mqd::compute_pipelinestat_enable, 1) &&
        mqd.write_u32(compute_mqd::static_thread_mgmt_se0, static_thread_mgmt_all) &&
        mqd.write_u32(compute_mqd::static_thread_mgmt_se1, static_thread_mgmt_all) &&
        mqd.write_u32(compute_mqd::static_thread_mgmt_se2, static_thread_mgmt_all) &&
        mqd.write_u32(compute_mqd::static_thread_mgmt_se3, static_thread_mgmt_all) &&
        mqd.write_u32(compute_mqd::compute_misc_reserved, compute_misc_reserved_value) &&
        mqd.write_u32(compute_mqd::cp_hqd_eop_base_addr_lo, low(eop)) &&
        mqd.write_u32(compute_mqd::cp_hqd_eop_base_addr_hi, high(eop)) &&
        mqd.write_u32(compute_mqd::cp_hqd_eop_control, eop_control) &&
        mqd.write_u32(compute_mqd::cp_hqd_dequeue_request, 0) &&
        mqd.write_u32(compute_mqd::cp_hqd_pq_wptr_lo, 0) &&
        mqd.write_u32(compute_mqd::cp_hqd_pq_wptr_hi, 0) &&
        mqd.write_u32(compute_mqd::cp_mqd_base_addr_lo, low(prop.mqd_address) & 0xfffffffcu) &&
        mqd.write_u32(compute_mqd::cp_mqd_base_addr_hi, high(prop.mqd_address)) &&
        mqd.write_u32(compute_mqd::cp_mqd_control, mqd_control) &&
        mqd.write_u32(compute_mqd::cp_hqd_pq_base_lo, low(pq)) &&
        mqd.write_u32(compute_mqd::cp_hqd_pq_base_hi, high(pq)) &&
        mqd.write_u32(compute_mqd::cp_hqd_pq_control, pq_control) &&
        mqd.write_u32(compute_mqd::cp_hqd_pq_rptr_report_addr_lo,
                      low(prop.rptr_address) & 0xfffffffcu) &&
        mqd.write_u32(compute_mqd::cp_hqd_pq_rptr_report_addr_hi,
                      high(prop.rptr_address) & 0xffffu) &&
        mqd.write_u32(compute_mqd::cp_hqd_pq_wptr_poll_addr_lo,
                      low(prop.wptr_address) & 0xfffffffcu) &&
        mqd.write_u32(compute_mqd::cp_hqd_pq_wptr_poll_addr_hi,
                      high(prop.wptr_address) & 0xffffu) &&
        mqd.write_u32(compute_mqd::cp_hqd_pq_doorbell_control, doorbell) &&
        mqd.write_u32(compute_mqd::cp_hqd_pq_rptr, rptr) &&
        mqd.write_u32(compute_mqd::cp_hqd_vmid, 0) &&
        mqd.write_u32(compute_mqd::cp_hqd_persistent_state, persistent) &&
        mqd.write_u32(compute_mqd::cp_hqd_ib_control, ib_control) &&
        mqd.write_u32(compute_mqd::cp_hqd_pipe_priority, prop.hqd_pipe_priority) &&
        mqd.write_u32(compute_mqd::cp_hqd_queue_priority, prop.hqd_queue_priority) &&
        mqd.write_u32(compute_mqd::cp_hqd_active, prop.hqd_active ? 1 : 0);
}

bool gfx11_gfx_mqd_init(const Gfx11GcAccess& access, Gfx11QueueSelect select,
                        const Gfx11MqdProperties& prop, sdma::CommandBuffer mqd) noexcept {
    if (!valid_queue_bytes(prop.queue_bytes) || !zero_mqd(mqd) || !gc_select(access, select)) {
        return false;
    }

    uint32_t mqd_control = 0;
    uint32_t vmid = 0;
    uint32_t priority = 0;
    uint32_t quantum = 0;
    uint32_t control = 0;
    uint32_t doorbell = 0;
    uint32_t rptr = 0;

    if (!gc_read(access, 0, reg_cp_gfx_mqd_control, mqd_control) ||
        !gc_read(access, 0, reg_cp_gfx_hqd_vmid, vmid) ||
        !gc_read(access, 0, reg_cp_gfx_hqd_queue_priority, priority) ||
        !gc_read(access, 0, reg_cp_gfx_hqd_quantum, quantum) ||
        !gc_read(access, 0, reg_cp_gfx_hqd_cntl, control) ||
        !gc_read(access, 0, reg_cp_rb_doorbell_control, doorbell) ||
        !gc_read(access, 0, reg_cp_gfx_hqd_rptr, rptr) || !gc_select(access, {0, 0, 0})) {
        return false;
    }

    const uint64_t base = prop.hqd_base_address >> 8;
    const uint32_t rb_bufsz = log2_exact(prop.queue_bytes / 4) - 1;
    mqd_control = set_field(mqd_control, gfx_mqd_vmid, 0);
    mqd_control = set_field(mqd_control, gfx_mqd_priv_state, 1);
    mqd_control = set_field(mqd_control, gfx_mqd_cache_policy, 0);
    // The CP_GFX_HQD_VMID value is computed and discarded; the MQD gets literal 0.
    vmid = set_field(vmid, gfx_hqd_vmid, 0);
    // gfx_v11_0_gfx_mqd_set_priority: PRIORITY_LEVEL 1 only for PIPE_PRIO_HIGH.
    priority = set_field(priority, gfx_priority_level,
                         prop.hqd_pipe_priority == pipe_priority_high ? 1 : 0);
    quantum = set_field(quantum, gfx_quantum_en, 1);
    control = set_field(control, gfx_rb_bufsz, rb_bufsz);
    control = set_field(control, gfx_rb_blksz, rb_bufsz - 2);
    doorbell = set_field(doorbell, rb_doorbell_offset, prop.doorbell_index);
    doorbell = set_field(doorbell, rb_doorbell_en, 1);

    return mqd.write_u32(gfx_mqd::cp_gfx_hqd_wptr, 0) &&
        mqd.write_u32(gfx_mqd::cp_gfx_hqd_wptr_hi, 0) &&
        mqd.write_u32(gfx_mqd::cp_mqd_base_addr, low(prop.mqd_address) & 0xfffffffcu) &&
        mqd.write_u32(gfx_mqd::cp_mqd_base_addr_hi, high(prop.mqd_address)) &&
        mqd.write_u32(gfx_mqd::cp_gfx_mqd_control, mqd_control) &&
        mqd.write_u32(gfx_mqd::cp_gfx_hqd_vmid, 0) &&
        mqd.write_u32(gfx_mqd::cp_gfx_hqd_queue_priority, priority) &&
        mqd.write_u32(gfx_mqd::cp_gfx_hqd_quantum, quantum) &&
        mqd.write_u32(gfx_mqd::cp_gfx_hqd_base, low(base)) &&
        mqd.write_u32(gfx_mqd::cp_gfx_hqd_base_hi, high(base)) &&
        mqd.write_u32(gfx_mqd::cp_gfx_hqd_rptr_addr, low(prop.rptr_address) & 0xfffffffcu) &&
        mqd.write_u32(gfx_mqd::cp_gfx_hqd_rptr_addr_hi, high(prop.rptr_address) & 0xffffu) &&
        mqd.write_u32(gfx_mqd::cp_rb_wptr_poll_addr_lo, low(prop.wptr_address) & 0xfffffffcu) &&
        mqd.write_u32(gfx_mqd::cp_rb_wptr_poll_addr_hi, high(prop.wptr_address) & 0xffffu) &&
        mqd.write_u32(gfx_mqd::cp_gfx_hqd_cntl, control) &&
        mqd.write_u32(gfx_mqd::cp_rb_doorbell_control, doorbell) &&
        mqd.write_u32(gfx_mqd::cp_gfx_hqd_rptr, rptr) &&
        mqd.write_u32(gfx_mqd::cp_gfx_hqd_active, 1);
}

bool gfx11_ring_write(Gfx11RingWriter& writer, uint32_t value) noexcept {
    const size_t dwords = writer.ring.size() / 4;

    if (dwords == 0 || (dwords & (dwords - 1)) != 0 ||
        !writer.ring.write_u32(static_cast<size_t>(writer.wptr & (dwords - 1)) * 4, value)) {
        return false;
    }

    ++writer.wptr;
    return true;
}

bool gfx11_ring_pad(Gfx11RingWriter& writer) noexcept {
    const uint32_t count = (gfx11_ring_align_mask + 1 -
                            (low(writer.wptr) & gfx11_ring_align_mask)) & gfx11_ring_align_mask;

    if (count == 0) {
        return true;
    }

    if (count == 1) {
        return gfx11_ring_write(writer, gfx11_ring_nop);
    }

    bool ok = gfx11_ring_write(writer, pm4::packet3(pm4_nop, count - 2 < nop_count_limit ?
                                                             count - 2 : nop_count_limit));

    for (uint32_t index = 1; ok && index < count; ++index) {
        ok = gfx11_ring_write(writer, gfx11_ring_nop);
    }

    return ok;
}

bool gfx11_emit_ring_test(Gfx11RingWriter& writer, uint32_t scratch_register) noexcept {
    return scratch_register >= set_uconfig_reg_start && scratch_register < set_uconfig_reg_end &&
           gfx11_ring_write(writer, pm4::packet3(pm4_set_uconfig_reg, 1)) &&
           gfx11_ring_write(writer, scratch_register - set_uconfig_reg_start) &&
           gfx11_ring_write(writer, gfx11_ring_test_value);
}

bool gfx11_write_test_ib(sdma::CommandBuffer ib, uint64_t destination) noexcept {
    return ib.write_u32(0, pm4::packet3(pm4::write_data, 3)) &&
           ib.write_u32(4, write_data_dst_sel_memory | write_data_wr_confirm) &&
           ib.write_u32(8, low(destination)) && ib.write_u32(12, high(destination)) &&
           ib.write_u32(16, gfx11_ring_test_value);
}

namespace {

// amdgpu_ring_patch_cond_exec: dwords after the count through the last written
// dword ((wptr - 1) - offset; the unmasked 64-bit positions give the same
// difference as Linux's wrap correction).
bool patch_cond_exec(Gfx11RingWriter& writer, uint64_t count_position) noexcept {
    const size_t dwords = writer.ring.size() / 4;
    return writer.ring.write_u32(static_cast<size_t>(count_position & (dwords - 1)) * 4,
                                 static_cast<uint32_t>(writer.wptr - 1 - count_position));
}

// One GmcRingOp with the gfx ring encoders: emit_wreg, emit_reg_wait and
// emit_reg_write_reg_wait (write ref to reg0, wait for reg1 & mask == mask).
bool emit_gmc_op(Gfx11RingWriter& writer, const GmcRingOp& op) noexcept {
    switch (op.kind) {
    case GmcRingOpKind::write:
        return gfx11_ring_write(writer, pm4::packet3(pm4::write_data, 3)) &&
               gfx11_ring_write(writer, write_data_register_gfx) && gfx11_ring_write(writer, op.reg0) &&
               gfx11_ring_write(writer, 0) && gfx11_ring_write(writer, op.value);
    case GmcRingOpKind::reg_wait:
        return gfx11_ring_write(writer, pm4::packet3(pm4_wait_reg_mem, 5)) &&
               gfx11_ring_write(writer, wait_reg_mem_register_equal) && gfx11_ring_write(writer, op.reg0) &&
               gfx11_ring_write(writer, 0) && gfx11_ring_write(writer, op.value) &&
               gfx11_ring_write(writer, op.mask) && gfx11_ring_write(writer, wait_reg_mem_poll_interval);
    case GmcRingOpKind::write_wait:
        return gfx11_ring_write(writer, pm4::packet3(pm4_wait_reg_mem, 5)) &&
               gfx11_ring_write(writer, wait_reg_mem_write_wait_equal | wait_reg_mem_engine_pfp) &&
               gfx11_ring_write(writer, op.reg0) && gfx11_ring_write(writer, op.reg1) &&
               gfx11_ring_write(writer, op.value) && gfx11_ring_write(writer, op.mask) &&
               gfx11_ring_write(writer, wait_reg_mem_poll_interval);
    }

    return false;
}

// amdgpu_vm_flush on the gfx ring (always a flush: CPU-updated VMs), wrapped in
// its own COND_EXEC whose count is patched after the PASID mapping.
bool emit_vm_flush(Gfx11RingWriter& writer, uint64_t cond, const Gfx11VmFrame& vm) noexcept {
    bool ok = gfx11_ring_write(writer, pm4::packet3(pm4_cond_exec, 3)) &&
              gfx11_ring_write(writer, low(cond)) && gfx11_ring_write(writer, high(cond)) &&
              gfx11_ring_write(writer, 0);
    const uint64_t count = writer.wptr;
    ok = ok && gfx11_ring_write(writer, 0);

    if (ok && vm.pipeline_sync) {
        ok = gfx11_ring_write(writer, pm4::packet3(pm4_wait_reg_mem, 5)) &&
             gfx11_ring_write(writer, wait_reg_mem_memory_equal_pfp) &&
             gfx11_ring_write(writer, low(vm.sync_address)) && gfx11_ring_write(writer, high(vm.sync_address)) &&
             gfx11_ring_write(writer, vm.sync_sequence) && gfx11_ring_write(writer, 0xffffffffu) &&
             gfx11_ring_write(writer, pipeline_sync_poll_interval);
    }

    for (uint32_t index = 0; ok && index < vm.flush.count; ++index) {
        ok = emit_gmc_op(writer, vm.flush.ops[index]);
    }

    // gfx_v11_0_ring_emit_vm_flush: sync PFP to ME after the flush.
    ok = ok && gfx11_ring_write(writer, pm4::packet3(pm4_pfp_sync_me, 0)) && gfx11_ring_write(writer, 0);

    if (ok && vm.pasid_mapping) {
        ok = emit_gmc_op(writer, {GmcRingOpKind::write, vm.pasid_register, 0, vm.pasid, 0});
    }

    return ok && patch_cond_exec(writer, count);
}

} // namespace

bool gfx11_emit_ib_frame(Gfx11RingWriter& writer, const Gfx11IbFrame& frame) noexcept {
    const bool gfx = frame.gfx;
    const uint64_t cond = frame.cond_exec_address;
    const Gfx11VmFrame* vm = frame.vm;
    // ref_and_mask_cp0 << pipe (gfx, PFP) or ref_and_mask_cp2 << pipe (ME1, ME).
    const uint32_t ref = (gfx ? hdp_flush_ref_cp0 : hdp_flush_ref_cp2) << (frame.pipe & 3u);
    const uint32_t inherit = !frame.mes_queue ? 0 :
                             gfx ? mes_gfx_ib_inherit_vmid : mes_compute_ib_inherit_vmid;
    const uint32_t vmid = vm != nullptr ? vm->vmid << indirect_buffer_vmid_shift : 0;
    const uint32_t control = frame.ib_dwords | (gfx ? 0 : indirect_buffer_valid) | inherit | vmid;
    const uint32_t context = frame.mes_queue ? frame.hw_queue_id | gfx11_fence_mes_queue_flag : 0;
    uint64_t cond_count = 0;

    if (frame.pipe > 3 || frame.ib_dwords == 0 || frame.ib_dwords > ib_size_mask ||
        (frame.ib_address & 3u) != 0 || (frame.fence_address & 3u) != 0 ||
        (frame.mes_queue && frame.hw_queue_id >= gfx11_fence_mes_queue_flag) ||
        (vm != nullptr && (!gfx || frame.mes_queue || vm->vmid == 0 || vm->vmid > 15 ||
                           vm->flush.count > 5 || (vm->sync_address & 3u) != 0))) {
        return false;
    }

    bool ok = true;

    if (frame.mem_sync) {
        ok = gfx11_ring_write(writer, pm4::packet3(pm4_acquire_mem, 6)) &&
             gfx11_ring_write(writer, 0) && gfx11_ring_write(writer, acquire_mem_size) &&
             gfx11_ring_write(writer, acquire_mem_size_hi) && gfx11_ring_write(writer, 0) &&
             gfx11_ring_write(writer, 0) && gfx11_ring_write(writer, acquire_mem_poll_interval) &&
             gfx11_ring_write(writer, acquire_mem_gcr_cntl);
    }

    if (ok && vm != nullptr) {
        ok = emit_vm_flush(writer, cond, *vm);
    }

    if (ok && gfx) {
        ok = gfx11_ring_write(writer, pm4::packet3(pm4_cond_exec, 3)) &&
             gfx11_ring_write(writer, low(cond)) && gfx11_ring_write(writer, high(cond)) &&
             gfx11_ring_write(writer, 0);
        cond_count = writer.wptr;
        ok = ok && gfx11_ring_write(writer, 0);
    }

    ok = ok && gfx11_ring_write(writer, pm4::packet3(pm4_wait_reg_mem, 5)) &&
         gfx11_ring_write(writer, wait_reg_mem_write_wait_equal |
                                      (gfx ? wait_reg_mem_engine_pfp : 0)) &&
         gfx11_ring_write(writer, frame.hdp_flush_req) &&
         gfx11_ring_write(writer, frame.hdp_flush_done) && gfx11_ring_write(writer, ref) &&
         gfx11_ring_write(writer, ref) && gfx11_ring_write(writer, wait_reg_mem_poll_interval);

    // A job: emit_cntxcntl, then emit_frame_cntl(start, not secure).
    if (ok && vm != nullptr) {
        ok = gfx11_ring_write(writer, pm4::packet3(pm4::context_control, 1)) &&
             gfx11_ring_write(writer, context_control_load_enable |
                                          (vm->context_switch ? context_control_switch : 0)) &&
             gfx11_ring_write(writer, 0) &&
             gfx11_ring_write(writer, pm4::packet3(pm4_frame_control, 0)) && gfx11_ring_write(writer, 0);
    }

    ok = ok && gfx11_ring_write(writer, pm4::packet3(pm4_indirect_buffer, 2)) &&
         gfx11_ring_write(writer, low(frame.ib_address)) &&
         gfx11_ring_write(writer, high(frame.ib_address)) && gfx11_ring_write(writer, control);

    if (ok && vm != nullptr) {
        ok = gfx11_ring_write(writer, pm4::packet3(pm4_frame_control, 0)) &&
             gfx11_ring_write(writer, frame_cmd_end);
    }

    if (ok && gfx) {
        ok = gfx11_ring_write(writer, pm4::packet3(pm4_cond_exec, 3)) &&
             gfx11_ring_write(writer, low(cond)) && gfx11_ring_write(writer, high(cond)) &&
             gfx11_ring_write(writer, 0) && gfx11_ring_write(writer, 0);
    }

    ok = ok && gfx11_ring_write(writer, pm4::packet3(pm4_release_mem, 6)) &&
         gfx11_ring_write(writer, release_mem_event) && gfx11_ring_write(writer, release_mem_data) &&
         gfx11_ring_write(writer, low(frame.fence_address)) &&
         gfx11_ring_write(writer, high(frame.fence_address)) &&
         gfx11_ring_write(writer, frame.fence_sequence) && gfx11_ring_write(writer, 0) &&
         gfx11_ring_write(writer, context);

    if (ok && gfx) {
        // amdgpu_ring_patch_cond_exec through the last fence dword.
        ok = patch_cond_exec(writer, cond_count);
    }

    return ok;
}

Gfx11QueuesStatus Gfx11Queues::finish(Gfx11QueuesStatus status) noexcept {
    report_.status = status;
    return status;
}

Gfx11QueuesStatus Gfx11Queues::quarantine(Gfx11QueuesStatus reason) noexcept {
    phase_ = Phase::faulted;
    return finish(reason);
}

bool Gfx11Queues::read(uint32_t segment, uint32_t reg, uint32_t& value) noexcept {
    return read_ip_register(io_, registers_.segment(segment), reg, value) && value != UINT32_MAX;
}

bool Gfx11Queues::write(uint32_t segment, uint32_t reg, uint32_t value) noexcept {
    return write_ip_register(io_, registers_.segment(segment), reg, value);
}

// soc21_grbm_select(adev, me, pipe, queue, 0): PIPEID[1:0], MEID[3:2], QUEUEID[10:8].
bool Gfx11Queues::grbm_select(uint32_t me, uint32_t pipe, uint32_t queue) noexcept {
    return write(1, reg_grbm_gfx_cntl, (pipe & 0x3u) | ((me & 0x3u) << 2) | ((queue & 0x7u) << 8));
}

Gfx11QueuesStatus Gfx11Queues::initialize(memory::GcRegisterBase registers, RegisterIo io,
                                          DoorbellIo doorbells, Nbio& nbio, VramLocation vram,
                                          Gfx11QueuesConfig config, Gfx11QueuesPolicy policy,
                                          Gfx11QueuesRecovery recovery) noexcept {
    if (phase_ != Phase::uninitialized && phase_ != Phase::ready) {
        return phase_ == Phase::faulted ? Gfx11QueuesStatus::quarantined : Gfx11QueuesStatus::busy;
    }

    if (report_.memory_retained) {
        return Gfx11QueuesStatus::busy;
    }

    phase_ = Phase::uninitialized;
    report_ = Gfx11QueuesReport();

    if (!registers.valid()) {
        return finish(Gfx11QueuesStatus::unsupported_ip);
    }

    uint32_t offset = 0;
    const uint32_t scratch = registers.segment(1) + reg_scratch_reg0;
    const uint32_t steering = registers.segment(1) + reg_pa_sc_tile_steering_override;

    // SCRATCH_REG0 must be addressable by SET_UCONFIG_REG and the clear-state
    // steering register by SET_CONTEXT_REG, as Linux computes both offsets.
    if (!discovery_register_offset(registers.segment(0), highest_segment0_register,
                                   registers.aperture_bytes(), offset) ||
        !discovery_register_offset(registers.segment(1), highest_segment1_register,
                                   registers.aperture_bytes(), offset) ||
        scratch < set_uconfig_reg_start || scratch >= set_uconfig_reg_end ||
        steering < pm4::set_context_reg_start) {
        return finish(Gfx11QueuesStatus::unsupported_ip);
    }

    // Highest doorbell: the 64-bit gfx ring doorbell (0x8b << 1) and its upper dword.
    const uint32_t gfx_doorbell = soc21_doorbell::gfx_ring0 << 1;

    if (!register_io_valid(io) || io.aperture_bytes != registers.aperture_bytes() ||
        !doorbell_io_valid(doorbells) || doorbells.aperture_dwords <= gfx_doorbell + 1 ||
        recovery.confirm_reset_quiescence == nullptr) {
        return finish(Gfx11QueuesStatus::unavailable);
    }

    if (vram.bytes == 0 || vram.mc_end < vram.mc_start || config.compute_rings == 0 ||
        config.compute_rings > gfx11_max_compute_rings || policy.poll_iterations == 0 ||
        policy.poll_iterations > maximum_polls) {
        return finish(Gfx11QueuesStatus::invalid_argument);
    }

    // gfx_v11_0_gfx_ring_init: me 0 pipe 0 queue 0 (amdgpu_gfx_graphics_queue_acquire
    // with one pipe), never high priority with a single gfx ring.
    Gfx11RingReport& gfx = report_.rings[0];
    gfx.type = Mes11QueueType::gfx;
    gfx.doorbell_index = gfx_doorbell;

    // amdgpu_gfx_compute_queue_acquire multipipe policy (GC > 9.0.0) on MEC1:
    // queue i is pipe i % 4, queue (i / 4) % 4; gfx_v11_0_sw_init then assigns
    // ring ids queue-major, pipe-minor, which is the same order. Ring 0 is the
    // high-priority compute queue when more than one exists.
    for (uint32_t index = 0; index < config.compute_rings; ++index) {
        Gfx11RingReport& ring = report_.rings[1 + index];
        ring.type = Mes11QueueType::compute;
        ring.me = 1;
        ring.pipe = index % mec_pipes;
        ring.queue = (index / mec_pipes) % mec_queues_per_pipe;
        ring.doorbell_index = (soc21_doorbell::mec_ring0 + index) << 1;
        ring.high_priority = config.compute_rings > 1 && index == 0;
    }

    registers_ = registers;
    io_ = io;
    doorbells_ = doorbells;
    nbio_ = &nbio;
    vram_ = vram;
    config_ = config;
    policy_ = policy;
    recovery_ = recovery;
    report_.compute_rings = config.compute_rings;
    phase_ = Phase::ready;
    return finish(Gfx11QueuesStatus::ready);
}

bool Gfx11Queues::retain_memory(memory::Manager& manager, const Gfx11QueueMemory& memory) noexcept {
    struct Need final {
        memory::Handle handle;
        uint32_t bytes;
    };

    const uint32_t compute = config_.compute_rings;
    Need needs[allocation_count]{};
    memory::AllocationInfo info[allocation_count]{};
    uint32_t count = 0;
    needs[count++] = {memory.gfx_ring, gfx11_ring_bytes};
    needs[count++] = {memory.gfx_mqd, gfx11_mqd_bytes};

    for (uint32_t index = 0; index < compute; ++index) {
        needs[count++] = {memory.compute_ring[index], gfx11_ring_bytes};
        needs[count++] = {memory.compute_mqd[index], gfx11_mqd_bytes};
    }

    needs[count++] = {memory.compute_eop, compute * gfx11_mec_hpd_bytes};
    needs[count++] = {memory.writeback, gfx11_writeback_bytes};
    uint32_t retained = 0;
    bool ok = true;

    for (; retained < count && ok; ++retained) {
        ok = retain_local_vram(manager, needs[retained].handle, vram_, needs[retained].bytes,
                               gfx11_queue_alignment, true, info[retained]);
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

    if (!ok) {
        while (retained > 0) {
            --retained;
            manager.unretain(needs[retained].handle);
        }

        return false;
    }

    manager_ = &manager;
    handles_ = memory;
    const memory::AllocationInfo& eop = info[count - 2];
    const memory::AllocationInfo& writeback = info[count - 1];
    writeback_cpu_ = writeback.backing.cpu_bytes;
    report_.writeback_mc = writeback.gpu.virtual_address;
    report_.compute_eop_mc = eop.gpu.virtual_address;
    report_.memory_retained = true;

    for (uint32_t ring = 0; ring < 1 + compute; ++ring) {
        const memory::AllocationInfo& ring_info = info[ring * 2];
        const memory::AllocationInfo& mqd_info = info[ring * 2 + 1];
        const uint32_t eop_offset = ring == 0 ? 0 : (ring - 1) * gfx11_mec_hpd_bytes;
        rings_[ring] = {ring_info.backing.cpu_bytes, ring_info.gpu.virtual_address,
                        mqd_info.backing.cpu_bytes, mqd_info.gpu.virtual_address,
                        ring == 0 ? nullptr : eop.backing.cpu_bytes + eop_offset};
        Gfx11RingReport& report = report_.rings[ring];
        report.ring_mc = ring_info.gpu.virtual_address;
        report.mqd_mc = mqd_info.gpu.virtual_address;
        report.eop_mc = ring == 0 ? 0 : eop.gpu.virtual_address + eop_offset;
        report.rptr_mc = slot_mc(ring, gfx11_writeback_rptr_slot);
        report.wptr_mc = slot_mc(ring, gfx11_writeback_wptr_slot);
        report.wptr = 0;
        // prepare_memory zeroes the fence slot: amdgpu_fence_driver_start_ring
        // with sync_seq 0.
        report.fence_mc = slot_mc(ring, gfx11_writeback_fence_slot);
        report.fence_seq = 0;
        report.fence_value = 0;
        report.ib_disabled = false;
        report.ib_test = Gfx11RingTest::not_run;
    }

    return true;
}

// Like retain_local_vram without the VRAM-aperture requirement: the IB buffer
// may later live in GART-mapped system memory. CPU visibility, GPU read access,
// alignment, the 48-bit MC limit and non-overlap with every retained queue
// buffer are checked before the single lease is taken.
bool Gfx11Queues::retain_ib(memory::Manager& manager, memory::Handle ib_buffer) noexcept {
    memory::AllocationInfo info{};
    sdma::GpuRange range{};

    if (&manager != manager_ || manager.describe(ib_buffer, info) != memory::Status::success ||
        manager.resolve({ib_buffer, 0}, gfx11_ib_test_buffer_bytes, memory::Access::read, range) !=
            memory::Status::success) {
        return false;
    }

    const uint64_t start = info.gpu.virtual_address;
    const uint64_t bytes = gfx11_ib_test_buffer_bytes;

    if ((start & (gfx11_ib_test_alignment - 1)) != 0 || info.gpu.byte_length < bytes ||
        start > sdma::gpu_virtual_address_max - bytes + 1 || info.backing.cpu_bytes == nullptr ||
        info.backing.cpu_byte_count < bytes ||
        overlaps(start, bytes, report_.writeback_mc, gfx11_writeback_bytes) ||
        overlaps(start, bytes, report_.compute_eop_mc,
                 static_cast<uint64_t>(config_.compute_rings) * gfx11_mec_hpd_bytes)) {
        return false;
    }

    for (uint32_t ring = 0; ring <= config_.compute_rings; ++ring) {
        if (overlaps(start, bytes, report_.rings[ring].ring_mc, gfx11_ring_bytes) ||
            overlaps(start, bytes, report_.rings[ring].mqd_mc, gfx11_mqd_bytes)) {
            return false;
        }
    }

    if (manager.retain(ib_buffer, memory::Access::read) != memory::Status::success) {
        return false;
    }

    ib_handle_ = ib_buffer;
    ib_cpu_ = info.backing.cpu_bytes;
    report_.ib_buffer_mc = start;
    report_.ib_buffer_retained = true;
    return true;
}

void Gfx11Queues::release_ib() noexcept {
    if (report_.ib_buffer_retained && manager_ != nullptr) {
        manager_->unretain(ib_handle_);
    }

    ib_handle_ = {};
    ib_cpu_ = nullptr;
    report_.ib_buffer_retained = false;
    report_.ib_buffer_mc = 0;
}

void Gfx11Queues::release_leases() noexcept {
    release_ib();

    if (report_.memory_retained && manager_ != nullptr) {
        manager_->unretain(handles_.gfx_ring);
        manager_->unretain(handles_.gfx_mqd);

        for (uint32_t index = 0; index < config_.compute_rings; ++index) {
            manager_->unretain(handles_.compute_ring[index]);
            manager_->unretain(handles_.compute_mqd[index]);
        }

        manager_->unretain(handles_.compute_eop);
        manager_->unretain(handles_.writeback);
    }

    manager_ = nullptr;
    handles_ = {};
    writeback_cpu_ = nullptr;
    report_.memory_retained = false;

    for (bool& backed_up : mqd_backed_up_) {
        backed_up = false;
    }

    report_.writeback_mc = 0;
    report_.compute_eop_mc = 0;
    report_.queues_mapped = 0;

    for (uint32_t ring = 0; ring < gfx11_max_rings; ++ring) {
        rings_[ring] = {};
        Gfx11RingReport& report = report_.rings[ring];
        report.ring_mc = 0;
        report.mqd_mc = 0;
        report.eop_mc = 0;
        report.rptr_mc = 0;
        report.wptr_mc = 0;
        report.wptr = 0;
        report.mapped = false;
        report.ib_disabled = false;
        report.fence_seq = 0;
        report.fence_value = 0;
        report.fence_mc = 0;
        report.ib_mc = 0;
        report.ib_test_mc = 0;
    }
}

uint64_t Gfx11Queues::slot_mc(uint32_t ring, uint32_t slot) const noexcept {
    return report_.writeback_mc +
           static_cast<uint64_t>(ring * gfx11_writeback_slots_per_ring + slot) *
               gfx11_writeback_slot_bytes;
}

bool Gfx11Queues::write_slot64(uint32_t ring, uint32_t slot, uint64_t value) noexcept {
    const sdma::CommandBuffer out(writeback_cpu_, gfx11_writeback_bytes);
    const uint32_t offset = (ring * gfx11_writeback_slots_per_ring + slot) *
                            gfx11_writeback_slot_bytes;
    return out.write_u32(offset, low(value)) && out.write_u32(offset + 4, high(value));
}

// The sw_init-time state: amdgpu_device_wb_init zeroes the writeback,
// amdgpu_ring_init sets cond_exec to 1 and amdgpu_fence_driver_start_ring the
// fence to sequence 0, amdgpu_ring_clear_ring fills each ring with its NOP and
// gfx_v11_0_mec_init zeroes the MEC HPD buffer. CPU stores only.
bool Gfx11Queues::prepare_memory() noexcept {
    if (!zero(writeback_cpu_, gfx11_writeback_bytes) ||
        !zero(rings_[1].eop_cpu, config_.compute_rings * gfx11_mec_hpd_bytes)) {
        return false;
    }

    for (uint32_t ring = 0; ring < 1 + config_.compute_rings; ++ring) {
        const sdma::CommandBuffer bytes(rings_[ring].cpu, gfx11_ring_bytes);

        if (!write_slot64(ring, gfx11_writeback_cond_exec_slot, 1)) {
            return false;
        }

        for (uint32_t offset = 0; offset < gfx11_ring_bytes; offset += 4) {
            if (!bytes.write_u32(offset, gfx11_ring_nop)) {
                return false;
            }
        }
    }

    return true;
}

// The amdgpu_ring_to_mqd_prop values of a kernel ring: VMID0 MC addresses of its
// ring, MQD, writeback rptr/wptr and (compute) MEC HPD slot; allow_tunneling,
// AMDGPU_GFX_PIPE_PRIO_HIGH and AMDGPU_GFX_QUEUE_PRIORITY_MAXIMUM only for the
// high-priority compute ring, zero priorities otherwise; hqd_active only for KIQ.
Gfx11MqdProperties Gfx11Queues::mqd_properties(uint32_t ring) const noexcept {
    const Gfx11RingReport& info = report_.rings[ring];
    return {info.mqd_mc, info.ring_mc, info.rptr_mc, info.wptr_mc, info.eop_mc, gfx11_ring_bytes,
            info.doorbell_index, info.high_priority ? pipe_priority_high : 0,
            info.high_priority ? queue_priority_maximum : 0, info.high_priority, false};
}

// gfx_v11_0_kcq_init_queue (first init): memset the MQD, then under
// soc21_grbm_select(me, pipe, queue, 0) amdgpu_ring_init_mqd ->
// gfx_v11_0_compute_mqd_init, reading registers in Linux order, then select 0.
bool Gfx11Queues::compute_mqd_init(uint32_t ring) noexcept {
    const Gfx11RingReport& info = report_.rings[ring];
    return gfx11_compute_mqd_init({io_, registers_.segment(0), registers_.segment(1)},
                                  {info.me, info.pipe, info.queue}, mqd_properties(ring),
                                  sdma::CommandBuffer(rings_[ring].mqd_cpu, gfx11_mqd_bytes));
}

// gfx_v11_0_kgq_init_queue (first init) -> gfx_v11_0_gfx_mqd_init under
// soc21_grbm_select(0, 0, 0, 0).
bool Gfx11Queues::gfx_mqd_init() noexcept {
    const Gfx11RingReport& info = report_.rings[0];
    return gfx11_gfx_mqd_init({io_, registers_.segment(0), registers_.segment(1)},
                              {info.me, info.pipe, info.queue}, mqd_properties(0),
                              sdma::CommandBuffer(rings_[0].mqd_cpu, gfx11_mqd_bytes));
}

// amdgpu_mes_map_legacy_queue input for ring (mqd_addr is amdgpu_bo_gpu_offset).
Mes11LegacyQueue Gfx11Queues::legacy_queue(uint32_t ring) const noexcept {
    const Gfx11RingReport& info = report_.rings[ring];
    return {info.type, info.doorbell_index, info.pipe, info.queue, info.mqd_mc, info.wptr_mc};
}

Gfx11QueuesStatus Gfx11Queues::map_queue(Mes11& mes, uint32_t ring) noexcept {
    report_.mes_status = mes.map_legacy_queue(legacy_queue(ring));

    if (report_.mes_status != Mes11Status::completed) {
        return quarantine(Gfx11QueuesStatus::mes_failure);
    }

    report_.rings[ring].mapped = true;
    ++report_.queues_mapped;
    return Gfx11QueuesStatus::running;
}

// amdgpu_ring_write: ring[wptr++ & buf_mask]; 64-bit pointers are not masked.
bool Gfx11Queues::ring_write(uint32_t ring, uint32_t value) noexcept {
    Gfx11RingReport& info = report_.rings[ring];
    Gfx11RingWriter writer{sdma::CommandBuffer(rings_[ring].cpu, gfx11_ring_bytes), info.wptr};
    const bool ok = gfx11_ring_write(writer, value);
    info.wptr = writer.wptr;
    return ok;
}

// amdgpu_ring_commit with gfx_v11_ring_insert_nop padding to align_mask + 1,
// then gfx_v11_0_ring_set_wptr_gfx/compute (doorbell path): the 64-bit wptr
// shadow in writeback, HDP flush (VRAM placement), WDOORBELL64.
Gfx11QueuesStatus Gfx11Queues::ring_commit(uint32_t ring) noexcept {
    Gfx11RingReport& info = report_.rings[ring];
    Gfx11RingWriter writer{sdma::CommandBuffer(rings_[ring].cpu, gfx11_ring_bytes), info.wptr};
    const bool ok = gfx11_ring_pad(writer);
    info.wptr = writer.wptr;

    if (!ok || !write_slot64(ring, gfx11_writeback_wptr_slot, info.wptr)) {
        return quarantine(Gfx11QueuesStatus::invalid_memory);
    }

    if (!nbio_->flush_hdp()) {
        return quarantine(Gfx11QueuesStatus::mmio_failure);
    }

    if (!doorbells_.write64(doorbells_.context, info.doorbell_index, info.wptr)) {
        return quarantine(Gfx11QueuesStatus::doorbell_failure);
    }

    return Gfx11QueuesStatus::running;
}

// gfx_v11_0_cp_gfx_start with amdgpu_async_gfx_ring (CP gfx already enabled by
// Gfx11Init::cp_start) and one gfx ring: CP_MAX_CONTEXT, CP_DEVICE_ID, then the
// clear-state preamble (same stream as gfx_v11_0_get_csb_buffer) on the gfx ring.
Gfx11QueuesStatus Gfx11Queues::cp_gfx_start() noexcept {
    if (!write(0, reg_cp_max_context, cp_max_context_value) ||
        !write(0, reg_cp_device_id, cp_device_id_value)) {
        return quarantine(Gfx11QueuesStatus::mmio_failure);
    }

    // amdgpu_ring_alloc(ring, gfx_v11_0_get_csb_size) rounds to 1024 <= max_dw.
    bool ok = ring_write(0, pm4::packet3(pm4::preamble_cntl, 0)) &&
        ring_write(0, pm4::preamble_begin_clear_state) &&
        ring_write(0, pm4::packet3(pm4::context_control, 1)) && ring_write(0, 0x80000000u) &&
        ring_write(0, 0x80000000u);

    for (const auto& extent : gfx11_clear_state::context_extents) {
        ok = ok && ring_write(0, pm4::packet3(pm4::set_context_reg, extent.count)) &&
             ring_write(0, extent.register_index - pm4::set_context_reg_start);

        for (uint32_t index = 0; ok && index < extent.count; ++index) {
            ok = ring_write(0, extent.values[index]);
        }
    }

    const uint32_t steering = registers_.segment(1) + reg_pa_sc_tile_steering_override -
                              pm4::set_context_reg_start;
    ok = ok && ring_write(0, pm4::packet3(pm4::set_context_reg, 1)) && ring_write(0, steering) &&
         ring_write(0, pa_sc_tile_steering_override_value) &&
         ring_write(0, pm4::packet3(pm4::preamble_cntl, 0)) &&
         ring_write(0, pm4::preamble_end_clear_state) &&
         ring_write(0, pm4::packet3(pm4::clear_state, 0)) && ring_write(0, 0);

    if (!ok) {
        return quarantine(Gfx11QueuesStatus::invalid_memory);
    }

    const Gfx11QueuesStatus status = ring_commit(0);
    report_.clear_state_emitted = status == Gfx11QueuesStatus::running;
    return status;
}

// gfx_v11_0_ring_test_ring for a gfx or compute ring (amdgpu_ring_test_helper).
Gfx11QueuesStatus Gfx11Queues::ring_test(uint32_t ring) noexcept {
    Gfx11RingReport& info = report_.rings[ring];
    const uint32_t scratch = registers_.segment(1) + reg_scratch_reg0;
    info.ring_test = Gfx11RingTest::failed;

    if (!write(1, reg_scratch_reg0, gfx11_ring_test_seed)) {
        report_.failed_ring = ring;
        return quarantine(Gfx11QueuesStatus::mmio_failure);
    }

    Gfx11RingWriter writer{sdma::CommandBuffer(rings_[ring].cpu, gfx11_ring_bytes), info.wptr};
    const bool emitted = gfx11_emit_ring_test(writer, scratch);
    info.wptr = writer.wptr;

    if (!emitted) {
        report_.failed_ring = ring;
        return quarantine(Gfx11QueuesStatus::invalid_memory);
    }

    const Gfx11QueuesStatus commit = ring_commit(ring);

    if (commit != Gfx11QueuesStatus::running) {
        report_.failed_ring = ring;
        return commit;
    }

    for (uint32_t poll = 0; poll < policy_.poll_iterations; ++poll) {
        uint32_t value = 0;

        if (!read_ip_register(io_, registers_.segment(1), reg_scratch_reg0, value)) {
            report_.failed_ring = ring;
            return quarantine(Gfx11QueuesStatus::mmio_failure);
        }

        if (value == gfx11_ring_test_value) {
            info.ring_test = Gfx11RingTest::passed;
            return Gfx11QueuesStatus::running;
        }

        io_.delay_microseconds(io_.context, 1);
    }

    // The CP may still fetch this ring and write SCRATCH_REG0 later.
    info.ring_test = Gfx11RingTest::timeout;
    report_.failed_ring = ring;
    return quarantine(Gfx11QueuesStatus::ring_test_failed);
}

Gfx11QueuesStatus Gfx11Queues::resume(memory::Manager& manager, const Gfx11QueueMemory& memory,
                                      Mes11& mes) noexcept {
    if (phase_ != Phase::ready) {
        return phase_ == Phase::faulted ? Gfx11QueuesStatus::quarantined :
               phase_ == Phase::uninitialized ? Gfx11QueuesStatus::unavailable :
                                                Gfx11QueuesStatus::busy;
    }

    const Mes11Report mes_report = mes.report();

    // adev->mes.enable_legacy_queue_map: without it Linux maps through the KIQ.
    if (!mes_report.scheduler_ready) {
        return finish(Gfx11QueuesStatus::unavailable);
    }

    if (!mes_report.legacy_queue_map) {
        return finish(Gfx11QueuesStatus::unsupported_firmware);
    }

    if (!retain_memory(manager, memory)) {
        return finish(Gfx11QueuesStatus::invalid_memory);
    }

    // CPU stores only so far: nothing references these buffers yet.
    if (!prepare_memory()) {
        release_leases();
        return finish(Gfx11QueuesStatus::invalid_memory);
    }

    const uint32_t last = config_.compute_rings;

    // gfx_v11_0_kcq_resume: every compute MQD, then amdgpu_gfx_enable_kcq ->
    // amdgpu_gfx_mes_enable_kcq. From the first GRBM select on, a failure
    // quarantines; a best-effort broadcast select is restored first.
    for (uint32_t ring = 1; ring <= last; ++ring) {
        if (!compute_mqd_init(ring)) {
            grbm_select(0, 0, 0);
            return quarantine(Gfx11QueuesStatus::mmio_failure);
        }

        backup_mqd(ring);
    }

    // amdgpu_device_flush_hdp and gfx11_kiq_set_resources(~0) plus the KIQ ring
    // test happen inside Mes11. Linux only logs a failure and maps anyway; a
    // failed Mes11 is quarantined, so the first map below then fails closed.
    report_.set_resources_mask = mes_kcq_queue_mask;
    report_.set_resources_status = mes.kiq_set_resources(mes_kcq_queue_mask);
    report_.mes_status = report_.set_resources_status;

    for (uint32_t ring = 1; ring <= last; ++ring) {
        const Gfx11QueuesStatus status = map_queue(mes, ring);

        if (status != Gfx11QueuesStatus::running) {
            return status;
        }
    }

    // gfx_v11_0_cp_async_gfx_ring_resume: KGQ MQD, amdgpu_gfx_enable_kgq
    // (amdgpu_device_flush_hdp, then MES map), then gfx_v11_0_cp_gfx_start.
    if (!gfx_mqd_init()) {
        grbm_select(0, 0, 0);
        return quarantine(Gfx11QueuesStatus::mmio_failure);
    }

    backup_mqd(0);

    if (!nbio_->flush_hdp()) {
        return quarantine(Gfx11QueuesStatus::mmio_failure);
    }

    Gfx11QueuesStatus status = map_queue(mes, 0);

    if (status == Gfx11QueuesStatus::running) {
        status = cp_gfx_start();
    }

    // gfx_v11_0_cp_resume: ring tests of every gfx ring, then every compute
    // ring, returning at the first failure.
    for (uint32_t ring = 0; status == Gfx11QueuesStatus::running && ring <= last; ++ring) {
        status = ring_test(ring);
    }

    if (status != Gfx11QueuesStatus::running) {
        return status;
    }

    phase_ = Phase::running;
    return finish(Gfx11QueuesStatus::running);
}

// amdgpu_ib_schedule(ring, 1, &ib, NULL, &f) for the gfx_v11_0_ring_test_ib IB
// (no job: vmid 0, fence_ctx 0, no VM flush, no CNTX_CNTL, no FRAME_CNTL; GC
// 11.0.0 bare metal has cp_gfx_shadow false, so emit_gfx_shadow emits nothing;
// gfx11 has no wave limit, insert_start/end or sw-ring mux; soc21 HDP6 has no
// invalidate_hdp). The ring stream is, in order:
//   gfx_v11_0_emit_mem_sync (ACQUIRE_MEM, 8 dwords; ib->flags EMIT_MEM_SYNC),
//   gfx only: amdgpu_ring_init_cond_exec (COND_EXEC on cond_exe_gpu_addr, 5),
//   amdgpu_device_flush_hdp -> gfx_v11_0_ring_emit_hdp_flush (WAIT_REG_MEM, 7),
//   gfx_v11_0_ring_emit_ib_gfx/compute (INDIRECT_BUFFER, 4),
//   gfx only: the second amdgpu_ring_init_cond_exec after emit_gfx_shadow, whose
//     returned offset Linux discards (its count stays 0),
//   amdgpu_fence_emit -> gfx_v11_0_ring_emit_fence (RELEASE_MEM, 8),
//   gfx only: amdgpu_ring_patch_cond_exec of the first COND_EXEC,
// and amdgpu_ring_commit pads with NOPs to align_mask 0xff.
bool Gfx11Queues::emit_ib_frame(uint32_t ring) noexcept {
    Gfx11RingReport& info = report_.rings[ring];
    Gfx11RingWriter writer{sdma::CommandBuffer(rings_[ring].cpu, gfx11_ring_bytes), info.wptr};
    // amdgpu_fence_emit: seq = ++ring->fence_drv.sync_seq (32-bit, wraps).
    ++info.fence_seq;
    const bool ok = gfx11_emit_ib_frame(writer, {ring == 0, true, false, 0, info.pipe,
        slot_mc(ring, gfx11_writeback_cond_exec_slot), hdp_flush_req_, hdp_flush_done_,
        info.ib_mc, gfx11_ib_test_dwords, info.fence_mc, info.fence_seq});
    info.wptr = writer.wptr;
    return ok;
}

// gfx_v11_0_ring_test_ib for one gfx or compute ring with the fence polled.
Gfx11QueuesStatus Gfx11Queues::ib_test(uint32_t ring, uint32_t timeout) noexcept {
    Gfx11RingReport& info = report_.rings[ring];
    const uint32_t test = (gfx11_writeback_ib_test_slot_base + ring) * gfx11_writeback_slot_bytes;
    const uint32_t fence = (ring * gfx11_writeback_slots_per_ring + gfx11_writeback_fence_slot) *
                           gfx11_writeback_slot_bytes;
    const sdma::CommandBuffer slots(writeback_cpu_, gfx11_writeback_bytes);
    const sdma::CommandBuffer ib = sdma::CommandBuffer(ib_cpu_, gfx11_ib_test_buffer_bytes)
                                       .subbuffer(ring * gfx11_ib_test_slot_bytes,
                                                  gfx11_ib_test_slot_bytes);
    info.ib_mc = report_.ib_buffer_mc + ring * gfx11_ib_test_slot_bytes;
    info.ib_test_mc = report_.writeback_mc + test;
    info.ib_test = Gfx11RingTest::failed;
    info.ib_polls = 0;

    // adev->wb.wb[index] = 0xCAFEDEAD; ib.ptr[0..4]: WRITE_DATA to memory with
    // WR_CONFIRM; ib.length_dw 5.
    if (!slots.write_u32(test, gfx11_ring_test_seed) ||
        !gfx11_write_test_ib(ib, info.ib_test_mc) || !emit_ib_frame(ring)) {
        report_.ib_failed_ring = ring;
        return quarantine(Gfx11QueuesStatus::invalid_memory);
    }

    const Gfx11QueuesStatus commit = ring_commit(ring);

    if (commit != Gfx11QueuesStatus::running) {
        report_.ib_failed_ring = ring;
        return commit;
    }

    const ByteView result(writeback_cpu_, gfx11_writeback_bytes);
    bool signalled = false;

    for (; info.ib_polls < timeout; ++info.ib_polls) {
        if (!result.read_u32(fence, info.fence_value)) {
            report_.ib_failed_ring = ring;
            return quarantine(Gfx11QueuesStatus::invalid_memory);
        }

        // amdgpu_fence_process: signalled once the written sequence reaches
        // seq in 32-bit wrap-around order.
        if (info.fence_value - info.fence_seq < 0x80000000u) {
            signalled = true;
            break;
        }

        io_.delay_microseconds(io_.context, 1);
    }

    uint32_t value = 0;

    if (signalled && !result.read_u32(test, value)) {
        report_.ib_failed_ring = ring;
        return quarantine(Gfx11QueuesStatus::invalid_memory);
    }

    if (signalled && value == gfx11_ring_test_value) {
        info.ib_test = Gfx11RingTest::passed;
        ++report_.ib_rings_passed;
        return Gfx11QueuesStatus::running;
    }

    // Linux sets ring->sched.ready = false. A timed-out frame may still run
    // later and a wrong value means an unexpected write: both quarantine this
    // ring and its memory.
    info.ib_test = signalled ? Gfx11RingTest::failed : Gfx11RingTest::timeout;
    info.ib_disabled = true;

    if (report_.ib_failed_ring == UINT32_MAX) {
        report_.ib_failed_ring = ring;
    }

    return Gfx11QueuesStatus::ib_test_failed;
}

Gfx11QueuesStatus Gfx11Queues::test_ib(memory::Manager& manager, memory::Handle ib_buffer,
                                       const Gfx11IbTestConfig& config) noexcept {
    if (phase_ != Phase::running) {
        return phase_ == Phase::faulted ? Gfx11QueuesStatus::quarantined :
               phase_ == Phase::uninitialized ? Gfx11QueuesStatus::unavailable :
                                                Gfx11QueuesStatus::busy;
    }

    // An IB buffer kept from a failed test is still quarantined.
    if (report_.ib_buffer_retained) {
        return Gfx11QueuesStatus::quarantined;
    }

    // FenceManager owns the rings' sync_seq once submission is enabled.
    if (report_.submission_enabled) {
        return Gfx11QueuesStatus::busy;
    }

    const uint32_t segment = config.nbio.valid() ? config.nbio.nbif_base(gfx11_hdp_flush_nbif_segment) : 0;
    uint32_t offset = 0;

    if (!config.nbio.valid() || config.timeout_microseconds == 0 ||
        config.timeout_microseconds > gfx11_ib_test_xgmi_timeout_us ||
        !discovery_register_offset(segment, gfx11_hdp_flush_done_register,
                                   config.nbio.aperture_bytes(), offset)) {
        return finish(Gfx11QueuesStatus::invalid_argument);
    }

    if (!retain_ib(manager, ib_buffer)) {
        return finish(Gfx11QueuesStatus::invalid_memory);
    }

    hdp_flush_req_ = segment + gfx11_hdp_flush_req_register;
    hdp_flush_done_ = segment + gfx11_hdp_flush_done_register;
    report_.ib_failed_ring = UINT32_MAX;
    report_.ib_rings_passed = 0;
    report_.ib_primary_failed = false;
    bool failed = false;

    // adev->rings order: gfx_0.0.0 then comp_1.p.q; sched.ready (ring test
    // passed) and not disabled by an earlier IB failure.
    for (uint32_t ring = 0; ring <= config_.compute_rings; ++ring) {
        Gfx11RingReport& info = report_.rings[ring];

        if (info.ib_disabled || info.ring_test != Gfx11RingTest::passed || !info.mapped) {
            failed = failed || info.ib_disabled;
            continue;
        }

        const Gfx11QueuesStatus status = ib_test(ring, config.timeout_microseconds);

        if (phase_ == Phase::faulted) {
            return status;
        }

        if (status == Gfx11QueuesStatus::ib_test_failed) {
            failed = true;

            // "oh, oh, that's really bad": accel_working = false, return.
            if (ring == 0) {
                report_.ib_primary_failed = true;
                break;
            }
        }
    }

    if (failed) {
        return finish(Gfx11QueuesStatus::ib_test_failed);
    }

    // Every fence signalled after its IB: the CP no longer reads the buffer.
    release_ib();
    return finish(Gfx11QueuesStatus::running);
}

// memcpy_fromio(mqd_backup[mqd_idx], mqd, sizeof(*mqd)) right after mqd_init.
void Gfx11Queues::backup_mqd(uint32_t ring) noexcept {
    const ByteView mqd(rings_[ring].mqd_cpu, gfx11_mqd_bytes);
    bool copied = true;

    for (uint32_t offset = 0; copied && offset < gfx11_mqd_bytes; ++offset) {
        copied = mqd.read_u8(offset, mqd_backup_[ring][offset]);
    }

    mqd_backed_up_[ring] = copied;
}

// memcpy_toio(mqd, mqd_backup[mqd_idx], sizeof(*mqd)).
bool Gfx11Queues::restore_mqd(uint32_t ring) noexcept {
    if (!mqd_backed_up_[ring]) {
        return false;
    }

    const sdma::CommandBuffer mqd(rings_[ring].mqd_cpu, gfx11_mqd_bytes);

    for (uint32_t offset = 0; offset < gfx11_mqd_bytes; offset += 4) {
        uint32_t value = 0;

        if (!ByteView(mqd_backup_[ring], gfx11_mqd_bytes).read_u32(offset, value) ||
            !mqd.write_u32(offset, value)) {
            return false;
        }
    }

    return true;
}

// ring->wptr = 0, *ring->wptr_cpu_addr = 0, amdgpu_ring_clear_ring (ring NOPs).
bool Gfx11Queues::clear_ring(uint32_t ring) noexcept {
    report_.rings[ring].wptr = 0;

    if (!write_slot64(ring, gfx11_writeback_wptr_slot, 0)) {
        return false;
    }

    const sdma::CommandBuffer bytes(rings_[ring].cpu, gfx11_ring_bytes);

    for (uint32_t offset = 0; offset < gfx11_ring_bytes; offset += 4) {
        if (!bytes.write_u32(offset, gfx11_ring_nop)) {
            return false;
        }
    }

    return true;
}

Gfx11QueuesStatus Gfx11Queues::resume_from_suspend(Mes11& mes) noexcept {
    if (phase_ != Phase::stopped) {
        return phase_ == Phase::faulted ? Gfx11QueuesStatus::quarantined :
               phase_ == Phase::uninitialized ? Gfx11QueuesStatus::unavailable :
                                                Gfx11QueuesStatus::busy;
    }

    if (report_.submission_enabled) {
        return Gfx11QueuesStatus::busy;
    }

    const uint32_t last = config_.compute_rings;

    if (report_.ib_buffer_retained) {
        return Gfx11QueuesStatus::quarantined;
    }

    for (uint32_t ring = 0; ring <= last; ++ring) {
        if (report_.rings[ring].ib_disabled) {
            return Gfx11QueuesStatus::quarantined;
        }

        if (!mqd_backed_up_[ring] || report_.rings[ring].mapped) {
            return finish(Gfx11QueuesStatus::unavailable);
        }
    }

    const Mes11Report mes_report = mes.report();

    if (!mes_report.scheduler_ready) {
        return finish(Gfx11QueuesStatus::unavailable);
    }

    if (!mes_report.legacy_queue_map) {
        return finish(Gfx11QueuesStatus::unsupported_firmware);
    }

    // gfx_v11_0_kcq_init_queue, in_suspend branch: CPU stores into retained memory.
    for (uint32_t ring = 1; ring <= last; ++ring) {
        if (!restore_mqd(ring) || !clear_ring(ring)) {
            return quarantine(Gfx11QueuesStatus::invalid_memory);
        }
    }

    report_.set_resources_mask = mes_kcq_queue_mask;
    report_.set_resources_status = mes.kiq_set_resources(mes_kcq_queue_mask);
    report_.mes_status = report_.set_resources_status;

    for (uint32_t ring = 1; ring <= last; ++ring) {
        const Gfx11QueuesStatus status = map_queue(mes, ring);

        if (status != Gfx11QueuesStatus::running) {
            return status;
        }
    }

    // gfx_v11_0_kgq_init_queue, in_suspend branch, then amdgpu_gfx_enable_kgq.
    if (!restore_mqd(0) || !clear_ring(0)) {
        return quarantine(Gfx11QueuesStatus::invalid_memory);
    }

    if (!nbio_->flush_hdp()) {
        return quarantine(Gfx11QueuesStatus::mmio_failure);
    }

    Gfx11QueuesStatus status = map_queue(mes, 0);

    if (status == Gfx11QueuesStatus::running) {
        status = cp_gfx_start();
    }

    for (uint32_t ring = 0; status == Gfx11QueuesStatus::running && ring <= last; ++ring) {
        status = ring_test(ring);
    }

    if (status != Gfx11QueuesStatus::running) {
        return status;
    }

    phase_ = Phase::running;
    return finish(Gfx11QueuesStatus::running);
}

Gfx11QueuesStatus Gfx11Queues::fini(Mes11& mes) noexcept {
    if (phase_ != Phase::running && phase_ != Phase::faulted) {
        return phase_ == Phase::stopped ? finish(Gfx11QueuesStatus::stopped) :
                                          Gfx11QueuesStatus::unavailable;
    }

    const bool faulted = phase_ == Phase::faulted;
    bool all = true;

    // amdgpu_gfx_disable_kgq (PREEMPT_QUEUES) then amdgpu_gfx_disable_kcq
    // (RESET_QUEUES): neither is PREEMPT_QUEUES_NO_UNMAP, so both send
    // REMOVE_QUEUE with unmap_legacy_queue and the queue type. Linux continues
    // after a failure; so does this loop, and it then quarantines.
    for (uint32_t ring = 0; ring <= config_.compute_rings; ++ring) {
        if (!report_.rings[ring].mapped) {
            continue;
        }

        report_.mes_status = mes.unmap_legacy_queue({legacy_queue(ring), Mes11UnmapAction::unmap,
                                                     0, 0});

        if (report_.mes_status == Mes11Status::completed) {
            report_.rings[ring].mapped = false;
            --report_.queues_mapped;
        } else {
            all = false;
        }
    }

    if (faulted) {
        return quarantine(Gfx11QueuesStatus::quarantined);
    }

    if (!all) {
        return quarantine(Gfx11QueuesStatus::mes_failure);
    }

    phase_ = Phase::stopped;
    return finish(Gfx11QueuesStatus::stopped);
}

Gfx11QueuesStatus Gfx11Queues::release() noexcept {
    if (phase_ != Phase::stopped) {
        return phase_ == Phase::faulted ? Gfx11QueuesStatus::quarantined : Gfx11QueuesStatus::busy;
    }

    if (report_.submission_enabled) {
        return Gfx11QueuesStatus::busy;
    }

    for (uint32_t ring = 0; ring <= config_.compute_rings; ++ring) {
        if (report_.rings[ring].ib_disabled) {
            return Gfx11QueuesStatus::quarantined;
        }
    }

    if (report_.ib_buffer_retained) {
        return Gfx11QueuesStatus::quarantined;
    }

    release_leases();
    phase_ = Phase::ready;
    return finish(Gfx11QueuesStatus::released);
}

Gfx11QueuesStatus Gfx11Queues::release_after_reset() noexcept {
    if (!report_.memory_retained && phase_ != Phase::faulted) {
        return Gfx11QueuesStatus::busy;
    }

    if (recovery_.confirm_reset_quiescence == nullptr ||
        !recovery_.confirm_reset_quiescence(recovery_.context)) {
        return Gfx11QueuesStatus::quarantined;
    }

    // The reset proof covers every outstanding fence and IB: detach the
    // FenceManager rings (whose writeback views end with these leases) first.
    for (uint32_t ring = 0; ring < gfx11_max_rings; ++ring) {
        if (report_.rings[ring].submit_enabled && fences_ != nullptr) {
            fences_->detach_ring(ring, true);
        }

        reclaim(ring, true);
        report_.rings[ring].submit_enabled = false;
        report_.rings[ring].submissions = 0;
        report_.rings[ring].consumed_position = 0;
        report_.rings[ring].free_dwords = 0;
    }

    fences_ = nullptr;
    wptr_store_ = {};
    report_.submission_enabled = false;
    vm_report_.enabled = false;
    reset_vmids();
    release_leases();

    for (uint32_t ring = 0; ring < gfx11_max_rings; ++ring) {
        report_.rings[ring].ring_test = Gfx11RingTest::not_run;
        report_.rings[ring].ib_test = Gfx11RingTest::not_run;
        report_.rings[ring].ib_polls = 0;
    }

    report_.clear_state_emitted = false;
    report_.failed_ring = UINT32_MAX;
    report_.ib_failed_ring = UINT32_MAX;
    report_.ib_rings_passed = 0;
    report_.ib_primary_failed = false;
    phase_ = Phase::ready;
    return finish(Gfx11QueuesStatus::released);
}

// amdgpu_fence_driver_start_ring for one ring that passed its ring and IB
// tests: the fence slot must hold the IB test's sequence (signalled).
bool Gfx11Queues::attach_fence_ring(uint32_t ring) noexcept {
    const Gfx11RingReport& info = report_.rings[ring];
    const uint32_t fence = (ring * gfx11_writeback_slots_per_ring + gfx11_writeback_fence_slot) *
                           gfx11_writeback_slot_bytes;
    ByteView slot(nullptr, 0);

    if (!ByteView(writeback_cpu_, gfx11_writeback_bytes).subview(fence, sizeof(uint32_t), slot)) {
        return false;
    }

    const FenceRingConfig config{ring == 0 ? FenceEngine::gfx : FenceEngine::compute, info.me,
                                 info.pipe, info.queue, 0, slot, info.fence_seq, info.wptr, 0};
    return fences_->attach_ring(ring, config) == FenceStatus::attached;
}

Gfx11QueuesStatus Gfx11Queues::enable_submission(FenceManager& fences,
                                                 const Gfx11SubmitConfig& config) noexcept {
    if (phase_ != Phase::running) {
        return phase_ == Phase::faulted ? Gfx11QueuesStatus::quarantined :
                                          Gfx11QueuesStatus::unavailable;
    }

    if (report_.submission_enabled || report_.ib_buffer_retained) {
        return Gfx11QueuesStatus::busy;
    }

    const uint32_t segment = config.nbio.valid() ?
        config.nbio.nbif_base(gfx11_hdp_flush_nbif_segment) : 0;
    uint32_t offset = 0;

    if (!config.nbio.valid() || config.wptr_store.store == nullptr ||
        !discovery_register_offset(segment, gfx11_hdp_flush_done_register,
                                   config.nbio.aperture_bytes(), offset)) {
        return Gfx11QueuesStatus::invalid_argument;
    }

    fences_ = &fences;
    wptr_store_ = config.wptr_store;
    hdp_flush_req_ = segment + gfx11_hdp_flush_req_register;
    hdp_flush_done_ = segment + gfx11_hdp_flush_done_register;
    // Process-VM jobs need both register tokens (VM flush and PASID LUT).
    vm_gmc_ = config.gmc;
    vm_osssys_ = config.osssys;
    reset_vmids();
    uint32_t attached = 0;

    // Linux sched.ready: the ring test and the IB test passed.
    for (uint32_t ring = 0; ring <= config_.compute_rings; ++ring) {
        Gfx11RingReport& info = report_.rings[ring];

        if (!info.mapped || info.ring_test != Gfx11RingTest::passed ||
            info.ib_test != Gfx11RingTest::passed || info.ib_disabled) {
            continue;
        }

        if (!attach_fence_ring(ring)) {
            for (uint32_t undo = 0; undo < ring; ++undo) {
                if (report_.rings[undo].submit_enabled) {
                    fences.detach_ring(undo, false);
                    report_.rings[undo].submit_enabled = false;
                }
            }

            fences_ = nullptr;
            wptr_store_ = {};
            return Gfx11QueuesStatus::invalid_memory;
        }

        info.submit_enabled = true;
        info.consumed_position = info.wptr;
        ++attached;
    }

    if (attached == 0) {
        fences_ = nullptr;
        wptr_store_ = {};
        return Gfx11QueuesStatus::unavailable;
    }

    report_.submission_enabled = true;
    vm_report_.enabled = vm_gmc_.valid() && vm_osssys_.valid() && report_.rings[0].submit_enabled;
    return Gfx11QueuesStatus::running;
}

// amdgpu_vmid_reset_all / a fresh id manager: no VMID has an owner, so every
// next use maps its PASID again (amdgpu_vmid_had_gpu_reset after a reset).
void Gfx11Queues::reset_vmids() noexcept {
    for (auto& id : vmids_) {
        id = {};
    }

    for (auto& pasid : vm_report_.vmid_pasid) {
        pasid = 0;
    }

    vmid_clock_ = 0;
    ring_context_pasid_ = 0;
}

// Returns the IB leases of signalled fences (all: every lease, after a reset
// proof). A lease of a hung ring stays until then.
void Gfx11Queues::reclaim(uint32_t ring, bool all) noexcept {
    for (auto& lease : leases_[ring]) {
        if (!lease.used) {
            continue;
        }

        if (!all && (fences_ == nullptr ||
                     fences_->query(ring, lease.sequence) != FenceStatus::signalled)) {
            continue;
        }

        if (manager_ != nullptr) {
            manager_->unretain(lease.ib);
        }

        lease = {};
        --report_.rings[ring].ib_leases;
    }
}

// The IB GPU address: a live allocation of the resume Manager readable for the
// whole IB, 4-byte aligned, below the 48-bit limit and outside every buffer
// this module owns.
bool Gfx11Queues::submit_ib_address(const Gfx11Submission& submission, uint64_t& address) noexcept {
    sdma::GpuRange range{};
    const uint64_t bytes = static_cast<uint64_t>(submission.ib_dwords) * 4u;

    if (manager_ == nullptr ||
        manager_->resolve({submission.ib, submission.ib_offset}, bytes, memory::Access::read,
                          range) != memory::Status::success) {
        return false;
    }

    address = range.allocation.virtual_address + range.byte_offset;

    if ((address & 3u) != 0 || address > sdma::gpu_virtual_address_max - bytes + 1 ||
        overlaps(address, bytes, report_.writeback_mc, gfx11_writeback_bytes) ||
        overlaps(address, bytes, report_.compute_eop_mc,
                 static_cast<uint64_t>(config_.compute_rings) * gfx11_mec_hpd_bytes)) {
        return false;
    }

    for (uint32_t ring = 0; ring <= config_.compute_rings; ++ring) {
        if (overlaps(address, bytes, report_.rings[ring].ring_mc, gfx11_ring_bytes) ||
            overlaps(address, bytes, report_.rings[ring].mqd_mc, gfx11_mqd_bytes)) {
            return false;
        }
    }

    return true;
}

// A process-VM job: gfx ring with VM jobs enabled, a 16-bit non-zero PASID, a
// dword-aligned IB inside the 48-bit VA space and a page directory address the
// GMC flush encoder accepts (checked again by gmc11_emit_flush_gpu_tlb).
bool Gfx11Queues::vm_job_valid(const Gfx11Submission& submission) const noexcept {
    const Gfx11VmJob& job = *submission.vm;
    const uint64_t bytes = static_cast<uint64_t>(submission.ib_dwords) * 4u;
    return submission.ring == 0 && job.pasid != 0 && job.pasid < job_pasid_limit && job.pd_address != 0 &&
           (job.ib_va & 3u) == 0 && job.ib_va < job_va_limit && bytes <= job_va_limit - job.ib_va;
}

// amdgpu_vmid_grab without reserved VMIDs: no idle VMID means wait (refused
// here, nothing written); a VMID this VM already owns is reused (concurrent
// flushes are allowed on GC 11), else the least recently used idle one is
// taken. A VMID is idle when the gfx-ring fence of its last job signalled (a
// value more than half the sequence space old, which the FenceManager no
// longer validates, signalled long ago: at most fence_slots are outstanding).
bool Gfx11Queues::grab_vmid(const Gfx11VmJob& job, uint32_t& index, bool& pasid_mapping) noexcept {
    uint32_t idle = gfx11_job_vmids;
    uint32_t used = gfx11_job_vmids;

    for (uint32_t candidate = 0; candidate < gfx11_job_vmids; ++candidate) {
        const JobVmid& id = vmids_[candidate];
        const FenceStatus last = id.last_sequence == 0 ? FenceStatus::signalled :
                                 fences_->query(0, id.last_sequence);
        const bool signalled = last == FenceStatus::signalled || last == FenceStatus::invalid_argument;

        if (signalled && (idle == gfx11_job_vmids || id.lru < vmids_[idle].lru)) {
            idle = candidate;
        }

        if (id.pasid == job.pasid && id.pd_address == job.pd_address && id.last_sequence != 0 &&
            (used == gfx11_job_vmids || id.lru > vmids_[used].lru)) {
            used = candidate;
        }
    }

    if (idle == gfx11_job_vmids) {
        ++vm_report_.vmid_busy;
        return false;
    }

    index = used != gfx11_job_vmids ? used : idle;
    pasid_mapping = vmids_[index].last_sequence == 0 || vmids_[index].pasid != job.pasid;
    return true;
}

// Ring space: the engine consumed the ring at least up to the end of the newest
// signalled frame (retired); the 32-bit RPTR writeback (gfx_v11_0_ring_get_rptr)
// may show more. Only an RPTR inside the unretired span counts: CP reports it
// in RPTR_BLOCK_SIZE steps, so a stale value lags and is ignored.
uint64_t Gfx11Queues::consumed_position(uint32_t ring, uint64_t retired) const noexcept {
    const uint64_t wptr = report_.rings[ring].wptr;
    const uint32_t mask = gfx11_ring_bytes / 4 - 1;
    uint32_t rptr = 0;

    if (retired > wptr || wptr - retired > mask + 1u ||
        !ByteView(writeback_cpu_, gfx11_writeback_bytes)
             .read_u32((ring * gfx11_writeback_slots_per_ring + gfx11_writeback_rptr_slot) *
                       gfx11_writeback_slot_bytes, rptr)) {
        return retired;
    }

    const uint64_t behind = (static_cast<uint32_t>(wptr) - rptr) & mask;
    return behind < wptr - retired ? wptr - behind : retired;
}

Gfx11QueuesStatus Gfx11Queues::submit(const Gfx11Submission& submission,
                                      uint32_t& sequence) noexcept {
    sequence = 0;

    if (phase_ != Phase::running) {
        return phase_ == Phase::faulted ? Gfx11QueuesStatus::quarantined :
               phase_ == Phase::stopped ? Gfx11QueuesStatus::busy : Gfx11QueuesStatus::unavailable;
    }

    if (!report_.submission_enabled) {
        return Gfx11QueuesStatus::unavailable;
    }

    const uint32_t ring = submission.ring;

    if (ring > config_.compute_rings || !report_.rings[ring].submit_enabled ||
        submission.ib_dwords == 0 || submission.ib_dwords > gfx11_max_ib_dwords) {
        return Gfx11QueuesStatus::invalid_argument;
    }

    const bool vm_job = submission.vm != nullptr;

    if (vm_job && (!vm_report_.enabled || !vm_job_valid(submission))) {
        return vm_report_.enabled ? Gfx11QueuesStatus::invalid_argument :
                                         Gfx11QueuesStatus::unavailable;
    }

    reclaim(ring, false);
    uint64_t ib = 0;

    if (!submit_ib_address(submission, ib)) {
        return Gfx11QueuesStatus::invalid_memory;
    }

    // The VM part is fully prepared before any fence or ring change.
    Gfx11VmFrame vm{};
    uint32_t vmid_index = 0;

    if (vm_job) {
        const Gfx11VmJob& job = *submission.vm;
        bool pasid_mapping = false;

        if (!grab_vmid(job, vmid_index, pasid_mapping)) {
            return Gfx11QueuesStatus::ring_full;
        }

        vm.vmid = gfx11_first_job_vmid + vmid_index;

        if (!gmc11_emit_flush_gpu_tlb(vm_gmc_, VmHub::gfxhub, vm.vmid, job.pd_address,
                                      gfx11_gfx_vm_invalidate_engine, vm.flush) ||
            !gpuvm_pasid_mapping_register(vm_osssys_, VmHub::gfxhub, vm.vmid, vm.pasid_register)) {
            return Gfx11QueuesStatus::invalid_argument;
        }

        vm.pipeline_sync = true;
        vm.sync_address = report_.rings[ring].fence_mc;
        vm.sync_sequence = report_.rings[ring].fence_seq;
        vm.pasid_mapping = pasid_mapping;
        vm.pasid = job.pasid;
        vm.context_switch = ring_context_pasid_ != job.pasid;
        // The IB is fetched through the job VMID at its process address.
        ib = job.ib_va;
    }

    FenceReservation reservation{};
    const FenceStatus reserved = fences_->begin_submission(ring, reservation);

    if (reserved == FenceStatus::busy) {
        return Gfx11QueuesStatus::ring_full;
    }

    if (reserved != FenceStatus::ready) {
        return reserved == FenceStatus::ring_hung ? Gfx11QueuesStatus::ring_hung :
                                                    Gfx11QueuesStatus::unavailable;
    }

    Gfx11RingReport& info = report_.rings[ring];
    const uint64_t consumed = consumed_position(ring, reservation.retired_position);
    const uint64_t used = info.wptr - consumed;
    const uint32_t ring_dwords = gfx11_ring_bytes / 4;
    info.consumed_position = consumed;
    info.free_dwords = used < ring_dwords ? static_cast<uint32_t>(ring_dwords - used) : 0;

    // amdgpu_ring_alloc's count_dw must fit without the WPTR reaching RPTR.
    if (used + gfx11_submit_alloc_dwords >= ring_dwords) {
        return Gfx11QueuesStatus::ring_full;
    }

    IbLease* lease = nullptr;

    for (auto& candidate : leases_[ring]) {
        if (!candidate.used) {
            lease = &candidate;
            break;
        }
    }

    if (lease == nullptr) {
        return Gfx11QueuesStatus::ring_full;
    }

    if (manager_->retain(submission.ib, memory::Access::read) != memory::Status::success) {
        return Gfx11QueuesStatus::invalid_memory;
    }

    // amdgpu_ib_schedule: amdgpu_ib_get set AMDGPU_IB_FLAG_EMIT_MEM_SYNC.
    Gfx11RingWriter writer{sdma::CommandBuffer(rings_[ring].cpu, gfx11_ring_bytes), info.wptr};
    const bool emitted = gfx11_emit_ib_frame(writer, {ring == 0, true, false, 0, info.pipe,
        slot_mc(ring, gfx11_writeback_cond_exec_slot), hdp_flush_req_, hdp_flush_done_, ib,
        submission.ib_dwords, info.fence_mc, reservation.sequence, vm_job ? &vm : nullptr}) &&
        gfx11_ring_pad(writer);

    if (!emitted || writer.wptr - info.wptr > gfx11_submit_alloc_dwords) {
        manager_->unretain(submission.ib);
        fences_->mark_hung(ring, FenceHangReason::submit_failure);
        return quarantine(Gfx11QueuesStatus::invalid_memory);
    }

    // The fence becomes known before the CP can write it.
    const FenceStatus published = fences_->publish_submission(ring, reservation.sequence, writer.wptr);

    if (published != FenceStatus::pending) {
        // amdgpu_ring_undo: nothing reached the CP; WPTR stays.
        manager_->unretain(submission.ib);
        return published == FenceStatus::ring_hung ? Gfx11QueuesStatus::ring_hung :
                                                     quarantine(Gfx11QueuesStatus::invalid_argument);
    }

    *lease = {true, reservation.sequence, submission.ib};
    ++info.ib_leases;
    info.wptr = writer.wptr;
    info.fence_seq = reservation.sequence;
    ++info.submissions;
    sequence = reservation.sequence;

    // amdgpu_vmid_grab bookkeeping (id->owner, pd_gpu_addr, active, LRU tail) and
    // ring->current_ctx. The frame is published from here on; a later doorbell
    // failure leaves the VMID owned by this job's (never signalling) fence.
    if (vm_job) {
        JobVmid& id = vmids_[vmid_index];
        id = {submission.vm->pasid, submission.vm->pd_address, reservation.sequence, ++vmid_clock_};
        vm_report_.vmid_pasid[vmid_index] = submission.vm->pasid;
        ++vm_report_.submissions;
        vm_report_.pasid_mappings += vm.pasid_mapping ? 1 : 0;
    }

    if (ring == 0) {
        ring_context_pasid_ = vm_job ? submission.vm->pasid : 0;
    }
    const uint32_t shadow = (ring * gfx11_writeback_slots_per_ring + gfx11_writeback_wptr_slot) *
                            gfx11_writeback_slot_bytes;

    // amdgpu_ring_commit -> gfx_v11_0_ring_set_wptr_gfx/compute (doorbell path):
    // atomic64_set of the shadow, then WDOORBELL64; HDP flush for VRAM backing.
    // The slot offset is inside the retained writeback page (checked by
    // retain_memory), 32-byte aligned within a 4 KiB aligned allocation.
    if (!wptr_store_.store(wptr_store_.context, writeback_cpu_ + shadow, info.wptr)) {
        fences_->mark_hung(ring, FenceHangReason::submit_failure);
        return quarantine(Gfx11QueuesStatus::invalid_memory);
    }

    if (!nbio_->flush_hdp()) {
        fences_->mark_hung(ring, FenceHangReason::submit_failure);
        return quarantine(Gfx11QueuesStatus::mmio_failure);
    }

    if (!doorbells_.write64(doorbells_.context, info.doorbell_index, info.wptr)) {
        fences_->mark_hung(ring, FenceHangReason::submit_failure);
        return quarantine(Gfx11QueuesStatus::doorbell_failure);
    }

    return Gfx11QueuesStatus::running;
}

Gfx11QueuesStatus Gfx11Queues::retire() noexcept {
    if (!report_.submission_enabled) {
        return Gfx11QueuesStatus::unavailable;
    }

    for (uint32_t ring = 0; ring <= config_.compute_rings; ++ring) {
        reclaim(ring, false);
    }

    return phase_ == Phase::running ? Gfx11QueuesStatus::running : report_.status;
}

Gfx11QueuesStatus Gfx11Queues::disable_submission() noexcept {
    if (!report_.submission_enabled) {
        return Gfx11QueuesStatus::unavailable;
    }

    Gfx11QueuesStatus status = Gfx11QueuesStatus::stopped;

    for (uint32_t ring = 0; ring <= config_.compute_rings; ++ring) {
        Gfx11RingReport& info = report_.rings[ring];

        if (!info.submit_enabled) {
            continue;
        }

        reclaim(ring, false);
        const FenceStatus detached = fences_->detach_ring(ring, false);

        if (detached == FenceStatus::detached) {
            // Every fence signalled: no IB lease can remain.
            reclaim(ring, true);
            info.submit_enabled = false;
        } else if (detached == FenceStatus::ring_hung) {
            status = Gfx11QueuesStatus::ring_hung;
        } else if (status != Gfx11QueuesStatus::ring_hung) {
            status = Gfx11QueuesStatus::busy;
        }
    }

    if (status != Gfx11QueuesStatus::stopped) {
        return status;
    }

    fences_ = nullptr;
    wptr_store_ = {};
    report_.submission_enabled = false;
    return phase_ == Phase::running ? Gfx11QueuesStatus::running : report_.status;
}

} // namespace rx7900::hardware

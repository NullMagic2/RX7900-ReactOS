/*
Module: rx7900.hardware.sdma6_init
Purpose: Program the SDMA 6.0.0 kernel rings, NBIO4.3 SDMA doorbell range and ring test.
Created: 2026-10-21
Architecture: Bounded register protocol; native owner performs MMIO and doorbell
stores; Manager owns the local-VRAM ring and writeback backing; the fence packet
comes from the shared sdma encoder.
Copyright 2020 Advanced Micro Devices, Inc.; Copyright 2021 Advanced Micro Devices, Inc.;
Copyright 2019 Advanced Micro Devices, Inc.; Copyright 2018 Advanced Micro Devices, Inc.;
Copyright 2008 Advanced Micro Devices, Inc.; Copyright 2008 Red Hat Inc.;
Copyright 2009 Jerome Glisse.
Adapted Linux v6.12 sdma_v6_0.c, sdma_v6_0_0_pkt_open.h, sdma_common.h, nbio_v4_3.c,
soc21.c, amdgpu_ring.c, amdgpu_ring.h, amdgpu_ib.c, amdgpu_fence.c, amdgpu_sdma.c,
amdgpu_device.c, amdgpu_drv.c, amdgpu_doorbell.h, soc15_hw_ip.h, gc_11_0_0 and
nbio_4_3_0 register headers. Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#include "sdma6_init.hpp"

namespace rx7900::hardware {
namespace {

// gc_11_0_0_offset.h SDMA0 registers. sdma_v6_0_get_reg_offset adds
// SDMA1_REG_OFFSET (0x600) for instance 1 to GC segment 0 registers, and
// SDMA1_HYP_DEC_REG_OFFSET (0x20) per instance to registers in the HYP_DEC
// range 0x5880..0x589a, which use GC segment 1 (F32_CNTL BASE_IDX 1).
constexpr uint32_t sdma1_reg_offset = 0x600;
constexpr uint32_t sdma0_hyp_dec_reg_start = 0x5880;
constexpr uint32_t sdma0_hyp_dec_reg_end = 0x589a;
constexpr uint32_t sdma1_hyp_dec_reg_offset = 0x20;

constexpr uint32_t sdma_cntl = 0x001c;
constexpr uint32_t sdma_sem_wait_fail_timer_cntl = 0x0022;
constexpr uint32_t sdma_watchdog_cntl = 0x002e;
constexpr uint32_t sdma_utcl1_cntl = 0x003c;
constexpr uint32_t sdma_utcl1_page = 0x003f;
constexpr uint32_t sdma_queue0_rb_cntl = 0x0080;
constexpr uint32_t sdma_queue0_rb_base = 0x0081;
constexpr uint32_t sdma_queue0_rb_base_hi = 0x0082;
constexpr uint32_t sdma_queue0_rb_rptr = 0x0083;
constexpr uint32_t sdma_queue0_rb_rptr_hi = 0x0084;
constexpr uint32_t sdma_queue0_rb_wptr = 0x0085;
constexpr uint32_t sdma_queue0_rb_wptr_hi = 0x0086;
constexpr uint32_t sdma_queue0_rb_rptr_addr_hi = 0x0088;
constexpr uint32_t sdma_queue0_rb_rptr_addr_lo = 0x0089;
constexpr uint32_t sdma_queue0_ib_cntl = 0x008a;
constexpr uint32_t sdma_queue0_doorbell = 0x0092;
constexpr uint32_t sdma_queue0_doorbell_offset = 0x00ab;
constexpr uint32_t sdma_queue0_rb_wptr_poll_addr_hi = 0x00b2;
constexpr uint32_t sdma_queue0_rb_wptr_poll_addr_lo = 0x00b3;
constexpr uint32_t sdma_queue0_minor_ptr_update = 0x00b5; // highest segment 0 register
constexpr uint32_t sdma_f32_cntl = 0x589a;                // segment 1

// gc_11_0_0_sh_mask.h.
constexpr uint32_t cntl_ctxempty_int_enable = 0x10000000u;
constexpr uint32_t watchdog_queue_hang_count_mask = 0x000000ffu;
constexpr uint32_t utcl1_cntl_redo_delay_mask = 0x0000001fu;
constexpr uint32_t utcl1_cntl_resp_mode_mask = 0x00000600u;
constexpr uint32_t utcl1_cntl_resp_mode_shift = 9;
constexpr uint32_t utcl1_page_llc_noalloc = 0x01000000u;
constexpr uint32_t rb_cntl_rb_enable = 0x00000001u;
constexpr uint32_t rb_cntl_rb_size_mask = 0x0000003eu;
constexpr uint32_t rb_cntl_rb_size_shift = 1;
constexpr uint32_t rb_cntl_wptr_poll_enable = 0x00000100u;
constexpr uint32_t rb_cntl_f32_wptr_poll_enable = 0x00000800u;
constexpr uint32_t rb_cntl_rptr_writeback_enable = 0x00001000u;
constexpr uint32_t rb_cntl_rb_priv = 0x00800000u;
constexpr uint32_t ib_cntl_ib_enable = 0x00000001u;
constexpr uint32_t doorbell_enable = 0x10000000u;
constexpr uint32_t doorbell_offset_mask = 0x0ffffffcu;
constexpr uint32_t doorbell_offset_shift = 2;
constexpr uint32_t f32_cntl_halt = 0x00000001u;
constexpr uint32_t f32_cntl_th1_reset = 0x00002000u;

// sdma_v6_0_gfx_resume field values.
// QUEUE_HANG_COUNT = max(adev->usec_timeout / 100000, 1), 100 ms per unit.
constexpr uint32_t watchdog_queue_hang_count =
    sdma6_usec_timeout / 100000 > 1 ? sdma6_usec_timeout / 100000 : 1;
constexpr uint32_t utcl1_resp_mode = 3;
constexpr uint32_t utcl1_redo_delay = 9;
// UTCL1_PAGE: Linux clears with the literal 0xFF0FFF (which also drops bits
// 31:24) and sets RD_L2_POLICY/WR_L2_POLICY to sdma_common.h
// CACHE_READ_POLICY_L2__DEFAULT (NOA, 2) and CACHE_WRITE_POLICY_L2__DEFAULT
// (BYPASS, 3), plus LLC_NOALLOC.
constexpr uint32_t utcl1_page_keep = 0x00ff0fffu;
constexpr uint32_t cache_read_policy_l2_default = 2;
constexpr uint32_t cache_write_policy_l2_default = 3;
constexpr uint32_t utcl1_page_policy = (cache_read_policy_l2_default << 12) |
                                       (cache_write_policy_l2_default << 14) |
                                       utcl1_page_llc_noalloc;
// RB_RPTR_ADDR_LO keeps bits 31:2 (Linux masks with 0xFFFFFFFC).
constexpr uint32_t rptr_addr_lo_mask = 0xfffffffcu;

// nbio_4_3_0_offset.h / _sh_mask.h S2A_DOORBELL_ENTRY_2_CTRL, segment 3, beyond
// BAR5 on Navi31 and therefore reached through Nbio's SMN fallback.
constexpr uint32_t nbif_s2a_segment = 3;
constexpr uint32_t s2a_doorbell_entry_2_ctrl = 0x4f7642;
constexpr uint32_t s2a_port2_enable = 0x00000001u;
constexpr uint32_t s2a_port2_awid_mask = 0x0000003eu;
constexpr uint32_t s2a_port2_awid_shift = 1;
constexpr uint32_t s2a_port2_range_offset_mask = 0x0001ff80u;
constexpr uint32_t s2a_port2_range_offset_shift = 7;
constexpr uint32_t s2a_port2_range_size_mask = 0x01fe0000u;
constexpr uint32_t s2a_port2_range_size_shift = 17;
constexpr uint32_t s2a_port2_awaddr_mask = 0xf0000000u;
constexpr uint32_t s2a_port2_awaddr_shift = 28;
// nbio_v4_3_sdma_doorbell_range: AWID 0xe and AWADDR_31_28_VALUE 0x3.
constexpr uint32_t s2a_sdma_awid = 0xe;
constexpr uint32_t s2a_sdma_awaddr = 0x3;

constexpr uint64_t mc_address_limit = 1ULL << 48;

// sdma_v6_0_0_pkt_open.h opcodes and fields used by the IB test.
constexpr uint32_t sdma_op_nop = 0;
constexpr uint32_t sdma_op_write = 2;            // SUB_OP WRITE_LINEAR 0
constexpr uint32_t sdma_op_indirect = 4;         // HEADER_VMID(0)
constexpr uint32_t sdma_op_poll_regmem = 8;
constexpr uint32_t sdma_op_cond_exe = 9;
constexpr uint32_t sdma_nop_count_shift = 16;    // SDMA_PKT_NOP_HEADER_COUNT
constexpr uint32_t sdma_nop_count_mask = 0x3fff;
constexpr uint32_t poll_regmem_hdp_flush = 1u << 26;
constexpr uint32_t poll_regmem_func_equal = 3u << 28;
// SDMA_PKT_POLL_REGMEM_DW5_RETRY_COUNT(0xfff) | DW5_INTERVAL(10).
constexpr uint32_t poll_regmem_retry_interval = (0xfffu << 16) | 10u;
// sdma_v6_0_ring_emit_ib: the IB base must be 32-byte aligned.
constexpr uint32_t indirect_base_mask = 0xffffffe0u;
// sdma_v6_0_ring_init_cond_exec: execute when *addr equals 1.
constexpr uint32_t cond_exe_reference = 1;
// nbio_v4_3_hdp_flush_reg ref_and_mask_sdma0, shifted by ring->me (instance).
constexpr uint32_t hdp_flush_ref_sdma0 = 0x00000400u;
// nbio_4_3_0_offset.h BIF_BX_PF0_GPU_HDP_FLUSH_REQ/DONE, NBIO BASE_IDX 2.
constexpr uint32_t nbif_hdp_flush_segment = 2;
constexpr uint32_t bif_bx_pf0_gpu_hdp_flush_req = 0x0106;
constexpr uint32_t bif_bx_pf0_gpu_hdp_flush_done = 0x0107;
// sdma_v6_0_ring_funcs align_mask 0xf; INDIRECT must end on 8 dwords.
constexpr uint32_t sdma_ring_align_mask = 0xf;
constexpr uint32_t ib_fence_slot = 0;
constexpr uint32_t ib_cond_exec_slot = 1;
constexpr uint32_t ib_test_slot = 2;

uint32_t ib_slot_offset(uint32_t instance, uint32_t slot) noexcept {
    return sdma6_ib_test_slots_offset +
           (instance * sdma6_ib_test_slots_per_instance + slot) * sdma6_ib_test_slot_stride;
}

constexpr uint32_t rptr_slot = 0;
constexpr uint32_t wptr_slot = 1;
constexpr uint32_t test_slot = 2;

uint32_t writeback_offset(uint32_t instance, uint32_t slot) noexcept {
    return (instance * sdma6_writeback_slots_per_instance + slot) * sdma6_writeback_slot_bytes;
}

bool power_of_two(uint32_t value) noexcept {
    return value != 0 && (value & (value - 1)) == 0;
}

// order_base_2 of an exact power of two.
uint32_t log2_exact(uint32_t value) noexcept {
    uint32_t order = 0;

    while (value > 1) {
        value >>= 1;
        ++order;
    }

    return order;
}

bool overlaps(uint64_t a, uint64_t a_bytes, uint64_t b, uint64_t b_bytes) noexcept {
    return a < b + b_bytes && b < a + a_bytes;
}

} // namespace

DiscoveryStatus validate_sdma6_discovery(ByteView binary, uint32_t aperture,
                                         Sdma6Registers& output) noexcept {
    output = {};

    if (aperture == 0 || (aperture & 3u) != 0) {
        return DiscoveryStatus::invalid_argument;
    }

    DiscoveredIp gc;
    DiscoveredIp sdma0;
    DiscoveredIp sdma1;
    auto status = discover_ip(binary, {discovery_gc_hardware_id, 0, 11, 0, 0, 2}, gc);

    if (status != DiscoveryStatus::validated) {
        return status;
    }

    status = discover_ip(binary, {discovery_sdma0_hardware_id, 0, 6, 0, 0, 1}, sdma0);

    if (status != DiscoveryStatus::validated) {
        return status;
    }

    status = discover_ip(binary, {discovery_sdma1_hardware_id, 0, 6, 0, 0, 1}, sdma1);

    if (status != DiscoveryStatus::validated) {
        return status;
    }

    uint32_t offset = 0;

    if (!discovery_register_offset(gc.segment(0), sdma_queue0_minor_ptr_update + sdma1_reg_offset,
                                   aperture, offset) ||
        !discovery_register_offset(gc.segment(1), sdma_f32_cntl + sdma1_hyp_dec_reg_offset,
                                   aperture, offset)) {
        return DiscoveryStatus::invalid_argument;
    }

    output.gc_ = gc;
    output.aperture_bytes_ = aperture;
    return DiscoveryStatus::validated;
}

// sdma_v6_0_get_reg_offset.
bool Sdma6::locate(uint32_t instance, uint32_t internal, uint32_t& base,
                   uint32_t& reg) const noexcept {
    if (instance >= sdma6_instance_count) {
        return false;
    }

    if (internal >= sdma0_hyp_dec_reg_start && internal <= sdma0_hyp_dec_reg_end) {
        base = registers_.gc_base(1);
        reg = internal + sdma1_hyp_dec_reg_offset * instance;
        return true;
    }

    base = registers_.gc_base(0);
    reg = instance == 1 ? internal + sdma1_reg_offset : internal;
    return true;
}

bool Sdma6::read(uint32_t instance, uint32_t internal, uint32_t& value) noexcept {
    uint32_t base = 0;
    uint32_t reg = 0;

    // All-ones is a dead or unmapped device; no SDMA register is used for it.
    return locate(instance, internal, base, reg) && read_ip_register(io_, base, reg, value) &&
           value != UINT32_MAX;
}

bool Sdma6::write(uint32_t instance, uint32_t internal, uint32_t value) noexcept {
    uint32_t base = 0;
    uint32_t reg = 0;

    return locate(instance, internal, base, reg) && write_ip_register(io_, base, reg, value);
}

bool Sdma6::modify(uint32_t instance, uint32_t internal, uint32_t clear, uint32_t set,
                   uint32_t& value) noexcept {
    if (!read(instance, internal, value)) {
        return false;
    }

    value = (value & ~clear) | set;
    return write(instance, internal, value);
}

uint32_t Sdma6::doorbell_index(uint32_t instance) const noexcept {
    return instance == 0 ? sdma6_ring0_doorbell_index : sdma6_ring1_doorbell_index;
}

Sdma6Status Sdma6::quarantine(Sdma6Status reason, uint32_t instance) noexcept {
    phase_ = Phase::faulted;
    report_.status = reason;
    report_.failed_instance = instance;
    return reason;
}

Sdma6Status Sdma6::initialize(Sdma6Registers registers, NbioRegisters nbio_registers,
                              RegisterIo io, Nbio& nbio, VramLocation vram, Sdma6Config config,
                              DoorbellIo doorbells, Sdma6Recovery recovery) noexcept {
    if (phase_ == Phase::started || phase_ == Phase::stopped || phase_ == Phase::faulted ||
        report_.memory_retained) {
        return Sdma6Status::busy;
    }

    phase_ = Phase::uninitialized;
    report_ = {};

    if (!registers.valid() || !nbio_registers.valid()) {
        report_.status = Sdma6Status::unsupported_ip;
        return report_.status;
    }

    if (!register_io_valid(io) || io.aperture_bytes != registers.aperture_bytes() ||
        io.aperture_bytes != nbio_registers.aperture_bytes() ||
        recovery.confirm_reset_quiescence == nullptr) {
        report_.status = Sdma6Status::unavailable;
        return report_.status;
    }

    const bool no_doorbells = doorbells.write32 == nullptr && doorbells.write64 == nullptr;
    // A 64-bit doorbell needs its dword and the following one inside the aperture.
    const bool doorbells_fit = doorbell_io_valid(doorbells) &&
                               sdma6_ring0_doorbell_index < doorbells.aperture_dwords - 1 &&
                               sdma6_ring1_doorbell_index < doorbells.aperture_dwords - 1;

    if (vram.bytes == 0 || vram.mc_end < vram.mc_start || !power_of_two(config.ring_bytes) ||
        config.ring_bytes < sdma6_minimum_ring_bytes || (!no_doorbells && !doorbells_fit)) {
        report_.status = Sdma6Status::invalid_argument;
        return report_.status;
    }

    registers_ = registers;
    nbio_registers_ = nbio_registers;
    io_ = io;
    nbio_ = &nbio;
    vram_ = vram;
    config_ = config;
    doorbells_ = no_doorbells ? DoorbellIo{} : doorbells;
    recovery_ = recovery;
    report_.ring_bytes = config.ring_bytes;
    report_.doorbell_mode = !no_doorbells;
    phase_ = Phase::ready;
    report_.status = Sdma6Status::ready;
    return report_.status;
}

bool Sdma6::retain_memory(memory::Manager& manager, memory::Handle ring0, memory::Handle ring1,
                          memory::Handle writeback) noexcept {
    memory::AllocationInfo rings[sdma6_instance_count]{};
    memory::AllocationInfo slots{};

    if (!retain_local_vram(manager, ring0, vram_, config_.ring_bytes, sdma6_ring_alignment, true,
                           rings[0])) {
        return false;
    }

    if (!retain_local_vram(manager, ring1, vram_, config_.ring_bytes, sdma6_ring_alignment, true,
                           rings[1])) {
        manager.unretain(ring0);
        return false;
    }

    if (!retain_local_vram(manager, writeback, vram_, sdma6_writeback_bytes,
                           sdma6_writeback_alignment, true, slots)) {
        manager.unretain(ring1);
        manager.unretain(ring0);
        return false;
    }

    const uint64_t ring0_mc = rings[0].gpu.virtual_address;
    const uint64_t ring1_mc = rings[1].gpu.virtual_address;
    const uint64_t slots_mc = slots.gpu.virtual_address;
    const uint64_t bytes = config_.ring_bytes;

    // retain_local_vram bounds every range inside VRAM, so these sums cannot wrap.
    if (overlaps(ring0_mc, bytes, ring1_mc, bytes) ||
        overlaps(ring0_mc, bytes, slots_mc, sdma6_writeback_bytes) ||
        overlaps(ring1_mc, bytes, slots_mc, sdma6_writeback_bytes) ||
        ring0_mc + bytes > mc_address_limit || ring1_mc + bytes > mc_address_limit ||
        slots_mc + sdma6_writeback_bytes > mc_address_limit) {
        manager.unretain(writeback);
        manager.unretain(ring1);
        manager.unretain(ring0);
        return false;
    }

    manager_ = &manager;
    handles_[0] = ring0;
    handles_[1] = ring1;
    handles_[2] = writeback;
    ring_cpu_[0] = rings[0].backing.cpu_bytes;
    ring_cpu_[1] = rings[1].backing.cpu_bytes;
    writeback_cpu_ = slots.backing.cpu_bytes;

    for (uint32_t instance = 0; instance < sdma6_instance_count; ++instance) {
        report_.ring_mc[instance] = rings[instance].gpu.virtual_address;
        report_.rptr_mc[instance] = slots_mc + writeback_offset(instance, rptr_slot);
        report_.wptr_mc[instance] = slots_mc + writeback_offset(instance, wptr_slot);
        report_.test_mc[instance] = slots_mc + writeback_offset(instance, test_slot);
    }

    report_.memory_retained = true;
    return true;
}

// Like retain_local_vram without the VRAM-aperture requirement (the IB buffer
// may be GART-mapped system memory): same manager, CPU visible, GPU read/write,
// aligned, below the 48-bit MC limit and disjoint from both rings and writeback.
bool Sdma6::retain_ib(memory::Manager& manager, memory::Handle ib_buffer) noexcept {
    memory::AllocationInfo info{};
    sdma::GpuRange range{};

    if (&manager != manager_ || manager.describe(ib_buffer, info) != memory::Status::success ||
        manager.resolve({ib_buffer, 0}, sdma6_ib_test_buffer_bytes, memory::Access::read_write,
                        range) != memory::Status::success) {
        return false;
    }

    const uint64_t start = info.gpu.virtual_address;
    const uint64_t bytes = sdma6_ib_test_buffer_bytes;
    const uint64_t writeback = report_.rptr_mc[0];

    if ((start & (sdma6_ib_test_alignment - 1)) != 0 || info.gpu.byte_length < bytes ||
        start > mc_address_limit - bytes || info.backing.cpu_bytes == nullptr ||
        info.backing.cpu_byte_count < bytes ||
        overlaps(start, bytes, report_.ring_mc[0], config_.ring_bytes) ||
        overlaps(start, bytes, report_.ring_mc[1], config_.ring_bytes) ||
        overlaps(start, bytes, writeback, sdma6_writeback_bytes)) {
        return false;
    }

    if (manager.retain(ib_buffer, memory::Access::read_write) != memory::Status::success) {
        return false;
    }

    ib_handle_ = ib_buffer;
    ib_cpu_ = info.backing.cpu_bytes;
    report_.ib_buffer_mc = start;
    report_.ib_buffer_retained = true;

    for (uint32_t instance = 0; instance < sdma6_instance_count; ++instance) {
        report_.ib_mc[instance] = start + instance * sdma6_ib_test_slot_bytes;
        report_.fence_mc[instance] = start + ib_slot_offset(instance, ib_fence_slot);
        report_.cond_exec_mc[instance] = start + ib_slot_offset(instance, ib_cond_exec_slot);
        report_.ib_test_mc[instance] = start + ib_slot_offset(instance, ib_test_slot);
    }

    return true;
}

void Sdma6::release_ib() noexcept {
    if (report_.ib_buffer_retained && manager_ != nullptr) {
        manager_->unretain(ib_handle_);
    }

    ib_handle_ = {};
    ib_cpu_ = nullptr;
    report_.ib_buffer_retained = false;
    report_.ib_buffer_mc = 0;
}

void Sdma6::release_leases() noexcept {
    release_ib();

    if (report_.memory_retained && manager_ != nullptr) {
        for (const auto& handle : handles_) {
            manager_->unretain(handle);
        }
    }

    manager_ = nullptr;

    for (auto& handle : handles_) {
        handle = {};
    }

    for (uint32_t instance = 0; instance < sdma6_instance_count; ++instance) {
        ring_cpu_[instance] = nullptr;
        report_.ring_mc[instance] = 0;
        report_.rptr_mc[instance] = 0;
        report_.wptr_mc[instance] = 0;
        report_.test_mc[instance] = 0;
        report_.wptr_bytes[instance] = 0;
        report_.ring_ready[instance] = false;
        report_.ib_disabled[instance] = false;
        report_.fence_seq[instance] = 0;
        report_.fence_value[instance] = 0;
    }

    writeback_cpu_ = nullptr;
    report_.memory_retained = false;
}

// amdgpu_ring_clear_ring fills each ring with ring->funcs->nop, which is
// SDMA_PKT_NOP_HEADER_OP(SDMA_OP_NOP) == 0; amdgpu_device_wb_init zeroes the
// writeback slots. Uncached BAR0 stores, published by the HDP flush.
bool Sdma6::clear_memory() noexcept {
    for (uint8_t* ring : ring_cpu_) {
        const sdma::CommandBuffer bytes(ring, config_.ring_bytes);

        for (uint32_t offset = 0; offset < config_.ring_bytes; offset += 4) {
            if (!bytes.write_u32(offset, 0)) {
                return false;
            }
        }
    }

    const sdma::CommandBuffer slots(writeback_cpu_, sdma6_writeback_bytes);

    for (uint32_t offset = 0; offset < sdma6_writeback_bytes; offset += 4) {
        if (!slots.write_u32(offset, 0)) {
            return false;
        }
    }

    return nbio_->flush_hdp();
}

// sdma_v6_0_ctxempty_int_enable (bare metal).
bool Sdma6::ctxempty_int_enable(bool on) noexcept {
    for (uint32_t instance = 0; instance < sdma6_instance_count; ++instance) {
        uint32_t value = 0;

        if (!modify(instance, sdma_cntl, cntl_ctxempty_int_enable,
                    on ? cntl_ctxempty_int_enable : 0, value)) {
            report_.failed_instance = instance;
            return false;
        }
    }

    return true;
}

// sdma_v6_0_gfx_stop.
bool Sdma6::gfx_stop() noexcept {
    for (uint32_t instance = 0; instance < sdma6_instance_count; ++instance) {
        uint32_t rb_cntl = 0;
        uint32_t ib_cntl = 0;

        if (!modify(instance, sdma_queue0_rb_cntl, rb_cntl_rb_enable, 0, rb_cntl)) {
            report_.failed_instance = instance;
            return false;
        }

        report_.rb_cntl[instance] = rb_cntl;
        report_.ring_ready[instance] = false;

        if (!modify(instance, sdma_queue0_ib_cntl, ib_cntl_ib_enable, 0, ib_cntl)) {
            report_.failed_instance = instance;
            return false;
        }

        report_.ib_cntl[instance] = ib_cntl;
    }

    return true;
}

// sdma_v6_0_enable (bare metal). Disabling first stops the gfx rings;
// sdma_v6_0_rlc_stop is empty in Linux v6.12.
bool Sdma6::enable(bool on) noexcept {
    if (!on && !gfx_stop()) {
        return false;
    }

    for (uint32_t instance = 0; instance < sdma6_instance_count; ++instance) {
        uint32_t value = 0;

        if (!modify(instance, sdma_f32_cntl, f32_cntl_halt, on ? 0 : f32_cntl_halt, value)) {
            report_.failed_instance = instance;
            return false;
        }

        report_.f32_cntl[instance] = value;
    }

    return true;
}

// nbio_v4_3_sdma_doorbell_range, called by gfx_resume for instance 0 only with
// ring 0's dword doorbell index and sdma_doorbell_range * num_instances.
bool Sdma6::sdma_doorbell_range() noexcept {
    const uint32_t base = nbio_registers_.nbif_base(nbif_s2a_segment);
    uint32_t value = 0;

    if (!nbio_->read_register(base, s2a_doorbell_entry_2_ctrl, value) || value == UINT32_MAX) {
        return false;
    }

    if (report_.doorbell_mode) {
        value &= ~(s2a_port2_awid_mask | s2a_port2_range_offset_mask | s2a_port2_range_size_mask |
                   s2a_port2_awaddr_mask);
        value |= s2a_port2_enable;
        value |= (s2a_sdma_awid << s2a_port2_awid_shift) & s2a_port2_awid_mask;
        value |= (sdma6_ring0_doorbell_index << s2a_port2_range_offset_shift) &
                 s2a_port2_range_offset_mask;
        value |= (sdma6_doorbell_range_dwords << s2a_port2_range_size_shift) &
                 s2a_port2_range_size_mask;
        value |= (s2a_sdma_awaddr << s2a_port2_awaddr_shift) & s2a_port2_awaddr_mask;
    } else {
        value &= ~s2a_port2_range_size_mask;
    }

    if (!nbio_->write_register(base, s2a_doorbell_entry_2_ctrl, value)) {
        return false;
    }

    report_.doorbell_range = value;
    return true;
}

// sdma_v6_0_gfx_resume for one instance up to the ring test, bare metal, little
// endian (no RB/IB/RPTR swap bits), in Linux store order.
bool Sdma6::program_ring(uint32_t instance) noexcept {
    const uint64_t ring = report_.ring_mc[instance];
    const uint64_t rptr = report_.rptr_mc[instance];
    const uint64_t wptr_poll = report_.wptr_mc[instance];
    uint32_t rb_cntl = 0;

    if (!write(instance, sdma_sem_wait_fail_timer_cntl, 0) ||
        !read(instance, sdma_queue0_rb_cntl, rb_cntl)) {
        return false;
    }

    rb_cntl &= ~rb_cntl_rb_size_mask;
    rb_cntl |= (log2_exact(config_.ring_bytes / 4) << rb_cntl_rb_size_shift) & rb_cntl_rb_size_mask;
    rb_cntl |= rb_cntl_rb_priv;

    if (!write(instance, sdma_queue0_rb_cntl, rb_cntl) ||
        !write(instance, sdma_queue0_rb_rptr, 0) || !write(instance, sdma_queue0_rb_rptr_hi, 0) ||
        !write(instance, sdma_queue0_rb_wptr, 0) || !write(instance, sdma_queue0_rb_wptr_hi, 0) ||
        !write(instance, sdma_queue0_rb_wptr_poll_addr_lo, static_cast<uint32_t>(wptr_poll)) ||
        !write(instance, sdma_queue0_rb_wptr_poll_addr_hi, static_cast<uint32_t>(wptr_poll >> 32)) ||
        !write(instance, sdma_queue0_rb_rptr_addr_hi, static_cast<uint32_t>(rptr >> 32)) ||
        !write(instance, sdma_queue0_rb_rptr_addr_lo,
               static_cast<uint32_t>(rptr) & rptr_addr_lo_mask)) {
        return false;
    }

    report_.rb_cntl[instance] = rb_cntl;
    rb_cntl |= rb_cntl_rptr_writeback_enable | rb_cntl_f32_wptr_poll_enable;
    rb_cntl &= ~rb_cntl_wptr_poll_enable;
    report_.wptr_bytes[instance] = 0;

    // RB_BASE holds address bits 39:8, RB_BASE_HI bits 63:40 (gpu_addr >> 40).
    // MINOR_PTR_UPDATE must be set before WPTR moves to a lower value.
    if (!write(instance, sdma_queue0_rb_base, static_cast<uint32_t>(ring >> 8)) ||
        !write(instance, sdma_queue0_rb_base_hi, static_cast<uint32_t>(ring >> 40)) ||
        !write(instance, sdma_queue0_minor_ptr_update, 1) ||
        !write(instance, sdma_queue0_rb_wptr, 0) || !write(instance, sdma_queue0_rb_wptr_hi, 0)) {
        return false;
    }

    uint32_t doorbell = 0;
    uint32_t doorbell_offset = 0;

    if (!read(instance, sdma_queue0_doorbell, doorbell) ||
        !read(instance, sdma_queue0_doorbell_offset, doorbell_offset)) {
        return false;
    }

    if (report_.doorbell_mode) {
        doorbell |= doorbell_enable;
        doorbell_offset &= ~doorbell_offset_mask;
        doorbell_offset |= (doorbell_index(instance) << doorbell_offset_shift) & doorbell_offset_mask;
    } else {
        doorbell &= ~doorbell_enable;
    }

    if (!write(instance, sdma_queue0_doorbell, doorbell) ||
        !write(instance, sdma_queue0_doorbell_offset, doorbell_offset)) {
        return false;
    }

    report_.doorbell[instance] = doorbell;
    report_.doorbell_offset[instance] = doorbell_offset;

    if (instance == 0 && !sdma_doorbell_range()) {
        return false;
    }

    uint32_t value = 0;

    if (!write(instance, sdma_queue0_minor_ptr_update, 0) ||
        !modify(instance, sdma_watchdog_cntl, watchdog_queue_hang_count_mask,
                watchdog_queue_hang_count, value) ||
        !modify(instance, sdma_utcl1_cntl, utcl1_cntl_resp_mode_mask | utcl1_cntl_redo_delay_mask,
                (utcl1_resp_mode << utcl1_cntl_resp_mode_shift) | utcl1_redo_delay, value) ||
        !modify(instance, sdma_utcl1_page, ~utcl1_page_keep, utcl1_page_policy, value) ||
        !modify(instance, sdma_f32_cntl, f32_cntl_halt | f32_cntl_th1_reset, 0, value)) {
        return false;
    }

    report_.f32_cntl[instance] = value;
    rb_cntl |= rb_cntl_rb_enable;

    if (!write(instance, sdma_queue0_rb_cntl, rb_cntl)) {
        return false;
    }

    report_.rb_cntl[instance] = rb_cntl;

    if (!modify(instance, sdma_queue0_ib_cntl, 0, ib_cntl_ib_enable, value)) {
        return false;
    }

    report_.ib_cntl[instance] = value;
    return true;
}

// sdma_v6_0_ring_set_wptr with the byte WPTR (Linux ring->wptr << 2). Every CPU
// store to the ring, test slot and WPTR shadow is published by one HDP flush
// first, because those bytes live in local VRAM behind HDP.
Sdma6Status Sdma6::set_wptr(uint32_t instance) noexcept {
    const uint64_t wptr = report_.wptr_bytes[instance];

    if (report_.doorbell_mode) {
        // Linux atomic64_set of the shadow. Two ordered dword stores are used
        // here; the high dword stays zero below 4 GiB of ring traffic, so the
        // engine cannot observe a torn value during the ring test.
        const sdma::CommandBuffer slots(writeback_cpu_, sdma6_writeback_bytes);
        const uint32_t shadow = writeback_offset(instance, wptr_slot);

        if (!slots.write_u32(shadow, static_cast<uint32_t>(wptr)) ||
            !slots.write_u32(shadow + 4, static_cast<uint32_t>(wptr >> 32))) {
            return quarantine(Sdma6Status::invalid_memory, instance);
        }
    }

    if (!nbio_->flush_hdp()) {
        return quarantine(Sdma6Status::mmio_failure, instance);
    }

    if (report_.doorbell_mode) {
        if (!doorbells_.write64(doorbells_.context, doorbell_index(instance), wptr)) {
            return quarantine(Sdma6Status::doorbell_failure, instance);
        }

        return Sdma6Status::started;
    }

    if (!write(instance, sdma_queue0_rb_wptr, static_cast<uint32_t>(wptr)) ||
        !write(instance, sdma_queue0_rb_wptr_hi, static_cast<uint32_t>(wptr >> 32))) {
        return quarantine(Sdma6Status::mmio_failure, instance);
    }

    return Sdma6Status::started;
}

// sdma_v6_0_ring_test_ring through amdgpu_ring_test_helper. Linux emits a
// 5-dword WRITE_LINEAR of 0xDEADBEEF; this driver reuses the shared SDMA6 FENCE
// encoder (4 dwords, MTYPE UC), which performs the same single dword store. The
// frame is padded with single-dword NOPs (zero) to the 16-dword fetch alignment,
// matching sdma_v6_0_ring_insert_nop without burst NOPs; the published WPTR is
// the same 64 bytes Linux publishes.
Sdma6Status Sdma6::ring_test(uint32_t instance) noexcept {
    const uint32_t test = writeback_offset(instance, test_slot);
    const sdma::CommandBuffer slots(writeback_cpu_, sdma6_writeback_bytes);

    if (!slots.write_u32(test, sdma6_ring_test_seed)) {
        return quarantine(Sdma6Status::invalid_memory, instance);
    }

    const uint64_t wptr = report_.wptr_bytes[instance];
    const size_t ring_offset = static_cast<size_t>(wptr & (config_.ring_bytes - 1));
    const sdma::CommandBuffer frame =
        sdma::CommandBuffer(ring_cpu_[instance], config_.ring_bytes)
            .subbuffer(ring_offset, sdma6_ring_test_bytes);
    const sdma::FenceRequest fence{{{report_.test_mc[instance], sizeof(uint32_t)}, 0},
                                   sdma6_ring_test_value, sdma::FenceWidth::bits32, false};
    sdma::EncodeResult encoded{};

    if (frame.size() != sdma6_ring_test_bytes ||
        sdma::encode_fence(fence, frame, encoded) != sdma::EncodeStatus::success) {
        return quarantine(Sdma6Status::invalid_memory, instance);
    }

    for (size_t offset = encoded.bytes_written; offset < sdma6_ring_test_bytes; offset += 4) {
        if (!frame.write_u32(offset, 0)) {
            return quarantine(Sdma6Status::invalid_memory, instance);
        }
    }

    report_.wptr_bytes[instance] = wptr + sdma6_ring_test_bytes;
    const Sdma6Status published = set_wptr(instance);

    if (published != Sdma6Status::started) {
        return published;
    }

    const ByteView result(writeback_cpu_, sdma6_writeback_bytes);
    uint32_t polls = 0;

    for (; polls < sdma6_usec_timeout; ++polls) {
        uint32_t value = 0;

        if (!result.read_u32(test, value)) {
            return quarantine(Sdma6Status::invalid_memory, instance);
        }

        if (value == sdma6_ring_test_value) {
            break;
        }

        io_.delay_microseconds(io_.context, 1);
    }

    report_.ring_test_polls[instance] = polls;

    // The engine may still execute the frame later and write the test slot, so
    // a timeout keeps every lease until reset quiescence is confirmed.
    if (polls >= sdma6_usec_timeout) {
        return quarantine(Sdma6Status::ring_test_timeout, instance);
    }

    report_.ring_ready[instance] = true;
    return Sdma6Status::started;
}

// sdma_v6_0_start, bare metal with PSP-loaded firmware: no load_microcode.
Sdma6Status Sdma6::start() noexcept {
    if (!enable(true) || !ctxempty_int_enable(true)) {
        return quarantine(Sdma6Status::mmio_failure, report_.failed_instance);
    }

    // sdma_v6_0_gfx_resume: each instance is programmed and ring-tested before
    // the next one is touched.
    for (uint32_t instance = 0; instance < sdma6_instance_count; ++instance) {
        if (!program_ring(instance)) {
            return quarantine(Sdma6Status::mmio_failure, instance);
        }

        const Sdma6Status tested = ring_test(instance);

        if (tested != Sdma6Status::started) {
            return tested;
        }
    }

    // sdma_v6_0_rlc_resume is empty in Linux v6.12.
    report_.started = true;
    report_.failed_instance = UINT32_MAX;
    phase_ = Phase::started;
    report_.status = Sdma6Status::started;
    return report_.status;
}

// amdgpu_ring_write into the kernel ring at a dword position (64-bit pointers
// are never masked; the ring index is).
bool Sdma6::ring_put(uint32_t instance, uint64_t& dwords, uint32_t value) noexcept {
    const uint64_t index = dwords & (config_.ring_bytes / 4 - 1);

    if (!sdma::CommandBuffer(ring_cpu_[instance], config_.ring_bytes)
             .write_u32(static_cast<size_t>(index) * 4, value)) {
        return false;
    }

    ++dwords;
    return true;
}

// sdma_v6_0_ring_insert_nop: with burst_nop the first NOP carries COUNT(count - 1).
bool Sdma6::insert_nop(uint32_t instance, uint64_t& dwords, uint32_t count) noexcept {
    for (uint32_t index = 0; index < count; ++index) {
        const uint32_t nop = report_.burst_nop && index == 0 ?
            sdma_op_nop | (((count - 1) & sdma_nop_count_mask) << sdma_nop_count_shift) : sdma_op_nop;

        if (!ring_put(instance, dwords, nop)) {
            return false;
        }
    }

    return true;
}

// amdgpu_ib_schedule(ring, 1, &ib, NULL, &f) on an SDMA6 ring (no job: vmid 0,
// no VM flush; soc21 has no invalidate_hdp; no insert_start/end):
//   sdma_v6_0_ring_emit_mem_sync (GCR_REQ, 5 dwords),
//   amdgpu_ring_init_cond_exec (COND_EXE on cond_exe_gpu_addr, 5),
//   amdgpu_device_flush_hdp -> sdma_v6_0_ring_emit_hdp_flush (POLL_REGMEM, 6),
//   sdma_v6_0_ring_emit_ib (NOPs to (2 - wptr) & 7, INDIRECT, 6; csa 0 since
//     vmid 0),
//   amdgpu_fence_emit -> sdma_v6_0_ring_emit_fence (FENCE MTYPE UC, 32-bit seq,
//     TRAP context 0 for AMDGPU_FENCE_FLAG_INT),
//   amdgpu_ring_patch_cond_exec, then amdgpu_ring_commit NOP padding to 16.
bool Sdma6::emit_ib_frame(uint32_t instance, uint64_t& dwords, uint64_t ib, uint32_t ib_dwords,
                          uint64_t fence_mc, uint64_t cond, uint32_t sequence) noexcept {
    uint8_t packet[sdma::cache_sync_packet_bytes + sdma::fence32_packet_bytes +
                   sdma::trap_packet_bytes]{};
    const sdma::CommandBuffer bytes(packet, sizeof(packet));
    sdma::EncodeResult sync{};
    sdma::EncodeResult fence{};
    const uint32_t ref = hdp_flush_ref_sdma0 << instance;
    const uint32_t segment = nbio_registers_.nbif_base(nbif_hdp_flush_segment);
    const sdma::FenceRequest request{{{fence_mc, sizeof(uint32_t)}, 0}, sequence,
                                     sdma::FenceWidth::bits32, true};

    if (sdma::encode_cache_sync(bytes, sync) != sdma::EncodeStatus::success ||
        sdma::encode_fence(request, bytes.subbuffer(sync.bytes_written,
                                                    sizeof(packet) - sync.bytes_written),
                           fence) != sdma::EncodeStatus::success) {
        return false;
    }

    const ByteView encoded(packet, sync.bytes_written + fence.bytes_written);
    bool ok = true;
    uint32_t value = 0;

    for (size_t offset = 0; ok && offset < sync.bytes_written; offset += 4) {
        ok = encoded.read_u32(offset, value) && ring_put(instance, dwords, value);
    }

    ok = ok && ring_put(instance, dwords, sdma_op_cond_exe) &&
         ring_put(instance, dwords, static_cast<uint32_t>(cond)) &&
         ring_put(instance, dwords, static_cast<uint32_t>(cond >> 32)) &&
         ring_put(instance, dwords, cond_exe_reference);
    const uint64_t cond_count = dwords;
    ok = ok && ring_put(instance, dwords, 0) &&
         ring_put(instance, dwords, sdma_op_poll_regmem | poll_regmem_hdp_flush | poll_regmem_func_equal) &&
         ring_put(instance, dwords, (segment + bif_bx_pf0_gpu_hdp_flush_done) << 2) &&
         ring_put(instance, dwords, (segment + bif_bx_pf0_gpu_hdp_flush_req) << 2) &&
         ring_put(instance, dwords, ref) && ring_put(instance, dwords, ref) &&
         ring_put(instance, dwords, poll_regmem_retry_interval) &&
         insert_nop(instance, dwords, (2u - static_cast<uint32_t>(dwords)) & 7u) &&
         ring_put(instance, dwords, sdma_op_indirect) &&
         ring_put(instance, dwords, static_cast<uint32_t>(ib) & indirect_base_mask) &&
         ring_put(instance, dwords, static_cast<uint32_t>(ib >> 32)) &&
         ring_put(instance, dwords, ib_dwords) && ring_put(instance, dwords, 0) &&
         ring_put(instance, dwords, 0);

    for (size_t offset = sync.bytes_written; ok && offset < encoded.size(); offset += 4) {
        ok = encoded.read_u32(offset, value) && ring_put(instance, dwords, value);
    }

    // amdgpu_ring_patch_cond_exec: (wptr - 1) - offset, wrap-corrected.
    uint64_t patch = cond_count;
    ok = ok && ring_put(instance, patch, static_cast<uint32_t>(dwords - 1 - cond_count)) &&
         insert_nop(instance, dwords,
                    (sdma_ring_align_mask + 1 - (static_cast<uint32_t>(dwords) & sdma_ring_align_mask)) &
                        sdma_ring_align_mask);
    return ok;
}

// sdma_v6_0_ring_test_ib for one instance with the fence polled.
Sdma6Status Sdma6::ib_test(uint32_t instance, uint32_t timeout) noexcept {
    const sdma::CommandBuffer buffer(ib_cpu_, sdma6_ib_test_buffer_bytes);
    const sdma::CommandBuffer ib = buffer.subbuffer(instance * sdma6_ib_test_slot_bytes,
                                                    sdma6_ib_test_slot_bytes);
    const uint32_t fence = ib_slot_offset(instance, ib_fence_slot);
    const uint32_t test = ib_slot_offset(instance, ib_test_slot);
    const uint64_t destination = report_.ib_test_mc[instance];
    report_.ib_test[instance] = Sdma6IbTest::failed;
    report_.ib_polls[instance] = 0;

    // amdgpu_fence_driver_start_ring state (slot = sync_seq), cond_exec 1,
    // destination seed, then ib.ptr[0..7]: WRITE_LINEAR one dword, three NOPs.
    bool ok = buffer.write_u32(fence, report_.fence_seq[instance]) &&
              buffer.write_u32(ib_slot_offset(instance, ib_cond_exec_slot), 1) &&
              buffer.write_u32(test, sdma6_ring_test_seed) && ib.write_u32(0, sdma_op_write) &&
              ib.write_u32(4, static_cast<uint32_t>(destination)) &&
              ib.write_u32(8, static_cast<uint32_t>(destination >> 32)) && ib.write_u32(12, 0) &&
              ib.write_u32(16, sdma6_ring_test_value);

    for (uint32_t offset = 20; ok && offset < sdma6_ib_test_dwords * 4; offset += 4) {
        ok = ib.write_u32(offset, sdma_op_nop);
    }

    // amdgpu_fence_emit: seq = ++ring->fence_drv.sync_seq (32-bit, wraps).
    ++report_.fence_seq[instance];
    uint64_t dwords = report_.wptr_bytes[instance] / 4;

    if (!ok || !emit_ib_frame(instance, dwords, report_.ib_mc[instance], sdma6_ib_test_dwords,
                              report_.fence_mc[instance], report_.cond_exec_mc[instance],
                              report_.fence_seq[instance])) {
        return quarantine(Sdma6Status::invalid_memory, instance);
    }

    report_.wptr_bytes[instance] = dwords * 4;

    const Sdma6Status published = set_wptr(instance);

    if (published != Sdma6Status::started) {
        return published;
    }

    const ByteView result(ib_cpu_, sdma6_ib_test_buffer_bytes);
    const uint32_t seq = report_.fence_seq[instance];
    bool signalled = false;

    for (; report_.ib_polls[instance] < timeout; ++report_.ib_polls[instance]) {
        if (!result.read_u32(fence, report_.fence_value[instance])) {
            return quarantine(Sdma6Status::invalid_memory, instance);
        }

        // amdgpu_fence_process: 32-bit wrap-around comparison with seq.
        if (report_.fence_value[instance] - seq < 0x80000000u) {
            signalled = true;
            break;
        }

        io_.delay_microseconds(io_.context, 1);
    }

    uint32_t value = 0;

    if (signalled && !result.read_u32(test, value)) {
        return quarantine(Sdma6Status::invalid_memory, instance);
    }

    if (signalled && value == sdma6_ring_test_value) {
        report_.ib_test[instance] = Sdma6IbTest::passed;
        return Sdma6Status::started;
    }

    // Linux ring->sched.ready = false; a late or misdirected write keeps this
    // ring and the IB buffer quarantined.
    report_.ib_test[instance] = signalled ? Sdma6IbTest::failed : Sdma6IbTest::timeout;
    report_.ib_disabled[instance] = true;
    report_.ring_ready[instance] = false;

    if (report_.ib_failed_instance == UINT32_MAX) {
        report_.ib_failed_instance = instance;
    }

    return Sdma6Status::ib_test_failed;
}

Sdma6Status Sdma6::test_ib(memory::Manager& manager, memory::Handle ib_buffer,
                           const Sdma6IbTestConfig& config) noexcept {
    if (phase_ != Phase::started) {
        return phase_ == Phase::faulted ? Sdma6Status::quarantined :
               phase_ == Phase::uninitialized ? Sdma6Status::unavailable : Sdma6Status::busy;
    }

    if (report_.ib_buffer_retained) {
        return Sdma6Status::quarantined;
    }

    // FenceManager owns the rings' sync_seq once submission is enabled.
    if (report_.submission_enabled) {
        return Sdma6Status::busy;
    }

    uint32_t offset = 0;

    if (config.timeout_microseconds == 0 ||
        config.timeout_microseconds > sdma6_ib_test_xgmi_timeout_us ||
        !discovery_register_offset(nbio_registers_.nbif_base(nbif_hdp_flush_segment),
                                   bif_bx_pf0_gpu_hdp_flush_done, nbio_registers_.aperture_bytes(),
                                   offset)) {
        report_.status = Sdma6Status::invalid_argument;
        return report_.status;
    }

    if (!retain_ib(manager, ib_buffer)) {
        report_.status = Sdma6Status::invalid_memory;
        return report_.status;
    }

    report_.burst_nop = config.firmware_feature_version >= sdma6_burst_nop_feature_version;
    report_.ib_failed_instance = UINT32_MAX;
    bool failed = false;

    for (uint32_t instance = 0; instance < sdma6_instance_count; ++instance) {
        if (report_.ib_disabled[instance] || !report_.ring_ready[instance]) {
            failed = failed || report_.ib_disabled[instance];
            continue;
        }

        const Sdma6Status status = ib_test(instance, config.timeout_microseconds);

        if (phase_ == Phase::faulted) {
            return status;
        }

        failed = failed || status == Sdma6Status::ib_test_failed;
    }

    if (failed) {
        report_.status = Sdma6Status::ib_test_failed;
        return report_.status;
    }

    // Both fences signalled after their IBs: the engines no longer read the buffer.
    release_ib();
    report_.status = Sdma6Status::started;
    return report_.status;
}

Sdma6Status Sdma6::hw_init(memory::Manager& manager, memory::Handle ring0, memory::Handle ring1,
                           memory::Handle writeback) noexcept {
    if (phase_ != Phase::ready || report_.memory_retained) {
        return phase_ == Phase::uninitialized ? Sdma6Status::unavailable : Sdma6Status::busy;
    }

    if (!retain_memory(manager, ring0, ring1, writeback)) {
        report_.status = Sdma6Status::invalid_memory;
        return report_.status;
    }

    if (!clear_memory()) {
        return quarantine(Sdma6Status::mmio_failure, UINT32_MAX);
    }

    return start();
}

Sdma6Status Sdma6::resume() noexcept {
    if (phase_ != Phase::stopped || !report_.memory_retained) {
        return phase_ == Phase::uninitialized ? Sdma6Status::unavailable : Sdma6Status::busy;
    }

    if (report_.ib_buffer_retained || report_.ib_disabled[0] || report_.ib_disabled[1]) {
        return Sdma6Status::quarantined;
    }

    // start() rewinds both rings to WPTR 0; FenceManager positions would regress.
    if (report_.submission_enabled) {
        return Sdma6Status::busy;
    }

    return start();
}

// sdma_v6_0_hw_fini (bare metal).
Sdma6Status Sdma6::hw_fini() noexcept {
    if (phase_ != Phase::started && phase_ != Phase::faulted) {
        return phase_ == Phase::uninitialized ? Sdma6Status::unavailable : Sdma6Status::busy;
    }

    if (phase_ == Phase::faulted) {
        // Best-effort stop of an uncertain engine; leases stay pinned.
        if (ctxempty_int_enable(false)) {
            enable(false);
        }

        report_.started = false;
        return Sdma6Status::quarantined;
    }

    if (!ctxempty_int_enable(false) || !enable(false)) {
        return quarantine(Sdma6Status::mmio_failure, report_.failed_instance);
    }

    report_.started = false;
    phase_ = Phase::stopped;
    report_.status = Sdma6Status::stopped;
    return report_.status;
}

Sdma6Status Sdma6::release() noexcept {
    if (phase_ != Phase::stopped) {
        return phase_ == Phase::faulted ? Sdma6Status::quarantined : Sdma6Status::busy;
    }

    if (report_.submission_enabled) {
        return Sdma6Status::busy;
    }

    if (report_.ib_buffer_retained || report_.ib_disabled[0] || report_.ib_disabled[1]) {
        return Sdma6Status::quarantined;
    }

    release_leases();
    phase_ = Phase::ready;
    report_.status = Sdma6Status::released;
    return report_.status;
}

Sdma6Status Sdma6::release_after_reset() noexcept {
    if (!report_.memory_retained && phase_ != Phase::faulted) {
        return Sdma6Status::busy;
    }

    if (recovery_.confirm_reset_quiescence == nullptr ||
        !recovery_.confirm_reset_quiescence(recovery_.context)) {
        return Sdma6Status::quarantined;
    }

    // The FenceManager views end with the writeback lease: detach them first.
    for (uint32_t instance = 0; instance < sdma6_instance_count; ++instance) {
        if (report_.submit_enabled[instance] && fences_ != nullptr) {
            fences_->detach_ring(fence_ring_sdma0 + instance, true);
        }

        reclaim(instance, true);
        report_.submit_enabled[instance] = false;
        report_.submit_fence_mc[instance] = 0;
        report_.submit_cond_exec_mc[instance] = 0;
        report_.submissions[instance] = 0;
        report_.consumed_position[instance] = 0;
        report_.free_dwords[instance] = 0;
    }

    fences_ = nullptr;
    wptr_store_ = {};
    report_.submission_enabled = false;
    release_leases();

    for (uint32_t instance = 0; instance < sdma6_instance_count; ++instance) {
        report_.ib_test[instance] = Sdma6IbTest::not_run;
        report_.ib_polls[instance] = 0;
    }

    report_.ib_failed_instance = UINT32_MAX;
    report_.started = false;
    phase_ = Phase::uninitialized;
    report_.status = Sdma6Status::released;
    return report_.status;
}

Sdma6Status Sdma6::enable_submission(FenceManager& fences, const Sdma6SubmitConfig& config) noexcept {
    if (phase_ != Phase::started) {
        return phase_ == Phase::faulted ? Sdma6Status::quarantined : Sdma6Status::unavailable;
    }

    if (report_.submission_enabled || report_.ib_buffer_retained) {
        return Sdma6Status::busy;
    }

    memory::AllocationInfo slots{};

    if ((report_.doorbell_mode && config.wptr_store.store == nullptr) || manager_ == nullptr ||
        manager_->describe(handles_[2], slots) != memory::Status::success ||
        slots.gpu.byte_length < sdma6_submission_writeback_bytes ||
        slots.backing.cpu_bytes != writeback_cpu_ ||
        slots.backing.cpu_byte_count < sdma6_submission_writeback_bytes) {
        return Sdma6Status::invalid_argument;
    }

    const sdma::CommandBuffer writeback(writeback_cpu_, sdma6_submission_writeback_bytes);
    fences_ = &fences;
    wptr_store_ = config.wptr_store;
    uint32_t attached = 0;

    for (uint32_t instance = 0; instance < sdma6_instance_count; ++instance) {
        if (!report_.ring_ready[instance] || report_.ib_test[instance] != Sdma6IbTest::passed ||
            report_.ib_disabled[instance]) {
            continue;
        }

        const uint32_t fence = (sdma6_writeback_fence_slot_base + 2 * instance) *
                               sdma6_writeback_slot_bytes;
        const uint32_t cond = fence + sdma6_writeback_slot_bytes;
        ByteView view(nullptr, 0);

        // amdgpu_fence_driver_start_ring: *cpu_addr = last_seq (the IB test's
        // signalled sequence); amdgpu_ring_init: *cond_exe_cpu_addr = 1.
        bool ok = writeback.write_u32(fence, report_.fence_seq[instance]) &&
                  writeback.write_u32(cond, cond_exe_reference) &&
                  ByteView(writeback_cpu_, sdma6_submission_writeback_bytes)
                      .subview(fence, sizeof(uint32_t), view) && nbio_->flush_hdp();
        ok = ok && fences.attach_ring(fence_ring_sdma0 + instance,
                                      {FenceEngine::sdma, 0, 0, 0, instance, view,
                                       report_.fence_seq[instance],
                                       report_.wptr_bytes[instance] / 4, 0}) == FenceStatus::attached;

        if (!ok) {
            for (uint32_t undo = 0; undo < instance; ++undo) {
                if (report_.submit_enabled[undo]) {
                    fences.detach_ring(fence_ring_sdma0 + undo, false);
                    report_.submit_enabled[undo] = false;
                }
            }

            fences_ = nullptr;
            wptr_store_ = {};
            return Sdma6Status::invalid_memory;
        }

        const uint64_t base = report_.rptr_mc[0];
        report_.submit_fence_mc[instance] = base + fence;
        report_.submit_cond_exec_mc[instance] = base + cond;
        report_.submit_enabled[instance] = true;
        report_.consumed_position[instance] = report_.wptr_bytes[instance] / 4;
        ++attached;
    }

    if (attached == 0) {
        fences_ = nullptr;
        wptr_store_ = {};
        return Sdma6Status::unavailable;
    }

    report_.submission_enabled = true;
    return Sdma6Status::started;
}

void Sdma6::reclaim(uint32_t instance, bool all) noexcept {
    for (auto& lease : leases_[instance]) {
        if (!lease.used) {
            continue;
        }

        if (!all && (fences_ == nullptr ||
                     fences_->query(fence_ring_sdma0 + instance, lease.sequence) !=
                         FenceStatus::signalled)) {
            continue;
        }

        if (manager_ != nullptr) {
            manager_->unretain(lease.ib);
        }

        lease = {};
        --report_.ib_leases[instance];
    }
}

// The IB GPU address: a live, readable allocation of the hw_init Manager,
// 32-byte aligned, below the 48-bit MC limit, outside both rings and the
// submission writeback.
bool Sdma6::submit_ib_address(const Sdma6Submission& submission, uint64_t& address) noexcept {
    sdma::GpuRange range{};
    const uint64_t bytes = static_cast<uint64_t>(submission.ib_dwords) * 4u;

    if (manager_ == nullptr ||
        manager_->resolve({submission.ib, submission.ib_offset}, bytes, memory::Access::read,
                          range) != memory::Status::success) {
        return false;
    }

    address = range.allocation.virtual_address + range.byte_offset;
    return (address & (sdma6_ib_alignment - 1)) == 0 && address <= mc_address_limit - bytes &&
           !overlaps(address, bytes, report_.ring_mc[0], config_.ring_bytes) &&
           !overlaps(address, bytes, report_.ring_mc[1], config_.ring_bytes) &&
           !overlaps(address, bytes, report_.rptr_mc[0], sdma6_submission_writeback_bytes);
}

// The end of the newest signalled frame, or the 64-bit byte RPTR writeback
// (sdma_v6_0_ring_get_rptr: >> 2) when it lies inside the unretired span.
uint64_t Sdma6::consumed_position(uint32_t instance, uint64_t retired) const noexcept {
    const uint64_t wptr = report_.wptr_bytes[instance] / 4;
    const uint64_t mask = config_.ring_bytes / 4 - 1;
    uint32_t rptr_bytes = 0;

    if (retired > wptr || wptr - retired > mask + 1 ||
        !ByteView(writeback_cpu_, sdma6_writeback_bytes)
             .read_u32(writeback_offset(instance, rptr_slot), rptr_bytes)) {
        return retired;
    }

    const uint64_t behind = (wptr - (rptr_bytes >> 2)) & mask;
    return behind < wptr - retired ? wptr - behind : retired;
}

// sdma_v6_0_ring_set_wptr for the submit path: the doorbell path stores the
// 64-bit shadow atomically (atomic64_set), flushes HDP and rings WDOORBELL64;
// the register path writes RB_WPTR then RB_WPTR_HI.
Sdma6Status Sdma6::publish_wptr(uint32_t instance) noexcept {
    const uint64_t wptr = report_.wptr_bytes[instance];

    if (report_.doorbell_mode &&
        !wptr_store_.store(wptr_store_.context,
                           writeback_cpu_ + writeback_offset(instance, wptr_slot), wptr)) {
        return quarantine(Sdma6Status::invalid_memory, instance);
    }

    if (!nbio_->flush_hdp()) {
        return quarantine(Sdma6Status::mmio_failure, instance);
    }

    if (report_.doorbell_mode) {
        return doorbells_.write64(doorbells_.context, doorbell_index(instance), wptr) ?
            Sdma6Status::started : quarantine(Sdma6Status::doorbell_failure, instance);
    }

    if (!write(instance, sdma_queue0_rb_wptr, static_cast<uint32_t>(wptr)) ||
        !write(instance, sdma_queue0_rb_wptr_hi, static_cast<uint32_t>(wptr >> 32))) {
        return quarantine(Sdma6Status::mmio_failure, instance);
    }

    return Sdma6Status::started;
}

Sdma6Status Sdma6::submit(const Sdma6Submission& submission, uint32_t& sequence) noexcept {
    sequence = 0;

    if (phase_ != Phase::started) {
        return phase_ == Phase::faulted ? Sdma6Status::quarantined :
               phase_ == Phase::stopped ? Sdma6Status::busy : Sdma6Status::unavailable;
    }

    if (!report_.submission_enabled) {
        return Sdma6Status::unavailable;
    }

    const uint32_t instance = submission.instance;

    if (instance >= sdma6_instance_count || !report_.submit_enabled[instance] ||
        submission.ib_dwords == 0 || submission.ib_dwords > sdma6_max_ib_dwords ||
        (submission.ib_dwords % sdma6_ib_dword_multiple) != 0) {
        return Sdma6Status::invalid_argument;
    }

    reclaim(instance, false);
    uint64_t ib = 0;

    if (!submit_ib_address(submission, ib)) {
        return Sdma6Status::invalid_memory;
    }

    const uint32_t ring = fence_ring_sdma0 + instance;
    FenceReservation reservation{};
    const FenceStatus reserved = fences_->begin_submission(ring, reservation);

    if (reserved == FenceStatus::busy) {
        return Sdma6Status::ring_full;
    }

    if (reserved != FenceStatus::ready) {
        return reserved == FenceStatus::ring_hung ? Sdma6Status::ring_hung : Sdma6Status::unavailable;
    }

    const uint64_t start = report_.wptr_bytes[instance] / 4;
    const uint64_t consumed = consumed_position(instance, reservation.retired_position);
    const uint64_t used = start - consumed;
    const uint32_t ring_dwords = config_.ring_bytes / 4;
    report_.consumed_position[instance] = consumed;
    report_.free_dwords[instance] = used < ring_dwords ? static_cast<uint32_t>(ring_dwords - used) : 0;

    if (used + sdma6_submit_alloc_dwords >= ring_dwords) {
        return Sdma6Status::ring_full;
    }

    IbLease* lease = nullptr;

    for (auto& candidate : leases_[instance]) {
        if (!candidate.used) {
            lease = &candidate;
            break;
        }
    }

    if (lease == nullptr) {
        return Sdma6Status::ring_full;
    }

    if (manager_->retain(submission.ib, memory::Access::read) != memory::Status::success) {
        return Sdma6Status::invalid_memory;
    }

    uint64_t dwords = start;

    if (!emit_ib_frame(instance, dwords, ib, submission.ib_dwords, report_.submit_fence_mc[instance],
                       report_.submit_cond_exec_mc[instance], reservation.sequence) ||
        dwords - start > sdma6_submit_alloc_dwords) {
        manager_->unretain(submission.ib);
        fences_->mark_hung(ring, FenceHangReason::submit_failure);
        return quarantine(Sdma6Status::invalid_memory, instance);
    }

    const FenceStatus published = fences_->publish_submission(ring, reservation.sequence, dwords);

    if (published != FenceStatus::pending) {
        // amdgpu_ring_undo: the engine was not told; WPTR stays.
        manager_->unretain(submission.ib);
        return published == FenceStatus::ring_hung ? Sdma6Status::ring_hung :
                                                     quarantine(Sdma6Status::invalid_argument, instance);
    }

    *lease = {true, reservation.sequence, submission.ib};
    ++report_.ib_leases[instance];
    report_.wptr_bytes[instance] = dwords * 4;
    report_.fence_seq[instance] = reservation.sequence;
    ++report_.submissions[instance];
    sequence = reservation.sequence;
    const Sdma6Status status = publish_wptr(instance);

    if (status != Sdma6Status::started) {
        fences_->mark_hung(ring, FenceHangReason::submit_failure);
    }

    return status;
}

Sdma6Status Sdma6::retire() noexcept {
    if (!report_.submission_enabled) {
        return Sdma6Status::unavailable;
    }

    for (uint32_t instance = 0; instance < sdma6_instance_count; ++instance) {
        reclaim(instance, false);
    }

    return phase_ == Phase::started ? Sdma6Status::started : report_.status;
}

Sdma6Status Sdma6::disable_submission() noexcept {
    if (!report_.submission_enabled) {
        return Sdma6Status::unavailable;
    }

    Sdma6Status status = Sdma6Status::stopped;

    for (uint32_t instance = 0; instance < sdma6_instance_count; ++instance) {
        if (!report_.submit_enabled[instance]) {
            continue;
        }

        reclaim(instance, false);
        const FenceStatus detached = fences_->detach_ring(fence_ring_sdma0 + instance, false);

        if (detached == FenceStatus::detached) {
            reclaim(instance, true);
            report_.submit_enabled[instance] = false;
        } else if (detached == FenceStatus::ring_hung) {
            status = Sdma6Status::ring_hung;
        } else if (status != Sdma6Status::ring_hung) {
            status = Sdma6Status::busy;
        }
    }

    if (status != Sdma6Status::stopped) {
        return status;
    }

    fences_ = nullptr;
    wptr_store_ = {};
    report_.submission_enabled = false;
    return phase_ == Phase::started ? Sdma6Status::started : report_.status;
}

} // namespace rx7900::hardware

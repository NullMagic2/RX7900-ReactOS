/*
Module: rx7900.hardware.irq_service
Purpose: Claim, acknowledge, decode and route IH6 vectors; enable Linux's
late-init interrupt sources.
Created: 2026-10-21
Architecture: Bounded ISR/DPC protocol over the Ih6 public API; register MMIO
through RegisterIo; the only outward call is the optional drain consumer (the
fence manager), made after dispatch while the drain is still exclusive.
Copyright 2014, 2018, 2021, 2022 Advanced Micro Devices, Inc.
Adapted Linux v6.12 amdgpu_irq.c, amdgpu_ih.c, ih_v6_0.c, gfx_v11_0.c, sdma_v6_0.c,
gmc_v11_0.c, amdgpu_gmc.c, gfxhub_v3_0.c, mmhub_v3_0.c, amdgpu_fence.c, smu_v13_0.c,
smu_cmn.h, amdgpu_mes_ctx.h, soc15_ih_clientid.h, irqsrcs_gfx_11_0_0.h,
irqsrcs_vmc_1_0.h, irqsrcs_nbif_7_4.h, gc_11_0_0_offset.h/_sh_mask.h,
mmhub_3_0_0_offset.h/_sh_mask.h and mp_13_0_0_offset.h/_sh_mask.h.
Notice and provenance: licenses/AMDGPU-SDMA.txt.
*/
#include "irq_service.hpp"

#include "vmhub_layout.hpp"

namespace rx7900::hardware {
namespace {

// soc15_ih_clientid.h, soc21 client ids.
constexpr uint32_t client_ih = 0x00;
constexpr uint32_t client_bif = 0x03;
constexpr uint32_t client_gfx = 0x0a;
constexpr uint32_t client_thm = 0x0f;
constexpr uint32_t client_vmc = 0x12;
constexpr uint32_t client_grbm_cp = 0x14;
constexpr uint32_t client_rom_smuio = 0x16;
constexpr uint32_t client_df = 0x17;
constexpr uint32_t client_mp1 = 0x1f;

// irqsrcs_gfx_11_0_0.h, irqsrcs_vmc_1_0.h, irqsrcs_nbif_7_4.h, smu_v13_0.c, smu_cmn.h.
constexpr uint32_t source_self_interrupt = 0;
constexpr uint32_t source_utcl2_fault = 0;   // GFX_11_0_0__SRCID__UTCL2_FAULT
constexpr uint32_t source_vm_fault = 0;      // VMC_1_0__SRCID__VM_FAULT
constexpr uint32_t source_sdma_trap = 49;
constexpr uint32_t source_rlc_gc_fed = 128;
constexpr uint32_t source_cp_eop = 181;
constexpr uint32_t source_cp_bad_opcode = 183;
constexpr uint32_t source_cp_priv_reg = 184;
constexpr uint32_t source_cp_priv_instr = 185;
constexpr uint32_t source_thm_l2h = 0;
constexpr uint32_t source_thm_h2l = 1;
constexpr uint32_t source_smuio_gpio19 = 83;
constexpr uint32_t source_smu_to_driver = 0xfe;
constexpr uint32_t source_df_ecc = 0;
constexpr uint32_t source_athub_err_event = 0x61;

// amdgpu_mes_ctx.h.
constexpr uint32_t fence_mes_queue_flag = 0x1000000u;
constexpr uint32_t fence_mes_queue_id_mask = fence_mes_queue_flag - 1;

// gc_11_0_0_offset.h (BASE_IDX 0) and sdma_v6_0.c SDMA1_REG_OFFSET.
constexpr uint32_t cp_int_cntl_ring0 = 0x1e0a;
constexpr uint32_t cp_me1_pipe0_int_cntl = 0x1e25;
constexpr uint32_t sdma0_cntl = 0x001c;
constexpr uint32_t sdma1_reg_offset = 0x600;
// mp_13_0_0_offset.h (BASE_IDX 0).
constexpr uint32_t mp1_smn_ih_sw_int_ctrl = 0x02c3;

// gc_11_0_0_sh_mask.h: CP_INT_CNTL_RING0 and CP_ME1_PIPE0_INT_CNTL share these.
constexpr uint32_t cp_priv_instr_int_enable = 0x00400000u;
constexpr uint32_t cp_priv_reg_int_enable = 0x00800000u;
constexpr uint32_t cp_opcode_error_int_enable = 0x01000000u;
constexpr uint32_t cp_time_stamp_int_enable = 0x04000000u;
constexpr uint32_t cp_generic0_int_enable = 0x80000000u;
constexpr uint32_t cp_eop_int_enable = cp_time_stamp_int_enable | cp_generic0_int_enable;
constexpr uint32_t sdma_cntl_trap_enable = 0x00000001u;
// hub->vm_cntx_cntl_vm_fault: RANGE, DUMMY_PAGE, PDE0, VALID, READ, WRITE and
// EXECUTE _PROTECTION_FAULT_ENABLE_INTERRUPT (identical in MMVM and GCVM).
constexpr uint32_t vm_fault_interrupts = 0x00000200u | 0x00000800u | 0x00002000u |
                                         0x00008000u | 0x00020000u | 0x00080000u |
                                         0x00200000u;
constexpr uint32_t fault_cntl_clear_status_addr = 0x00000001u;
constexpr uint32_t mp1_sw_int_ack = 0x00000100u;

constexpr uint32_t gfx_pipe_mask = (1u << irq_gfx_pipes) - 1;
constexpr uint32_t compute_pipe_mask = (1u << irq_mec_pipes) - 1;
constexpr uint32_t sdma_instance_mask = (1u << irq_sdma_instances) - 1;

uint32_t min_u32(uint32_t a, uint32_t b) noexcept {
    return a < b ? a : b;
}

const VmHubLayout& hub_layout(VmHub hub) noexcept {
    return hub == VmHub::mmhub ? mmhub_3_0_0_layout : gfxhub_3_0_layout;
}

} // namespace

DiscoveryStatus validate_irq_discovery(ByteView binary, uint32_t aperture,
                                       IrqRegisters& output) noexcept {
    output = {};
    GmcRegisters gmc;
    SmuRegisters smu;
    auto status = validate_gmc_discovery(binary, aperture, gmc);

    if (status != DiscoveryStatus::validated) {
        return status;
    }

    status = validate_smu_discovery(binary, aperture, smu);

    if (status != DiscoveryStatus::validated) {
        return status;
    }

    const uint32_t mmhub_top = mmhub_3_0_0_layout.context0_control + irq_vm_contexts - 1;
    const uint32_t gfxhub_top = gfxhub_3_0_layout.context0_control + irq_vm_contexts - 1;
    const uint32_t gc_top = cp_me1_pipe0_int_cntl + irq_mec_pipes - 1;
    uint32_t offset = 0;

    if (!discovery_register_offset(gmc.base(VmHub::mmhub), mmhub_top, aperture, offset) ||
        !discovery_register_offset(gmc.base(VmHub::gfxhub), gfxhub_top, aperture, offset) ||
        !discovery_register_offset(gmc.base(VmHub::gfxhub), gc_top, aperture, offset) ||
        !discovery_register_offset(smu.segment0_dword_base(), mp1_smn_ih_sw_int_ctrl, aperture,
                                   offset)) {
        return DiscoveryStatus::invalid_argument;
    }

    output.gmc_ = gmc;
    output.smu_ = smu;
    output.aperture_bytes_ = aperture;
    return DiscoveryStatus::validated;
}

IrqStatus IrqService::fail(IrqStatus status) noexcept {
    report_.status = status;
    return status;
}

IrqStatus IrqService::attach(Ih6& ih, RegisterIo io, IrqRegisters registers) noexcept {
    if (phase_ != Phase::detached) {
        return IrqStatus::busy;
    }

    if (!registers.valid()) {
        return fail(IrqStatus::unsupported_ip);
    }

    if (!register_io_valid(io) || io.aperture_bytes != registers.aperture_bytes()) {
        return fail(IrqStatus::invalid_argument);
    }

    if (!bind_rings(ih)) {
        return fail(IrqStatus::unavailable);
    }

    io_ = io;
    registers_ = registers;
    report_ = {};
    vm_faults_ = IrqVmFaultLog();
    lost_unconsumed_ = false;
    phase_ = Phase::attached;
    report_.status = IrqStatus::attached;
    return report_.status;
}

// The enabled IH owner's two rings, power-of-two sized: views, mask, backlog
// bound and the ring 0 RPTR; nothing else changes on failure.
bool IrqService::bind_rings(Ih6& ih) noexcept {
    const Ih6Report ih_report = ih.report();
    const uint32_t bytes = ih_report.ring_bytes;
    ByteView ring0(nullptr, 0);
    ByteView ring1(nullptr, 0);

    if (!ih_report.enabled || ih_report.status != Ih6Status::enabled ||
        bytes < ih6_minimum_ring_bytes || bytes > ih6_maximum_ring_bytes ||
        (bytes & (bytes - 1)) != 0 || !ih.ring_view(Ih6Ring::primary, ring0) ||
        !ih.ring_view(Ih6Ring::page_fault, ring1) || ring0.size() != bytes ||
        ring1.size() != bytes) {
        return false;
    }

    ih_ = &ih;
    ring_mask_ = bytes - 1;
    // Half the ring keeps acknowledged-but-unread vectors clear of new writes.
    backlog_limit_ = bytes / 2;
    pending_start_ = ih.rptr(Ih6Ring::primary);
    return true;
}

IrqStatus IrqService::suspend() noexcept {
    if (phase_ != Phase::attached) {
        return phase_ == Phase::faulted ? IrqStatus::quarantined : IrqStatus::unavailable;
    }

    if (armed_ || drain_active_ || report_.pending_bytes != 0) {
        return IrqStatus::busy;
    }

    ring_views_[0] = ByteView(nullptr, 0);
    ring_views_[1] = ByteView(nullptr, 0);
    ring1_update_ = false;
    ring1_more_ = false;
    phase_ = Phase::suspended;
    report_.status = IrqStatus::suspended;
    return report_.status;
}

IrqStatus IrqService::resume(Ih6& ih) noexcept {
    if (phase_ != Phase::suspended) {
        return phase_ == Phase::faulted ? IrqStatus::quarantined : IrqStatus::busy;
    }

    if (ih_ != &ih || !bind_rings(ih)) {
        return IrqStatus::unavailable;
    }

    // The ISR's discard generation advances: no batch decoded before the
    // suspend can be committed against the zeroed rings.
    ++generation_;
    report_.pending_bytes = 0;
    lost_unconsumed_ = false;
    // Power loss cleared every source enable (amdgpu_irq_gpu_reset_resume_helper
    // re-applies them on Linux); enable_sources programs them again.
    report_.sources_enabled = false;
    phase_ = Phase::attached;
    report_.status = IrqStatus::attached;
    return report_.status;
}

void IrqService::arm_locked() noexcept {
    if (phase_ == Phase::attached) {
        armed_ = true;
        report_.armed = true;
    }
}

void IrqService::disarm_locked() noexcept {
    armed_ = false;
    report_.armed = false;
}

bool IrqService::modify(uint32_t base, uint32_t reg, uint32_t set, uint32_t& written) noexcept {
    uint32_t value = 0;

    // All-ones is a dead or unmapped device; never write it back.
    if (!read_ip_register(io_, base, reg, value) || value == UINT32_MAX) {
        return false;
    }

    value |= set;

    if (!write_ip_register(io_, base, reg, value)) {
        return false;
    }

    written = value;
    return true;
}

IrqStatus IrqService::enable_sources(IrqSourcePlan plan) noexcept {
    if (phase_ != Phase::attached) {
        return phase_ == Phase::faulted ? IrqStatus::quarantined : IrqStatus::unavailable;
    }

    if (report_.sources_enabled) {
        return IrqStatus::busy;
    }

    if ((plan.gfx_pipes & ~gfx_pipe_mask) != 0 || (plan.compute_pipes & ~compute_pipe_mask) != 0 ||
        (plan.sdma_instances & ~sdma_instance_mask) != 0) {
        return IrqStatus::invalid_argument;
    }

    const uint32_t gc = registers_.gc_base();
    bool ok = true;

    // amdgpu_fence_driver_hw_init: gfx ring pipes (gfx_v11_0_set_gfx_eop_interrupt_state),
    // compute ring pipes (set_compute_eop_interrupt_state, MEC1 only), SDMA
    // instances (sdma_v6_0_set_trap_irq_state). The first get of a type programs it.
    for (uint32_t pipe = 0; ok && pipe < irq_gfx_pipes; ++pipe) {
        if ((plan.gfx_pipes & (1u << pipe)) != 0) {
            ok = modify(gc, cp_int_cntl_ring0 + pipe, cp_eop_int_enable,
                        report_.cp_int_cntl_ring[pipe]);
        }
    }

    for (uint32_t pipe = 0; ok && pipe < irq_mec_pipes; ++pipe) {
        if ((plan.compute_pipes & (1u << pipe)) != 0) {
            ok = modify(gc, cp_me1_pipe0_int_cntl + pipe, cp_eop_int_enable,
                        report_.cp_me1_pipe_int_cntl[pipe]);
        }
    }

    for (uint32_t instance = 0; ok && instance < irq_sdma_instances; ++instance) {
        if ((plan.sdma_instances & (1u << instance)) != 0) {
            ok = modify(gc, sdma0_cntl + instance * sdma1_reg_offset, sdma_cntl_trap_enable,
                        report_.sdma_cntl[instance]);
        }
    }

    // gmc_v11_0_late_init: amdgpu_gmc_set_vm_fault_masks(MMHUB0, true), then
    // GFXHUB0 (not in s0ix), every context 0..15 at ctx_distance.
    const VmHub hubs[2]{VmHub::mmhub, VmHub::gfxhub};

    for (uint32_t index = 0; ok && index < 2; ++index) {
        const VmHubLayout& layout = hub_layout(hubs[index]);
        const uint32_t base = index == 0 ? registers_.mmhub_base() : gc;
        const uint32_t distance = layout.context1_control - layout.context0_control;

        for (uint32_t context = 0; ok && context < irq_vm_contexts; ++context) {
            ok = modify(base, layout.context0_control + context * distance, vm_fault_interrupts,
                        report_.vm_context_cntl[index][context]);
        }
    }

    // gfx_v11_0_late_init: priv_reg, priv_inst, bad_op. GC 11.0.0 has one ME0
    // pipe (num_pipe_per_me 1) and amdgpu owns MEC1 pipes 0..3; PRIV_INSTR has
    // no MEC field. Each get is one read-modify-write pass, as in Linux.
    ok = ok && modify(gc, cp_int_cntl_ring0, cp_priv_reg_int_enable, report_.cp_int_cntl_ring[0]);

    for (uint32_t pipe = 0; ok && pipe < irq_mec_pipes; ++pipe) {
        ok = modify(gc, cp_me1_pipe0_int_cntl + pipe, cp_priv_reg_int_enable,
                    report_.cp_me1_pipe_int_cntl[pipe]);
    }

    ok = ok && modify(gc, cp_int_cntl_ring0, cp_priv_instr_int_enable, report_.cp_int_cntl_ring[0]);
    ok = ok && modify(gc, cp_int_cntl_ring0, cp_opcode_error_int_enable, report_.cp_int_cntl_ring[0]);

    for (uint32_t pipe = 0; ok && pipe < irq_mec_pipes; ++pipe) {
        ok = modify(gc, cp_me1_pipe0_int_cntl + pipe, cp_opcode_error_int_enable,
                    report_.cp_me1_pipe_int_cntl[pipe]);
    }

    if (!ok) {
        // Some sources may already be live; servicing continues, the state is reported.
        return fail(IrqStatus::mmio_failure);
    }

    report_.sources_enabled = true;
    report_.status = IrqStatus::enabled;
    return report_.status;
}

void IrqService::discard_oldest_locked(uint32_t bytes) noexcept {
    if (bytes == 0) {
        return;
    }

    pending_start_ = (pending_start_ + bytes) & ring_mask_;
    report_.pending_bytes -= bytes;
    report_.entries_lost += bytes / ih6_entry_bytes;
    report_.events_lost = true;
    lost_unconsumed_ = true;
    ++generation_;
}

// amdgpu_ih_process for ring 0 up to the RPTR update; decoding is deferred.
bool IrqService::isr() noexcept {
    ++report_.isr_calls;

    if (!armed_) {
        return false;
    }

    bool claimed = false;

    for (uint32_t pass = 0; pass < irq_isr_max_passes; ++pass) {
        uint32_t wptr = 0;
        const Ih6Status status = ih_->read_wptr(Ih6Ring::primary, wptr);

        if (status != Ih6Status::enabled && status != Ih6Status::overflow) {
            // The IH owner is quarantined or disabled; RPTR can no longer be moved.
            phase_ = Phase::faulted;
            armed_ = false;
            report_.armed = false;
            report_.status = IrqStatus::quarantined;
            break;
        }

        const uint32_t rptr = ih_->rptr(Ih6Ring::primary);
        const bool contiguous =
            ((pending_start_ + report_.pending_bytes) & ring_mask_) == rptr;

        if (status == Ih6Status::overflow || !contiguous) {
            // Ih6 resumed at wptr + 32: every unread vector may be overwritten.
            if (status == Ih6Status::overflow) {
                ++report_.overflows;
            }

            report_.entries_lost += report_.pending_bytes / ih6_entry_bytes;
            report_.pending_bytes = 0;
            report_.events_lost = true;
            lost_unconsumed_ = true;
            pending_start_ = rptr;
            ++generation_;
        } else if (wptr == rptr) {
            break;
        }

        if ((wptr % ih6_entry_bytes) != 0) {
            phase_ = Phase::faulted;
            armed_ = false;
            report_.armed = false;
            report_.status = IrqStatus::quarantined;
            break;
        }

        report_.pending_bytes += (wptr - rptr) & ring_mask_;

        if (report_.pending_bytes > backlog_limit_) {
            discard_oldest_locked(report_.pending_bytes - backlog_limit_);
        }

        claimed = true;

        if (ih_->set_rptr(Ih6Ring::primary, wptr) != Ih6Status::enabled) {
            phase_ = Phase::faulted;
            armed_ = false;
            report_.armed = false;
            report_.status = IrqStatus::quarantined;
            break;
        }
    }

    if (claimed) {
        ++report_.isr_claimed;
    }

    return claimed;
}

bool IrqService::collect_trampoline(void* argument) noexcept {
    auto& call = *static_cast<LockedCall*>(argument);
    return call.self->collect_locked(call);
}

bool IrqService::commit_trampoline(void* argument) noexcept {
    auto& call = *static_cast<LockedCall*>(argument);
    return call.self->commit_locked(call);
}

bool IrqService::finish_trampoline(void* argument) noexcept {
    auto& call = *static_cast<LockedCall*>(argument);
    return call.self->finish_locked(call);
}

bool IrqService::collect_locked(LockedCall& call) noexcept {
    if (phase_ != Phase::attached) {
        call.status = phase_ == Phase::faulted ? IrqStatus::quarantined : IrqStatus::unavailable;
        return false;
    }

    if (drain_active_) {
        call.status = IrqStatus::busy;
        return false;
    }

    ByteView ring0(nullptr, 0);
    ByteView ring1(nullptr, 0);

    if (!ih_->ring_view(Ih6Ring::primary, ring0) || !ih_->ring_view(Ih6Ring::page_fault, ring1)) {
        call.status = IrqStatus::unavailable;
        return false;
    }

    drain_active_ = true;
    hardware_access_ = armed_;
    ring_views_[0] = ring0;
    ring_views_[1] = ring1;
    Batch& primary = batches_[0];
    primary.start = pending_start_;
    primary.count = min_u32(report_.pending_bytes / ih6_entry_bytes, irq_drain_batch_entries);
    primary.generation = generation_;

    // ih_v6_0_self_irq stores the ring 1 WPTR; amdgpu_irq_handle_ih1 then runs
    // amdgpu_ih_process on ring 1, here inside the same bounded drain.
    if (ring1_update_) {
        ih_->record_self_interrupt(static_cast<uint32_t>(Ih6Ring::page_fault), ring1_update_wptr_);
        ring1_update_ = false;
    }

    Batch& faults = batches_[1];
    faults.count = 0;
    faults.start = 0;
    faults.generation = 0;
    ring1_more_ = false;
    uint32_t wptr = 0;
    const Ih6Status status = ih_->read_wptr(Ih6Ring::page_fault, wptr);

    if (status == Ih6Status::overflow) {
        // Ring 1 drains on full (RB_FULL_DRAIN_ENABLE); the lost count is unknown.
        ++report_.overflows;
        report_.events_lost = true;
        lost_unconsumed_ = true;
    } else if (status != Ih6Status::enabled) {
        call.status = IrqStatus::unavailable;
        return true;
    }

    const uint32_t rptr = ih_->rptr(Ih6Ring::page_fault);
    const uint32_t available = ((wptr - rptr) & ring_mask_) / ih6_entry_bytes;
    faults.start = rptr;
    faults.count = min_u32(available, irq_drain_batch_entries);
    ring1_more_ = available > faults.count;
    call.status = IrqStatus::attached;
    return true;
}

bool IrqService::read_batch(ByteView ring, Batch& batch) const noexcept {
    for (uint32_t index = 0; index < batch.count; ++index) {
        const uint32_t offset = (batch.start + index * ih6_entry_bytes) & ring_mask_;

        // amdgpu_ih_decode_iv_helper field layout; entries never straddle the wrap.
        if (!sdma::decode_interrupt(ring, offset, batch.vectors[index])) {
            batch.count = index;
            return false;
        }
    }

    return true;
}

bool IrqService::commit_locked(LockedCall& call) noexcept {
    Batch& primary = batches_[0];

    // A changed generation means the ISR discarded (and counted) the oldest
    // vectors while they were decoded; the decoded copies may be torn.
    if (primary.generation == generation_ && primary.start == pending_start_) {
        const uint32_t bytes = primary.count * ih6_entry_bytes;
        pending_start_ = (pending_start_ + bytes) & ring_mask_;
        report_.pending_bytes -= bytes;
    } else {
        primary.count = 0;
    }

    // Losses up to this point are reported to this drain's consumer.
    call.events_lost = lost_unconsumed_;
    lost_unconsumed_ = false;
    Batch& faults = batches_[1];

    if (faults.count != 0 &&
        ih_->set_rptr(Ih6Ring::page_fault,
                      (faults.start + faults.count * ih6_entry_bytes) & ring_mask_) !=
            Ih6Status::enabled) {
        // Already decoded vectors are still dispatched, as Linux would.
        phase_ = Phase::faulted;
        armed_ = false;
        report_.armed = false;
        report_.status = IrqStatus::quarantined;
        call.status = IrqStatus::quarantined;
    }

    return true;
}

bool IrqService::finish_locked(LockedCall& call) noexcept {
    if (dispatch_ring1_update_) {
        ring1_update_ = true;
        ring1_update_wptr_ = dispatch_ring1_wptr_;
    }

    call.more_pending = phase_ == Phase::attached &&
                        (report_.pending_bytes != 0 || ring1_more_ || ring1_update_);
    drain_active_ = false;
    return true;
}

void IrqService::count(IrqSource source) noexcept {
    ++report_.sequence[static_cast<uint32_t>(source)];
}

// gfx_v11_0_eop_irq.
void IrqService::dispatch_eop(const sdma::InterruptVector& vector) noexcept {
    if ((vector.source_data[0] & fence_mes_queue_flag) != 0) {
        report_.last_mes_queue = vector.source_data[0] & fence_mes_queue_id_mask;
        count(IrqSource::mes_eop);
        return;
    }

    const uint32_t me = (vector.ring & 0x0c) >> 2;
    const uint32_t pipe = vector.ring & 0x03;
    const uint32_t queue = (vector.ring & 0x70) >> 4;

    if (me == 0) {
        ++report_.gfx_eop[pipe == 0 ? 0 : 1];
        count(IrqSource::gfx_eop);
    } else if (me <= irq_mec_count) {
        ++report_.compute_eop[me - 1][pipe][queue];
        count(IrqSource::compute_eop);
    } else {
        count(IrqSource::unhandled);
    }
}

// gfx_v11_0_handle_priv_fault: Linux faults the scheduler of the matching ring
// (BUG for an ME above 2); here every fault latches queue quarantine.
void IrqService::dispatch_cp_fault(const sdma::InterruptVector& vector, IrqSource source) noexcept {
    const uint32_t me = (vector.ring & 0x0c) >> 2;
    const uint32_t pipe = vector.ring & 0x03;
    const uint32_t queue = (vector.ring & 0x70) >> 4;
    count(source);
    report_.last_cp_fault = {source, me, pipe, queue, vector.vmid, vector.pasid,
                             report_.sequence[static_cast<uint32_t>(source)]};
    report_.queue_fault = true;

    if (me >= irq_cp_me_count) {
        ++report_.cp_fault_invalid_me;
        return;
    }

    ++report_.cp_fault_count[me][pipe][queue];
    report_.cp_fault_source[me][pipe][queue] = source;
}

// sdma_v6_0_process_trap_irq.
void IrqService::dispatch_sdma_trap(const sdma::InterruptVector& vector) noexcept {
    if ((vector.source_data[0] & fence_mes_queue_flag) != 0) {
        report_.last_mes_queue = vector.source_data[0] & fence_mes_queue_id_mask;
        count(IrqSource::sdma_mes_trap);
        return;
    }

    const uint32_t queue = vector.ring & 0x0f;
    const uint32_t instance = (vector.ring & 0xf0) >> 4;

    // Linux rejects an instance above 1 (-EINVAL) and ignores queues other than 0.
    if (instance >= irq_sdma_instances || queue != 0) {
        count(IrqSource::unhandled);
        return;
    }

    ++report_.sdma_trap[instance];
    count(IrqSource::sdma_trap);
}

// gmc_v11_0_process_interrupt (bare metal).
void IrqService::dispatch_vm_fault(const sdma::InterruptVector& vector,
                                   bool page_fault_ring) noexcept {
    const IrqSource source =
        vector.client == client_vmc ? IrqSource::mmhub_fault : IrqSource::gfxhub_fault;
    const VmHub hub = vector.client == client_vmc ? VmHub::mmhub : VmHub::gfxhub;
    const VmHubLayout& layout = hub_layout(hub);
    const uint32_t base = hub == VmHub::mmhub ? registers_.mmhub_base() : registers_.gc_base();
    IrqVmFault fault{};
    fault.address = (static_cast<uint64_t>(vector.source_data[0]) << 12) |
                    (static_cast<uint64_t>(vector.source_data[1] & 0xf) << 44);
    fault.client = vector.client;
    fault.source = vector.source;
    fault.ring_id = vector.ring;
    fault.vmid = vector.vmid;
    fault.pasid = vector.pasid;
    fault.register_hub = hub;
    fault.cache_hub = vector.vmid_source ? VmHub::mmhub : VmHub::gfxhub;
    fault.page_fault_ring = page_fault_ring;

    if (hardware_access_) {
        uint32_t status = 0;
        uint32_t written = 0;
        bool read = true;

        // vmid_src GFXHUB(0): a dummy read lets the fast GRBM interface update status.
        if (!vector.vmid_source) {
            read = read_ip_register(io_, base, layout.fault_status, status);
        }

        read = read && read_ip_register(io_, base, layout.fault_status, status);
        fault.status_read = read;
        fault.status = read ? status : 0;
        // WREG32_P(vm_l2_pro_fault_cntl, 1, ~1): set CLEAR_PROTECTION_FAULT_STATUS_ADDR.
        fault.status_cleared =
            modify(base, layout.fault_control, fault_cntl_clear_status_addr, written);
    }

    count(source);
    fault.sequence = report_.sequence[static_cast<uint32_t>(source)];
    report_.last_vm_fault = fault;
    report_.vm_fault = true;
    ++vm_faults_.total;
    vm_faults_.entries[(vm_faults_.total - 1) % irq_vm_fault_log_entries] = fault;
}

// smu_v13_0_irq_process, MP1 branch: acknowledge, then record the context id
// for the PASSIVE_LEVEL owner (AC/DC acknowledgement message, throttling log,
// THM limit changes for fan abnormal/recovery).
void IrqService::dispatch_smu(const sdma::InterruptVector& vector) noexcept {
    uint32_t written = 0;

    if (!hardware_access_ ||
        !modify(registers_.mp1_base(), mp1_smn_ih_sw_int_ctrl, mp1_sw_int_ack, written)) {
        ++report_.smu_ack_failures;
    }

    const uint32_t context = vector.source_data[0];
    report_.smu_last_context = context;

    if (context < irq_smu_context_count) {
        ++report_.smu_context[context];
    }

    count(IrqSource::smu_to_driver);
}

// amdgpu_irq_dispatch with the sources registered on Navi31.
void IrqService::dispatch(const sdma::InterruptVector& vector, bool page_fault_ring) noexcept {
    if (vector.client >= irq_client_count) {
        count(IrqSource::invalid);
        return;
    }

    switch (vector.client) {
    case client_ih:
        if (vector.source == source_self_interrupt) {
            // ih_v6_0_self_irq: only ring id 1 carries a WPTR.
            if (vector.ring == static_cast<uint32_t>(Ih6Ring::page_fault)) {
                report_.ring1_wptr = vector.source_data[0];
                dispatch_ring1_update_ = true;
                dispatch_ring1_wptr_ = vector.source_data[0];
            }

            count(IrqSource::self_interrupt);
            return;
        }
        break;
    case client_grbm_cp:
        switch (vector.source) {
        case source_cp_eop: dispatch_eop(vector); return;
        case source_cp_priv_reg: dispatch_cp_fault(vector, IrqSource::cp_priv_reg); return;
        case source_cp_priv_instr: dispatch_cp_fault(vector, IrqSource::cp_priv_instr); return;
        case source_cp_bad_opcode: dispatch_cp_fault(vector, IrqSource::cp_bad_opcode); return;
        default: break;
        }
        break;
    case client_gfx:
        if (vector.source == source_sdma_trap) {
            dispatch_sdma_trap(vector);
            return;
        }

        if (vector.source == source_utcl2_fault) {
            dispatch_vm_fault(vector, page_fault_ring);
            return;
        }

        if (vector.source == source_rlc_gc_fed) {
            ++report_.ras_gc_fed;
            count(IrqSource::ras);
            return;
        }
        break;
    case client_vmc:
        if (vector.source == source_vm_fault) {
            dispatch_vm_fault(vector, page_fault_ring);
            return;
        }
        break;
    case client_thm:
        if (vector.source == source_thm_l2h) {
            report_.thermal_work = true;
            count(IrqSource::thermal_high);
            return;
        }

        if (vector.source == source_thm_h2l) {
            count(IrqSource::thermal_low);
            return;
        }
        break;
    case client_rom_smuio:
        if (vector.source == source_smuio_gpio19) {
            report_.hardware_ctf = true;
            count(IrqSource::thermal_ctf);
            return;
        }
        break;
    case client_mp1:
        if (vector.source == source_smu_to_driver) {
            dispatch_smu(vector);
            return;
        }
        break;
    case client_df:
        if (vector.source == source_df_ecc) {
            ++report_.ras_umc_ecc;
            count(IrqSource::ras);
            return;
        }
        break;
    case client_bif:
        if (vector.source == source_athub_err_event) {
            ++report_.ras_athub;
            count(IrqSource::ras);
            return;
        }
        break;
    default:
        break;
    }

    // Linux hands unhandled vectors to amdkfd; none exists here.
    ++report_.unknown_by_client[vector.client];
    count(IrqSource::unknown);
}

IrqDrain IrqService::drain(IrqSynchronize sync) noexcept {
    return drain(sync, {nullptr, nullptr});
}

IrqDrain IrqService::drain(IrqSynchronize sync, IrqConsumer consumer) noexcept {
    IrqDrain result{};

    if (sync.run == nullptr) {
        result.status = IrqStatus::invalid_argument;
        return result;
    }

    LockedCall call{this, IrqStatus::unavailable, false, false};

    if (!sync.run(sync.context, collect_trampoline, &call)) {
        result.status = call.status;
        return result;
    }

    dispatch_ring1_update_ = false;
    dispatch_ring1_wptr_ = 0;
    // Device-written ring bytes are read only inside the acknowledged (ring 0)
    // or unacknowledged-but-published (ring 1) spans recorded under the lock.
    read_batch(ring_views_[0], batches_[0]);
    read_batch(ring_views_[1], batches_[1]);
    sync.run(sync.context, commit_trampoline, &call);

    for (uint32_t ring = 0; ring < ih6_ring_count; ++ring) {
        const Batch& batch = batches_[ring];

        for (uint32_t index = 0; index < batch.count; ++index) {
            dispatch(batch.vectors[index], ring == static_cast<uint32_t>(Ih6Ring::page_fault));
        }

        report_.dispatched[ring] += batch.count;
        result.dispatched[ring] = batch.count;
    }

    // Still the only active drain: no dispatch can change the view meanwhile.
    if (consumer.consume != nullptr) {
        consumer.consume(consumer.context, {&report_, &vm_faults_, call.events_lost});
    }

    sync.run(sync.context, finish_trampoline, &call);
    result.status = call.status;
    result.more_pending = call.more_pending;
    return result;
}

} // namespace rx7900::hardware

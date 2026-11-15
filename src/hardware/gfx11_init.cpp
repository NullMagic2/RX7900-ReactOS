/*
Module: rx7900.hardware.gfx11_init
Purpose: Execute the GC11 RLC/CP register sequence of gfx_v11_0_hw_init (PSP load).
Created: 2026-10-21
Architecture: Bounded register protocol; clear-state data from gfx11_clear_state.
Copyright 2021 Advanced Micro Devices, Inc. Adapted Linux v6.12 gfx_v11_0.c
(hw_init, wait_for_rlc_autoload_complete, config_gfx_rs64, init_golden_registers,
constants_init, init_compute_vmid, init_gds_vmid, rlc_resume, init_csb,
get_csb_size/buffer, rlc_enable_srm, cp_resume, cp_set_doorbell_range,
cp_compute_enable, cp_gfx_enable, enable_gui_idle_interrupt), soc21.c
(grbm_select), soc15.c (program_register_sequence), gmc_v11_0.c (apertures,
first_kfd_vmid) and GC11.0.0 headers. Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#include "gfx11_init.hpp"
#include "gfx11_clear_state.hpp"
#include "pm4.hpp"

namespace rx7900::hardware {
namespace {

// Register index and BASE_IDX segment from gc_11_0_0_offset.h.
struct Reg final {
    size_t segment;
    uint32_t index;
};

constexpr Reg cp_stat{0, 0xf40};
constexpr Reg rlcs_bootload_status{1, 0x4e82};
constexpr Reg gb_addr_config{0, 0x13de};
constexpr Reg grbm_gfx_cntl{1, 0x900};
constexpr Reg pfp_start{0, 0x1e44};
constexpr Reg pfp_start_hi{0, 0x1e59};
constexpr Reg me_cntl{1, 0x803};
constexpr Reg me_start{0, 0x1e45};
constexpr Reg me_start_hi{0, 0x1e79};
constexpr Reg mec_start{1, 0x2900};
constexpr Reg mec_start_hi{1, 0x2938};
constexpr Reg mec_rs64_cntl{1, 0x2904};
constexpr Reg tcp_cntl{1, 0x19a2};
constexpr Reg grbm_cntl{0, 0xda0};
constexpr Reg sh_mem_config{1, 0x9e4};
constexpr Reg sh_mem_bases{1, 0x9e3};
constexpr Reg spi_gdbg_per_vmid_cntl{0, 0x1f72};
constexpr Reg gds_vmid0_base{0, 0x20a0};
constexpr Reg gds_vmid0_size{0, 0x20a1};
constexpr Reg gds_gws_vmid0{0, 0x20c0};
constexpr Reg gds_oa_vmid0{0, 0x20d0};
constexpr Reg rlc_csib_addr_hi{1, 0x988};
constexpr Reg rlc_csib_addr_lo{1, 0x987};
constexpr Reg rlc_csib_length{1, 0x989};
constexpr Reg rlc_srm_cntl{1, 0x4c80};
constexpr Reg cp_int_cntl_ring0{0, 0x1e0a};
constexpr Reg cp_int_cntl_ring1{0, 0x1e0b};
constexpr Reg cp_rb_doorbell_lower{0, 0x1dfa};
constexpr Reg cp_rb_doorbell_upper{0, 0x1dfb};
constexpr Reg cp_mec_doorbell_lower{0, 0x1dfc};
constexpr Reg cp_mec_doorbell_upper{0, 0x1dfd};
constexpr Reg imu_scratch0{1, 0x4068};
// PA_SC_TILE_STEERING_OVERRIDE (segment 1) for the clear-state context offset.
constexpr uint32_t pa_sc_tile_steering_override = 0xd7;

constexpr uint32_t bootload_complete = 0x80000000u;
constexpr uint32_t pfp_pipe_resets = 0x40000u | 0x80000u;
constexpr uint32_t me_pipe_resets = 0x100000u | 0x200000u;
constexpr uint32_t me_halt = 0x10000000u;
constexpr uint32_t pfp_halt = 0x4000000u;
constexpr uint32_t mec_invalidate_icache = 0x10u;
constexpr uint32_t mec_pipe_resets = 0xf0000u;
constexpr uint32_t mec_pipe_active = 0x3c000000u;
constexpr uint32_t mec_halt = 0x40000000u;
constexpr uint32_t tcp_cntl_golden = 0x20000000u;
constexpr uint32_t grbm_read_timeout = 0xffu;
// DEFAULT_SH_MEM_CONFIG: ADDRESS_MODE_64 (0), ALIGNMENT_MODE_UNALIGNED (3) << 2,
// INITIAL_INST_PREFETCH 3 << 14.
constexpr uint32_t default_sh_mem_config = (3u << 2) | (3u << 14);
// gmc_v11_0 apertures >> 48: private 0x1000..., shared 0x2000...
constexpr uint32_t gfx_sh_mem_bases = (0x2000u << 16) | 0x1000u;
// gfx_v11_0_init_compute_vmid: LDS_APP_BASE 1 shared, SCRATCH_APP_BASE 2 private.
constexpr uint32_t compute_sh_mem_bases = (1u << 16) | 2u;
constexpr uint32_t spi_trap_enable = 0x8u;
constexpr uint32_t vmid_count = 16;
constexpr uint32_t first_kfd_vmid = 8;
constexpr uint32_t srm_enable = 0x1u | 0x2u;
constexpr uint32_t gui_idle_interrupts = 0x40000u | 0x80000u | 0x100000u | 0x200000u;
constexpr uint32_t maximum_polls = 1000000;
constexpr uint32_t compute_enable_delay = 50;

uint32_t clear_state_dwords() noexcept {
    // begin(2) + context control(3) + per extent (2 + count) + steering(3) +
    // end(2) + clear state(2), exactly gfx_v11_0_get_csb_size.
    uint32_t count = 2 + 3 + 3 + 2 + 2;

    for (const auto& extent : gfx11_clear_state::context_extents) {
        count += 2 + extent.count;
    }

    return count;
}

} // namespace

uint32_t Gfx11Init::clear_state_bytes() noexcept {
    return clear_state_dwords() * 4;
}

Gfx11Status Gfx11Init::fail(Gfx11Status status) noexcept {
    report_.status = status;

    if (status == Gfx11Status::mmio_failure || status == Gfx11Status::timeout) {
        faulted_ = true;
    }

    return status;
}

bool Gfx11Init::read(size_t segment, uint32_t reg, uint32_t& value) noexcept {
    return read_ip_register(io_, registers_.segment(segment), reg, value) && value != UINT32_MAX;
}

bool Gfx11Init::write(size_t segment, uint32_t reg, uint32_t value) noexcept {
    return write_ip_register(io_, registers_.segment(segment), reg, value);
}

bool Gfx11Init::modify(size_t segment, uint32_t reg, uint32_t clear, uint32_t set) noexcept {
    uint32_t value = 0;
    return read(segment, reg, value) && write(segment, reg, (value & ~clear) | set);
}

bool Gfx11Init::grbm_select(uint32_t me, uint32_t pipe, uint32_t queue, uint32_t vmid) noexcept {
    // soc21_grbm_select: PIPEID[1:0], MEID[3:2], VMID[7:4], QUEUEID[10:8].
    const uint32_t value = (pipe & 0x3u) | ((me & 0x3u) << 2) | ((vmid & 0xfu) << 4) |
                           ((queue & 0x7u) << 8);
    return write(grbm_gfx_cntl.segment, grbm_gfx_cntl.index, value);
}

Gfx11Status Gfx11Init::initialize(memory::GcRegisterBase registers, RegisterIo io, Nbio& nbio,
                                  VramLocation vram, Gfx11Policy policy,
                                  Gfx11Recovery recovery) noexcept {
    if (report_.memory_retained) {
        return Gfx11Status::busy;
    }

    report_ = {};
    initialized_ = false;
    faulted_ = false;

    if (!registers.valid()) {
        return fail(Gfx11Status::unsupported_ip);
    }

    uint32_t offset = 0;

    // Highest indices used per segment must fit the mapped BAR5 aperture.
    if (!register_io_valid(io) || io.aperture_bytes != registers.aperture_bytes() ||
        policy.poll_iterations == 0 || policy.poll_iterations > maximum_polls ||
        !discovery_register_offset(registers.segment(0), gds_oa_vmid0.index + vmid_count,
                                   io.aperture_bytes, offset) ||
        !discovery_register_offset(registers.segment(1), rlcs_bootload_status.index,
                                   io.aperture_bytes, offset)) {
        return fail(Gfx11Status::unavailable);
    }

    registers_ = registers;
    io_ = io;
    nbio_ = &nbio;
    vram_ = vram;
    policy_ = policy;
    recovery_ = recovery;
    initialized_ = true;
    report_.status = Gfx11Status::ready;
    return report_.status;
}

Gfx11Status Gfx11Init::wait_for_autoload() noexcept {
    if (!initialized_ || faulted_) {
        return fail(faulted_ ? Gfx11Status::quarantined : Gfx11Status::unavailable);
    }

    for (uint32_t poll = 0; poll < policy_.poll_iterations; ++poll) {
        if (!read(cp_stat.segment, cp_stat.index, report_.cp_status) ||
            !read(rlcs_bootload_status.segment, rlcs_bootload_status.index,
                  report_.bootload_status)) {
            return fail(Gfx11Status::mmio_failure);
        }

        if (report_.cp_status == 0 && (report_.bootload_status & bootload_complete) != 0) {
            report_.autoload_complete = true;
            report_.status = Gfx11Status::completed;
            return report_.status;
        }

        io_.delay_microseconds(io_.context, 1);
    }

    return fail(Gfx11Status::timeout);
}

Gfx11Status Gfx11Init::read_gb_addr_config() noexcept {
    if (!report_.autoload_complete || faulted_) {
        return fail(faulted_ ? Gfx11Status::quarantined : Gfx11Status::busy);
    }

    if (!read_ip_register(io_, registers_.segment(gb_addr_config.segment), gb_addr_config.index,
                          report_.gb_addr_config)) {
        return fail(Gfx11Status::mmio_failure);
    }

    report_.gb_addr_config_invalid = report_.gb_addr_config == 0;
    report_.status = Gfx11Status::completed;
    return report_.status;
}

Gfx11Status Gfx11Init::configure_rs64(Gfx11StartAddresses a) noexcept {
    if (!report_.autoload_complete || faulted_) {
        return fail(faulted_ ? Gfx11Status::quarantined : Gfx11Status::busy);
    }

    bool ok = true;

    for (uint32_t pipe = 0; ok && pipe < 2; ++pipe) {
        ok = grbm_select(0, pipe, 0, 0) &&
             write(pfp_start.segment, pfp_start.index, (a.pfp_high << 30) | (a.pfp_low >> 2)) &&
             write(pfp_start_hi.segment, pfp_start_hi.index, a.pfp_high >> 2);
    }

    uint32_t control = 0;
    ok = ok && grbm_select(0, 0, 0, 0) && read(me_cntl.segment, me_cntl.index, control) &&
         write(me_cntl.segment, me_cntl.index, control | pfp_pipe_resets) &&
         write(me_cntl.segment, me_cntl.index, control & ~pfp_pipe_resets);

    for (uint32_t pipe = 0; ok && pipe < 2; ++pipe) {
        ok = grbm_select(0, pipe, 0, 0) &&
             write(me_start.segment, me_start.index, (a.me_high << 30) | (a.me_low >> 2)) &&
             write(me_start_hi.segment, me_start_hi.index, a.me_high >> 2);
    }

    ok = ok && grbm_select(0, 0, 0, 0) && read(me_cntl.segment, me_cntl.index, control) &&
         write(me_cntl.segment, me_cntl.index, control | me_pipe_resets) &&
         write(me_cntl.segment, me_cntl.index, control & ~me_pipe_resets);

    for (uint32_t pipe = 0; ok && pipe < 4; ++pipe) {
        ok = grbm_select(1, pipe, 0, 0) &&
             write(mec_start.segment, mec_start.index, (a.mec_low >> 2) | (a.mec_high << 30)) &&
             write(mec_start_hi.segment, mec_start_hi.index, a.mec_high >> 2);
    }

    ok = ok && grbm_select(0, 0, 0, 0) && read(mec_rs64_cntl.segment, mec_rs64_cntl.index, control) &&
         write(mec_rs64_cntl.segment, mec_rs64_cntl.index, control | mec_pipe_resets) &&
         write(mec_rs64_cntl.segment, mec_rs64_cntl.index, control & ~mec_pipe_resets);

    if (!ok) {
        return fail(Gfx11Status::mmio_failure);
    }

    report_.rs64_configured = true;
    report_.status = Gfx11Status::completed;
    return report_.status;
}

Gfx11Status Gfx11Init::golden_registers() noexcept {
    if (!initialized_ || faulted_) {
        return fail(faulted_ ? Gfx11Status::quarantined : Gfx11Status::unavailable);
    }

    // soc15_program_register_sequence with and_mask != 0xffffffff.
    if (!modify(tcp_cntl.segment, tcp_cntl.index, tcp_cntl_golden, tcp_cntl_golden)) {
        return fail(Gfx11Status::mmio_failure);
    }

    report_.status = Gfx11Status::completed;
    return report_.status;
}

Gfx11Status Gfx11Init::constants_init() noexcept {
    if (!initialized_ || faulted_) {
        return fail(faulted_ ? Gfx11Status::quarantined : Gfx11Status::unavailable);
    }

    bool ok = modify(grbm_cntl.segment, grbm_cntl.index, grbm_read_timeout, grbm_read_timeout);

    // SH_MEM for every GFXHUB VMID (id_mgr num_ids 16).
    for (uint32_t vmid = 0; ok && vmid < vmid_count; ++vmid) {
        ok = grbm_select(0, 0, 0, vmid) &&
             write(sh_mem_config.segment, sh_mem_config.index, default_sh_mem_config) &&
             (vmid == 0 || write(sh_mem_bases.segment, sh_mem_bases.index, gfx_sh_mem_bases));
    }

    ok = ok && grbm_select(0, 0, 0, 0);

    // gfx_v11_0_init_compute_vmid for the KFD VMIDs 8..15.
    for (uint32_t vmid = first_kfd_vmid; ok && vmid < vmid_count; ++vmid) {
        ok = grbm_select(0, 0, 0, vmid) &&
             write(sh_mem_config.segment, sh_mem_config.index, default_sh_mem_config) &&
             write(sh_mem_bases.segment, sh_mem_bases.index, compute_sh_mem_bases) &&
             modify(spi_gdbg_per_vmid_cntl.segment, spi_gdbg_per_vmid_cntl.index, 0, spi_trap_enable);
    }

    ok = ok && grbm_select(0, 0, 0, 0);

    for (uint32_t vmid = first_kfd_vmid; ok && vmid < vmid_count; ++vmid) {
        ok = write(0, gds_vmid0_base.index + 2 * vmid, 0) && write(0, gds_vmid0_size.index + 2 * vmid, 0) &&
             write(0, gds_gws_vmid0.index + vmid, 0) && write(0, gds_oa_vmid0.index + vmid, 0);
    }

    // gfx_v11_0_init_gds_vmid for VMIDs 1..15.
    for (uint32_t vmid = 1; ok && vmid < vmid_count; ++vmid) {
        ok = write(0, gds_vmid0_base.index + 2 * vmid, 0) && write(0, gds_vmid0_size.index + 2 * vmid, 0) &&
             write(0, gds_gws_vmid0.index + vmid, 0) && write(0, gds_oa_vmid0.index + vmid, 0);
    }

    if (!ok) {
        return fail(Gfx11Status::mmio_failure);
    }

    report_.status = Gfx11Status::completed;
    return report_.status;
}

Gfx11Status Gfx11Init::rlc_resume(memory::Manager& manager, memory::Handle clear_state) noexcept {
    if (!report_.rs64_configured || faulted_ || report_.memory_retained) {
        return fail(faulted_ ? Gfx11Status::quarantined : Gfx11Status::busy);
    }

    memory::AllocationInfo info{};

    if (!retain_local_vram(manager, clear_state, vram_, clear_state_bytes(), memory::page_bytes,
                           true, info)) {
        return fail(Gfx11Status::invalid_memory);
    }

    manager_ = &manager;
    clear_state_ = clear_state;
    report_.memory_retained = true;
    report_.clear_state_mc = info.gpu.virtual_address;
    const sdma::CommandBuffer buffer(info.backing.cpu_bytes, clear_state_bytes());
    uint32_t at = 0;
    const auto put = [&](uint32_t value) noexcept {
        buffer.write_u32(at, value);
        at += 4;
    };

    // gfx_v11_0_get_csb_buffer.
    put(pm4::packet3(pm4::preamble_cntl, 0));
    put(pm4::preamble_begin_clear_state);
    put(pm4::packet3(pm4::context_control, 1));
    put(0x80000000u);
    put(0x80000000u);

    for (const auto& extent : gfx11_clear_state::context_extents) {
        put(pm4::packet3(pm4::set_context_reg, extent.count));
        put(extent.register_index - pm4::set_context_reg_start);

        for (uint32_t index = 0; index < extent.count; ++index) {
            put(extent.values[index]);
        }
    }

    // pa_sc_tile_steering_override is 0 in gfx_v11_0_constants_init.
    const uint32_t steering = registers_.segment(1) + pa_sc_tile_steering_override -
                              pm4::set_context_reg_start;
    put(pm4::packet3(pm4::set_context_reg, 1));
    put(steering);
    put(0);
    put(pm4::packet3(pm4::preamble_cntl, 0));
    put(pm4::preamble_end_clear_state);
    put(pm4::packet3(pm4::clear_state, 0));
    put(0);
    report_.clear_state_dwords = at / 4;

    const uint64_t address = info.gpu.virtual_address;

    if (!nbio_->flush_hdp() ||
        !write(rlc_csib_addr_hi.segment, rlc_csib_addr_hi.index, static_cast<uint32_t>(address >> 32)) ||
        !write(rlc_csib_addr_lo.segment, rlc_csib_addr_lo.index,
               static_cast<uint32_t>(address) & 0xfffffffcu) ||
        !write(rlc_csib_length.segment, rlc_csib_length.index, report_.clear_state_dwords) ||
        !modify(rlc_srm_cntl.segment, rlc_srm_cntl.index, 0, srm_enable)) {
        return fail(Gfx11Status::mmio_failure);
    }

    report_.rlc_resumed = true;
    report_.status = Gfx11Status::completed;
    return report_.status;
}

Gfx11Status Gfx11Init::cp_start() noexcept {
    if (!report_.rlc_resumed || faulted_) {
        return fail(faulted_ ? Gfx11Status::quarantined : Gfx11Status::busy);
    }

    // enable_gui_idle_interrupt(false) for ME0 pipes 0 and 1 (dGPU only).
    bool ok = modify(cp_int_cntl_ring0.segment, cp_int_cntl_ring0.index, gui_idle_interrupts, 0) &&
              modify(cp_int_cntl_ring1.segment, cp_int_cntl_ring1.index, gui_idle_interrupts, 0);

    // cp_set_doorbell_range (doorbell indices are 64-bit, in bytes).
    ok = ok && write(0, cp_rb_doorbell_lower.index, (soc21_doorbell::gfx_ring0 * 2) << 2) &&
         write(0, cp_rb_doorbell_upper.index, (soc21_doorbell::gfx_userqueue_end * 2) << 2) &&
         write(0, cp_mec_doorbell_lower.index, (soc21_doorbell::kiq * 2) << 2) &&
         write(0, cp_mec_doorbell_upper.index, (soc21_doorbell::userqueue_end * 2) << 2);

    // amdgpu_async_gfx_ring: RS64 compute enable, then GFX unhalt.
    uint32_t control = 0;
    ok = ok && read(mec_rs64_cntl.segment, mec_rs64_cntl.index, control) &&
         write(mec_rs64_cntl.segment, mec_rs64_cntl.index,
               (control & ~(mec_invalidate_icache | mec_pipe_resets | mec_halt)) | mec_pipe_active);

    if (!ok) {
        return fail(Gfx11Status::mmio_failure);
    }

    io_.delay_microseconds(io_.context, compute_enable_delay);

    if (!modify(me_cntl.segment, me_cntl.index, me_halt | pfp_halt, 0)) {
        return fail(Gfx11Status::mmio_failure);
    }

    report_.cp_gfx_enable_timeout = true;

    for (uint32_t poll = 0; poll < policy_.poll_iterations; ++poll) {
        if (!read(cp_stat.segment, cp_stat.index, report_.cp_status)) {
            return fail(Gfx11Status::mmio_failure);
        }

        if (report_.cp_status == 0) {
            report_.cp_gfx_enable_timeout = false;
            break;
        }

        io_.delay_microseconds(io_.context, 1);
    }

    // hw_init tail: IMU version from GFX_IMU_SCRATCH_0 when not otherwise known.
    if (!read_ip_register(io_, registers_.segment(imu_scratch0.segment), imu_scratch0.index,
                          report_.imu_version)) {
        return fail(Gfx11Status::mmio_failure);
    }

    report_.cp_started = true;
    report_.status = Gfx11Status::completed;
    return report_.status;
}

Gfx11Status Gfx11Init::cp_stop() noexcept {
    if (!initialized_) {
        return fail(Gfx11Status::unavailable);
    }

    // gfx_v11_0_cp_gfx_enable(false): Linux logs a halt timeout and continues.
    if (!modify(me_cntl.segment, me_cntl.index, 0, me_halt | pfp_halt)) {
        return fail(Gfx11Status::mmio_failure);
    }

    report_.cp_halt_timeout = true;

    for (uint32_t poll = 0; poll < policy_.poll_iterations; ++poll) {
        if (!read(cp_stat.segment, cp_stat.index, report_.cp_status)) {
            return fail(Gfx11Status::mmio_failure);
        }

        if (report_.cp_status == 0) {
            report_.cp_halt_timeout = false;
            break;
        }

        io_.delay_microseconds(io_.context, 1);
    }

    // gfx_v11_0_cp_compute_enable(false), RS64 path, then the 50 us settle.
    uint32_t control = 0;
    bool ok = read(mec_rs64_cntl.segment, mec_rs64_cntl.index, control) &&
              write(mec_rs64_cntl.segment, mec_rs64_cntl.index,
                    (control & ~mec_pipe_active) | mec_invalidate_icache | mec_pipe_resets | mec_halt);

    if (ok) {
        io_.delay_microseconds(io_.context, compute_enable_delay);
    }

    ok = ok && modify(cp_int_cntl_ring0.segment, cp_int_cntl_ring0.index, gui_idle_interrupts, 0) &&
         modify(cp_int_cntl_ring1.segment, cp_int_cntl_ring1.index, gui_idle_interrupts, 0);

    if (!ok) {
        return fail(Gfx11Status::mmio_failure);
    }

    report_.cp_started = false;
    report_.cp_stopped = true;
    report_.status = faulted_ ? Gfx11Status::quarantined : Gfx11Status::stopped;
    return report_.status;
}

Gfx11Status Gfx11Init::release_after_reset() noexcept {
    if (recovery_.confirm_reset_quiescence == nullptr ||
        !recovery_.confirm_reset_quiescence(recovery_.context)) {
        return fail(Gfx11Status::quarantined);
    }

    if (report_.memory_retained && manager_ != nullptr) {
        manager_->unretain(clear_state_);
    }

    manager_ = nullptr;
    clear_state_ = {};
    report_ = {};
    report_.status = Gfx11Status::released;
    initialized_ = false;
    faulted_ = false;
    return report_.status;
}

} // namespace rx7900::hardware

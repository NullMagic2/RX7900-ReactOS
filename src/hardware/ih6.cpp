/*
Module: rx7900.hardware.ih6
Purpose: Program the IH6 ring registers, NBIO4.3 IH control and doorbell range.
Created: 2026-10-21
Architecture: Bounded register protocol; native owner performs MMIO, doorbells
and bus mastering; Manager owns the local-VRAM ring backing.
Copyright 2021 Advanced Micro Devices, Inc.; Copyright 2018 Advanced Micro Devices, Inc.;
Copyright 2014 Advanced Micro Devices, Inc.
Adapted Linux v6.12 ih_v6_0.c, amdgpu_ih.c/.h, nbio_v4_3.c, soc21.c,
amdgpu_doorbell.h, soc15_hw_ip.h, osssys_6_0_0 and nbio_4_3_0 register headers.
Notice and provenance: licenses/AMDGPU-SDMA.txt.
*/
#include "ih6.hpp"

namespace rx7900::hardware {
namespace {

// osssys_6_0_0_offset.h, all BASE_IDX 0.
constexpr uint32_t ih_rb_cntl = 0x0080;
constexpr uint32_t ih_rb_base = 0x0081;
constexpr uint32_t ih_rb_base_hi = 0x0082;
constexpr uint32_t ih_rb_rptr = 0x0083;
constexpr uint32_t ih_rb_wptr = 0x0084;
constexpr uint32_t ih_rb_wptr_addr_hi = 0x0085;
constexpr uint32_t ih_rb_wptr_addr_lo = 0x0086;
constexpr uint32_t ih_doorbell_rptr = 0x0087;
constexpr uint32_t ih_rb_cntl_ring1 = 0x008c;
constexpr uint32_t ih_rb_base_ring1 = 0x008d;
constexpr uint32_t ih_rb_base_hi_ring1 = 0x008e;
constexpr uint32_t ih_rb_rptr_ring1 = 0x008f;
constexpr uint32_t ih_rb_wptr_ring1 = 0x0090;
constexpr uint32_t ih_doorbell_rptr_ring1 = 0x0093;
constexpr uint32_t ih_cntl2 = 0x00c1;
constexpr uint32_t ih_int_flood_cntl = 0x00d5;
constexpr uint32_t ih_storm_client_list_cntl = 0x00da;
constexpr uint32_t ih_clk_ctrl = 0x00db;
constexpr uint32_t ih_msi_storm_ctrl = 0x00f1;
constexpr uint32_t ih_ring1_client_cfg_index = 0x0185;
constexpr uint32_t ih_ring1_client_cfg_data = 0x0186;
constexpr uint32_t ih_chicken = 0x018d; // highest register checked for BAR5 fit

// osssys_6_0_0_sh_mask.h. IH_RB_CNTL_RING1 shares the IH_RB_CNTL layout; Linux
// applies the IH_RB_CNTL field macros to both rings.
constexpr uint32_t rb_enable = 0x00000001u;
constexpr uint32_t rb_size_mask = 0x0000003eu;
constexpr uint32_t rb_size_shift = 1;
constexpr uint32_t wptr_writeback_enable = 0x00000100u;
constexpr uint32_t rb_full_drain_enable = 0x00000200u;
constexpr uint32_t rb_used_int_threshold_mask = 0x0000f000u; // IH_RB_CNTL_RING1
constexpr uint32_t wptr_overflow_enable = 0x00010000u;
constexpr uint32_t enable_intr = 0x00020000u;
constexpr uint32_t mc_snoop = 0x00100000u;
constexpr uint32_t rptr_rearm = 0x00200000u;
constexpr uint32_t mc_ro = 0x00400000u;
constexpr uint32_t mc_vmid_mask = 0x0f000000u;
constexpr uint32_t mc_space_mask = 0x70000000u;
constexpr uint32_t mc_space_shift = 28;
constexpr uint32_t wptr_overflow_clear = 0x80000000u;
// ih_v6_0_rb_cntl: MC_SPACE 4 for a GPU (VMID0) address, 2 only for bus addresses.
constexpr uint32_t mc_space_gpu_address = 4;
constexpr uint32_t rb_base_hi_mask = 0x000000ffu;
constexpr uint32_t rb_wptr_overflow = 0x00000001u;
constexpr uint32_t wptr_addr_hi_mask = 0x0000ffffu;
constexpr uint32_t doorbell_offset_mask = 0x03ffffffu;
constexpr uint32_t doorbell_enable = 0x10000000u;
constexpr uint32_t self_iv_timeout_mask = 0x0000001fu;
constexpr uint32_t self_iv_enable = 0x00000100u;
// force_update_wptr_for_self_int(adev, 0, 8, enabled).
constexpr uint32_t self_iv_timeout = 8;
constexpr uint32_t flood_cntl_enable = 0x00000008u;
constexpr uint32_t client18_is_storm_client = 0x00040000u;
constexpr uint32_t msi_storm_delay_mask = 0x00000fffu;
constexpr uint32_t msi_storm_delay = 3;
constexpr uint32_t ring1_cfg_index_mask = 0x00000007u;
constexpr uint32_t ring1_cfg_client_id_mask = 0x000000ffu;
constexpr uint32_t ring1_cfg_source_id_mask = 0x0000ff00u;
constexpr uint32_t ring1_cfg_source_match = 0x00010000u;
// SOC21_IH_CLIENTID_VMC (UTCL2/MMHUB faults) is redirected to ring 1.
constexpr uint32_t ring1_cfg_client_vmc = 0x0a;
// IH_CLK_CTRL DBUS_MUX, OSSSYS_SHARE, LIMIT_SMN, DYN and REG soft overrides.
constexpr uint32_t clk_soft_overrides = 0x08000000u | 0x10000000u | 0x20000000u |
                                        0x40000000u | 0x80000000u;

// nbio_4_3_0_offset.h / _sh_mask.h.
constexpr uint32_t bif_bx0_interrupt_cntl = 0x00f1;  // segment 2
constexpr uint32_t bif_bx0_interrupt_cntl2 = 0x00f2; // segment 2
constexpr uint32_t s2a_doorbell_entry_1_ctrl = 0x4f7641; // segment 3, beyond BAR5
constexpr uint32_t ih_dummy_rd_override = 0x00000001u;
constexpr uint32_t ih_req_nonsnoop_en = 0x00000008u;
constexpr uint32_t s2a_port1_enable = 0x00000001u;
constexpr uint32_t s2a_port1_awid_mask = 0x0000003eu;
constexpr uint32_t s2a_port1_range_offset_mask = 0x0001ff80u;
constexpr uint32_t s2a_port1_range_offset_shift = 7;
constexpr uint32_t s2a_port1_range_size_mask = 0x01fe0000u;
constexpr uint32_t s2a_port1_range_size_shift = 17;
constexpr uint32_t s2a_port1_awaddr_mask = 0xf0000000u;
// nbio_v4_3_ih_doorbell_range: ring 0 and ring 1 doorbells.
constexpr uint32_t s2a_ih_range_size = 2;

constexpr uint64_t mc_address_limit = 1ULL << 48;
constexpr uint64_t dummy_page_limit = 1ULL << 40;

bool power_of_two(uint32_t value) noexcept {
    return value != 0 && (value & (value - 1)) == 0;
}

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

DiscoveryStatus validate_ih6_discovery(ByteView binary, uint32_t aperture,
                                       Ih6Registers& output) noexcept {
    output = {};

    if (aperture == 0 || (aperture & 3u) != 0) {
        return DiscoveryStatus::invalid_argument;
    }

    DiscoveredIp ip;
    const auto status = discover_ip(binary, {discovery_osssys_hardware_id, 0, 6, 0, 0, 1}, ip);

    if (status != DiscoveryStatus::validated) {
        return status;
    }

    uint32_t offset = 0;

    if (!discovery_register_offset(ip.segment(0), ih_chicken, aperture, offset)) {
        return DiscoveryStatus::invalid_argument;
    }

    output.ip_ = ip;
    output.aperture_bytes_ = aperture;
    return DiscoveryStatus::validated;
}

Ih6::RingRegisters Ih6::ring_registers(uint32_t index) noexcept {
    if (index == 0) {
        return {ih_rb_base, ih_rb_base_hi, ih_rb_cntl, ih_rb_wptr, ih_rb_rptr, ih_doorbell_rptr};
    }

    return {ih_rb_base_ring1, ih_rb_base_hi_ring1, ih_rb_cntl_ring1, ih_rb_wptr_ring1,
            ih_rb_rptr_ring1, ih_doorbell_rptr_ring1};
}

bool Ih6::read(uint32_t reg, uint32_t& value) noexcept {
    // All-ones is a dead or unmapped device; no IH register is read for it.
    return read_ip_register(io_, registers_.segment0_dword_base(), reg, value) &&
           value != UINT32_MAX;
}

bool Ih6::write(uint32_t reg, uint32_t value) noexcept {
    return write_ip_register(io_, registers_.segment0_dword_base(), reg, value);
}

bool Ih6::modify(uint32_t reg, uint32_t clear, uint32_t set) noexcept {
    uint32_t value = 0;

    if (!read(reg, value)) {
        return false;
    }

    return write(reg, (value & ~clear) | set);
}

Ih6Status Ih6::quarantine(Ih6Status reason) noexcept {
    phase_ = Phase::faulted;
    report_.status = reason;
    return reason;
}

Ih6Status Ih6::initialize(Ih6Registers registers, NbioRegisters nbio_registers, RegisterIo io,
                          Nbio& nbio, VramLocation vram, Ih6Config config,
                          Ih6Platform platform) noexcept {
    if (phase_ == Phase::enabled || phase_ == Phase::disabled || phase_ == Phase::faulted ||
        report_.memory_retained) {
        return Ih6Status::busy;
    }

    phase_ = Phase::uninitialized;
    report_ = {};

    if (!registers.valid() || !nbio_registers.valid()) {
        report_.status = Ih6Status::unsupported_ip;
        return report_.status;
    }

    if (!register_io_valid(io) || io.aperture_bytes != registers.aperture_bytes() ||
        io.aperture_bytes != nbio_registers.aperture_bytes() ||
        platform.enable_bus_master == nullptr || platform.confirm_reset_quiescence == nullptr) {
        report_.status = Ih6Status::unavailable;
        return report_.status;
    }

    if (vram.bytes == 0 || vram.mc_end < vram.mc_start || !power_of_two(config.ring_bytes) ||
        config.ring_bytes < ih6_minimum_ring_bytes || config.ring_bytes > ih6_maximum_ring_bytes ||
        config.dummy_page_bus_address == 0 ||
        (config.dummy_page_bus_address & (memory::page_bytes - 1)) != 0 ||
        config.dummy_page_bus_address >= dummy_page_limit) {
        report_.status = Ih6Status::invalid_argument;
        return report_.status;
    }

    registers_ = registers;
    nbio_registers_ = nbio_registers;
    io_ = io;
    nbio_ = &nbio;
    vram_ = vram;
    config_ = config;
    platform_ = platform;
    report_.ring_bytes = config.ring_bytes;
    report_.doorbell_mode = platform.write_doorbell32 != nullptr;
    phase_ = Phase::ready;
    report_.status = Ih6Status::ready;
    return report_.status;
}

bool Ih6::retain_memory(memory::Manager& manager, memory::Handle ring0, memory::Handle ring1,
                        memory::Handle writeback) noexcept {
    memory::AllocationInfo rings[ih6_ring_count]{};
    memory::AllocationInfo slot{};

    if (!retain_local_vram(manager, ring0, vram_, config_.ring_bytes, ih6_ring_alignment, true,
                           rings[0])) {
        return false;
    }

    if (!retain_local_vram(manager, ring1, vram_, config_.ring_bytes, ih6_ring_alignment, true,
                           rings[1])) {
        manager.unretain(ring0);
        return false;
    }

    if (!retain_local_vram(manager, writeback, vram_, ih6_writeback_bytes,
                           ih6_writeback_alignment, true, slot)) {
        manager.unretain(ring1);
        manager.unretain(ring0);
        return false;
    }

    const uint64_t ring0_mc = rings[0].gpu.virtual_address;
    const uint64_t ring1_mc = rings[1].gpu.virtual_address;
    const uint64_t slot_mc = slot.gpu.virtual_address;
    const uint64_t bytes = config_.ring_bytes;

    // retain_local_vram bounds every range inside VRAM, so these sums cannot wrap.
    if (overlaps(ring0_mc, bytes, ring1_mc, bytes) ||
        overlaps(ring0_mc, bytes, slot_mc, ih6_writeback_bytes) ||
        overlaps(ring1_mc, bytes, slot_mc, ih6_writeback_bytes) ||
        ring0_mc + bytes > mc_address_limit || ring1_mc + bytes > mc_address_limit ||
        slot_mc + ih6_writeback_bytes > mc_address_limit) {
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
    writeback_cpu_ = slot.backing.cpu_bytes;
    report_.ring_mc[0] = ring0_mc;
    report_.ring_mc[1] = ring1_mc;
    report_.writeback_mc = slot_mc;
    report_.memory_retained = true;
    return true;
}

void Ih6::release_leases() noexcept {
    if (report_.memory_retained && manager_ != nullptr) {
        for (const auto& handle : handles_) {
            manager_->unretain(handle);
        }
    }

    manager_ = nullptr;

    for (auto& handle : handles_) {
        handle = {};
    }

    ring_cpu_[0] = nullptr;
    ring_cpu_[1] = nullptr;
    writeback_cpu_ = nullptr;
    report_.ring_mc[0] = 0;
    report_.ring_mc[1] = 0;
    report_.writeback_mc = 0;
    report_.memory_retained = false;
}

bool Ih6::clear_memory() noexcept {
    // Uncached BAR0 stores; the HDP flush publishes them before IH is enabled.
    for (uint8_t* ring : ring_cpu_) {
        const sdma::CommandBuffer bytes(ring, config_.ring_bytes);

        for (uint32_t offset = 0; offset < config_.ring_bytes; offset += 4) {
            if (!bytes.write_u32(offset, 0)) {
                return false;
            }
        }
    }

    ring1_wptr_ = 0;
    return sdma::CommandBuffer(writeback_cpu_, ih6_writeback_bytes).write_u32(0, 0) &&
           nbio_->flush_hdp();
}

// ih_v6_0_toggle_ring_interrupts.
bool Ih6::toggle_ring(uint32_t index, bool enable) noexcept {
    const RingRegisters regs = ring_registers(index);
    uint32_t value = 0;

    if (!read(regs.cntl, value)) {
        return false;
    }

    value = enable ? (value | rb_enable) : (value & ~rb_enable);

    if (enable) {
        // A 0 then 1 transition of WPTR_OVERFLOW_CLEAR clears a latched overflow;
        // it is dropped again so a new overflow can be detected.
        value &= ~wptr_overflow_clear;

        if (!write(regs.cntl, value)) {
            return false;
        }

        value |= wptr_overflow_clear;

        if (!write(regs.cntl, value)) {
            return false;
        }

        value &= ~wptr_overflow_clear;
    }

    // ENABLE_INTR exists only in ring 0.
    if (index == 0) {
        value = enable ? (value | enable_intr) : (value & ~enable_intr);
    }

    if (!write(regs.cntl, value)) {
        return false;
    }

    report_.rb_cntl[index] = value;

    if (enable) {
        return true;
    }

    if (!write(regs.rptr, 0) || !write(regs.wptr, 0)) {
        return false;
    }

    report_.rptr[index] = 0;
    return true;
}

// ih_v6_0_toggle_interrupts: ring 0 then ring 1.
bool Ih6::toggle_interrupts(bool enable) noexcept {
    return toggle_ring(0, enable) && toggle_ring(1, enable);
}

// ih_v6_0_enable_ring with ih_v6_0_rb_cntl and ih_v6_0_doorbell_rptr.
bool Ih6::enable_ring(uint32_t index) noexcept {
    const RingRegisters regs = ring_registers(index);
    const uint64_t gpu = report_.ring_mc[index];
    uint32_t value = 0;

    // RB_BASE holds address bits 39:8 (WREG32 keeps the low 32 bits of gpu >> 8).
    if (!write(regs.base, static_cast<uint32_t>(gpu >> 8)) ||
        !write(regs.base_hi, static_cast<uint32_t>(gpu >> 40) & rb_base_hi_mask) ||
        !read(regs.cntl, value)) {
        return false;
    }

    value &= ~(mc_space_mask | rb_size_mask | mc_ro | mc_vmid_mask);
    value |= mc_space_gpu_address << mc_space_shift;
    value |= wptr_overflow_clear | wptr_overflow_enable | wptr_writeback_enable | mc_snoop;
    value |= (log2_exact(config_.ring_bytes / 4) << rb_size_shift) & rb_size_mask;

    if (index == 0) {
        value = config_.msi_enabled ? (value | rptr_rearm) : (value & ~rptr_rearm);
    } else {
        value &= ~wptr_overflow_enable;
        value |= rb_full_drain_enable;
    }

    if (!write(regs.cntl, value)) {
        return false;
    }

    report_.rb_cntl[index] = value;

    // Only ring 0 has a writeback address; ring 1 learns WPTR from self-interrupts.
    if (index == 0 &&
        (!write(ih_rb_wptr_addr_lo, static_cast<uint32_t>(report_.writeback_mc)) ||
         !write(ih_rb_wptr_addr_hi,
                static_cast<uint32_t>(report_.writeback_mc >> 32) & wptr_addr_hi_mask))) {
        return false;
    }

    if (!write(regs.wptr, 0) || !write(regs.rptr, 0)) {
        return false;
    }

    report_.rptr[index] = 0;
    uint32_t doorbell = 0;

    if (report_.doorbell_mode) {
        const uint32_t doorbell_index =
            index == 0 ? ih6_ring0_doorbell_index : ih6_ring1_doorbell_index;
        doorbell = (doorbell_index & doorbell_offset_mask) | doorbell_enable;
    }

    if (!write(regs.doorbell_rptr, doorbell)) {
        return false;
    }

    report_.doorbell_rptr[index] = doorbell;
    return true;
}

// nbio_v4_3_ih_control. Linux leaves IH_REQ_NONSNOOP_EN clear because its ring
// is in system memory; the same Linux comment requires it set for a ring in
// non-cacheable memory such as VRAM, which is where this project places it.
bool Ih6::nbio_ih_control() noexcept {
    const uint32_t base = 2;
    uint32_t value = 0;

    if (!nbio_->write_register(nbio_registers_.nbif_base(base), bif_bx0_interrupt_cntl2,
                               static_cast<uint32_t>(config_.dummy_page_bus_address >> 8)) ||
        !nbio_->read_register(nbio_registers_.nbif_base(base), bif_bx0_interrupt_cntl, value) ||
        value == UINT32_MAX) {
        return false;
    }

    value &= ~ih_dummy_rd_override;
    value |= ih_req_nonsnoop_en;

    if (!nbio_->write_register(nbio_registers_.nbif_base(base), bif_bx0_interrupt_cntl, value)) {
        return false;
    }

    report_.interrupt_cntl = value;
    return true;
}

// nbio_v4_3_ih_doorbell_range for ring 0. The register lies beyond BAR5 on
// Navi31 and is reached through Nbio's SMN fallback, as amdgpu_device_wreg.
bool Ih6::nbio_doorbell_range() noexcept {
    const uint32_t base = 3;
    uint32_t value = 0;

    if (!nbio_->read_register(nbio_registers_.nbif_base(base), s2a_doorbell_entry_1_ctrl, value) ||
        value == UINT32_MAX) {
        return false;
    }

    if (report_.doorbell_mode) {
        value &= ~(s2a_port1_awid_mask | s2a_port1_range_offset_mask | s2a_port1_range_size_mask |
                   s2a_port1_awaddr_mask);
        value |= s2a_port1_enable;
        value |= (ih6_ring0_doorbell_index << s2a_port1_range_offset_shift) &
                 s2a_port1_range_offset_mask;
        value |= s2a_ih_range_size << s2a_port1_range_size_shift;
    } else {
        value &= ~s2a_port1_range_size_mask;
    }

    if (!nbio_->write_register(nbio_registers_.nbif_base(base), s2a_doorbell_entry_1_ctrl, value)) {
        return false;
    }

    report_.doorbell_range = value;
    return true;
}

// force_update_wptr_for_self_int(adev, 0, 8, enable).
bool Ih6::force_wptr_update(bool enable) noexcept {
    uint32_t cntl2 = 0;
    uint32_t ring1 = 0;

    if (!read(ih_cntl2, cntl2) || !read(ih_rb_cntl_ring1, ring1)) {
        return false;
    }

    cntl2 = (cntl2 & ~self_iv_timeout_mask) | self_iv_timeout;
    cntl2 = enable ? (cntl2 | self_iv_enable) : (cntl2 & ~self_iv_enable);
    ring1 &= ~rb_used_int_threshold_mask;

    if (!write(ih_rb_cntl_ring1, ring1) || !write(ih_cntl2, cntl2)) {
        return false;
    }

    report_.rb_cntl[1] = ring1;
    return true;
}

// ih_v6_0_irq_init. IH_CHICKEN MC_SPACE_GPA_ENABLE is skipped exactly as Linux
// does for PSP firmware loading (and because the rings use GPU addresses).
Ih6Status Ih6::irq_init() noexcept {
    if (!toggle_interrupts(false) || !nbio_ih_control() || !enable_ring(0) || !enable_ring(1) ||
        !nbio_doorbell_range() ||
        !modify(ih_storm_client_list_cntl, 0, client18_is_storm_client) ||
        !modify(ih_int_flood_cntl, 0, flood_cntl_enable) ||
        !modify(ih_msi_storm_ctrl, msi_storm_delay_mask, msi_storm_delay) ||
        !modify(ih_ring1_client_cfg_index, ring1_cfg_index_mask, 0) ||
        !modify(ih_ring1_client_cfg_data,
                ring1_cfg_client_id_mask | ring1_cfg_source_id_mask | ring1_cfg_source_match,
                ring1_cfg_client_vmc | ring1_cfg_source_match)) {
        return quarantine(Ih6Status::mmio_failure);
    }

    if (!platform_.enable_bus_master(platform_.context)) {
        return quarantine(Ih6Status::platform_failure);
    }

    if (!toggle_interrupts(true) || !force_wptr_update(true)) {
        return quarantine(Ih6Status::mmio_failure);
    }

    report_.enabled = true;
    phase_ = Phase::enabled;
    report_.status = Ih6Status::enabled;
    return report_.status;
}

Ih6Status Ih6::hw_init(memory::Manager& manager, memory::Handle ring0, memory::Handle ring1,
                       memory::Handle writeback) noexcept {
    if (phase_ != Phase::ready || report_.memory_retained) {
        return phase_ == Phase::uninitialized ? Ih6Status::unavailable : Ih6Status::busy;
    }

    if (!retain_memory(manager, ring0, ring1, writeback)) {
        report_.status = Ih6Status::invalid_memory;
        return report_.status;
    }

    if (!clear_memory()) {
        return quarantine(Ih6Status::mmio_failure);
    }

    return irq_init();
}

Ih6Status Ih6::resume() noexcept {
    if (phase_ != Phase::disabled || !report_.memory_retained) {
        return phase_ == Phase::uninitialized ? Ih6Status::unavailable : Ih6Status::busy;
    }

    if (!clear_memory()) {
        return quarantine(Ih6Status::mmio_failure);
    }

    return irq_init();
}

// ih_v6_0_irq_disable.
Ih6Status Ih6::disable() noexcept {
    if (phase_ != Phase::enabled) {
        return phase_ == Phase::uninitialized ? Ih6Status::unavailable : Ih6Status::busy;
    }

    if (!force_wptr_update(false) || !toggle_interrupts(false)) {
        return quarantine(Ih6Status::mmio_failure);
    }

    io_.delay_microseconds(io_.context, ih6_disable_settle_microseconds);
    report_.enabled = false;
    phase_ = Phase::disabled;
    report_.status = Ih6Status::disabled;
    return report_.status;
}

// ih_v6_0_update_clockgating_state: write only when the value changes.
Ih6Status Ih6::set_clockgating(bool gate) noexcept {
    if (phase_ == Phase::uninitialized || phase_ == Phase::faulted) {
        return phase_ == Phase::faulted ? Ih6Status::quarantined : Ih6Status::unavailable;
    }

    uint32_t current = 0;

    if (!read(ih_clk_ctrl, current)) {
        return quarantine(Ih6Status::mmio_failure);
    }

    const uint32_t desired = gate ? (current & ~clk_soft_overrides) :
                                    (current | clk_soft_overrides);

    if (desired != current && !write(ih_clk_ctrl, desired)) {
        return quarantine(Ih6Status::mmio_failure);
    }

    report_.clock_gated = gate;
    return report_.status;
}

Ih6Status Ih6::release() noexcept {
    if (phase_ != Phase::disabled) {
        return phase_ == Phase::faulted ? Ih6Status::quarantined : Ih6Status::busy;
    }

    release_leases();
    phase_ = Phase::ready;
    report_.status = Ih6Status::released;
    return report_.status;
}

Ih6Status Ih6::release_after_reset() noexcept {
    if (!report_.memory_retained && phase_ != Phase::faulted) {
        return Ih6Status::busy;
    }

    if (platform_.confirm_reset_quiescence == nullptr ||
        !platform_.confirm_reset_quiescence(platform_.context)) {
        return Ih6Status::quarantined;
    }

    release_leases();
    report_.enabled = false;
    phase_ = Phase::uninitialized;
    report_.status = Ih6Status::released;
    return report_.status;
}

// ih_v6_0_get_wptr.
Ih6Status Ih6::read_wptr(Ih6Ring ring, uint32_t& wptr) noexcept {
    const uint32_t index = static_cast<uint32_t>(ring);

    if (phase_ != Phase::enabled || index >= ih6_ring_count) {
        return phase_ == Phase::faulted ? Ih6Status::quarantined : Ih6Status::unavailable;
    }

    const RingRegisters regs = ring_registers(index);
    const uint32_t mask = config_.ring_bytes - 1;
    uint32_t value = ring1_wptr_;

    // The writeback dword is device-written uncached VRAM: one checked read.
    if (index == 0 && !ByteView(writeback_cpu_, ih6_writeback_bytes).read_u32(0, value)) {
        return quarantine(Ih6Status::invalid_memory);
    }

    if ((value & rb_wptr_overflow) == 0) {
        wptr = value & mask;
        return Ih6Status::enabled;
    }

    if (!read(regs.wptr, value)) {
        return quarantine(Ih6Status::mmio_failure);
    }

    if ((value & rb_wptr_overflow) == 0) {
        wptr = value & mask;
        return Ih6Status::enabled;
    }

    value &= ~rb_wptr_overflow;
    // Linux resumes at the oldest vector not yet overwritten (wptr + 32).
    report_.rptr[index] = (value + ih6_entry_bytes) & mask;
    uint32_t cntl = 0;

    if (!read(regs.cntl, cntl) || !write(regs.cntl, cntl | wptr_overflow_clear) ||
        !write(regs.cntl, cntl & ~wptr_overflow_clear)) {
        return quarantine(Ih6Status::mmio_failure);
    }

    report_.rb_cntl[index] = cntl & ~wptr_overflow_clear;
    ++report_.overflows;
    wptr = value & mask;
    return Ih6Status::overflow;
}

// ih_v6_0_set_rptr. The doorbell path omits Linux's rptr_cpu shadow store:
// IH6 never has its address programmed, so nothing reads it.
Ih6Status Ih6::set_rptr(Ih6Ring ring, uint32_t rptr) noexcept {
    const uint32_t index = static_cast<uint32_t>(ring);

    if (phase_ != Phase::enabled || index >= ih6_ring_count) {
        return phase_ == Phase::faulted ? Ih6Status::quarantined : Ih6Status::unavailable;
    }

    if (rptr >= config_.ring_bytes || (rptr % ih6_entry_bytes) != 0) {
        return Ih6Status::invalid_argument;
    }

    bool written = false;

    if (report_.doorbell_mode) {
        const uint32_t doorbell = index == 0 ? ih6_ring0_doorbell_index : ih6_ring1_doorbell_index;
        written = platform_.write_doorbell32(platform_.context, doorbell, rptr);
    } else {
        written = write(ring_registers(index).rptr, rptr);
    }

    if (!written) {
        return quarantine(report_.doorbell_mode ? Ih6Status::platform_failure :
                                                  Ih6Status::mmio_failure);
    }

    report_.rptr[index] = rptr;
    return Ih6Status::enabled;
}

bool Ih6::record_self_interrupt(uint32_t ring_id, uint32_t source_data0) noexcept {
    if (phase_ != Phase::enabled || ring_id != static_cast<uint32_t>(Ih6Ring::page_fault)) {
        return false;
    }

    ring1_wptr_ = source_data0;
    return true;
}

bool Ih6::ring_view(Ih6Ring ring, ByteView& view) const noexcept {
    const uint32_t index = static_cast<uint32_t>(ring);

    if (phase_ != Phase::enabled || index >= ih6_ring_count) {
        return false;
    }

    view = ByteView(ring_cpu_[index], config_.ring_bytes);
    return true;
}

uint32_t Ih6::rptr(Ih6Ring ring) const noexcept {
    const uint32_t index = static_cast<uint32_t>(ring);
    return index < ih6_ring_count ? report_.rptr[index] : 0;
}

} // namespace rx7900::hardware

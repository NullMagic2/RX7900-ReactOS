/*
Module: rx7900.hardware.gmc11
Purpose: Program GART, system aperture, L1/L2, contexts and TLB invalidation per hub.
Created: 2026-10-21
Architecture: Bounded register sequence shared by MMHUB3.0 and GFXHUB3.0.
Copyright 2021 Advanced Micro Devices, Inc.; Copyright 2009 Jerome Glisse.
Adapted Linux v6.12 gmc_v11_0.c (hw_init, gart_enable, flush_gpu_tlb,
emit_flush_gpu_tlb, get_vm_pde), mmhub_v3_0.c, gfxhub_v3_0.c (get_invalidate_req),
gfx_v11_0.c (gfxhub_enable), amdgpu_gmc.c (gart_location, pd_addr, set_agp_default,
fw_reg_write_reg_wait), amdgpu_mes.c (reg_write_reg_wait), mes_v11_0.c (misc_op),
mes_v11_api_def.h, amdgpu_gart.c and amdgpu_vm.c (adjust_size).
Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#include "gmc11.hpp"

namespace rx7900::hardware {
namespace {

constexpr uint64_t gmc_hole_start = 0x0000800000000000ULL;
constexpr uint64_t mc_mask = 0x0000ffffffffffffULL;
constexpr uint64_t four_gib = 0x100000000ULL;
// amdgpu_gmc_set_agp_default: AGP disabled.
constexpr uint64_t agp_start = 0xffffffffffffULL;
constexpr uint64_t agp_end = 0;
// amdgpu_vm_adjust_size(256 TiB, 9, 3, 48): three levels, block size 9, max_pfn 2^36.
constexpr uint32_t vm_levels = 3;
constexpr uint32_t vm_block_size = 9;
constexpr uint64_t vm_max_pfn = 1ULL << 36;
constexpr uint32_t mtype_uc = 3;
constexpr uint32_t user_contexts = 15;
constexpr uint32_t invalidate_engines = 18;
constexpr uint64_t pte_address_mask = 0x0000fffffffff000ULL;
// AMDGPU_PTE_EXECUTABLE (bit 4), part of gmc_v11_0 gart_pte_flags.
constexpr uint32_t pte_executable = 0x10u;
constexpr uint64_t pte_valid = 1;
constexpr uint32_t maximum_polls = 1000000;
constexpr uint32_t page_bytes = 4096;

uint32_t low(uint64_t value) noexcept { return static_cast<uint32_t>(value); }
uint32_t high(uint64_t value) noexcept { return static_cast<uint32_t>(value >> 32); }

uint32_t highest_register(const VmHubLayout& layout) noexcept {
    // Context1 control plus 14 contexts, engine 17 and the 18th range register
    // are the furthest registers touched; every other one lies below them.
    uint32_t top = layout.context1_control + user_contexts - 1;
    const uint32_t candidates[]{layout.invalidate0_ack + gart_invalidate_engine,
        layout.invalidate0_range_high + (invalidate_engines - 1) * 2,
        layout.context1_end_high + (user_contexts - 1) * 2, layout.default_address_msb,
        layout.l1_tlb_control, layout.agp_base};

    for (const auto candidate : candidates) {
        top = candidate > top ? candidate : top;
    }

    return top;
}

// mes_v11_api_def.h: MES_API_TYPE_SCHEDULER, MES_SCH_API_MISC, header dwsize,
// MESAPI_MISC__WAIT_REG_MEM, WRM_OPERATION__WR_WAIT_WR_REG and the union
// MESAPI__MISC byte offsets of opcode and wait_reg_mem fields.
constexpr uint32_t mes_api_type_scheduler = 1;
constexpr uint32_t mes_sch_api_misc = 14;
constexpr uint32_t mes_misc_wait_reg_mem = 4;
constexpr uint32_t wrm_operation_wr_wait_wr_reg = 1;
constexpr size_t misc_opcode_offset = 4;
constexpr size_t misc_wrm_op_offset = 24;
constexpr size_t misc_wrm_reference_offset = 28;
constexpr size_t misc_wrm_mask_offset = 32;
constexpr size_t misc_wrm_reg1_offset = 36;
constexpr size_t misc_wrm_reg2_offset = 40;
// AMDGPU_PDE address bits 47:6 plus flags; gmc_v11_0_get_vm_pde BUG_ON mask.
constexpr uint64_t pd_reject_mask = 0xffff000000000000ULL;

} // namespace

uint32_t gmc11_invalidate_request(uint32_t vmid, uint32_t flush_type) noexcept {
    using namespace vmhub_field;
    return ((1u << (vmid & 15u)) & 0xffffu) | ((flush_type & gmc_max_flush_type) << invalidate_flush_type_shift) |
           invalidate_l2_ptes | invalidate_l2_pde0 | invalidate_l2_pde1 | invalidate_l2_pde2 |
           invalidate_l1_ptes;
}

bool encode_mes_reg_write_reg_wait(uint32_t reg0, uint32_t reg1, uint32_t reference,
                                   uint32_t mask, sdma::CommandBuffer frame) noexcept {
    if (frame.size() < gmc_mes_frame_bytes) {
        return false;
    }

    for (uint32_t offset = 0; offset < gmc_mes_frame_bytes; offset += 4) {
        frame.write_u32(offset, 0);
    }

    const uint32_t header = mes_api_type_scheduler | (mes_sch_api_misc << 4) |
                            ((gmc_mes_frame_bytes / 4) << 12);
    return frame.write_u32(0, header) && frame.write_u32(misc_opcode_offset, mes_misc_wait_reg_mem) &&
           frame.write_u32(misc_wrm_op_offset, wrm_operation_wr_wait_wr_reg) &&
           frame.write_u32(misc_wrm_reference_offset, reference) &&
           frame.write_u32(misc_wrm_mask_offset, mask) && frame.write_u32(misc_wrm_reg1_offset, reg0) &&
           frame.write_u32(misc_wrm_reg2_offset, reg1);
}

bool gmc11_emit_flush_gpu_tlb(const GmcRegisters& registers, VmHub hub, uint32_t vmid,
                              uint64_t pd_addr, uint32_t engine, GmcRingFlush& output) noexcept {
    if (!registers.valid() || vmid > 15 || engine > gart_invalidate_engine ||
        (pd_addr & pd_reject_mask) != 0) {
        return false;
    }

    const VmHubLayout& r = hub == VmHub::mmhub ? mmhub_3_0_0_layout : gfxhub_3_0_layout;
    const uint32_t base = registers.base(hub);
    const uint32_t semaphore = base + r.invalidate0_semaphore + engine;
    // ctx_addr_distance: CONTEXT1_PAGE_TABLE_BASE_ADDR_LO32 - CONTEXT0_..._LO32.
    const uint32_t context_distance = r.context1_base_low - r.context0_base_low;
    const uint32_t base_low = base + r.context0_base_low + context_distance * vmid;
    GmcRingFlush flush{};

    // gmc_v11_0_use_invalidate_semaphore: MMHUB on bare metal only.
    if (hub == VmHub::mmhub) {
        flush.ops[flush.count++] = {GmcRingOpKind::reg_wait, semaphore, 0, 1, 1};
    }

    flush.ops[flush.count++] = {GmcRingOpKind::write, base_low, 0, low(pd_addr), 0};
    flush.ops[flush.count++] = {GmcRingOpKind::write, base_low + 1, 0, high(pd_addr), 0};
    flush.ops[flush.count++] = {GmcRingOpKind::write_wait, base + r.invalidate0_request + engine,
                                base + r.invalidate0_ack + engine,
                                gmc11_invalidate_request(vmid, 0), 1u << vmid};

    if (hub == VmHub::mmhub) {
        flush.ops[flush.count++] = {GmcRingOpKind::write, semaphore, 0, 0, 0};
    }

    output = flush;
    return true;
}

bool compute_gart_layout(const VramLocation& vram, GartLayout& output) noexcept {
    const uint64_t max_mc = (mc_mask < gmc_hole_start - 1 ? mc_mask : gmc_hole_start - 1);

    if (vram.bytes == 0 || vram.mc_end >= max_mc) {
        return false;
    }

    const uint64_t size_before = vram.mc_start;
    const uint64_t aligned_end = (vram.mc_end + 1 + four_gib - 1) & ~(four_gib - 1);
    const uint64_t size_after = aligned_end > max_mc + 1 ? 0 : max_mc + 1 - aligned_end;

    // Linux would shrink the GART; the 512 MiB default must fit here.
    if (gart_bytes > (size_before > size_after ? size_before : size_after)) {
        return false;
    }

    const uint64_t start = (max_mc - gart_bytes + 1) & ~(four_gib - 1);
    output = {start, start + gart_bytes - 1};
    return true;
}

DiscoveryStatus validate_gmc_discovery(ByteView binary, uint32_t aperture,
                                       GmcRegisters& output) noexcept {
    output = {};

    if (aperture == 0 || (aperture & 3u) != 0) {
        return DiscoveryStatus::invalid_argument;
    }

    DiscoveredIp mmhub;
    DiscoveredIp gc;
    auto status = discover_ip(binary, {discovery_mmhub_hardware_id, 0, 3, 0, 0, 1}, mmhub);

    if (status != DiscoveryStatus::validated) {
        return status;
    }

    status = discover_ip(binary, {discovery_gc_hardware_id, 0, 11, 0, 0, 1}, gc);

    if (status != DiscoveryStatus::validated) {
        return status;
    }

    uint32_t offset = 0;

    if (!discovery_register_offset(mmhub.segment(0), highest_register(mmhub_3_0_0_layout),
                                   aperture, offset) ||
        !discovery_register_offset(gc.segment(0), highest_register(gfxhub_3_0_layout),
                                   aperture, offset) ||
        !discovery_register_offset(gc.segment(0), gc_cp_debug, aperture, offset)) {
        return DiscoveryStatus::invalid_argument;
    }

    output.mmhub_ = mmhub;
    output.gc_ = gc;
    output.aperture_bytes_ = aperture;
    return DiscoveryStatus::validated;
}

GmcStatus Gmc11::finish(GmcStatus status) noexcept {
    report_.status = status;
    return status;
}

bool Gmc11::read(VmHub hub, uint32_t reg, uint32_t& value) noexcept {
    return read_ip_register(io_, registers_.base(hub), reg, value) && value != UINT32_MAX;
}

bool Gmc11::write(VmHub hub, uint32_t reg, uint32_t value) noexcept {
    return write_ip_register(io_, registers_.base(hub), reg, value);
}

bool Gmc11::modify(VmHub hub, uint32_t reg, uint32_t clear, uint32_t set) noexcept {
    uint32_t value = 0;
    return read(hub, reg, value) && write(hub, reg, (value & ~clear) | set);
}

GmcStatus Gmc11::initialize(GmcRegisters registers, RegisterIo io, Nbio& nbio,
                            VramLocation vram, GmcPolicy policy, GmcRecovery recovery) noexcept {
    if (report_.memory_retained) {
        return finish(GmcStatus::busy);
    }

    initialized_ = false;
    faulted_ = false;
    report_ = {};

    if (!registers.valid()) {
        return finish(GmcStatus::unsupported_ip);
    }

    if (!register_io_valid(io) || io.aperture_bytes != registers.aperture_bytes() ||
        policy.poll_iterations == 0 || policy.poll_iterations > maximum_polls ||
        !compute_gart_layout(vram, report_.gart)) {
        return finish(GmcStatus::unavailable);
    }

    registers_ = registers;
    io_ = io;
    nbio_ = &nbio;
    vram_ = vram;
    policy_ = policy;
    recovery_ = recovery;
    initialized_ = true;
    return finish(GmcStatus::ready);
}

GmcStatus Gmc11::prepare_gart(memory::Manager& manager, memory::Handle table,
                              memory::Handle scratch, uint64_t dummy_page_bus) noexcept {
    if (!initialized_ || report_.memory_retained) {
        return finish(initialized_ ? GmcStatus::busy : GmcStatus::unavailable);
    }

    // The dummy page is a 4 KiB-aligned system DMA address representable in a PTE.
    if (dummy_page_bus == 0 || (dummy_page_bus & (page_bytes - 1)) != 0 ||
        (dummy_page_bus & ~pte_address_mask) != 0) {
        return finish(GmcStatus::invalid_argument);
    }

    memory::AllocationInfo table_info{};
    memory::AllocationInfo scratch_info{};

    if (!retain_local_vram(manager, table, vram_, gart_table_bytes, page_bytes, true, table_info)) {
        return finish(GmcStatus::invalid_memory);
    }

    if (!retain_local_vram(manager, scratch, vram_, page_bytes, page_bytes, false, scratch_info)) {
        manager.unretain(table);
        return finish(GmcStatus::invalid_memory);
    }

    manager_ = &manager;
    table_ = table;
    scratch_ = scratch;
    table_cpu_ = table_info.backing.cpu_bytes;
    report_.memory_retained = true;
    report_.table_mc = table_info.gpu.virtual_address;
    report_.dummy_page_bus = dummy_page_bus;
    // amdgpu_gmc_pd_addr on a VRAM BO: get_vm_pde translates MC to VRAM "PA"
    // (no translate_further on GMC11) and ORs AMDGPU_PTE_VALID.
    report_.page_directory = (table_info.gpu.virtual_address - vram_.mc_start +
                              vram_.physical_offset) | pte_valid;
    report_.scratch_physical = scratch_info.gpu.virtual_address - vram_.mc_start +
                               vram_.physical_offset;

    const sdma::CommandBuffer entries(table_cpu_, gart_table_bytes);
    const uint64_t unbound = dummy_page_bus & pte_address_mask;

    for (uint32_t offset = 0; offset < gart_table_bytes; offset += 8) {
        entries.write_u32(offset, low(unbound));
        entries.write_u32(offset + 4, high(unbound));
    }

    if (!nbio_->flush_hdp()) {
        return finish(GmcStatus::mmio_failure);
    }

    return finish(GmcStatus::prepared);
}

GmcStatus Gmc11::gart_enable(VmHub hub) noexcept {
    using namespace vmhub_field;
    const VmHubLayout& r = layout(hub);
    const GartLayout& gart = report_.gart;
    const uint64_t pd = report_.page_directory;

    // init_gart_aperture_regs.
    bool ok = write(hub, r.context0_base_low, low(pd)) && write(hub, r.context0_base_high, high(pd)) &&
        write(hub, r.context0_start_low, static_cast<uint32_t>(gart.start >> 12)) &&
        write(hub, r.context0_start_high, static_cast<uint32_t>(gart.start >> 44)) &&
        write(hub, r.context0_end_low, static_cast<uint32_t>(gart.end >> 12)) &&
        write(hub, r.context0_end_high, static_cast<uint32_t>(gart.end >> 44));

    // init_system_aperture_regs.
    const uint64_t aperture_low = vram_.mc_start < agp_start ? vram_.mc_start : agp_start;
    const uint64_t aperture_high = vram_.mc_end > agp_end ? vram_.mc_end : agp_end;
    ok = ok && write(hub, r.agp_base, 0) &&
        write(hub, r.agp_bot, static_cast<uint32_t>(agp_start >> 24)) &&
        write(hub, r.agp_top, static_cast<uint32_t>(agp_end >> 24)) &&
        write(hub, r.system_aperture_low, static_cast<uint32_t>(aperture_low >> 18)) &&
        write(hub, r.system_aperture_high, static_cast<uint32_t>(aperture_high >> 18)) &&
        write(hub, r.default_address_lsb, static_cast<uint32_t>(report_.scratch_physical >> 12)) &&
        write(hub, r.default_address_msb, static_cast<uint32_t>(report_.scratch_physical >> 44)) &&
        write(hub, r.fault_default_low, static_cast<uint32_t>(report_.dummy_page_bus >> 12)) &&
        write(hub, r.fault_default_high, static_cast<uint32_t>(report_.dummy_page_bus >> 44)) &&
        modify(hub, r.fault_control2, 0, fault2_active_page_migration_retry);

    // init_tlb_regs: L1 TLB on, system access mode 3, advanced driver model, UC.
    ok = ok && modify(hub, r.l1_tlb_control,
        l1_enable_tlb | l1_system_access_mode | l1_unmapped_access | l1_advanced_driver_model |
            l1_eco_bits | l1_mtype,
        l1_enable_tlb | (3u << l1_system_access_mode_shift) | l1_advanced_driver_model |
            (mtype_uc << l1_mtype_shift));

    // init_cache_regs (translate_further is false on GMC11).
    ok = ok && modify(hub, r.l2_control,
        l2_enable_cache | l2_fragment_processing | l2_default_page_out_to_system |
            l2_pde0_tag_generation | l2_pde_fault_classification | l2_context1_identity_mode |
            l2_identity_fragment_size,
        l2_enable_cache | l2_default_page_out_to_system | (1u << l2_context1_identity_mode_shift)) &&
        modify(hub, r.l2_control2, 0, l2_invalidate_all_l1_tlbs | l2_invalidate_l2_cache) &&
        write(hub, r.l2_control3, (r.l2_control3_default & ~(l3_bank_select | l3_bigk_fragment)) |
            9u | (6u << l3_bigk_fragment_shift)) &&
        write(hub, r.l2_control4,
            r.l2_control4_default & ~(l4_pde_request_physical | l4_pte_request_physical)) &&
        write(hub, r.l2_control5, r.l2_control5_default & ~l5_smallk_fragment);

    // enable_system_domain: VMID0 flat (depth 0), no retry.
    ok = ok && modify(hub, r.context0_control, context_enable | context_depth | context_retry_fault,
                      context_enable);

    // disable_identity_aperture.
    ok = ok && write(hub, r.identity_low_low, 0xffffffffu) && write(hub, r.identity_low_high, 0xf) &&
        write(hub, r.identity_high_low, 0) && write(hub, r.identity_high_high, 0) &&
        write(hub, r.identity_offset_low, 0) && write(hub, r.identity_offset_high, 0);

    // setup_vmid_config: contexts 1..15; amdgpu_noretry -1 makes the retry bit 0.
    for (uint32_t context = 0; ok && context < user_contexts; ++context) {
        const uint32_t control = context_enable | (vm_levels << context_depth_shift) |
            context1_fault_defaults | ((vm_block_size - 9) << context_block_size_shift);
        ok = modify(hub, r.context1_control + context,
                    context_enable | context_depth | context1_fault_defaults | context_block_size |
                        context_retry_fault,
                    control) &&
            write(hub, r.context1_start_low + context * 2, 0) &&
            write(hub, r.context1_start_high + context * 2, 0) &&
            write(hub, r.context1_end_low + context * 2, low(vm_max_pfn - 1)) &&
            write(hub, r.context1_end_high + context * 2, high(vm_max_pfn - 1));
    }

    // program_invalidation: full address range on all 18 engines.
    for (uint32_t engine = 0; ok && engine < invalidate_engines; ++engine) {
        ok = write(hub, r.invalidate0_range_low + engine * 2, 0xffffffffu) &&
             write(hub, r.invalidate0_range_high + engine * 2, 0x1f);
    }

    return ok ? GmcStatus::enabled : GmcStatus::mmio_failure;
}

GmcStatus Gmc11::fault_defaults(VmHub hub) noexcept {
    using namespace vmhub_field;

    // gfxhub_v3_0_set_fault_enable_default: do not halt CP on a page fault.
    if (hub == VmHub::gfxhub &&
        !modify(hub, gc_cp_debug, 0, gc_cp_debug_utcl1_error_halt_disable)) {
        return GmcStatus::mmio_failure;
    }

    // value true: every default-page redirect enabled, crash bits untouched.
    return modify(hub, layout(hub).fault_control, fault_enable_defaults, fault_enable_defaults) ?
               GmcStatus::enabled : GmcStatus::mmio_failure;
}

GmcStatus Gmc11::flush_tlb_firmware(VmHub hub, uint32_t vmid, uint32_t flush_type,
                                    const GmcFirmwareInvalidation& firmware) noexcept {
    if (!initialized_ || faulted_ ||
        (hub == VmHub::mmhub ? !report_.mmhub_enabled : !report_.gfxhub_enabled)) {
        return finish(faulted_ ? GmcStatus::quarantined : GmcStatus::unavailable);
    }

    if (vmid > 15 || flush_type > gmc_max_flush_type || firmware.submit == nullptr) {
        return finish(GmcStatus::invalid_argument);
    }

    const VmHubLayout& r = layout(hub);
    const uint32_t base = registers_.base(hub);
    uint8_t frame[gmc_mes_frame_bytes]{};
    encode_mes_reg_write_reg_wait(base + r.invalidate0_request + gart_invalidate_engine,
                                  base + r.invalidate0_ack + gart_invalidate_engine,
                                  gmc11_invalidate_request(vmid, flush_type), 1u << vmid,
                                  sdma::CommandBuffer(frame, sizeof(frame)));

    // gmc_v11_0_flush_gpu_tlb flushes HDP before choosing the MES path.
    if (!nbio_->flush_hdp()) {
        faulted_ = true;
        return finish(GmcStatus::mmio_failure);
    }

    if (!firmware.submit(firmware.context, ByteView(frame, sizeof(frame)),
                         gmc_mes_misc_api_status_offset)) {
        faulted_ = true;
        return finish(GmcStatus::timeout);
    }

    ++report_.flushes;
    ++report_.firmware_flushes;
    return finish(GmcStatus::flushed);
}

GmcStatus Gmc11::flush_tlb(VmHub hub, uint32_t vmid, uint32_t flush_type) noexcept {
    using namespace vmhub_field;

    if (!initialized_ || faulted_ || vmid > 15 ||
        (hub == VmHub::mmhub ? !report_.mmhub_enabled : !report_.gfxhub_enabled)) {
        return finish(faulted_ ? GmcStatus::quarantined : GmcStatus::unavailable);
    }

    if (flush_type > gmc_max_flush_type) {
        return finish(GmcStatus::invalid_argument);
    }

    const VmHubLayout& r = layout(hub);
    const uint32_t semaphore = r.invalidate0_semaphore + gart_invalidate_engine;
    const uint32_t request = r.invalidate0_request + gart_invalidate_engine;
    const uint32_t ack = r.invalidate0_ack + gart_invalidate_engine;
    const bool use_semaphore = hub == VmHub::mmhub;
    const uint32_t value = gmc11_invalidate_request(vmid, flush_type);
    uint32_t observed = 0;

    if (!nbio_->flush_hdp()) {
        faulted_ = true;
        return finish(GmcStatus::mmio_failure);
    }

    if (use_semaphore) {
        bool acquired = false;

        for (uint32_t poll = 0; poll < policy_.poll_iterations; ++poll) {
            if (!read(hub, semaphore, observed)) {
                faulted_ = true;
                return finish(GmcStatus::mmio_failure);
            }

            if ((observed & 1u) != 0) {
                acquired = true;
                break;
            }

            io_.delay_microseconds(io_.context, 1);
        }

        // Linux logs a semaphore timeout and still issues the invalidation.
        report_.semaphore_timeout = report_.semaphore_timeout || !acquired;
    }

    if (!write(hub, request, value)) {
        faulted_ = true;
        return finish(GmcStatus::mmio_failure);
    }

    bool acknowledged = false;

    for (uint32_t poll = 0; poll < policy_.poll_iterations; ++poll) {
        if (!read(hub, ack, observed)) {
            faulted_ = true;
            return finish(GmcStatus::mmio_failure);
        }

        if ((observed & (1u << vmid)) != 0) {
            acknowledged = true;
            break;
        }

        io_.delay_microseconds(io_.context, 1);
    }

    if (use_semaphore && !write(hub, semaphore, 0)) {
        faulted_ = true;
        return finish(GmcStatus::mmio_failure);
    }

    // Additional private MMHUB invalidation (RESERVED_CACHE_PRIVATE_INVALIDATION).
    if (hub == VmHub::mmhub &&
        (!modify(hub, r.bank_select_reserved_cid2, 0, private_invalidation) ||
         !read(hub, r.bank_select_reserved_cid2, observed))) {
        faulted_ = true;
        return finish(GmcStatus::mmio_failure);
    }

    // A missing ACK leaves translation state unknown: fail closed.
    if (!acknowledged) {
        faulted_ = true;
        return finish(GmcStatus::timeout);
    }

    ++report_.flushes;
    return finish(GmcStatus::flushed);
}

GmcStatus Gmc11::enable_mmhub() noexcept {
    if (!initialized_ || !report_.memory_retained || report_.mmhub_enabled || faulted_) {
        return finish(faulted_ ? GmcStatus::quarantined : GmcStatus::busy);
    }

    auto status = gart_enable(VmHub::mmhub);

    if (status == GmcStatus::enabled && !nbio_->flush_hdp()) {
        status = GmcStatus::mmio_failure;
    }

    if (status == GmcStatus::enabled) {
        status = fault_defaults(VmHub::mmhub);
    }

    if (status != GmcStatus::enabled) {
        faulted_ = true;
        return finish(status);
    }

    report_.mmhub_enabled = true;
    status = flush_tlb(VmHub::mmhub, 0);
    return status == GmcStatus::flushed ? finish(GmcStatus::enabled) : status;
}

GmcStatus Gmc11::enable_gfxhub() noexcept {
    if (!initialized_ || !report_.memory_retained || report_.gfxhub_enabled || faulted_) {
        return finish(faulted_ ? GmcStatus::quarantined : GmcStatus::busy);
    }

    auto status = gart_enable(VmHub::gfxhub);

    if (status == GmcStatus::enabled && !nbio_->flush_hdp()) {
        status = GmcStatus::mmio_failure;
    }

    if (status == GmcStatus::enabled) {
        status = fault_defaults(VmHub::gfxhub);
    }

    if (status != GmcStatus::enabled) {
        faulted_ = true;
        return finish(status);
    }

    report_.gfxhub_enabled = true;
    status = flush_tlb(VmHub::gfxhub, 0);
    return status == GmcStatus::flushed ? finish(GmcStatus::enabled) : status;
}

GmcStatus Gmc11::map_system_pages(uint64_t gart_offset, ByteView dma_page_addresses,
                                  memory::Access access) noexcept {
    if (!initialized_ || faulted_ || !report_.memory_retained) {
        return finish(faulted_ ? GmcStatus::quarantined : GmcStatus::unavailable);
    }

    const uint64_t pages = dma_page_addresses.size() / 8;

    if (pages == 0 || dma_page_addresses.size() % 8 != 0 || (gart_offset & (page_bytes - 1)) != 0 ||
        gart_offset > gart_bytes || pages > (gart_bytes - gart_offset) / page_bytes) {
        return finish(GmcStatus::invalid_argument);
    }

    const sdma::CommandBuffer table(table_cpu_, gart_table_bytes);
    const auto window = table.subbuffer(static_cast<size_t>(gart_offset / page_bytes * 8),
                                        static_cast<size_t>(pages * 8));
    memory::PageTableResult result{};

    // The shared GFX11 encoder validates the whole list before writing.
    if (memory::encode_system_ptes(report_.gart.start + gart_offset, dma_page_addresses, access,
                                   window, result) != memory::PteStatus::prepared) {
        return finish(GmcStatus::invalid_argument);
    }

    // gmc_v11_0 gart_pte_flags add AMDGPU_PTE_EXECUTABLE to the shared encoder's
    // valid|system|snooped|MTYPE_UC|R|W entry.
    for (uint64_t page = 0; page < pages; ++page) {
        uint32_t pte_low = 0;
        ByteView(table_cpu_, gart_table_bytes)
            .read_u32(static_cast<size_t>((gart_offset / page_bytes + page) * 8), pte_low);
        window.write_u32(static_cast<size_t>(page * 8), pte_low | pte_executable);
    }

    const auto status = invalidate_gart();
    return status == GmcStatus::flushed ? finish(GmcStatus::mapped) : status;
}

GmcStatus Gmc11::unmap_system_pages(uint64_t gart_offset, uint64_t pages) noexcept {
    if (!initialized_ || !report_.memory_retained) {
        return finish(GmcStatus::unavailable);
    }

    if (pages == 0 || (gart_offset & (page_bytes - 1)) != 0 || gart_offset > gart_bytes ||
        pages > (gart_bytes - gart_offset) / page_bytes) {
        return finish(GmcStatus::invalid_argument);
    }

    // amdgpu_gart_unbind: the dummy page with no flags, then invalidate_tlb.
    const sdma::CommandBuffer table(table_cpu_, gart_table_bytes);
    const uint64_t unbound = report_.dummy_page_bus & pte_address_mask;

    for (uint64_t page = 0; page < pages; ++page) {
        const auto entry = static_cast<size_t>((gart_offset / page_bytes + page) * 8);
        table.write_u32(entry, low(unbound));
        table.write_u32(entry + 4, high(unbound));
    }

    if (faulted_) {
        return finish(GmcStatus::quarantined);
    }

    return invalidate_gart();
}

GmcStatus Gmc11::reset_gart_table() noexcept {
    if (!initialized_ || !report_.memory_retained || table_cpu_ == nullptr) {
        return finish(GmcStatus::unavailable);
    }

    if (faulted_) {
        return finish(GmcStatus::quarantined);
    }

    if (report_.mmhub_enabled || report_.gfxhub_enabled) {
        return finish(GmcStatus::busy);
    }

    const sdma::CommandBuffer entries(table_cpu_, gart_table_bytes);
    const uint64_t unbound = report_.dummy_page_bus & pte_address_mask;

    for (uint32_t offset = 0; offset < gart_table_bytes; offset += 8) {
        if (!entries.write_u32(offset, low(unbound)) || !entries.write_u32(offset + 4, high(unbound))) {
            return finish(GmcStatus::invalid_memory);
        }
    }

    if (!nbio_->flush_hdp()) {
        faulted_ = true;
        return finish(GmcStatus::mmio_failure);
    }

    return finish(GmcStatus::prepared);
}

// amdgpu_gart_invalidate_tlb: HDP publication and a VMID0 flush of every enabled hub.
GmcStatus Gmc11::invalidate_gart() noexcept {
    const VmHub hubs[]{VmHub::mmhub, VmHub::gfxhub};

    for (const auto hub : hubs) {
        const bool enabled = hub == VmHub::mmhub ? report_.mmhub_enabled : report_.gfxhub_enabled;

        const GmcStatus flushed = !enabled ? GmcStatus::flushed :
            firmware_.submit != nullptr ? flush_tlb_firmware(hub, 0, 0, firmware_) : flush_tlb(hub, 0);

        if (flushed != GmcStatus::flushed) {
            return report_.status;
        }
    }

    if (!report_.mmhub_enabled && !report_.gfxhub_enabled && !nbio_->flush_hdp()) {
        return finish(GmcStatus::mmio_failure);
    }

    return finish(GmcStatus::flushed);
}

GmcStatus Gmc11::disable_hub(VmHub hub) noexcept {
    using namespace vmhub_field;

    if (!initialized_) {
        return finish(GmcStatus::unavailable);
    }

    const VmHubLayout& r = layout(hub);
    bool ok = true;

    for (uint32_t context = 0; ok && context < 16; ++context) {
        ok = write(hub, r.context0_control + context, 0);
    }

    ok = ok && modify(hub, r.l1_tlb_control, l1_enable_tlb | l1_advanced_driver_model, 0) &&
         modify(hub, r.l2_control, l2_enable_cache, 0) && write(hub, r.l2_control3, 0);

    if (!ok) {
        faulted_ = true;
        return finish(GmcStatus::mmio_failure);
    }

    (hub == VmHub::mmhub ? report_.mmhub_enabled : report_.gfxhub_enabled) = false;
    return finish(faulted_ ? GmcStatus::quarantined : GmcStatus::disabled);
}

GmcStatus Gmc11::release_after_reset() noexcept {
    if (recovery_.confirm_reset_quiescence == nullptr ||
        !recovery_.confirm_reset_quiescence(recovery_.context)) {
        return finish(GmcStatus::quarantined);
    }

    if (report_.memory_retained && manager_ != nullptr) {
        manager_->unretain(table_);
        manager_->unretain(scratch_);
    }

    manager_ = nullptr;
    table_ = {};
    scratch_ = {};
    table_cpu_ = nullptr;
    report_.memory_retained = false;
    report_.mmhub_enabled = false;
    report_.gfxhub_enabled = false;
    initialized_ = false;
    faulted_ = false;
    return finish(GmcStatus::released);
}

} // namespace rx7900::hardware

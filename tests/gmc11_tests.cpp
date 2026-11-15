/*
Module: rx7900.tests.gmc11
Purpose: Verify GART layout, hub programming values, TLB invalidation and failures.
Created: 2026-10-21
Architecture: Synthetic register/VRAM machine; no memory controller is programmed.
*/
#include <algorithm>
#include <memory>

#include "asic_test_device.hpp"
#include "gmc11.hpp"

namespace {

using namespace rx7900::hardware;
using rx7900_test::SimulatedNavi31;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

constexpr uint64_t dummy_page = 0x0000001234567000ULL;

struct Fixture final {
    std::unique_ptr<SimulatedNavi31> device = std::make_unique<SimulatedNavi31>();
    rx7900_test::VramBackend backend{};
    std::unique_ptr<rx7900::memory::Manager> manager = std::make_unique<rx7900::memory::Manager>();
    Nbio nbio{};
    std::unique_ptr<Gmc11> gmc = std::make_unique<Gmc11>();
    GmcRegisters registers{};
    rx7900::memory::Handle table{};
    rx7900::memory::Handle scratch{};

    Fixture() {
        backend.device = device.get();
        manager->initialize({rx7900_test::fb_base + rx7900_test::arena_offset,
                             rx7900_test::arena_bytes}, backend.backend());
        const auto binary = rx7900_test::navi31_discovery();
        NbioRegisters nbio_registers;
        validate_nbio_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                nbio_registers);
        validate_gmc_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, registers);
        nbio.initialize(nbio_registers, device->io());
        nbio.program_common();
        manager->allocate(gart_table_bytes, 0x1000, rx7900::memory::Access::read_write, true, table);
        manager->allocate(0x1000, 0x1000, rx7900::memory::Access::read_write, false, scratch);
        gmc->initialize(registers, device->io(), nbio, rx7900_test::navi31_vram(), {100},
                        {device.get(), SimulatedNavi31::confirm});
    }

    uint32_t mm(uint32_t reg) { return device->reg(rx7900_test::mmhub_base + reg); }
    uint32_t gc(uint32_t reg) { return device->reg(rx7900_test::gc_base + reg); }

    uint64_t address(rx7900::memory::Handle handle) {
        rx7900::memory::AllocationInfo info{};
        manager->describe(handle, info);
        return info.gpu.virtual_address;
    }
};

void layout_and_discovery() {
    GartLayout gart{};
    expect(compute_gart_layout(rx7900_test::navi31_vram(), gart) &&
           gart.start == 0x00007fff00000000ULL && gart.end == 0x00007fff1fffffffULL,
           "512 MiB GART at the top of the 47-bit MC space, 4 GiB aligned");
    VramLocation high = rx7900_test::navi31_vram();
    high.mc_end = 0x00007fffffffffffULL;
    expect(!compute_gart_layout(high, gart), "VRAM reaching the MC hole leaves no GART room");

    const auto binary = rx7900_test::navi31_discovery();
    GmcRegisters registers;
    expect(validate_gmc_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                  registers) == DiscoveryStatus::validated &&
           registers.base(VmHub::mmhub) == rx7900_test::mmhub_base &&
           registers.base(VmHub::gfxhub) == rx7900_test::gc_base, "MMHUB3.0.0 and GC11.0.0 validated");
    const auto no_gc = rx7900_test::discovery_with({
        {discovery_mmhub_hardware_id, 3, 0, {rx7900_test::mmhub_base}}});
    expect(validate_gmc_discovery({no_gc.data(), no_gc.size()}, rx7900_test::aperture_bytes,
                                  registers) == DiscoveryStatus::invalid_binary && !registers.valid(),
           "missing GC record refused");
    expect(validate_gmc_discovery({binary.data(), binary.size()}, 0x60000, registers) ==
           DiscoveryStatus::invalid_argument, "hub registers outside BAR5 refused");
}

void mmhub_programming() {
    Fixture fixture;
    expect(fixture.gmc->enable_mmhub() == GmcStatus::busy, "hubs need a prepared GART");
    expect(fixture.gmc->prepare_gart(*fixture.manager, fixture.table, fixture.scratch, 0x123) ==
           GmcStatus::invalid_argument, "unaligned dummy page refused");
    expect(fixture.gmc->prepare_gart(*fixture.manager, fixture.table, fixture.scratch, dummy_page) ==
           GmcStatus::prepared, "GART table prepared");
    const uint64_t table = fixture.address(fixture.table);
    const uint64_t physical = table - rx7900_test::fb_base + (0x20ULL << 24);
    expect(fixture.device->vram_u32(table) == 0x34567000u && fixture.device->vram_u32(table + 4) == 0x12 &&
           fixture.device->vram_u32(table + gart_table_bytes - 8) == 0x34567000u,
           "every GART entry starts as the unbound dummy-page pattern without flags");
    expect(fixture.gmc->report().page_directory == (physical | 1),
           "page directory is the VRAM physical address with VALID");

    const auto flushes = fixture.device->hdp_flushes;
    expect(fixture.gmc->enable_mmhub() == GmcStatus::enabled, "MMHUB GART enabled");
    const auto& r = mmhub_3_0_0_layout;
    const uint64_t scratch = fixture.address(fixture.scratch) - rx7900_test::fb_base + (0x20ULL << 24);
    expect(fixture.mm(r.context0_base_low) == static_cast<uint32_t>(physical | 1) &&
           fixture.mm(r.context0_start_low) == 0xfff00000u && fixture.mm(r.context0_start_high) == 7 &&
           fixture.mm(r.context0_end_low) == 0xfff1ffffu && fixture.mm(r.context0_end_high) == 7,
           "VMID0 page table base and GART range");
    expect(fixture.mm(r.agp_base) == 0 && fixture.mm(r.agp_bot) == 0xffffff && fixture.mm(r.agp_top) == 0 &&
           fixture.mm(r.system_aperture_low) ==
               static_cast<uint32_t>(rx7900_test::fb_base >> 18) &&
           fixture.mm(r.system_aperture_high) ==
               static_cast<uint32_t>(rx7900_test::navi31_vram().mc_end >> 18) &&
           fixture.mm(r.default_address_lsb) == static_cast<uint32_t>(scratch >> 12) &&
           fixture.mm(r.fault_default_low) == static_cast<uint32_t>(dummy_page >> 12) &&
           (fixture.mm(r.fault_control2) & 0x40000) != 0,
           "AGP disabled, FB system aperture, scratch default page and dummy fault page");
    expect(fixture.mm(r.l1_tlb_control) == (1u | (3u << 3) | 0x40u | (3u << 11)) &&
           fixture.mm(r.l2_control) == (1u | 0x800u | (1u << 19)) &&
           fixture.mm(r.l2_control2) == 3u && fixture.mm(r.l2_control3) == 0x80130009u &&
           fixture.mm(r.l2_control4) == 0x1u && fixture.mm(r.l2_control5) == 0x3fe0u,
           "L1 TLB, L2 cache and bank/fragment values without translate_further");
    expect(fixture.mm(r.context0_control) == 1u && fixture.mm(r.identity_low_low) == 0xffffffffu &&
           fixture.mm(r.identity_low_high) == 0xfu, "flat VMID0 and disabled identity aperture");

    bool contexts = true;

    for (uint32_t context = 0; context < 15; ++context) {
        contexts = contexts && fixture.mm(r.context1_control + context) == 0x555407u &&
                   fixture.mm(r.context1_end_low + context * 2) == 0xffffffffu &&
                   fixture.mm(r.context1_end_high + context * 2) == 0xfu;
    }

    bool engines = true;

    for (uint32_t engine = 0; engine < 18; ++engine) {
        engines = engines && fixture.mm(r.invalidate0_range_low + engine * 2) == 0xffffffffu &&
                  fixture.mm(r.invalidate0_range_high + engine * 2) == 0x1fu;
    }

    expect(contexts, "contexts 1..15: 3 levels, block size 9, every fault default, no retry");
    expect(engines, "all 18 invalidation engines cover the full range");
    expect((fixture.mm(r.fault_control) & 0x1ffcu) == 0x1ffcu &&
           (fixture.mm(r.fault_control) & 0xc0000000u) == 0, "fault defaults redirect, no crash");
    expect(fixture.device->mmhub_invalidations == 1 &&
           fixture.mm(r.invalidate0_request + 17) == 0x00f80001u &&
           fixture.mm(r.invalidate0_semaphore + 17) == 0 &&
           (fixture.mm(r.bank_select_reserved_cid2) & (1u << 25)) != 0 &&
           fixture.device->hdp_flushes >= flushes + 2,
           "engine 17 VMID0 legacy flush with semaphore release and private invalidation");
    expect(fixture.gmc->enable_mmhub() == GmcStatus::busy, "MMHUB enabled once");

    expect(fixture.gmc->enable_gfxhub() == GmcStatus::enabled &&
           fixture.gc(gfxhub_3_0_layout.l2_control3) == 0x80130009u &&
           (fixture.gc(gc_cp_debug) & gc_cp_debug_utcl1_error_halt_disable) != 0 &&
           fixture.device->gfxhub_invalidations == 1 &&
           fixture.gc(gfxhub_3_0_layout.bank_select_reserved_cid2) == 0,
           "GFXHUB uses the same sequence, CP_DEBUG halt disable, no private invalidation");

    const uint8_t page[8]{0x00, 0x10, 0x20, 0x30, 0x00, 0x00, 0x00, 0x00};
    expect(fixture.gmc->map_system_pages(0x2000, {page, sizeof(page)}, rx7900::memory::Access::read_write) ==
           GmcStatus::mapped && (fixture.device->vram_u32(table + 16) & 0xfffff000u) == 0x30201000u &&
           (fixture.device->vram_u32(table + 16) & 1u) != 0 && fixture.device->mmhub_invalidations == 2 &&
           fixture.device->gfxhub_invalidations == 2, "system page bound and both hubs flushed");
    expect(fixture.gmc->map_system_pages(gart_bytes, {page, sizeof(page)},
                                         rx7900::memory::Access::read_write) == GmcStatus::invalid_argument,
           "binding past the GART refused");
    expect((fixture.device->vram_u32(table + 16) & 0xfffu) == 0x077u &&
           fixture.device->vram_u32(table + 20) == 0x00030000u,
           "GART PTE: valid, system, snooped, executable, readable, writable, MTYPE_UC");
    const uint32_t dummy_low = fixture.device->vram_u32(table + 8);
    const uint32_t dummy_high = fixture.device->vram_u32(table + 12);
    expect(fixture.gmc->unmap_system_pages(0x2000, 1) == GmcStatus::flushed &&
           fixture.device->vram_u32(table + 16) == dummy_low && fixture.device->vram_u32(table + 20) == dummy_high &&
           (dummy_low & 0xfffu) == 0 && fixture.device->mmhub_invalidations == 3 &&
           fixture.device->gfxhub_invalidations == 3, "unbind restores the flagless dummy entry and flushes");
    expect(fixture.gmc->unmap_system_pages(0x2001, 1) == GmcStatus::invalid_argument &&
           fixture.gmc->unmap_system_pages(gart_bytes, 1) == GmcStatus::invalid_argument,
           "misaligned or out-of-range unbind refused");

    expect(fixture.gmc->release_after_reset() == GmcStatus::quarantined, "release needs reset proof");
    fixture.device->reset_confirmed = true;
    expect(fixture.gmc->release_after_reset() == GmcStatus::released, "confirmed reset releases leases");
}

void failures() {
    Fixture ack;
    ack.gmc->prepare_gart(*ack.manager, ack.table, ack.scratch, dummy_page);
    ack.device->hub_ack_bits = 0;
    expect(ack.gmc->enable_mmhub() == GmcStatus::timeout && ack.gmc->flush_tlb(VmHub::mmhub, 0) ==
           GmcStatus::quarantined, "missing invalidation ACK quarantines the GMC");

    Fixture semaphore;
    semaphore.gmc->prepare_gart(*semaphore.manager, semaphore.table, semaphore.scratch, dummy_page);
    semaphore.device->hub_semaphore_granted = false;
    expect(semaphore.gmc->enable_mmhub() == GmcStatus::enabled && semaphore.gmc->report().semaphore_timeout,
           "semaphore timeout is reported and the flush proceeds, as Linux does");

    Fixture small;
    rx7900::memory::Handle tiny{};
    small.manager->allocate(0x1000, 0x1000, rx7900::memory::Access::read_write, true, tiny);
    expect(small.gmc->prepare_gart(*small.manager, tiny, small.scratch, dummy_page) ==
           GmcStatus::invalid_memory, "undersized GART table refused");

    Fixture writes;
    writes.gmc->prepare_gart(*writes.manager, writes.table, writes.scratch, dummy_page);
    writes.device->writes_ok = false;
    expect(writes.gmc->enable_mmhub() == GmcStatus::mmio_failure &&
           writes.gmc->enable_gfxhub() == GmcStatus::quarantined, "MMIO failure quarantines");

    Gmc11 fresh;
    expect(fresh.flush_tlb(VmHub::mmhub, 0) == GmcStatus::unavailable, "uninitialized flush refused");
}

struct MesStub final {
    uint32_t frame[gmc_mes_frame_bytes / 4]{};
    uint32_t status_offset = 0;
    unsigned calls = 0;

    static bool submit(void* context, rx7900::ByteView frame, uint32_t offset) noexcept {
        auto& self = *static_cast<MesStub*>(context);
        ++self.calls;
        self.status_offset = offset;

        for (uint32_t index = 0; index < gmc_mes_frame_bytes / 4; ++index) {
            frame.read_u32(index * 4, self.frame[index]);
        }

        return true;
    }
};

void process_vm_flushes() {
    expect(gmc11_invalidate_request(0, 0) == 0x00f80001u && gmc11_invalidate_request(5, 2) == 0x00fa0020u &&
           gmc11_invalidate_request(15, 7) == 0x00ff8000u,
           "get_invalidate_req: VMID bit, FLUSH_TYPE 18:16, L2 PTE/PDE0-2 and L1 invalidation");

    Fixture fixture;
    fixture.gmc->prepare_gart(*fixture.manager, fixture.table, fixture.scratch, dummy_page);
    MesStub mes;
    expect(fixture.gmc->flush_tlb_firmware(VmHub::mmhub, 5, 1, {&mes, MesStub::submit}) ==
           GmcStatus::unavailable && mes.calls == 0, "MES flush of a disabled hub refused");
    fixture.gmc->enable_mmhub();
    fixture.gmc->enable_gfxhub();
    const auto& r = mmhub_3_0_0_layout;
    const auto invalidations = fixture.device->mmhub_invalidations;
    expect(fixture.gmc->flush_tlb(VmHub::mmhub, 5, 1) == GmcStatus::flushed &&
           fixture.mm(r.invalidate0_request + 17) == 0x00f90020u && fixture.mm(r.invalidate0_semaphore + 17) == 0 &&
           fixture.device->mmhub_invalidations == invalidations + 1,
           "register path for VMID 5, lightweight flush, engine 17 with semaphore release");
    expect(fixture.gmc->flush_tlb(VmHub::mmhub, 5, 8) == GmcStatus::invalid_argument &&
           fixture.gmc->flush_tlb(VmHub::mmhub, 16, 0) == GmcStatus::unavailable &&
           fixture.device->mmhub_invalidations == invalidations + 1, "flush type 8 and VMID 16 refused");

    const auto hdp = fixture.device->hdp_flushes;
    expect(fixture.gmc->flush_tlb_firmware(VmHub::mmhub, 5, 1, {&mes, MesStub::submit}) == GmcStatus::flushed &&
           mes.calls == 1 && fixture.device->mmhub_invalidations == invalidations + 1 &&
           fixture.device->hdp_flushes == hdp + 1 && fixture.gmc->report().firmware_flushes == 1,
           "MES path: HDP flush and one frame, no register invalidation");
    expect(mes.frame[0] == 0x000400e1u && mes.frame[1] == 4 && mes.frame[6] == 1 && mes.frame[7] == 0x00f90020u &&
           mes.frame[8] == 0x20 && mes.frame[9] == rx7900_test::mmhub_base + r.invalidate0_request + 17 &&
           mes.frame[10] == rx7900_test::mmhub_base + r.invalidate0_ack + 17 && mes.frame[63] == 0 &&
           mes.status_offset == gmc_mes_misc_api_status_offset,
           "amdgpu_mes_reg_write_reg_wait frame with absolute REQ/ACK offsets");
    expect(fixture.gmc->flush_tlb_firmware(VmHub::gfxhub, 5, 9, {&mes, MesStub::submit}) ==
               GmcStatus::invalid_argument &&
           fixture.gmc->flush_tlb_firmware(VmHub::gfxhub, 5, 0, {nullptr, nullptr}) ==
               GmcStatus::invalid_argument && mes.calls == 1, "invalid flush type or no submitter refused");

    uint8_t small[gmc_mes_frame_bytes - 4]{};
    expect(!encode_mes_reg_write_reg_wait(1, 2, 3, 4, {small, sizeof(small)}), "undersized frame refused");

    GmcRingFlush ring{};
    const uint64_t pd = 0x0000000123456001ULL;
    expect(gmc11_emit_flush_gpu_tlb(fixture.registers, VmHub::mmhub, 3, pd, 5, ring) && ring.count == 5 &&
           ring.ops[0].kind == GmcRingOpKind::reg_wait && ring.ops[0].reg0 == 0x1a756 &&
           ring.ops[0].value == 1 && ring.ops[0].mask == 1 &&
           ring.ops[1].kind == GmcRingOpKind::write && ring.ops[1].reg0 == 0x1a7b1 &&
           ring.ops[1].value == 0x23456001u && ring.ops[2].reg0 == 0x1a7b2 && ring.ops[2].value == 1 &&
           ring.ops[3].kind == GmcRingOpKind::write_wait && ring.ops[3].reg0 == 0x1a768 &&
           ring.ops[3].reg1 == 0x1a77a && ring.ops[3].value == 0x00f80008u && ring.ops[3].mask == 8 &&
           ring.ops[4].kind == GmcRingOpKind::write && ring.ops[4].reg0 == 0x1a756 && ring.ops[4].value == 0,
           "emit_flush_gpu_tlb MMHUB: semaphore, CONTEXT3 base, REQ/ACK on engine 5, release");
    expect(gmc11_emit_flush_gpu_tlb(fixture.registers, VmHub::gfxhub, 3, pd, 5, ring) && ring.count == 3 &&
           ring.ops[0].reg0 == 0x2959 && ring.ops[1].reg0 == 0x295a && ring.ops[2].reg0 == 0x2910 &&
           ring.ops[2].reg1 == 0x2922, "emit_flush_gpu_tlb GFXHUB: no semaphore");
    expect(!gmc11_emit_flush_gpu_tlb(fixture.registers, VmHub::gfxhub, 16, pd, 5, ring) &&
           !gmc11_emit_flush_gpu_tlb(fixture.registers, VmHub::gfxhub, 1, pd, 18, ring) &&
           !gmc11_emit_flush_gpu_tlb(fixture.registers, VmHub::gfxhub, 1, 1ULL << 48, 0, ring) &&
           !gmc11_emit_flush_gpu_tlb(GmcRegisters{}, VmHub::gfxhub, 1, pd, 0, ring),
           "VMID, engine, PD address and token bounds");
}

} // namespace

// gmc_v11_0_suspend / resume: the table loses its VRAM contents; it is refilled
// with the dummy pattern, rebound and the MMHUB enabled again.
void suspend_resume_table() {
    Fixture fixture;
    expect(fixture.gmc->reset_gart_table() == GmcStatus::unavailable, "no table before prepare_gart");
    fixture.gmc->prepare_gart(*fixture.manager, fixture.table, fixture.scratch, dummy_page);
    fixture.gmc->enable_mmhub();
    expect(fixture.gmc->reset_gart_table() == GmcStatus::busy, "refused while a hub may walk the table");
    const uint8_t page[8]{0x00, 0x30, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00};
    expect(fixture.gmc->map_system_pages(0x5000, {page, sizeof(page)}, rx7900::memory::Access::read_write) ==
           GmcStatus::mapped && fixture.gmc->disable_hub(VmHub::mmhub) == GmcStatus::disabled,
           "page bound, then the MMHUB suspended");
    const uint64_t table = fixture.address(fixture.table);
    std::fill(fixture.device->vram.begin(), fixture.device->vram.end(), static_cast<uint8_t>(0xa5));
    const auto flushes = fixture.device->hdp_flushes;
    expect(fixture.gmc->reset_gart_table() == GmcStatus::prepared &&
           fixture.device->vram_u32(table + 5 * 8) == 0x34567000u &&
           fixture.device->vram_u32(table + gart_table_bytes - 4) == 0x12 &&
           fixture.device->hdp_flushes == flushes + 1,
           "lost table refilled with the dummy page pattern and published");
    expect(fixture.gmc->map_system_pages(0x5000, {page, sizeof(page)}, rx7900::memory::Access::read_write) ==
           GmcStatus::mapped && (fixture.device->vram_u32(table + 5 * 8) & 3u) == 3u &&
           fixture.gmc->enable_mmhub() == GmcStatus::enabled &&
           fixture.device->cpu(fixture.gmc->gart().start + 0x5000) == fixture.device->system(0x0000002000003000ULL),
           "rebound entry translates again once the MMHUB is re-enabled");
}

int main() {
    layout_and_discovery();
    mmhub_programming();
    failures();
    process_vm_flushes();
    suspend_resume_table();
    return checks.finish("gmc11");
}

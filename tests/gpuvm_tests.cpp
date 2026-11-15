/*
Module: rx7900.tests.gpuvm
Purpose: Verify process page-table values, fragments, table lifetime, PASIDs and VM TLB invalidation.
Created: 2026-10-22
Architecture: Synthetic register/VRAM machine shared with the GMC tests; expected
entries are computed by hand from the Linux v6.12 amdgpu_vm_pt.c rules.
*/
#include <memory>

#include "asic_test_device.hpp"
#include "gpuvm.hpp"

namespace {

using namespace rx7900::hardware;
using rx7900::memory::Access;
using rx7900_test::SimulatedNavi31;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

constexpr uint64_t dummy_page = 0x0000001234567000ULL;
constexpr uint64_t fb_offset = 0x20ULL << 24;
constexpr uint64_t pde_pte = 1ULL << 54;
constexpr uint32_t osssys_base = 0x4260;
constexpr GpuVmMapFlags read_write{Access::read_write, false, GpuVmMtype::nc, false};

bool confirm_idle(void* context) noexcept { return *static_cast<bool*>(context); }

struct Firmware final {
    uint32_t frame[gmc_mes_frame_bytes / 4]{};
    uint32_t status_offset = 0;
    unsigned submissions = 0;
    bool complete = true;

    static bool submit(void* context, rx7900::ByteView frame, uint32_t offset) noexcept {
        auto& self = *static_cast<Firmware*>(context);
        ++self.submissions;
        self.status_offset = offset;

        for (uint32_t index = 0; index < gmc_mes_frame_bytes / 4; ++index) {
            frame.read_u32(index * 4, self.frame[index]);
        }

        return self.complete;
    }
};

struct Fixture final {
    std::unique_ptr<SimulatedNavi31> device = std::make_unique<SimulatedNavi31>();
    rx7900_test::VramBackend backend{};
    std::unique_ptr<rx7900::memory::Manager> manager = std::make_unique<rx7900::memory::Manager>();
    Nbio nbio{};
    std::unique_ptr<Gmc11> gmc = std::make_unique<Gmc11>();
    std::unique_ptr<GpuVm> vm = std::make_unique<GpuVm>();
    Ih6Registers osssys{};
    rx7900::memory::Handle pool{};
    bool idle = false;

    explicit Fixture(uint32_t tables = 32) {
        backend.device = device.get();
        manager->initialize({rx7900_test::fb_base + rx7900_test::arena_offset, rx7900_test::arena_bytes},
                            backend.backend());
        const auto binary = rx7900_test::navi31_discovery();
        NbioRegisters nbio_registers;
        GmcRegisters registers;
        validate_nbio_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, nbio_registers);
        validate_gmc_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, registers);
        validate_ih6_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, osssys);
        nbio.initialize(nbio_registers, device->io());
        nbio.program_common();
        rx7900::memory::Handle table{};
        rx7900::memory::Handle scratch{};
        manager->allocate(gart_table_bytes, 0x1000, Access::read_write, true, table);
        manager->allocate(0x1000, 0x1000, Access::read_write, false, scratch);
        gmc->initialize(registers, device->io(), nbio, rx7900_test::navi31_vram(), {100},
                        {device.get(), SimulatedNavi31::confirm});
        gmc->prepare_gart(*manager, table, scratch, dummy_page);
        gmc->enable_mmhub();
        gmc->enable_gfxhub();
        manager->allocate(static_cast<uint64_t>(tables) * gpuvm_table_bytes, 0x1000, Access::read_write,
                          true, pool);
        vm->init(*manager, {rx7900_test::navi31_vram(), pool, tables, {&idle, confirm_idle}}, nbio);
    }

    uint64_t mc(rx7900::memory::Handle handle) {
        rx7900::memory::AllocationInfo info{};
        manager->describe(handle, info);
        return info.gpu.virtual_address;
    }

    uint64_t vram(uint64_t size, uint64_t alignment) {
        rx7900::memory::Handle handle{};
        manager->allocate(size, alignment, Access::read_write, true, handle);
        return mc(handle);
    }

    static uint64_t pa(uint64_t mc) { return mc - rx7900_test::fb_base + fb_offset; }
    uint64_t table_pa(uint32_t slot) { return pa(mc(pool)) + slot * 4096ULL; }

    uint64_t entry(uint32_t slot, uint32_t index) {
        const uint64_t address = mc(pool) + slot * 4096ULL + index * 8ULL;
        return device->vram_u32(address) | (static_cast<uint64_t>(device->vram_u32(address + 4)) << 32);
    }
};

void pasids() {
    std::unique_ptr<PasidAllocator> pasid = std::make_unique<PasidAllocator>();
    uint32_t id = 0;
    expect(pasid->allocate(16, id) == GpuVmStatus::ready && id == 0x8000,
           "amdgpu_pasid_alloc(16) starts at 0x8000");
    expect(pasid->allocate(16, id) == GpuVmStatus::ready && id == 0x8001, "lowest free id next");
    expect(pasid->release(0x8000) == GpuVmStatus::released && pasid->allocate(16, id) ==
           GpuVmStatus::ready && id == 0x8000, "a released id is reused first");
    expect(pasid->allocate(0, id) == GpuVmStatus::invalid_argument &&
           pasid->allocate(17, id) == GpuVmStatus::invalid_argument, "only 1..16 bit ranges");
    expect(pasid->release(0) == GpuVmStatus::invalid_argument &&
           pasid->release(0x9000) == GpuVmStatus::invalid_argument &&
           pasid->release(0x10000) == GpuVmStatus::invalid_argument, "only allocated ids are released");

    for (uint32_t count = 2; count < 0x8000; ++count) {
        pasid->allocate(16, id);
    }

    expect(id == 0xffff && pasid->allocated(0xffff), "the 16-bit range fills up to 0xffff");
    expect(pasid->allocate(16, id) == GpuVmStatus::ready && id == 0x4000,
           "then the 15-bit range [0x4000, 0x7fff], as the bits loop does");

    for (uint32_t count = 0x4001; count < 0x8000; ++count) {
        pasid->allocate(16, id);
    }

    for (uint32_t count = 1; count < 0x4000; ++count) {
        pasid->allocate(16, id);
    }

    expect(id == 1 && pasid->allocate(16, id) == GpuVmStatus::exhausted && !pasid->allocated(0),
           "every id 1..0xffff handed out, PASID 0 never");
}

void init_and_first_page() {
    Fixture f;
    const auto report = f.vm->report();
    expect(report.status == GpuVmStatus::ready && report.root_mc == f.mc(f.pool) &&
           report.page_table_base_addr == f.table_pa(0) && report.pd_address == (f.table_pa(0) | 1),
           "root PD: ADD_QUEUE base is the VRAM PA, pd_addr adds VALID");
    bool cleared = true;

    for (uint32_t index = 0; index < gpuvm_table_entries; ++index) {
        cleared = cleared && f.entry(0, index) == pde_pte;
    }

    expect(cleared, "amdgpu_vm_pt_clear: every root entry is AMDGPU_PDE_PTE");

    const uint64_t page = f.vram(0x1000, 0x1000);
    const auto flushes = f.device->hdp_flushes;
    expect(f.vm->map_vram(0x10000, page, 0x1000, read_write) == GpuVmStatus::mapped, "4 KiB VRAM mapped");
    expect(f.entry(3, 16) == (Fixture::pa(page) | 0x61) && f.entry(3, 15) == 0x10 &&
           f.entry(3, 17) == 0x10, "PTE: PA | VALID | READABLE | WRITEABLE, MTYPE_NC, frag 0; "
           "untouched PTB entries keep EXECUTABLE");
    expect(f.entry(2, 0) == (f.table_pa(3) | 1) && f.entry(1, 0) == (f.table_pa(2) | 1) &&
           f.entry(0, 0) == (f.table_pa(1) | 1) && f.entry(2, 1) == pde_pte && f.entry(0, 1) == pde_pte,
           "PDB0/PDB1/root PDEs are table PA | VALID; other PD entries stay PDE_PTE");
    expect(f.device->hdp_flushes == flushes + 2 && f.vm->report().tables_live == 4 &&
           f.vm->report().tlb_seq == 0, "PTE commit and PDE commit each flush HDP; no TLB flush needed");

    GpuVmWalk walk{};
    expect(f.vm->walk(0x10000, walk) == GpuVmStatus::ready && walk.count == 4 &&
           walk.entries[3].level == GpuVmLevel::ptb && walk.entries[3].index == 16 &&
           walk.entries[3].value == (Fixture::pa(page) | 0x61), "walk reaches the PTE through all four levels");

    const GpuVmMapFlags exec_uc{Access::read, true, GpuVmMtype::wc, true};
    const uint64_t second = f.vram(0x1000, 0x1000);
    expect(f.vm->map_vram(0x11000, second, 0x1000, exec_uc) == GpuVmStatus::mapped &&
           f.entry(3, 17) == (Fixture::pa(second) | 0x31 | (3ULL << 48)),
           "read-only executable mapping of an uncached BO forces MTYPE_UC");
    expect(f.vm->map_vram(0x11000, second, 0x1000, read_write) == GpuVmStatus::overlap &&
           f.vm->map_vram(0x10800, second, 0x1000, read_write) == GpuVmStatus::invalid_argument &&
           f.vm->map_vram(0x20000, second + 1, 0x1000, read_write) == GpuVmStatus::invalid_argument &&
           f.vm->map_vram(0x20000, rx7900_test::fb_base - 0x1000, 0x1000, read_write) ==
               GpuVmStatus::invalid_argument, "overlap, unaligned VA/MC and non-VRAM MC refused");
}

void huge_and_fragments() {
    Fixture f;
    const uint64_t block = f.vram(4u << 20, 4u << 20);
    const uint64_t va = 0x100000000ULL;
    expect(f.vm->map_vram(va, block, 4u << 20, read_write) == GpuVmStatus::mapped, "4 MiB VRAM mapped");
    expect(f.entry(2, 0) == (Fixture::pa(block) | pde_pte | 0x500 | 0x61) &&
           f.entry(2, 1) == ((Fixture::pa(block) + (2u << 20)) | pde_pte | 0x500 | 0x61) &&
           f.entry(2, 2) == pde_pte && f.entry(1, 4) == (f.table_pa(2) | 1) &&
           f.vm->report().tables_live == 3,
           "fragment 10 at PDB0: two 2 MiB huge PDEs with PDE_PTE and FRAG(10), no PTB");
    GpuVmWalk walk{};
    expect(f.vm->walk(va + 0x234000, walk) == GpuVmStatus::ready && walk.count == 3 &&
           (walk.entries[2].value & pde_pte) != 0, "walk stops at the huge PDB0 entry");

    // VA 1 GiB, PA one page past a 2 MiB boundary: congruence limits fragments to 0.
    const uint64_t skew = f.vram(4u << 20, 2u << 20) + 0x1000;
    const uint64_t skew_va = 0x40000000ULL;
    expect(f.vm->map_vram(skew_va, skew, 2u << 20, read_write) == GpuVmStatus::mapped &&
           f.entry(4, 0) == (Fixture::pa(skew) | 0x61) &&
           f.entry(4, 511) == ((Fixture::pa(skew) + 511 * 0x1000ULL) | 0x61) &&
           f.entry(3, 0) == (f.table_pa(4) | 1) && f.entry(1, 1) == (f.table_pa(3) | 1),
           "incongruent PA: 512 frag-0 PTEs in a new PTB");

    // Same alignment but congruent: Linux's fragment 9 at the PTB is not reached;
    // 2 MiB VA alignment with exactly 2 MiB gives a PDB0 huge entry with FRAG(9).
    const uint64_t aligned = f.vram(2u << 20, 2u << 20);
    expect(f.vm->map_vram(0x40200000ULL, aligned, 2u << 20, read_write) == GpuVmStatus::mapped &&
           f.entry(3, 1) == (Fixture::pa(aligned) | pde_pte | 0x480 | 0x61),
           "2 MiB aligned congruent mapping: one PDB0 entry with FRAG(9)");

    const auto retired_before = f.vm->report().tables_retired;
    expect(f.vm->unmap(skew_va) == GpuVmStatus::unmapped && f.entry(3, 0) == (pde_pte | 0x480) &&
           f.vm->report().tables_retired == retired_before + 1 && f.vm->report().tlb_seq == 1 &&
           f.vm->tlb_flush_needed(),
           "unmap writes a cleared PDE_PTE|FRAG(9) entry, retires the PTB and needs a flush");
    expect(f.vm->unmap(va) == GpuVmStatus::unmapped && f.entry(2, 0) == (pde_pte | 0x500) &&
           f.entry(2, 1) == ((2u << 20) | pde_pte | 0x500) && f.vm->report().tlb_seq == 2,
           "cleared huge entries carry the advancing dst offset as Linux writes it");
    expect(f.vm->unmap(va) == GpuVmStatus::not_mapped && f.vm->unmap(va + 0x1000) == GpuVmStatus::not_mapped,
           "unmap needs the exact start of a live mapping");

    // A new 4 KiB mapping in the retired region must not reuse the retired slot.
    const uint64_t page = f.vram(0x1000, 0x1000);
    expect(f.vm->map_vram(skew_va, page, 0x1000, read_write) == GpuVmStatus::mapped &&
           f.entry(3, 0) == (f.table_pa(5) | 1) && f.entry(5, 0) == (Fixture::pa(page) | 0x61),
           "a fresh PTB replaces the huge clear entry; the retired table stays unused");
}

void system_pages() {
    Fixture f;
    constexpr uint64_t a = 0x0000001000200000ULL;
    constexpr uint64_t c = 0x0000001000900000ULL;
    constexpr uint64_t e = 0x0000001000a40000ULL;
    uint8_t list[3 * 8]{};
    const rx7900::sdma::CommandBuffer out(list, sizeof(list));
    const uint64_t pages[3]{a, a + 0x1000, c};

    for (uint32_t index = 0; index < 3; ++index) {
        out.write_u32(index * 8, static_cast<uint32_t>(pages[index]));
        out.write_u32(index * 8 + 4, static_cast<uint32_t>(pages[index] >> 32));
    }

    expect(f.vm->map_system(0x200000, {list, sizeof(list)}, true, read_write) == GpuVmStatus::mapped,
           "three GTT pages mapped");
    expect(f.entry(3, 0) == (a | 0x67 | 0x80) && f.entry(3, 1) == ((a + 0x1000) | 0x67 | 0x80) &&
           f.entry(3, 2) == (c | 0x67), "contiguous run gets FRAG(1), the lone page frag 0; "
           "VALID|SYSTEM|SNOOPED|R|W");

    const uint64_t scattered[3]{a, c, e};

    for (uint32_t index = 0; index < 3; ++index) {
        out.write_u32(index * 8, static_cast<uint32_t>(scattered[index]));
        out.write_u32(index * 8 + 4, static_cast<uint32_t>(scattered[index] >> 32));
    }

    const GpuVmMapFlags uc{Access::write, false, GpuVmMtype::cc, false};
    expect(f.vm->map_system(0x400000, {list, sizeof(list)}, false, uc) == GpuVmStatus::mapped &&
           f.entry(4, 0) == (a | 0x43 | (2ULL << 48)) && f.entry(4, 1) == (c | 0x43 | (2ULL << 48)) &&
           f.entry(4, 2) == (e | 0x43 | (2ULL << 48)),
           "scattered pages through amdgpu_vm_map_gart; USWC pages without SNOOPED; MTYPE_CC");

    // The MES self-test metadata shape: 52 contiguous pages at pfn 16 give the
    // amdgpu_vm_pte_fragment sequence 16..31 FRAG(4), 32..63 FRAG(5), 64..67 FRAG(2).
    Fixture meta;
    uint8_t run[52 * 8]{};
    const rx7900::sdma::CommandBuffer pages52(run, sizeof(run));
    constexpr uint64_t base = 0x0000001000010000ULL;

    for (uint32_t index = 0; index < 52; ++index) {
        pages52.write_u32(index * 8, static_cast<uint32_t>(base + index * 0x1000ULL));
        pages52.write_u32(index * 8 + 4, static_cast<uint32_t>((base + index * 0x1000ULL) >> 32));
    }

    expect(meta.vm->map_system(0x10000, {run, sizeof(run)}, true, read_write) == GpuVmStatus::mapped &&
           meta.entry(3, 16) == (base | 0x67 | (4u << 7)) &&
           meta.entry(3, 31) == ((base + 15 * 0x1000ULL) | 0x67 | (4u << 7)) &&
           meta.entry(3, 32) == ((base + 16 * 0x1000ULL) | 0x67 | (5u << 7)) &&
           meta.entry(3, 63) == ((base + 47 * 0x1000ULL) | 0x67 | (5u << 7)) &&
           meta.entry(3, 64) == ((base + 48 * 0x1000ULL) | 0x67 | (2u << 7)) &&
           meta.entry(3, 67) == ((base + 51 * 0x1000ULL) | 0x67 | (2u << 7)) &&
           meta.entry(3, 68) == 0x10 && meta.vm->report().entries_written == 52 + 3,
           "contiguous GTT run: Linux fragment sequence 4, 5, 2 and the exact store count");
    expect(meta.vm->unmap(0x10000) == GpuVmStatus::unmapped && meta.entry(3, 16) == 0x210 &&
           meta.entry(3, 32) == (0x10000 | 0x10 | (5u << 7)) &&
           meta.entry(3, 67) == (0x33000 | 0x10 | (2u << 7)) && meta.vm->report().tables_retired == 0,
           "amdgpu_vm_clear_freed: flags 0 plus EXECUTABLE, same fragments, dst offsets kept");

    uint8_t bad[8]{0x01, 0x10, 0, 0, 0, 0, 0, 0};
    expect(f.vm->map_system(0x800000, {bad, sizeof(bad)}, true, read_write) == GpuVmStatus::invalid_argument &&
           f.vm->map_system(0x800000, {list, 7}, true, read_write) == GpuVmStatus::invalid_argument,
           "unaligned DMA page and partial list refused");
}

void bounds_and_capacity() {
    Fixture f;
    const uint64_t page = f.vram(0x2000, 0x1000);
    const uint64_t top = (gpuvm_max_pfn - 1) << 12;
    expect(f.vm->map_vram(top, page, 0x1000, read_write) == GpuVmStatus::mapped &&
           f.entry(0, 511) == (f.table_pa(1) | 1) && f.entry(1, 511) == (f.table_pa(2) | 1) &&
           f.entry(2, 511) == (f.table_pa(3) | 1) && f.entry(3, 511) == (Fixture::pa(page) | 0x61),
           "the last pfn (2^36 - 1) uses entry 511 at every level");
    expect(f.vm->map_vram(top, page, 0x2000, read_write) == GpuVmStatus::invalid_argument &&
           f.vm->map_vram(1ULL << 48, page, 0x1000, read_write) == GpuVmStatus::invalid_argument,
           "lpfn >= max_pfn refused");
    GpuVmWalk walk{};
    expect(f.vm->walk(1ULL << 48, walk) == GpuVmStatus::invalid_argument, "walk beyond the VM refused");

    // Two pages straddling the first 512 GiB root entry: sibling and ancestor walk.
    const uint64_t straddle = (1ULL << 39) - 0x1000;
    expect(f.vm->map_vram(straddle, page, 0x2000, read_write) == GpuVmStatus::mapped &&
           f.entry(0, 0) == (f.table_pa(4) | 1) && f.entry(0, 1) == (f.table_pa(7) | 1) &&
           f.entry(6, 511) == (Fixture::pa(page) | 0x61) &&
           f.entry(9, 0) == ((Fixture::pa(page) + 0x1000) | 0x61),
           "a range crossing root entries 0 and 1 builds both subtrees");

    Fixture small(3);
    const uint64_t one = small.vram(0x1000, 0x1000);
    const auto written = small.vm->report().entries_written;
    const auto flushes = small.device->hdp_flushes;
    expect(small.vm->map_vram(0x10000, one, 0x1000, read_write) == GpuVmStatus::out_of_tables &&
           small.vm->report().entries_written == written && small.device->hdp_flushes == flushes &&
           small.entry(0, 0) == pde_pte && small.vm->report().tables_live == 1,
           "too few tables: refused before any store, metadata restored");

    Fixture many(256);
    const uint64_t pages = many.vram(gpuvm_max_mappings * 0x1000ULL, 0x1000);
    bool all = true;

    for (uint32_t index = 0; index < gpuvm_max_mappings; ++index) {
        all = all && many.vm->map_vram(0x100000 + index * 0x2000ULL, pages + index * 0x1000ULL, 0x1000,
                                       read_write) == GpuVmStatus::mapped;
    }

    expect(all && many.vm->map_vram(0x900000, page, 0x1000, read_write) == GpuVmStatus::out_of_mappings,
           "64 live mappings, the 65th refused");
}

void invalidation() {
    Fixture f;
    const uint64_t block = f.vram(4u << 20, 2u << 20) + 0x1000;
    f.vm->map_vram(0x40000000ULL, block, 2u << 20, read_write);
    f.vm->unmap(0x40000000ULL);
    std::unique_ptr<GpuVmInvalidator> invalidator = std::make_unique<GpuVmInvalidator>();
    expect(f.vm->flush_tlb(*invalidator, 2) == GpuVmStatus::invalid_argument, "flush needs a PASID");
    expect(f.vm->set_pasid(0x8000) == GpuVmStatus::ready, "PASID set");
    expect(f.vm->flush_tlb(*invalidator, 2) == GpuVmStatus::unavailable, "uninitialized invalidator refused");

    const auto io = f.device->io();
    invalidator->initialize(*f.gmc, f.osssys, io, {nullptr, nullptr}, false);
    f.device->reg(osssys_base + 9) = 0x8000;
    f.device->reg(osssys_base + 3) = 0x8001;
    const auto gfx = f.device->gfxhub_invalidations;
    const auto mm = f.device->mmhub_invalidations;
    expect(f.vm->flush_tlb(*invalidator, 2) == GpuVmStatus::flushed &&
           f.device->gfxhub_invalidations == gfx + 1 && f.device->mmhub_invalidations == mm &&
           f.device->reg(rx7900_test::gc_base + gfxhub_3_0_layout.invalidate0_request + 17) == 0x00fa0200u,
           "LUT VMID 9 holds the PASID: one GFXHUB engine-17 flush, VMID bit 9, FLUSH_TYPE 2");
    expect(f.vm->report().tables_retired == 0 && !f.vm->tlb_flush_needed() &&
           f.vm->flush_tlb(*invalidator, 2) == GpuVmStatus::flushed && f.device->gfxhub_invalidations == gfx + 1,
           "flushed retired tables are free; an unchanged tlb_seq needs no hardware access");

    uint32_t vmids = 0;
    expect(invalidator->flush_pasid(0x8000, 0, true, vmids) == GpuVmStatus::flushed && vmids == (1u << 9) &&
           f.device->gfxhub_invalidations == gfx + 2 && f.device->mmhub_invalidations == mm + 1,
           "all_hub flushes GFXHUB then MMHUB");
    expect(invalidator->flush_pasid(0x7777, 0, false, vmids) == GpuVmStatus::flushed && vmids == 0 &&
           invalidator->flush_pasid(0, 0, false, vmids) == GpuVmStatus::invalid_argument &&
           invalidator->flush_pasid(0x8000, 8, false, vmids) == GpuVmStatus::invalid_argument,
           "unmapped PASID is a no-op; PASID 0 and flush type 8 refused");

    std::unique_ptr<GpuVmInvalidator> gfxoff = std::make_unique<GpuVmInvalidator>();
    gfxoff->initialize(*f.gmc, f.osssys, io, {nullptr, nullptr}, true);
    expect(gfxoff->flush_pasid(0x8000, 0, false, vmids) == GpuVmStatus::unavailable &&
           f.device->gfxhub_invalidations == gfx + 2, "GFXOFF allowed: no GFXHUB register flush without MES");

    Firmware firmware;
    std::unique_ptr<GpuVmInvalidator> mes = std::make_unique<GpuVmInvalidator>();
    mes->initialize(*f.gmc, f.osssys, io, {&firmware, Firmware::submit}, true);
    const auto hdp = f.device->hdp_flushes;
    expect(mes->flush_pasid(0x8000, 2, false, vmids) == GpuVmStatus::flushed && firmware.submissions == 1 &&
           f.device->gfxhub_invalidations == gfx + 2 && f.device->hdp_flushes == hdp + 1,
           "MES path: one MISC frame after an HDP flush, no register invalidation");
    const uint32_t request = rx7900_test::gc_base + gfxhub_3_0_layout.invalidate0_request + 17;
    const uint32_t ack = rx7900_test::gc_base + gfxhub_3_0_layout.invalidate0_ack + 17;
    expect(firmware.frame[0] == 0x000400e1u && firmware.frame[1] == 4 && firmware.frame[6] == 1 &&
           firmware.frame[7] == 0x00fa0200u && firmware.frame[8] == (1u << 9) && firmware.frame[9] == request &&
           firmware.frame[10] == ack && firmware.frame[2] == 0 && firmware.frame[11] == 0 &&
           firmware.status_offset == 8,
           "MESAPI_MISC WAIT_REG_MEM WR_WAIT_WR_REG: REQ17/ACK17, reference, VMID mask, api_status at 8");

    firmware.complete = false;
    expect(mes->flush_pasid(0x8000, 0, false, vmids) == GpuVmStatus::quarantined &&
           f.gmc->flush_tlb(VmHub::gfxhub, 0) == GmcStatus::quarantined,
           "a MES frame without completion quarantines the GMC");

    Fixture broken;
    std::unique_ptr<GpuVmInvalidator> reads = std::make_unique<GpuVmInvalidator>();
    reads->initialize(*broken.gmc, broken.osssys, broken.device->io(), {nullptr, nullptr}, false);
    broken.device->reads_ok = false;
    expect(reads->flush_pasid(0x8000, 0, false, vmids) == GpuVmStatus::mmio_failure,
           "a failed LUT read fails closed");
}

void helpers_and_fini() {
    Fixture f;
    uint32_t reg = 0;
    expect(gpuvm_pasid_mapping_register(f.osssys, VmHub::gfxhub, 5, reg) && reg == osssys_base + 5 &&
           gpuvm_pasid_mapping_register(f.osssys, VmHub::mmhub, 5, reg) && reg == osssys_base + 0x15 &&
           !gpuvm_pasid_mapping_register(f.osssys, VmHub::mmhub, 16, reg),
           "IH_VMID_n_LUT and IH_VMID_n_LUT_MM for kernel-ring PASID mappings");

    const uint64_t page = f.vram(0x1000, 0x1000);
    f.vm->map_vram(0x10000, page, 0x1000, read_write);
    rx7900::memory::AllocationInfo info{};
    f.manager->describe(f.pool, info);
    const auto leases = info.leases;
    expect(f.vm->fini() == GpuVmStatus::retained && f.vm->report().memory_retained,
           "fini refused while an engine may walk the tables");
    f.idle = true;
    f.manager->describe(f.pool, info);
    expect(f.vm->fini() == GpuVmStatus::released && !f.vm->report().memory_retained &&
           f.manager->describe(f.pool, info) == rx7900::memory::Status::success && info.leases == leases - 1 &&
           f.vm->map_vram(0x10000, page, 0x1000, read_write) == GpuVmStatus::unavailable,
           "idle fini returns the pool lease and closes the VM");

    Fixture faulty;
    faulty.device->writes_ok = false;
    expect(faulty.vm->map_vram(0x10000, faulty.vram(0x1000, 0x1000), 0x1000, read_write) ==
               GpuVmStatus::mmio_failure &&
           faulty.vm->map_vram(0x20000, faulty.vram(0x1000, 0x1000), 0x1000, read_write) ==
               GpuVmStatus::quarantined, "a failed HDP flush quarantines the VM");
}

} // namespace

int main() {
    pasids();
    init_and_first_page();
    huge_and_fragments();
    system_pages();
    bounds_and_capacity();
    invalidation();
    helpers_and_fini();
    return checks.finish("gpuvm");
}

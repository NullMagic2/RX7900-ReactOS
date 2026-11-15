/*
Module: rx7900.tests.mes11
Purpose: Verify MES KIQ/scheduler register order, MQDs, PM4/API frames, completion and faults.
Created: 2026-10-21
Architecture: Shared MES/KIQ model (mes_test_model.hpp) over the shared synthetic Navi31 machine;
it is not hardware evidence. Expected values are derived from Linux v6.12
mes_v11_0.c, gfx_v11_0.c, mes_v11_api_def.h and v11_structs.h.
*/
#include <map>
#include <memory>
#include <utility>

#include "gfx11_queues.hpp"
#include "mes_process_model.hpp"
#include "mes_test_model.hpp"

namespace {

using namespace rx7900::hardware;
using rx7900_test::SimulatedNavi31;
using namespace rx7900_test::mes_model;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

struct Fixture final {
    MesModel model;
    rx7900_test::VramBackend backend{};
    std::unique_ptr<rx7900::memory::Manager> manager = std::make_unique<rx7900::memory::Manager>();
    Nbio nbio{};
    Mes11Registers registers{};
    std::unique_ptr<Mes11> mes = std::make_unique<Mes11>();
    Mes11Memory memory{};
    bool reset_confirmed = false;

    static bool confirm(void* context) noexcept { return *static_cast<bool*>(context); }

    explicit Fixture(Mes11Policy policy = {1000, 2100000}) {
        backend.device = model.device;
        manager->initialize({rx7900_test::fb_base + rx7900_test::arena_offset,
                             rx7900_test::arena_bytes}, backend.backend());
        const auto binary = rx7900_test::navi31_discovery();
        NbioRegisters nbio_registers;
        validate_nbio_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                nbio_registers);
        validate_mes11_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                 registers);
        nbio.initialize(nbio_registers, model.device->io());
        nbio.program_common();
        allocate(mes11_kiq_ring_bytes, memory.kiq_ring);
        allocate(mes11_scheduler_ring_bytes, memory.scheduler_ring);
        allocate(mes11_mqd_bytes, memory.kiq_mqd);
        allocate(mes11_mqd_bytes, memory.scheduler_mqd);
        allocate(mes11_eop_bytes, memory.kiq_eop);
        allocate(mes11_eop_bytes, memory.scheduler_eop);
        allocate(mes11_writeback_bytes, memory.writeback);
        mes->initialize(registers, model.io(), model.doorbells(), nbio, rx7900_test::navi31_vram(),
                        {sched_uc, kiq_uc, 2}, policy, {&reset_confirmed, confirm});
    }

    void allocate(uint32_t bytes, rx7900::memory::Handle& handle) {
        // Manager allocations are whole pages, like the Linux BOs behind them.
        const uint32_t pages = (bytes + 0xfff) & ~0xfffu;
        manager->allocate(pages, 0x1000, rx7900::memory::Access::read_write, true, handle);
    }

    uint64_t address(rx7900::memory::Handle handle) {
        rx7900::memory::AllocationInfo info{};
        manager->describe(handle, info);
        return info.gpu.virtual_address;
    }

    uint32_t vram(uint64_t address) { return model.device->vram_u32(address); }
    uint64_t wb() { return address(memory.writeback); }

    bool release_all() {
        const rx7900::memory::Handle handles[7]{memory.kiq_ring, memory.scheduler_ring,
            memory.kiq_mqd, memory.scheduler_mqd, memory.kiq_eop, memory.scheduler_eop,
            memory.writeback};
        bool ok = true;

        for (const auto& handle : handles) {
            ok = ok && manager->release(handle) == rx7900::memory::Status::success;
        }

        return ok && manager->empty();
    }
};

void discovery_and_header() {
    const auto binary = rx7900_test::navi31_discovery();
    Mes11Registers registers;
    expect(validate_mes11_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                    registers) == DiscoveryStatus::validated &&
           registers.gc_segment(0) == gc0 && registers.gc_segment(1) == gc1 &&
           registers.gc_base(0) == gc0 && registers.gc_base(1) == gc1 && registers.gc_base(2) == 0 &&
           registers.mmhub_base(0) == rx7900_test::mmhub_base && registers.mmhub_base(1) == 0 &&
           registers.osssys_base(0) == 0x4260 && registers.osssys_base(4) == 0,
           "GC/MMHUB/OSSSYS bases copied with their real counts, absent indices zero");

    const auto six = rx7900_test::discovery_with({
        {discovery_gc_hardware_id, 11, 0, {gc0, gc1, 0x1c000, 0x2402c00, 0x11, 0x22}},
        {discovery_mmhub_hardware_id, 3, 0, {0x1a000, 0x2408800}},
        {mes11_osssys_hardware_id, 6, 0, {0x4260}}});
    expect(validate_mes11_discovery({six.data(), six.size()}, rx7900_test::aperture_bytes,
                                    registers) == DiscoveryStatus::validated &&
           registers.gc_base(3) == 0x2402c00 && registers.gc_base(4) == 0x11 &&
           registers.gc_base(5) == 0 && registers.mmhub_base(1) == 0x2408800,
           "six GC bases probed; MESAPI copies the first five");

    const auto one_gc = rx7900_test::discovery_with({
        {discovery_gc_hardware_id, 11, 0, {gc0}},
        {discovery_mmhub_hardware_id, 3, 0, {0x1a000}}, {mes11_osssys_hardware_id, 6, 0, {0x4260}}});
    expect(validate_mes11_discovery({one_gc.data(), one_gc.size()}, rx7900_test::aperture_bytes,
                                    registers) == DiscoveryStatus::unsupported && !registers.valid(),
           "GC without the BASE_IDX 1 segment refused");
    const auto no_osssys = rx7900_test::discovery_with({
        {discovery_gc_hardware_id, 11, 0, {gc0, gc1}}, {discovery_mmhub_hardware_id, 3, 0, {0x1a000}}});
    expect(validate_mes11_discovery({no_osssys.data(), no_osssys.size()},
                                    rx7900_test::aperture_bytes, registers) ==
           DiscoveryStatus::invalid_binary, "missing OSSSYS refused");
    const auto wrong_osssys = rx7900_test::discovery_with({
        {discovery_gc_hardware_id, 11, 0, {gc0, gc1}}, {discovery_mmhub_hardware_id, 3, 0, {0x1a000}},
        {mes11_osssys_hardware_id, 6, 1, {0x4260}}});
    expect(validate_mes11_discovery({wrong_osssys.data(), wrong_osssys.size()},
                                    rx7900_test::aperture_bytes, registers) ==
           DiscoveryStatus::unsupported, "OSSSYS 6.0.1 refused");
    expect(validate_mes11_discovery({binary.data(), binary.size()}, 0x30000, registers) ==
           DiscoveryStatus::invalid_argument, "MES registers outside BAR5 refused");

    std::vector<uint8_t> header(64, 0);
    const rx7900::sdma::CommandBuffer out(header.data(), header.size());
    out.write_u32(0, 0x1000);
    out.write_u32(4, 64);
    out.write_u32(8, 1);
    out.write_u32(56, 0x00040000u);
    out.write_u32(60, 0x1);
    uint64_t start = 0;
    expect(mes11_uc_start_address({header.data(), header.size()}, start) &&
           start == 0x0000000100040000ULL, "mes_uc_start_addr_lo/hi decoded");
    expect(!mes11_uc_start_address({header.data(), 60}, start) && start == 0x0000000100040000ULL,
           "truncated MES header refused, output unchanged");
    out.write_u32(8, 2);
    expect(!mes11_uc_start_address({header.data(), header.size()}, start), "header v2 refused");
}

void initialize_refusals() {
    Fixture fixture;
    expect(fixture.mes->report().status == Mes11Status::ready &&
           fixture.mes->report().aggregated_doorbell_base == 0x800,
           "aggregated doorbells follow 0x321 + 0x400 kernel doorbells, page aligned (0x800)");
    Mes11 other;
    expect(other.initialize({}, fixture.model.io(), fixture.model.doorbells(), fixture.nbio,
                            rx7900_test::navi31_vram(), {sched_uc, kiq_uc, 2}, {1000, 2100000},
                            {nullptr, Fixture::confirm}) == Mes11Status::unsupported_ip,
           "invalid register token refused");
    expect(other.initialize(fixture.registers, fixture.model.io(), {nullptr, 0, nullptr, nullptr},
                            fixture.nbio, rx7900_test::navi31_vram(), {sched_uc, kiq_uc, 2},
                            {1000, 2100000}, {&fixture.reset_confirmed, Fixture::confirm}) ==
           Mes11Status::unavailable, "no doorbell aperture: refused, Linux has no MMIO MES path");
    expect(other.initialize(fixture.registers, fixture.model.io(), fixture.model.doorbells(0x200),
                            fixture.nbio, rx7900_test::navi31_vram(), {sched_uc, kiq_uc, 2},
                            {1000, 2100000}, {&fixture.reset_confirmed, Fixture::confirm}) ==
           Mes11Status::unavailable, "doorbell BAR without the MES page refused");
    expect(other.initialize(fixture.registers, fixture.model.io(), fixture.model.doorbells(),
                            fixture.nbio, rx7900_test::navi31_vram(), {sched_uc, kiq_uc, 3},
                            {1000, 2100000}, {&fixture.reset_confirmed, Fixture::confirm}) ==
           Mes11Status::invalid_argument, "three SDMA instances refused");
    expect(other.initialize(fixture.registers, fixture.model.io(), fixture.model.doorbells(),
                            fixture.nbio, rx7900_test::navi31_vram(), {sched_uc + 2, kiq_uc, 2},
                            {1000, 2100000}, {&fixture.reset_confirmed, Fixture::confirm}) ==
           Mes11Status::invalid_argument, "unaligned ucode start refused");
    expect(other.initialize(fixture.registers, fixture.model.io(), fixture.model.doorbells(),
                            fixture.nbio, rx7900_test::navi31_vram(), {sched_uc, kiq_uc, 2},
                            {0, 2100000}, {&fixture.reset_confirmed, Fixture::confirm}) ==
           Mes11Status::invalid_argument, "zero poll budget refused");
    expect(fixture.mes->hw_init() == Mes11Status::unavailable &&
           fixture.mes->map_legacy_queue({Mes11QueueType::compute, 0x6, 0, 0, 0x1000, 0x2000}) ==
               Mes11Status::unavailable && fixture.model.writes.empty(),
           "hw_init/map before kiq_hw_init refused without MMIO");
    expect(fixture.mes->self_test(*fixture.manager, {}, {}, {}) == Mes11Status::unavailable &&
           fixture.model.writes.empty() && !fixture.mes->self_test_report().ran &&
           !fixture.mes->self_test_report().memory_retained,
           "MES self-test before the scheduler is ready refused without MMIO or leases");

    Mes11Memory wrong = fixture.memory;
    wrong.kiq_ring = fixture.memory.scheduler_ring;
    expect(fixture.mes->kiq_hw_init(*fixture.manager, wrong) == Mes11Status::invalid_memory &&
           fixture.model.writes.empty() && !fixture.mes->report().memory_retained,
           "32 KiB allocation for the 1 MiB KIQ ring refused before any register write");
    rx7900::memory::Handle stale{};
    fixture.manager->allocate(mes11_writeback_bytes, 0x1000, rx7900::memory::Access::read_write,
                              true, stale);
    fixture.manager->release(stale);
    wrong = fixture.memory;
    wrong.writeback = stale;
    expect(fixture.mes->kiq_hw_init(*fixture.manager, wrong) == Mes11Status::invalid_memory &&
           fixture.model.writes.empty(), "released writeback handle refused, earlier leases rolled back");
    expect(           fixture.release_all(), "failed retention left no leases");
}

void bring_up() {
    Fixture fixture;
    auto& model = fixture.model;
    expect(fixture.mes->kiq_hw_init(*fixture.manager, fixture.memory) == Mes11Status::scheduler_ready,
           "firmware 0x50 >= 0x47: kiq_hw_init runs hw_init and the scheduler is ready");
    const Mes11Report report = fixture.mes->report();
    expect(report.scheduler_version == 0x1050 && report.kiq_version == 0x1049 &&
           report.legacy_queue_map && report.scheduler_ready && !report.kiq_ring_ready &&
           report.mes_enabled && report.memory_retained, "versions, legacy map and readiness");

    const uint64_t kiq_mqd = fixture.address(fixture.memory.kiq_mqd);
    const uint64_t sched_mqd = fixture.address(fixture.memory.scheduler_mqd);
    const uint64_t kiq_ring = fixture.address(fixture.memory.kiq_ring);
    const uint64_t sched_ring = fixture.address(fixture.memory.scheduler_ring);
    const uint64_t kiq_eop = fixture.address(fixture.memory.kiq_eop);
    const uint64_t sched_eop = fixture.address(fixture.memory.scheduler_eop);
    const uint64_t wb = fixture.wb();
    const auto lo = [](uint64_t value) { return static_cast<uint32_t>(value); };
    const auto hi = [](uint64_t value) { return static_cast<uint32_t>(value >> 32); };

    const std::vector<std::pair<uint32_t, uint32_t>> expected{
        {mes_cntl, 0x00030000u}, {grbm, sched_select}, {pc_start, lo(sched_uc >> 2)},
        {pc_start_hi, hi(sched_uc >> 2)}, {grbm, kiq_select}, {pc_start, lo(kiq_uc >> 2)},
        {pc_start_hi, hi(kiq_uc >> 2)}, {grbm, 0}, {mes_cntl, 0x0c000000u},
        {grbm, sched_select}, {grbm, kiq_select}, {grbm, 0},
        {rlc_cp_schedulers, 0x68}, {rlc_cp_schedulers, 0xe8},
        {grbm, kiq_select}, {gc0 + 0x1fac, 0}, {gc0 + 0x1fb8, 0},
        {gc0 + 0x1fa9, lo(kiq_mqd)}, {gc0 + 0x1faa, hi(kiq_mqd)}, {gc0 + 0x1fcb, 0},
        {gc0 + 0x1fb1, lo(kiq_ring >> 8)}, {gc0 + 0x1fb2, hi(kiq_ring >> 8)},
        {gc0 + 0x1fb4, lo(wb)}, {gc0 + 0x1fb5, hi(wb)}, {gc0 + 0x1fba, 0xd8308011u},
        {gc0 + 0x1fb6, lo(wb + 32)}, {gc0 + 0x1fb7, hi(wb + 32)}, {gc0 + 0x1fb8, 0x40000060u},
        {gc0 + 0x1fad, 0x0be05501u}, {gc0 + 0x1fab, 1}, {grbm, 0},
        {scratch, 0xcafedeadu}};
    expect(model.writes == expected,
           "Linux order: enable, PC start per pipe, versions, kiq_setting, KIQ HQD, ring test seed");
    expect(model.device->reg(scratch) == 0xdeadbeefu,
           "KIQ executed the WRITE_DATA ring test");
    expect(!model.device->delays.empty() && model.device->delays.front() == 500,
           "500 us settle after unhalting MES");

    expect(fixture.vram(kiq_mqd) == 0xc0310800u && fixture.vram(kiq_mqd + 44) == 1 &&
           fixture.vram(kiq_mqd + 92) == 0xffffffffu && fixture.vram(kiq_mqd + 108) == 0xffffffffu &&
           fixture.vram(kiq_mqd + 100) == 0 && fixture.vram(kiq_mqd + 128) == 7,
           "MQD header, pipeline stats, static thread masks and misc_reserved");
    expect(fixture.vram(kiq_mqd + 660) == lo(kiq_eop >> 8) && fixture.vram(kiq_mqd + 664) == 0 &&
           fixture.vram(kiq_mqd + 668) == 8 && fixture.vram(kiq_mqd + 648) == 0x100 &&
           fixture.vram(kiq_mqd + 520) == 1 && fixture.vram(kiq_mqd + 524) == 0 &&
           fixture.vram(kiq_mqd + 596) == 0x00300000u && fixture.vram(kiq_mqd + 540) == 0 &&
           fixture.vram(kiq_mqd + 728) == 0 && fixture.vram(kiq_mqd + 552) == 0,
           "EOP 2048 bytes (EOP_SIZE 8), MQD control, active, IB control and pointers");
    expect(fixture.vram(sched_mqd + 580) == 0xd830800cu &&
           fixture.vram(sched_mqd + 572) == 0x40000058u &&
           fixture.vram(sched_mqd + 544) == lo(sched_ring >> 8) &&
           fixture.vram(sched_mqd + 556) == lo(wb + 160) && fixture.vram(sched_mqd + 564) == lo(wb + 192) &&
           fixture.vram(sched_mqd + 512) == lo(sched_mqd) &&
           fixture.vram(sched_mqd + 660) == lo(sched_eop >> 8) &&
           fixture.vram(sched_mqd + 528) == 0x0be05501u,
           "scheduler MQD: 32 KiB queue, doorbell 0x16, ring/rptr/wptr/EOP addresses");

    expect(model.kiq_doorbells.size() == 1 && model.kiq_doorbells.at(0) == 256 &&
           report.kiq_wptr == 256 && model.kiq_packets.size() == 2,
           "MAP_QUEUES and ring test committed together, padded to 256 dwords");
    const std::vector<uint32_t> map{0xc005a200u, 0x34080000u, 0x58, lo(sched_mqd), hi(sched_mqd),
                                    lo(wb + 192), hi(wb + 192)};
    const std::vector<uint32_t> test{0xc0033700u, 0x10000u, scratch, 0, 0xdeadbeefu};
    expect(model.kiq_packets.at(0) == map && model.kiq_packets.at(1) == test,
           "gfx11_kiq_map_queues (me 2, eng_sel 5) and KIQ WRITE_DATA bytes");
    expect(fixture.vram(kiq_ring + 12 * 4) == kiq_nop && fixture.vram(kiq_ring + 255 * 4) == kiq_nop &&
           fixture.vram(kiq_ring + 256 * 4) == kiq_nop, "NOP padding and cleared ring");
    expect(model.vram64(wb + 32) == 256 && model.vram64(wb + 192) == 256 &&
           fixture.vram(wb + 4 * 32) == 1 && fixture.vram(wb + 9 * 32) == 1,
           "wptr writeback mirrors doorbells; cond_exec slots set to 1");

    expect(model.mes_frames.size() == 4 && model.mes_doorbells.size() == 2 &&
           model.mes_doorbells.at(0) == 128 && model.mes_doorbells.at(1) == 256,
           "SET_HW_RESOURCES and QUERY_SCHEDULER_STATUS, each followed by a fence frame");
    std::vector<uint32_t> hw(64, 0);
    hw.at(0) = 0x40001;
    hw.at(1) = 0xffffff00u;
    hw.at(2) = 0xffffff00u;
    hw.at(3) = 0x1000;
    hw.at(5) = hw.at(6) = hw.at(7) = hw.at(8) = 0xc;
    hw.at(13) = 0xfffffffeu;
    hw.at(15) = hw.at(16) = 0xfc;

    for (uint32_t level = 0; level < 5; ++level) {
        hw.at(17 + level) = 0x800 + level * 2;
    }

    hw.at(22) = lo(wb + 320);
    hw.at(23) = hi(wb + 320);
    hw.at(24) = lo(wb + 384);
    hw.at(25) = hi(wb + 384);
    hw.at(26) = gc0;
    hw.at(27) = gc1;
    hw.at(34) = rx7900_test::mmhub_base;
    hw.at(42) = 0x4260;
    hw.at(50) = lo(wb + 480);
    hw.at(51) = hi(wb + 480);
    hw.at(52) = 1;
    hw.at(54) = 0x447;
    hw.at(55) = 50;
    expect(model.mes_frames.at(0) == hw, "MESAPI_SET_HW_RESOURCES exact 256-byte frame");
    std::vector<uint32_t> fence(64, 0);
    fence.at(0) = 0x400b1;
    fence.at(2) = lo(wb + 224);
    fence.at(3) = hi(wb + 224);
    fence.at(4) = 1;
    expect(model.mes_frames.at(1) == fence, "fence frame: QUERY_SCHEDULER_STATUS to ring fence, seq 1");
    std::vector<uint32_t> query(64, 0);
    query.at(0) = 0x400b1;
    query.at(2) = lo(wb + 480);
    query.at(3) = hi(wb + 480);
    query.at(4) = 1;
    expect(model.mes_frames.at(2) == query && model.mes_frames.at(3).at(4) == 2 &&
           fixture.vram(wb + 224) == 2 && report.fence_sequence == 2 && report.api_submissions == 2,
           "mes_v11_0_query_sched_status frame and second fence");
    expect(model.flushed_before_every_doorbell, "HDP flushed before every doorbell");

    const auto writes = model.writes.size();
    expect(fixture.mes->hw_init() == Mes11Status::scheduler_ready && model.writes.size() == writes &&
           model.mes_frames.size() == 4, "second hw_init returns at the ready check without MMIO");
    expect(fixture.mes->kiq_hw_init(*fixture.manager, fixture.memory) == Mes11Status::busy,
           "repeated kiq_hw_init refused while running");
    expect(fixture.mes->release() == Mes11Status::busy, "release refused while MES runs");
}

void legacy_queues() {
    Fixture fixture;
    auto& model = fixture.model;
    fixture.mes->kiq_hw_init(*fixture.manager, fixture.memory);
    const uint64_t wb = fixture.wb();
    const Mes11LegacyQueue compute{Mes11QueueType::compute, 0x6, 1, 2, 0x0000008004321000ULL,
                                   0x0000008004322008ULL};
    expect(fixture.mes->map_legacy_queue(compute) == Mes11Status::completed,
           "compute queue mapped through MESAPI ADD_QUEUE");
    std::vector<uint32_t> add(64, 0);
    add.at(0) = 0x40021;
    add.at(18) = 0x6;
    add.at(19) = 0x04321000u;
    add.at(20) = 0x80;
    add.at(21) = 0x04322008u;
    add.at(22) = 0x80;
    add.at(27) = 1;
    add.at(36) = 0x2000;
    add.at(37) = static_cast<uint32_t>(wb + 480);
    add.at(38) = static_cast<uint32_t>((wb + 480) >> 32);
    add.at(39) = 1;
    add.at(48) = 1;
    add.at(49) = 2;
    expect(model.mes_frames.size() == 6 && model.mes_frames.at(4) == add,
           "ADD_QUEUE with map_legacy_kq, pipe/queue, doorbell, MQD and wptr");

    const Mes11LegacyQueue gfx{Mes11QueueType::gfx, 0x116, 0, 0, 0x0000008004323000ULL,
                               0x0000008004324000ULL};
    expect(fixture.mes->unmap_legacy_queue({gfx, Mes11UnmapAction::preempt_no_unmap,
                                            0x0000008004325000ULL, 0x1234567800000009ULL}) ==
           Mes11Status::completed, "gfx queue preempted without unmap");
    std::vector<uint32_t> preempt(64, 0);
    preempt.at(0) = 0x40031;
    preempt.at(1) = 0x116;
    preempt.at(4) = 4;
    preempt.at(5) = static_cast<uint32_t>(wb + 480);
    preempt.at(6) = static_cast<uint32_t>((wb + 480) >> 32);
    preempt.at(7) = 1;
    preempt.at(11) = 0x04325000u;
    preempt.at(12) = 0x80;
    preempt.at(13) = 9;
    expect(model.mes_frames.at(6) == preempt, "REMOVE_QUEUE preempt_legacy_gfx_queue, trail fence low dword");
    expect(fixture.mes->unmap_legacy_queue({compute, Mes11UnmapAction::unmap, 0, 0}) ==
           Mes11Status::completed && model.mes_frames.at(8).at(4) == 8 &&
           model.mes_frames.at(8).at(14) == 1 && model.mes_frames.at(8).at(9) == 1 &&
           model.mes_frames.at(8).at(10) == 2 && model.mes_frames.at(8).at(1) == 6,
           "REMOVE_QUEUE unmap_legacy_queue with queue type");

    const auto frames = model.mes_frames.size();
    Mes11LegacyQueue bad = compute;
    bad.pipe = 4;
    expect(fixture.mes->map_legacy_queue(bad) == Mes11Status::invalid_argument, "MEC pipe 4 refused");
    bad = compute;
    bad.doorbell_index = 0x10000;
    expect(fixture.mes->map_legacy_queue(bad) == Mes11Status::invalid_argument,
           "doorbell outside the aperture refused");
    bad = compute;
    bad.wptr_address = 0x0000008004322004ULL;
    expect(fixture.mes->map_legacy_queue(bad) == Mes11Status::invalid_argument,
           "wptr not 8-byte aligned refused");
    bad = compute;
    bad.type = static_cast<Mes11QueueType>(3);
    expect(fixture.mes->map_legacy_queue(bad) == Mes11Status::invalid_argument, "unknown queue type refused");
    expect(fixture.mes->unmap_legacy_queue({gfx, Mes11UnmapAction::preempt_no_unmap, 0, 0}) ==
           Mes11Status::invalid_argument, "preempt without trail fence refused");
    expect(model.mes_frames.size() == frames && fixture.mes->report().status == Mes11Status::completed,
           "argument refusals submit nothing and do not quarantine");

    expect(fixture.mes->kiq_set_resources(0xffffffffffffffffULL) == Mes11Status::completed &&
           model.kiq_packets.size() == 4 && model.kiq_doorbells.back() == 512, "KIQ set resources");
    const std::vector<uint32_t> resources{0xc006a000u, 0xa0000u, 0xffffffffu, 0xffffffffu, 0, 0, 0, 0};
    expect(model.kiq_packets.at(2) == resources && !fixture.mes->report().kiq_ring_ready,
           "gfx11_kiq_set_resources bytes; MES keeps exclusive KIQ ownership");

    // 70 more submissions wrap the 8192-dword scheduler ring.
    bool all = true;

    for (int index = 0; index < 70; ++index) {
        all = all && fixture.mes->query_sched_status() == Mes11Status::completed;
    }

    const auto& last = model.mes_frames.back();
    expect(all && fixture.mes->report().scheduler_wptr > 8192 && last.at(0) == 0x400b1 &&
           last.at(4) == fixture.mes->report().fence_sequence && fixture.vram(wb + 224) == last.at(4) &&
           model.mes_frames.at(model.mes_frames.size() - 2).at(2) == static_cast<uint32_t>(wb + 480),
           "ring wraps with frames intact and fences in order");
}

void old_firmware() {
    Fixture fixture;
    fixture.model.banked_reg(sched_select, gp3_lo) = 0x00001046u;
    expect(fixture.mes->kiq_hw_init(*fixture.manager, fixture.memory) == Mes11Status::kiq_ready &&
           !fixture.mes->report().legacy_queue_map && fixture.model.mes_frames.empty() &&
           fixture.model.kiq_doorbells.empty(), "firmware 0x46: only the KIQ pipe comes up");
    expect(fixture.mes->query_sched_status() == Mes11Status::unavailable,
           "no API before the MES IP hw_init");
    expect(fixture.mes->hw_init() == Mes11Status::scheduler_ready && fixture.model.mes_frames.size() == 4,
           "MES IP hw_init brings up the scheduler later");
    expect(fixture.mes->map_legacy_queue({Mes11QueueType::compute, 0x6, 0, 0, 0x1000, 0x2000}) ==
           Mes11Status::unsupported_firmware, "legacy map refused below 0x47 (Linux uses the KIQ)");
}

void shutdown() {
    Fixture fixture;
    auto& model = fixture.model;
    fixture.mes->kiq_hw_init(*fixture.manager, fixture.memory);
    model.writes.clear();
    expect(fixture.mes->kiq_hw_fini() == Mes11Status::disabled, "kiq_hw_fini halts MES");
    const std::vector<std::pair<uint32_t, uint32_t>> expected{
        {grbm, sched_select}, {hqd_dequeue, 1}, {hqd_doorbell, 0x80000058u}, {hqd_doorbell, 0},
        {gc0 + 0x1fdf, 0}, {gc0 + 0x1fe0, 0}, {gc0 + 0x1fb3, 0}, {grbm, 0}, {mes_cntl, 0x40030010u}};
    expect(model.writes == expected && model.banked_reg(sched_select, hqd_active) == 0,
           "scheduler HQD dequeued, doorbell HIT, pointers cleared, then halt/reset/icache");
    expect(!fixture.mes->report().scheduler_ready && !fixture.mes->report().mes_enabled &&
           fixture.mes->hw_fini() == Mes11Status::disabled, "hw_fini has no bare-metal effect");
    expect(fixture.mes->map_legacy_queue({Mes11QueueType::compute, 0x6, 0, 0, 0x1000, 0x2000}) ==
           Mes11Status::unavailable, "no API after halt");
    expect(fixture.mes->release() == Mes11Status::released && !fixture.mes->report().memory_retained &&
           fixture.release_all(), "leases returned after the confirmed halt");

    Fixture hang;
    hang.mes->kiq_hw_init(*hang.manager, hang.memory);
    hang.model.dequeue_hang = true;
    expect(hang.mes->kiq_hw_fini() == Mes11Status::timeout && hang.mes->report().dequeue_timeout &&
           hang.model.device->reg(mes_cntl) == 0x40030010u,
           "dequeue timeout: Linux sequence completes, MES halted, then quarantine");
    expect(hang.mes->release() == Mes11Status::quarantined &&
           hang.mes->release_after_reset() == Mes11Status::quarantined &&
           hang.mes->report().memory_retained, "quarantined leases kept without reset proof");
    hang.reset_confirmed = true;
    expect(hang.mes->release_after_reset() == Mes11Status::released && hang.release_all(),
           "confirmed reset releases every lease");
}

void failures() {
    Fixture dead({100, 200});
    dead.model.mes_dead = true;
    expect(dead.mes->kiq_hw_init(*dead.manager, dead.memory) == Mes11Status::timeout &&
           !dead.mes->report().scheduler_ready, "MES not answering SET_HW_RESOURCES times out");
    expect(dead.mes->hw_init() == Mes11Status::quarantined &&
           dead.mes->query_sched_status() == Mes11Status::quarantined &&
           dead.mes->kiq_set_resources(1) == Mes11Status::quarantined, "every operation quarantined");
    expect(dead.mes->kiq_hw_fini() == Mes11Status::quarantined &&
           dead.model.device->reg(mes_cntl) == 0x40030010u && dead.mes->report().memory_retained,
           "a quarantined MES can still be halted; leases stay");
    dead.reset_confirmed = true;
    expect(dead.mes->release_after_reset() == Mes11Status::released && dead.release_all(),
           "reset releases the quarantined leases");

    Fixture status;
    status.mes->kiq_hw_init(*status.manager, status.memory);
    status.model.ignore_opcode = 2;
    expect(status.mes->map_legacy_queue({Mes11QueueType::compute, 0x6, 0, 0, 0x1000, 0x2000}) ==
           Mes11Status::timeout && status.mes->query_sched_status() == Mes11Status::quarantined,
           "fence signalled but API status zero: Linux -ETIMEDOUT, quarantined");

    Fixture kiq({50, 2100000});
    kiq.model.kiq_executes = false;
    expect(kiq.mes->kiq_hw_init(*kiq.manager, kiq.memory) == Mes11Status::ring_test_failed &&
           kiq.model.mes_frames.empty(), "KIQ ring test failure stops before any MES API");

    Fixture mmio;
    mmio.model.fail_write_index = rlc_cp_schedulers;
    expect(mmio.mes->kiq_hw_init(*mmio.manager, mmio.memory) == Mes11Status::mmio_failure &&
           mmio.mes->report().memory_retained && mmio.mes->release() == Mes11Status::quarantined,
           "failed kiq_setting store quarantines with leases kept");

    Fixture bell;
    bell.model.doorbell_ok = false;
    expect(bell.mes->kiq_hw_init(*bell.manager, bell.memory) == Mes11Status::doorbell_failure,
           "failed doorbell store quarantines");
}

// MES process queues: doorbell/queue-id allocators, ADD_QUEUE/REMOVE_QUEUE with
// process and gang fields, SUSPEND/RESUME of all gangs, the shared MQD init.
struct ProcessFixture final {
    Fixture base;
    rx7900_test::process_model::GttBackend gtt{};
    rx7900_test::process_model::ProcessEngines engines{};

    explicit ProcessFixture(uint32_t gp3 = 0x00001063u, Mes11Policy policy = {1000, 2100000}) :
        base(policy) {
        engines.model = &base.model;
        engines.gtt = &gtt;
        base.model.bank[{sched_select, gp3_lo}] = gp3;
        base.model.device->reg(rx7900_test::process_model::context15_cntl) =
            rx7900_test::process_model::vm_cntx_cntl;
        base.model.device->reg(rx7900_test::process_model::sdma_ib_cntl) = 0x100;
        base.mes->initialize(base.registers, base.model.io(), engines.io(), base.nbio,
                             rx7900_test::navi31_vram(), {sched_uc, kiq_uc, 2}, policy,
                             {&base.reset_confirmed, Fixture::confirm});
    }

    Mes11& mes() { return *base.mes; }
    bool up() { return mes().kiq_hw_init(*base.manager, base.memory) == Mes11Status::scheduler_ready; }

    std::vector<std::vector<uint32_t>> frames(uint32_t opcode) {
        std::vector<std::vector<uint32_t>> result;

        for (const auto& frame : base.model.mes_frames) {
            if (((frame.at(0) >> 4) & 0xff) == opcode && (opcode != 2 || (frame.at(36) & (1u << 13)) == 0)) {
                result.push_back(frame);
            }
        }

        return result;
    }
};

Mes11AddQueue sample_queue(uint32_t doorbell, Mes11QueueType type = Mes11QueueType::compute) {
    return {type, 0x8001, 0x0000000030421000ULL, 0x0000fffffffff000ULL, 0x1111, 0x0000001000123000ULL,
            0x2222, 0x0000001000124000ULL, 2, 3, doorbell, 0x0000001000125000ULL,
            0x0000000000412020ULL, 0x0000001000126020ULL};
}

void process_allocators() {
    Mes11 idle;
    uint32_t value = 0;
    expect(idle.allocate_doorbell(Mes11QueueType::gfx, value) == Mes11Status::unavailable &&
           idle.allocate_queue_id(value) == Mes11Status::unavailable,
           "allocators need initialize (the doorbell page base)");

    ProcessFixture fixture;
    Mes11& mes = fixture.mes();
    uint32_t gfx = 0;
    uint32_t compute = 0;
    uint32_t sdma = 0;
    expect(mes.allocate_doorbell(Mes11QueueType::gfx, gfx) == Mes11Status::completed && gfx == 0x80a &&
           mes.allocate_doorbell(Mes11QueueType::compute, compute) == Mes11Status::completed &&
           compute == 0x80c &&
           mes.allocate_doorbell(Mes11QueueType::sdma, sdma) == Mes11Status::completed && sdma == 0xa00,
           "amdgpu_mes_kernel_doorbell_get: bits 5/6 after the aggregated doorbells, SDMA from bit 0x100");
    expect(mes.free_doorbell(gfx) == Mes11Status::completed &&
           mes.allocate_doorbell(Mes11QueueType::sdma, value) == Mes11Status::completed && value == 0xa02 &&
           mes.allocate_doorbell(Mes11QueueType::gfx, value) == Mes11Status::completed && value == 0x80a,
           "freed bits are reused lowest first");
    expect(mes.free_doorbell(0x800) == Mes11Status::invalid_argument &&
           mes.free_doorbell(0x80b) == Mes11Status::invalid_argument &&
           mes.free_doorbell(0x80e) == Mes11Status::invalid_argument && !mes.doorbell_allocated(0x808) &&
           mes.doorbell_allocated(0x80c), "aggregated, odd and free doorbells are not freed");
    expect(mes.report().doorbells_allocated == 4, "four doorbells counted");
    unsigned sdma_more = 0;

    while (mes.allocate_doorbell(Mes11QueueType::sdma, value) == Mes11Status::completed) {
        ++sdma_more;
    }

    expect(sdma_more == 254 && mes.allocate_doorbell(Mes11QueueType::sdma, value) ==
           Mes11Status::doorbell_failure && mes.allocate_doorbell(Mes11QueueType::compute, value) ==
           Mes11Status::completed && value == 0x80e, "512-slot page: SDMA range exhausted, gfx range not");
    uint32_t a = 0;
    uint32_t b = 0;
    uint32_t c = 0;
    expect(mes.allocate_queue_id(a) == Mes11Status::completed && a == 1 &&
           mes.allocate_queue_id(b) == Mes11Status::completed && b == 2 &&
           mes.free_queue_id(a) == Mes11Status::completed &&
           mes.allocate_queue_id(c) == Mes11Status::completed && c == 1 &&
           mes.free_queue_id(0) == Mes11Status::invalid_argument &&
           mes.free_queue_id(7) == Mes11Status::invalid_argument, "queue_id_idr: lowest free id from 1");
    unsigned ids = 2;

    while (mes.allocate_queue_id(value) == Mes11Status::completed) {
        ++ids;
    }

    expect(ids == mes11_max_queue_ids - 1, "ids 1..511 then busy");

    // A registered process queue must retire before the scheduler may halt or
    // release its backing. A confirmed reset can forget the hardware ownership.
    ProcessFixture held;
    held.up();
    held.mes().allocate_doorbell(Mes11QueueType::gfx, value);
    held.mes().add_hw_queue(sample_queue(value, Mes11QueueType::gfx));
    expect(held.mes().kiq_hw_fini() == Mes11Status::busy &&
           held.mes().release() == Mes11Status::busy && held.mes().report().memory_retained &&
           held.mes().report().scheduler_ready,
           "halt and release refused while a process queue is registered");
    held.base.reset_confirmed = true;
    expect(held.mes().release_after_reset() == Mes11Status::released &&
           held.mes().report().process_queues == 0 && held.mes().report().doorbells_allocated == 0 &&
           !held.mes().doorbell_allocated(value), "reset forgets queues, doorbells and ids");
}

void process_queues() {
    ProcessFixture fixture;
    Mes11& mes = fixture.mes();
    uint32_t doorbell = 0;
    mes.allocate_doorbell(Mes11QueueType::compute, doorbell);
    expect(mes.add_hw_queue(sample_queue(doorbell)) == Mes11Status::unavailable &&
           mes.suspend_all() == Mes11Status::unavailable, "process API needs the scheduler phase");
    expect(fixture.up(), "scheduler 0x63 ready");
    const size_t before = fixture.base.model.mes_frames.size();
    Mes11AddQueue bad = sample_queue(doorbell);
    bad.pasid = 0;
    Mes11AddQueue bad_db = sample_queue(doorbell + 2);
    Mes11AddQueue bad_mqd = sample_queue(doorbell);
    bad_mqd.mqd_address += 0x100;
    Mes11AddQueue bad_prio = sample_queue(doorbell);
    bad_prio.gang_global_priority_level = 5;
    expect(mes.add_hw_queue(bad) == Mes11Status::invalid_argument &&
           mes.add_hw_queue(bad_db) == Mes11Status::invalid_argument &&
           mes.add_hw_queue(bad_mqd) == Mes11Status::invalid_argument &&
           mes.add_hw_queue(bad_prio) == Mes11Status::invalid_argument &&
           fixture.base.model.mes_frames.size() == before,
           "PASID 0, unallocated doorbell, unaligned MQD, priority 5: refused without a frame");
    fixture.base.model.device->reg(rx7900_test::process_model::context15_cntl) = 0x01fffe05u;
    expect(mes.add_hw_queue(sample_queue(doorbell)) == Mes11Status::unsupported_ip &&
           mes.report().scheduler_ready && fixture.base.model.mes_frames.size() == before,
           "GCVM context with an unexpected page-table structure refused");
    fixture.base.model.device->reg(rx7900_test::process_model::context15_cntl) =
        rx7900_test::process_model::vm_cntx_cntl;
    expect(mes.add_hw_queue(sample_queue(doorbell)) == Mes11Status::completed &&
           mes.report().process_queues == 1, "ADD_QUEUE completed");
    const auto adds = fixture.frames(2);
    bool add_ok = adds.size() == 1;

    if (add_ok) {
        const auto& f = adds[0];
        using rx7900_test::process_model::frame64;
        add_ok = f.at(0) == 0x00040021u && f.at(1) == 0x8001 && frame64(f, 8) == 0x30421000ULL &&
            frame64(f, 16) == 0 && frame64(f, 24) == 0x0000fffffffff000ULL && frame64(f, 32) == 0x1111 &&
            frame64(f, 40) == 0x0000001000123000ULL && frame64(f, 48) == 0x2222 &&
            frame64(f, 56) == 0x0000001000124000ULL && f.at(16) == 2 && f.at(17) == 3 &&
            f.at(18) == doorbell && frame64(f, 76) == 0x0000001000125000ULL &&
            frame64(f, 84) == 0x412020ULL && f.at(27) == 1 && f.at(29) == 0 &&
            f.at(35) == rx7900_test::process_model::vm_cntx_cntl && f.at(36) == 0 &&
            frame64(f, 148) != 0 && frame64(f, 156) == 1;
    }

    expect(add_ok, "mes_v11_0_add_hw_queue: pid, PT base, VA range, quanta, contexts, priorities, "
                   "doorbell, MQD, VA wptr (API 1), type compute, gds 0, vm_context_cntl, flags 0");
    expect(mes.remove_hw_queue({doorbell + 2, 0x0000001000124000ULL}) == Mes11Status::invalid_argument &&
           mes.remove_hw_queue({doorbell, 0x0000001000124000ULL}) == Mes11Status::completed &&
           mes.report().process_queues == 0, "REMOVE_QUEUE of the added doorbell");
    const auto removes = fixture.frames(3);
    expect(removes.size() == 1 && removes[0].at(0) == 0x00040031u && removes[0].at(1) == doorbell &&
           rx7900_test::process_model::frame64(removes[0], 8) == 0x0000001000124000ULL &&
           removes[0].at(4) == 0 && removes[0].at(13) == 0 && removes[0].at(14) == 0,
           "REMOVE_QUEUE: doorbell and gang context only");

    // SUSPEND/RESUME all gangs (firmware 0x63).
    expect(mes.suspend_all() == Mes11Status::completed && mes.report().suspend_resume_supported &&
           mes.report().gangs_suspended, "SUSPEND completed");
    const auto suspends = fixture.frames(6);
    bool suspend_ok = suspends.size() == 1 && suspends[0].at(0) == 0x00040061u && suspends[0].at(1) == 1;

    for (uint32_t dword = 2; suspend_ok && dword < 7; ++dword) {
        suspend_ok = suspends[0].at(dword) == 0;
    }

    expect(suspend_ok && rx7900_test::process_model::frame64(suspends[0], 28) != 0 &&
           rx7900_test::process_model::frame64(suspends[0], 36) == 1,
           "SUSPEND: suspend_all_gangs 1, gang context/fence 0, api_status at byte 28");
    expect(mes.resume_all() == Mes11Status::completed && !mes.report().gangs_suspended,
           "RESUME completed");
    const auto resumes = fixture.frames(7);
    expect(resumes.size() == 1 && resumes[0].at(0) == 0x00040071u && resumes[0].at(1) == 1 &&
           resumes[0].at(2) == 0 && resumes[0].at(3) == 0 &&
           rx7900_test::process_model::frame64(resumes[0], 16) != 0 &&
           rx7900_test::process_model::frame64(resumes[0], 24) == 1,
           "RESUME: resume_all_gangs 1, api_status at byte 16");
    mes.free_doorbell(doorbell);
    expect(mes.kiq_hw_fini() == Mes11Status::disabled && mes.release() == Mes11Status::released,
           "MES teardown once the queue and doorbell are gone");

    // Firmware below 0x63: amdgpu_mes_suspend/resume return 0 without a frame.
    ProcessFixture old(0x00001050u);
    old.up();
    const size_t frames = old.base.model.mes_frames.size();
    expect(old.mes().suspend_all() == Mes11Status::completed &&
           old.mes().resume_all() == Mes11Status::completed &&
           !old.mes().report().suspend_resume_supported && !old.mes().report().gangs_suspended &&
           old.base.model.mes_frames.size() == frames, "suspend/resume skipped below 0x63");

    // API version 2: ADD_QUEUE carries the VMID0 wptr address.
    ProcessFixture api2(0x00002063u);
    api2.up();
    api2.mes().allocate_doorbell(Mes11QueueType::sdma, doorbell);
    expect(api2.mes().add_hw_queue(sample_queue(doorbell, Mes11QueueType::sdma)) == Mes11Status::completed &&
           api2.frames(2).size() == 1 &&
           rx7900_test::process_model::frame64(api2.frames(2)[0], 84) == 0x0000001000126020ULL &&
           api2.frames(2)[0].at(27) == 2, "API 2: wptr_mc; SDMA type 2");

    // A SUSPEND MES does not complete: timeout and quarantine.
    ProcessFixture lost(0x00001063u, {1000, 200});
    lost.up();
    lost.engines.drop_opcode = 6;
    lost.base.model.ignore_opcode = 6;
    expect(lost.mes().suspend_all() == Mes11Status::timeout && !lost.mes().report().scheduler_ready &&
           !lost.mes().report().gangs_suspended && lost.mes().resume_all() == Mes11Status::quarantined,
           "lost SUSPEND quarantines MES");
}

void queue_mqd() {
    ProcessFixture fixture;
    uint8_t mqd[mes11_mqd_bytes]{};
    uint32_t doorbell = 0;
    fixture.mes().allocate_doorbell(Mes11QueueType::sdma, doorbell);
    const Gfx11MqdProperties properties{0x0000001000125000ULL, 0x0000000000400000ULL,
        0x0000000000402000ULL, 0x0000000000402020ULL, 0, 16384, doorbell, 1, 0, false, false};
    expect(fixture.mes().init_queue_mqd(Mes11QueueType::sdma, properties,
                                        rx7900::sdma::CommandBuffer(mqd, sizeof(mqd))) ==
           Mes11Status::unavailable, "MQD init needs the scheduler phase");
    fixture.up();
    expect(fixture.mes().init_queue_mqd(Mes11QueueType::sdma, properties,
                                        rx7900::sdma::CommandBuffer(mqd, 256)) ==
           Mes11Status::invalid_argument, "short MQD buffer refused");
    expect(fixture.mes().init_queue_mqd(Mes11QueueType::sdma, properties,
                                        rx7900::sdma::CommandBuffer(mqd, sizeof(mqd))) ==
           Mes11Status::completed, "SDMA MQD built");
    using rx7900_test::process_model::u32;
    expect(u32(mqd, 0) == ((12u << 1) | (1u << 11) | (1u << 12) | (4u << 16)) &&
           u32(mqd, 4) == 0x4000 && u32(mqd, 32) == 0x402000 && u32(mqd, 108) == 0x402020 &&
           u32(mqd, 36) == 0x100 && u32(mqd, 76) == (doorbell << 2) && u32(mqd, 68) == 0x10000000u &&
           u32(mqd, 112) == 0x4000 && u32(mqd, 100) == 0xf,
           "sdma_v6_0_mqd_init: RB_SIZE 12 for 16 KiB, VA base/rptr/wptr, IB_CNTL, doorbell");
    uint32_t gfx_doorbell = 0;
    fixture.mes().allocate_doorbell(Mes11QueueType::gfx, gfx_doorbell);
    Gfx11MqdProperties gfx = properties;
    gfx.doorbell_index = gfx_doorbell;
    fixture.base.model.writes.clear();
    expect(fixture.mes().init_queue_mqd(Mes11QueueType::gfx, gfx,
                                        rx7900::sdma::CommandBuffer(mqd, sizeof(mqd))) ==
           Mes11Status::completed && u32(mqd, 572) == ((gfx_doorbell << 2) | 0x40000000u) &&
           u32(mqd, 544) == 0x4000 && (u32(mqd, 580) & 0x3f) == 11 &&
           !fixture.base.model.writes.empty() && fixture.base.model.writes.back().first == grbm &&
           fixture.base.model.writes.back().second == 0,
           "gfx MQD under ME0 pipe0 selection, selection restored to 0");
}

} // namespace

int main() {
    discovery_and_header();
    initialize_refusals();
    bring_up();
    legacy_queues();
    old_firmware();
    shutdown();
    failures();
    process_allocators();
    process_queues();
    queue_mqd();
    return checks.finish("mes11_tests");
}

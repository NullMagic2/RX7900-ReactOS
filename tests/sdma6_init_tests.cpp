/*
Module: rx7900.tests.sdma6_init
Purpose: Verify SDMA6 start order, doorbell/register WPTR paths, ring and IB tests, halt and quarantine.
Created: 2026-10-21
Architecture: Test-local GC/NBIO register model and SDMA engine model on the shared
VRAM fixture, with a real FenceManager on a model platform for submissions; expected values are derived by hand from Linux v6.12 sdma_v6_0.c,
sdma_v6_0_0_pkt_open.h, amdgpu_ib.c, amdgpu_fence.c, amdgpu_sdma.c, nbio_v4_3.c,
soc21.c and the gc_11_0_0/nbio_4_3_0 register headers.
*/
#include <map>
#include <memory>

#include "asic_test_device.hpp"
#include "sdma6_init.hpp"

namespace {

using namespace rx7900::hardware;
using rx7900::ByteView;
using rx7900::memory::Access;
using rx7900::memory::Handle;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

constexpr uint32_t gc0 = rx7900_test::gc_base;
constexpr uint32_t gc1 = 0xa000;
constexpr uint32_t nbif3 = rx7900_test::nbif_bases[3];
constexpr uint32_t rsmu_index = rx7900_test::nbif_bases[1];
constexpr uint32_t rsmu_data = rx7900_test::nbif_bases[1] + 1;
constexpr uint32_t hdp_flush_index = hdp_remap_byte_offset / 4;
constexpr uint32_t s2a_smn = (nbif3 + 0x4f7642u) * 4;
// BIF_BX_PF0_GPU_HDP_FLUSH_REQ/DONE (NBIF segment 2 + 0x106/0x107).
constexpr uint32_t hdp_flush_req = rx7900_test::nbif_bases[2] + 0x106;
constexpr uint32_t hdp_flush_done = rx7900_test::nbif_bases[2] + 0x107;

// Absolute dword index of an SDMA register for instance 0 or 1.
constexpr uint32_t q(uint32_t instance, uint32_t internal) { return gc0 + internal + instance * 0x600; }
constexpr uint32_t f32(uint32_t instance) { return gc1 + 0x589a + instance * 0x20; }

constexpr uint32_t cntl = 0x1c;
constexpr uint32_t sem = 0x22;
constexpr uint32_t watchdog = 0x2e;
constexpr uint32_t utcl1_cntl = 0x3c;
constexpr uint32_t utcl1_page = 0x3f;
constexpr uint32_t rb_cntl = 0x80;
constexpr uint32_t rb_base = 0x81;
constexpr uint32_t rb_base_hi = 0x82;
constexpr uint32_t rb_rptr = 0x83;
constexpr uint32_t rb_rptr_hi = 0x84;
constexpr uint32_t rb_wptr = 0x85;
constexpr uint32_t rb_wptr_hi = 0x86;
constexpr uint32_t rptr_addr_hi = 0x88;
constexpr uint32_t rptr_addr_lo = 0x89;
constexpr uint32_t ib_cntl = 0x8a;
constexpr uint32_t doorbell = 0x92;
constexpr uint32_t doorbell_offset = 0xab;
constexpr uint32_t poll_hi = 0xb2;
constexpr uint32_t poll_lo = 0xb3;
constexpr uint32_t minor_ptr = 0xb5;

struct Access64 final {
    enum class Kind { mmio, smn, doorbell };
    Kind kind;
    uint32_t target;
    uint64_t value;

    bool operator==(const Access64& other) const {
        return kind == other.kind && target == other.target && value == other.value;
    }
};

Access64 mmio(uint32_t target, uint64_t value) { return {Access64::Kind::mmio, target, value}; }
Access64 smn(uint32_t target, uint64_t value) { return {Access64::Kind::smn, target, value}; }
Access64 ring_doorbell(uint32_t index, uint64_t value) { return {Access64::Kind::doorbell, index, value}; }

// Plain BAR5 register file with an RSMU-reached SMN space, a doorbell BAR and a
// minimal SDMA engine: when a WPTR is published to an unhalted, enabled ring it
// executes NOP (with burst count), GCR_REQ, COND_EXE, POLL_REGMEM HDP flush
// (NBIO reflects FLUSH_REQ into FLUSH_DONE), INDIRECT (WRITE_LINEAR and NOP in
// the IB), FENCE and TRAP packets from VRAM and writes the 64-bit RPTR back.
class SdmaDevice final {
public:
    std::vector<uint32_t> registers = std::vector<uint32_t>(rx7900_test::aperture_bytes / 4, 0);
    std::map<uint32_t, uint32_t> smn_space;
    std::vector<Access64> log;
    rx7900_test::SimulatedNavi31* vram = nullptr;
    uint64_t engine_rptr[2]{};
    uint64_t shadow_at_doorbell[2]{};
    unsigned executions[2]{};
    unsigned delay_calls = 0;
    uint32_t smn_address = 0;
    uint32_t fail_write = UINT32_MAX;
    uint32_t all_ones_read = UINT32_MAX;
    uint32_t hang_instance = UINT32_MAX;
    // WRITE_LINEAR from an IB on this instance stores a wrong value.
    uint32_t corrupt_ib_instance = UINT32_MAX;
    std::vector<uint32_t> opcodes[2];
    std::vector<std::vector<uint32_t>> ib_packets[2];
    std::vector<uint32_t> hdp_refs[2];
    unsigned traps[2]{};
    uint32_t doorbell_dwords = 0x400;
    bool doorbell_ok = true;
    bool reset_confirmed = false;

    SdmaDevice() {
        // State left by firmware/a previous owner; unrelated bits must survive.
        for (uint32_t instance = 0; instance < 2; ++instance) {
            at(q(instance, cntl)) = 0x00000002u;
            at(f32(instance)) = 0x00002001u;
            at(q(instance, rb_cntl)) = 0x0004003eu;
            at(q(instance, ib_cntl)) = 0x00000100u;
            at(q(instance, doorbell_offset)) = 0xf0000003u;
            at(q(instance, watchdog)) = 0x12345678u;
            at(q(instance, utcl1_cntl)) = 0x0000ffffu;
            at(q(instance, utcl1_page)) = 0xabcdefffu;
        }

        smn_space[s2a_smn] = 0x0202007eu;
    }

    uint32_t& at(uint32_t index) { return registers.at(index); }

    RegisterIo io() noexcept { return {this, rx7900_test::aperture_bytes, read, write, delay}; }

    DoorbellIo doorbell_io(bool enabled) noexcept {
        if (!enabled) {
            return {nullptr, 0, nullptr, nullptr};
        }

        return {this, doorbell_dwords, doorbell32, doorbell64};
    }

    Sdma6Recovery recovery() noexcept { return {this, confirm}; }

    static bool read(void* context, uint32_t offset, uint32_t& value) noexcept {
        auto& self = *static_cast<SdmaDevice*>(context);
        const uint32_t index = offset / 4;

        if ((offset & 3) != 0 || index >= self.registers.size()) {
            return false;
        }

        if (index == self.all_ones_read) {
            value = UINT32_MAX;
            return true;
        }

        if (index == rsmu_data) {
            value = self.smn_space[self.smn_address];
            return true;
        }

        value = self.registers.at(index);
        return true;
    }

    static bool write(void* context, uint32_t offset, uint32_t value) noexcept {
        auto& self = *static_cast<SdmaDevice*>(context);
        const uint32_t index = offset / 4;

        if ((offset & 3) != 0 || index >= self.registers.size() || index == self.fail_write) {
            return false;
        }

        if (index == rsmu_index) {
            self.smn_address = value;
            self.registers.at(index) = value;
            return true;
        }

        if (index == rsmu_data) {
            self.smn_space[self.smn_address] = value;
            self.log.push_back(smn(self.smn_address, value));
            return true;
        }

        self.registers.at(index) = value;
        self.log.push_back(mmio(index, value));

        for (uint32_t instance = 0; instance < 2; ++instance) {
            if (index == q(instance, rb_rptr)) {
                self.engine_rptr[instance] = 0;
            }

            // Register WPTR path: the engine samples WPTR when its high half lands.
            if (index == q(instance, rb_wptr_hi) && (self.at(q(instance, doorbell)) & 0x10000000u) == 0) {
                self.execute(instance, (static_cast<uint64_t>(value) << 32) | self.at(q(instance, rb_wptr)));
            }
        }

        return true;
    }

    static void delay(void* context, uint32_t) noexcept {
        ++static_cast<SdmaDevice*>(context)->delay_calls;
    }

    static bool doorbell32(void*, uint32_t, uint32_t) noexcept { return false; }

    static bool doorbell64(void* context, uint32_t index, uint64_t value) noexcept {
        auto& self = *static_cast<SdmaDevice*>(context);
        self.log.push_back(ring_doorbell(index, value));

        if (!self.doorbell_ok) {
            return false;
        }

        const uint32_t range = self.smn_space[s2a_smn];
        const uint32_t first = (range >> 7) & 0x3ff;
        const uint32_t count = (range >> 17) & 0xff;

        if ((range & 1) == 0 || index < first || index >= first + count) {
            return true;
        }

        for (uint32_t instance = 0; instance < 2; ++instance) {
            if ((self.at(q(instance, doorbell)) & 0x10000000u) != 0 &&
                ((self.at(q(instance, doorbell_offset)) & 0x0ffffffcu) >> 2) == index) {
                const uint64_t shadow = (static_cast<uint64_t>(self.at(q(instance, poll_hi))) << 32) |
                                        self.at(q(instance, poll_lo));
                self.shadow_at_doorbell[instance] = self.vram->vram_u32(shadow) |
                    (static_cast<uint64_t>(self.vram->vram_u32(shadow + 4)) << 32);
                self.execute(instance, value);
            }
        }

        return true;
    }

    static bool confirm(void* context) noexcept {
        return static_cast<SdmaDevice*>(context)->reset_confirmed;
    }

    void execute(uint32_t instance, uint64_t wptr) {
        if (instance == hang_instance || (at(f32(instance)) & 1) != 0 ||
            (at(q(instance, rb_cntl)) & 1) == 0) {
            return;
        }

        ++executions[instance];
        const uint64_t ring = (static_cast<uint64_t>(at(q(instance, rb_base))) << 8) |
                              (static_cast<uint64_t>(at(q(instance, rb_base_hi))) << 40);
        const uint64_t size = 4ULL << ((at(q(instance, rb_cntl)) >> 1) & 0x1f);
        uint64_t& rptr = engine_rptr[instance];

        const auto dw = [&](uint64_t byte) { return vram->vram_u32(ring + (byte & (size - 1))); };

        while (rptr < wptr) {
            const uint32_t header = dw(rptr);
            const uint32_t op = header & 0xff;

            if (op == 0) {
                // NOP: SDMA_PKT_NOP_HEADER_COUNT extra dwords follow the header.
                rptr += 4 * (1 + ((header >> 16) & 0x3fff));
                continue;
            }

            opcodes[instance].push_back(header);

            if (header == 0x00030005u) {
                const uint64_t address = dw(rptr + 4) | (static_cast<uint64_t>(dw(rptr + 8)) << 32);
                vram->put_vram_u32(address, dw(rptr + 12));
                rptr += 16;
            } else if (header == 17) {
                rptr += 20;
            } else if (header == 9) {
                // COND_EXE: execute the next count dwords only when *addr == reference.
                const uint64_t address = dw(rptr + 4) | (static_cast<uint64_t>(dw(rptr + 8)) << 32);
                const bool run = vram->vram_u32(address) == dw(rptr + 12);
                rptr += 20 + (run ? 0 : 4ULL * dw(rptr + 16));
            } else if (header == 0x34000008u) {
                const uint32_t done = dw(rptr + 4) >> 2;
                const uint32_t req = dw(rptr + 8) >> 2;
                at(req) = dw(rptr + 12);
                at(done) = at(req);
                hdp_refs[instance].push_back(dw(rptr + 12));

                if ((at(done) & dw(rptr + 16)) != dw(rptr + 12)) {
                    return;
                }

                rptr += 24;
            } else if (header == 4) {
                const uint64_t base = (dw(rptr + 4) & 0xffffffe0u) |
                                      (static_cast<uint64_t>(dw(rptr + 8)) << 32);
                run_ib(instance, base, dw(rptr + 12));
                rptr += 24;
            } else if (header == 6) {
                ++traps[instance];
                rptr += 8;
            } else {
                return;
            }
        }

        const uint64_t writeback = (static_cast<uint64_t>(at(q(instance, rptr_addr_hi))) << 32) |
                                   at(q(instance, rptr_addr_lo));
        vram->put_vram_u32(writeback, static_cast<uint32_t>(rptr));
        vram->put_vram_u32(writeback + 4, static_cast<uint32_t>(rptr >> 32));
    }

    // IB contents: WRITE_LINEAR (op 2, sub-op 0) of COUNT + 1 dwords, and NOPs.
    void run_ib(uint32_t instance, uint64_t base, uint32_t length) {
        uint32_t position = 0;

        while (position < length) {
            const uint32_t header = vram->vram_u32(base + position * 4);

            if ((header & 0xff) == 0) {
                ++position;
                continue;
            }

            if (header != 2) {
                return;
            }

            const uint64_t address = vram->vram_u32(base + position * 4 + 4) |
                (static_cast<uint64_t>(vram->vram_u32(base + position * 4 + 8)) << 32);
            const uint32_t count = vram->vram_u32(base + position * 4 + 12) + 1;
            std::vector<uint32_t> packet;

            for (uint32_t index = 0; index < 4 + count; ++index) {
                packet.push_back(vram->vram_u32(base + (position + index) * 4));
            }

            for (uint32_t index = 0; index < count; ++index) {
                vram->put_vram_u32(address + index * 4, instance == corrupt_ib_instance ?
                                   0x0badf00du : vram->vram_u32(base + (position + 4 + index) * 4));
            }

            ib_packets[instance].push_back(packet);
            position += 4 + count;
        }
    }
};

std::vector<uint8_t> sdma_discovery(uint8_t sdma1_major = 6, uint8_t gc_segments = 2,
                                    uint32_t segment1 = gc1, bool sdma1 = true) {
    const auto& n = rx7900_test::nbif_bases;
    std::vector<rx7900_test::IpRecord> records{
        {discovery_nbif_hardware_id, 4, 3, {n[0], n[1], n[2], n[3], n[4], n[5]}},
        {discovery_mmhub_hardware_id, 3, 0, {rx7900_test::mmhub_base}},
        {discovery_gc_hardware_id, 11, 0,
         gc_segments == 2 ? std::vector<uint32_t>{gc0, segment1} : std::vector<uint32_t>{gc0}},
        {discovery_sdma0_hardware_id, 6, 0, {gc0}},
    };

    if (sdma1) {
        records.push_back({discovery_sdma1_hardware_id, sdma1_major, 0, {gc0 + 0x600}});
    }

    return rx7900_test::discovery_with(records);
}

struct Fixture final {
    std::unique_ptr<rx7900_test::SimulatedNavi31> vram = std::make_unique<rx7900_test::SimulatedNavi31>();
    std::unique_ptr<SdmaDevice> device = std::make_unique<SdmaDevice>();
    rx7900_test::VramBackend backend{};
    std::unique_ptr<rx7900::memory::Manager> manager = std::make_unique<rx7900::memory::Manager>();
    Nbio nbio{};
    NbioRegisters nbio_registers{};
    Sdma6Registers registers{};
    std::unique_ptr<Sdma6> sdma = std::make_unique<Sdma6>();
    Handle ring0{};
    Handle ring1{};
    Handle writeback{};
    Handle ib{};
    uint32_t ring_bytes;

    explicit Fixture(bool doorbells = true, uint32_t bytes = sdma6_default_ring_bytes)
        : ring_bytes(bytes) {
        backend.device = vram.get();
        device->vram = vram.get();
        manager->initialize({rx7900_test::fb_base + rx7900_test::arena_offset,
                             rx7900_test::arena_bytes}, backend.backend());
        const auto binary = sdma_discovery();
        validate_nbio_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                nbio_registers);
        validate_sdma6_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                 registers);
        nbio.initialize(nbio_registers, device->io());
        nbio.program_common();
        manager->allocate(bytes, sdma6_ring_alignment, Access::read_write, true, ring0);
        manager->allocate(bytes, sdma6_ring_alignment, Access::read_write, true, ring1);
        manager->allocate(rx7900::memory::page_bytes, rx7900::memory::page_bytes,
                          Access::read_write, true, writeback);
        manager->allocate(rx7900::memory::page_bytes, rx7900::memory::page_bytes,
                          Access::read_write, true, ib);
        sdma->initialize(registers, nbio_registers, device->io(), nbio, rx7900_test::navi31_vram(),
                         {bytes}, device->doorbell_io(doorbells), device->recovery());
        device->log.clear();
    }

    uint64_t address(Handle handle) const {
        rx7900::memory::AllocationInfo info{};
        manager->describe(handle, info);
        return info.gpu.virtual_address;
    }

    uint32_t leases(Handle handle) const {
        rx7900::memory::AllocationInfo info{};
        manager->describe(handle, info);
        return info.leases;
    }

    bool no_leases() const { return leases(ring0) == 0 && leases(ring1) == 0 && leases(writeback) == 0; }
    bool all_leased() const { return leases(ring0) == 1 && leases(ring1) == 1 && leases(writeback) == 1; }

    Sdma6Status init() { return sdma->hw_init(*manager, ring0, ring1, writeback); }

    Sdma6Status test_ib(uint32_t feature = 0, uint32_t timeout = sdma6_ib_test_timeout_us) {
        return sdma->test_ib(*manager, ib, {feature, timeout});
    }

    std::vector<uint32_t> dwords(uint64_t address, uint32_t count) const {
        std::vector<uint32_t> out;

        for (uint32_t index = 0; index < count; ++index) {
            out.push_back(vram->vram_u32(address + index * 4));
        }

        return out;
    }
};

struct StartState final {
    uint32_t f32_enabled;   // F32_CNTL after sdma_v6_0_enable(true)
    uint32_t rb_cntl_first; // first RB_CNTL store of gfx_resume
};

// sdma_v6_0_gfx_resume for one instance, hand-derived from the fixture state.
void expected_instance(std::vector<Access64>& out, const Fixture& fixture, uint32_t instance,
                       bool doorbells, StartState state) {
    const uint64_t ring = fixture.address(instance == 0 ? fixture.ring0 : fixture.ring1);
    const uint64_t wb = fixture.address(fixture.writeback);
    const uint64_t rptr = wb + instance * 96;
    const uint64_t wptr = rptr + 32;
    const uint32_t index = instance == 0 ? 0x200u : 0x214u;
    const auto r = [instance](uint32_t internal) { return q(instance, internal); };
    const std::vector<Access64> steps{
        mmio(r(sem), 0),
        mmio(r(rb_cntl), state.rb_cntl_first),
        mmio(r(rb_rptr), 0), mmio(r(rb_rptr_hi), 0), mmio(r(rb_wptr), 0), mmio(r(rb_wptr_hi), 0),
        mmio(r(poll_lo), static_cast<uint32_t>(wptr)), mmio(r(poll_hi), static_cast<uint32_t>(wptr >> 32)),
        mmio(r(rptr_addr_hi), static_cast<uint32_t>(rptr >> 32)),
        mmio(r(rptr_addr_lo), static_cast<uint32_t>(rptr) & 0xfffffffcu),
        mmio(r(rb_base), static_cast<uint32_t>(ring >> 8)), mmio(r(rb_base_hi), static_cast<uint32_t>(ring >> 40)),
        mmio(r(minor_ptr), 1), mmio(r(rb_wptr), 0), mmio(r(rb_wptr_hi), 0),
        mmio(r(doorbell), doorbells ? 0x10000000u : 0),
        mmio(r(doorbell_offset), doorbells ? (0xf0000003u | (index << 2)) : 0xf0000003u),
    };
    out.insert(out.end(), steps.begin(), steps.end());

    if (instance == 0) {
        // AWID 0xe, RANGE_OFFSET 0x200, RANGE_SIZE 40, AWADDR 3, ENABLE; or size 0.
        out.push_back(smn(s2a_smn, doorbells ? 0x3251005du : 0x0200007eu));
    }

    const std::vector<Access64> tail{
        mmio(r(minor_ptr), 0),
        mmio(r(watchdog), 0x12345601u),  // QUEUE_HANG_COUNT 1
        mmio(r(utcl1_cntl), 0x0000ffe9u), // RESP_MODE 3, REDO_DELAY 9
        mmio(r(utcl1_page), 0x01cdefffu), // & 0xFF0FFF | NOA<<12 | BYPASS<<14 | LLC_NOALLOC
        mmio(f32(instance), 0),           // HALT 0, TH1_RESET 0
        mmio(r(rb_cntl), state.rb_cntl_first | 0x00001801u),
        mmio(r(ib_cntl), 0x00000101u),
        // Ring test: HDP flush of the frame, then the WPTR of one 16-dword frame.
        mmio(hdp_flush_index, 0),
    };
    out.insert(out.end(), tail.begin(), tail.end());

    if (doorbells) {
        out.push_back(ring_doorbell(index, 64));
    } else {
        out.push_back(mmio(r(rb_wptr), 64));
        out.push_back(mmio(r(rb_wptr_hi), 0));
    }
}

std::vector<Access64> expected_start(const Fixture& fixture, bool doorbells, StartState state) {
    std::vector<Access64> out{
        mmio(f32(0), state.f32_enabled), mmio(f32(1), state.f32_enabled),
        mmio(q(0, cntl), 0x10000002u), mmio(q(1, cntl), 0x10000002u),
    };
    expected_instance(out, fixture, 0, doorbells, state);
    expected_instance(out, fixture, 1, doorbells, state);
    return out;
}

const std::vector<Access64> expected_fini{
    mmio(q(0, cntl), 0x00000002u), mmio(q(1, cntl), 0x00000002u),
    mmio(q(0, rb_cntl), 0x00841816u), mmio(q(0, ib_cntl), 0x00000100u),
    mmio(q(1, rb_cntl), 0x00841816u), mmio(q(1, ib_cntl), 0x00000100u),
    mmio(f32(0), 0x00000001u), mmio(f32(1), 0x00000001u),
};

bool ring_frame_ok(Fixture& fixture, uint32_t instance) {
    const uint64_t ring = fixture.address(instance == 0 ? fixture.ring0 : fixture.ring1);
    const uint64_t test = fixture.address(fixture.writeback) + instance * 96 + 64;
    auto& v = *fixture.vram;
    bool ok = v.vram_u32(ring) == 0x00030005u && v.vram_u32(ring + 4) == static_cast<uint32_t>(test) &&
              v.vram_u32(ring + 8) == static_cast<uint32_t>(test >> 32) &&
              v.vram_u32(ring + 12) == 0xdeadbeefu;

    for (uint32_t offset = 16; offset < fixture.ring_bytes; offset += 4) {
        ok = ok && v.vram_u32(ring + offset) == 0;
    }

    return ok;
}

void discovery_validation() {
    Sdma6Registers registers;
    auto binary = sdma_discovery();
    expect(validate_sdma6_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                    registers) == DiscoveryStatus::validated && registers.valid() &&
           registers.gc_base(0) == gc0 && registers.gc_base(1) == gc1,
           "GC 11.0.0 with SDMA0/SDMA1 6.0.0 validated");
    binary = sdma_discovery(5);
    expect(validate_sdma6_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                    registers) != DiscoveryStatus::validated && !registers.valid(),
           "SDMA1 5.x rejected");
    binary = sdma_discovery(6, 2, gc1, false);
    expect(validate_sdma6_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                    registers) != DiscoveryStatus::validated, "missing SDMA1 rejected");
    binary = sdma_discovery(6, 1);
    expect(validate_sdma6_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                    registers) != DiscoveryStatus::validated,
           "GC without the HYP_DEC segment rejected");
    binary = sdma_discovery(6, 2, rx7900_test::aperture_bytes / 4 - 0x5800);
    expect(validate_sdma6_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                    registers) == DiscoveryStatus::invalid_argument,
           "SDMA1 F32_CNTL beyond BAR5 rejected");
    binary = sdma_discovery(6, 2, rx7900_test::aperture_bytes / 4 - 0x58bb);
    expect(validate_sdma6_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                    registers) == DiscoveryStatus::validated,
           "SDMA1 F32_CNTL as the last BAR5 dword accepted");
    binary = sdma_discovery();
    expect(validate_sdma6_discovery({binary.data(), binary.size()}, 6, registers) ==
           DiscoveryStatus::invalid_argument, "unaligned aperture rejected");
}

void initialize_arguments() {
    Fixture fixture;
    auto& device = *fixture.device;
    const auto vram = rx7900_test::navi31_vram();
    const auto attempt = [&](Sdma6Config config, DoorbellIo doorbells, Sdma6Recovery recovery) {
        Sdma6 sdma;
        return sdma.initialize(fixture.registers, fixture.nbio_registers, device.io(), fixture.nbio,
                               vram, config, doorbells, recovery);
    };

    expect(attempt({sdma6_default_ring_bytes}, device.doorbell_io(true), device.recovery()) ==
           Sdma6Status::ready, "doorbell configuration ready");
    expect(attempt({sdma6_default_ring_bytes}, device.doorbell_io(false), device.recovery()) ==
           Sdma6Status::ready, "register-path configuration ready");
    expect(attempt({sdma6_default_ring_bytes + 4096}, device.doorbell_io(true), device.recovery()) ==
           Sdma6Status::invalid_argument, "non power-of-two ring rejected");
    expect(attempt({2048}, device.doorbell_io(true), device.recovery()) ==
           Sdma6Status::invalid_argument, "sub-page ring rejected");
    DoorbellIo partial = device.doorbell_io(true);
    partial.write32 = nullptr;
    expect(attempt({sdma6_default_ring_bytes}, partial, device.recovery()) ==
           Sdma6Status::invalid_argument, "partial doorbell callbacks rejected");
    device.doorbell_dwords = sdma6_ring1_doorbell_index + 1;
    expect(attempt({sdma6_default_ring_bytes}, device.doorbell_io(true), device.recovery()) ==
           Sdma6Status::invalid_argument, "ring 1 64-bit doorbell beyond aperture rejected");
    device.doorbell_dwords = sdma6_ring1_doorbell_index + 2;
    expect(attempt({sdma6_default_ring_bytes}, device.doorbell_io(true), device.recovery()) ==
           Sdma6Status::ready, "ring 1 doorbell as last qword accepted");
    device.doorbell_dwords = 0x400;
    expect(attempt({sdma6_default_ring_bytes}, device.doorbell_io(true), {nullptr, nullptr}) ==
           Sdma6Status::unavailable, "reset confirmation callback required");
    Sdma6 sdma;
    expect(sdma.initialize({}, fixture.nbio_registers, device.io(), fixture.nbio, vram,
                           {sdma6_default_ring_bytes}, device.doorbell_io(true),
                           device.recovery()) == Sdma6Status::unsupported_ip,
           "invalid SDMA token rejected");
    expect(sdma.initialize(fixture.registers, {}, device.io(), fixture.nbio, vram,
                           {sdma6_default_ring_bytes}, device.doorbell_io(true),
                           device.recovery()) == Sdma6Status::unsupported_ip,
           "invalid NBIO token rejected");
    RegisterIo small = device.io();
    small.aperture_bytes = rx7900_test::aperture_bytes / 2;
    expect(sdma.initialize(fixture.registers, fixture.nbio_registers, small, fixture.nbio, vram,
                           {sdma6_default_ring_bytes}, device.doorbell_io(true),
                           device.recovery()) == Sdma6Status::unavailable,
           "register aperture mismatch rejected");
    expect(sdma.initialize(fixture.registers, fixture.nbio_registers, device.io(), fixture.nbio, {},
                           {sdma6_default_ring_bytes}, device.doorbell_io(true),
                           device.recovery()) == Sdma6Status::invalid_argument,
           "empty VRAM location rejected");
    Sdma6 fresh;
    expect(fresh.hw_init(*fixture.manager, fixture.ring0, fixture.ring1, fixture.writeback) ==
           Sdma6Status::unavailable && fixture.no_leases(), "hw_init before initialize unavailable");
    expect(fresh.hw_fini() == Sdma6Status::unavailable && fresh.resume() == Sdma6Status::unavailable,
           "fini/resume before initialize unavailable");
    expect(device.log.empty(), "argument validation touches no hardware");
}

void doorbell_start() {
    Fixture fixture(true);
    auto& device = *fixture.device;

    expect(fixture.init() == Sdma6Status::started, "doorbell hw_init starts both rings");
    std::vector<Access64> expected{mmio(hdp_flush_index, 0)};
    const auto start = expected_start(fixture, true, {0x00002000u, 0x00840016u});
    expected.insert(expected.end(), start.begin(), start.end());
    expect(device.log == expected, "doorbell start follows Linux store order and values");
    expect(device.executions[0] == 1 && device.executions[1] == 1, "each engine ran its ring test once");
    expect(device.shadow_at_doorbell[0] == 64 && device.shadow_at_doorbell[1] == 64,
           "WPTR poll shadow published before the doorbell");
    expect(ring_frame_ok(fixture, 0) && ring_frame_ok(fixture, 1),
           "ring holds the shared FENCE packet, NOP padding and cleared remainder");
    const auto report = fixture.sdma->report();
    const uint64_t wb = fixture.address(fixture.writeback);
    expect(fixture.vram->vram_u32(wb + 64) == 0xdeadbeefu && fixture.vram->vram_u32(wb + 96 + 64) == 0xdeadbeefu,
           "test slots hold the ring-test value");
    expect(fixture.vram->vram_u32(wb) == 64 && fixture.vram->vram_u32(wb + 4) == 0 &&
           fixture.vram->vram_u32(wb + 96) == 64, "RPTR writeback reached the frame end");
    expect(report.ring_ready[0] && report.ring_ready[1] && report.started && report.doorbell_mode &&
           report.memory_retained && report.wptr_bytes[0] == 64 && report.wptr_bytes[1] == 64 &&
           report.ring_test_polls[0] == 0 && report.failed_instance == UINT32_MAX,
           "report records ready rings and WPTR");
    expect(report.rb_cntl[0] == 0x00841817u && report.ib_cntl[1] == 0x101u &&
           report.doorbell_offset[1] == 0xf0000853u && report.doorbell_range == 0x3251005du &&
           report.f32_cntl[0] == 0 && report.ring_mc[1] == fixture.address(fixture.ring1) &&
           report.test_mc[1] == wb + 96 + 64, "report mirrors programmed registers");
    expect(fixture.all_leased(), "one lease per ring and writeback while started");
    expect(device.delay_calls == 0, "completed ring test needs no delay");
    expect(fixture.init() == Sdma6Status::busy && fixture.sdma->resume() == Sdma6Status::busy &&
           fixture.sdma->release() == Sdma6Status::busy, "started protocol refuses re-entry");
    expect(fixture.sdma->initialize(fixture.registers, fixture.nbio_registers, device.io(),
                                    fixture.nbio, rx7900_test::navi31_vram(),
                                    {sdma6_default_ring_bytes}, device.doorbell_io(true),
                                    device.recovery()) == Sdma6Status::busy,
           "initialize refused while started");
}

void register_path_start() {
    Fixture fixture(false);
    auto& device = *fixture.device;

    expect(fixture.init() == Sdma6Status::started, "register-path hw_init starts both rings");
    std::vector<Access64> expected{mmio(hdp_flush_index, 0)};
    const auto start = expected_start(fixture, false, {0x00002000u, 0x00840016u});
    expected.insert(expected.end(), start.begin(), start.end());
    expect(device.log == expected, "register path writes RB_WPTR/RB_WPTR_HI and disables doorbells");
    expect(device.executions[0] == 1 && device.executions[1] == 1 &&
           !fixture.sdma->report().doorbell_mode, "register WPTR ran both ring tests");
    expect(device.smn_space[s2a_smn] == 0x0200007eu, "SDMA doorbell range size cleared");
}

void small_ring() {
    Fixture fixture(true, sdma6_minimum_ring_bytes);

    expect(fixture.init() == Sdma6Status::started &&
           fixture.sdma->report().rb_cntl[0] == 0x00841815u,
           "4 KiB ring programs RB_SIZE order 10");
}

void fini_resume_release() {
    Fixture fixture(true);
    auto& device = *fixture.device;

    fixture.init();
    device.log.clear();
    expect(fixture.sdma->hw_fini() == Sdma6Status::stopped, "hw_fini stops");
    expect(device.log == expected_fini, "hw_fini: CTXEMPTY off, RB/IB off, then both F32 halted");
    expect(fixture.all_leased() && !fixture.sdma->report().started &&
           !fixture.sdma->report().ring_ready[0], "leases kept after hw_fini");
    expect(fixture.sdma->hw_fini() == Sdma6Status::busy, "second hw_fini refused");

    device.log.clear();
    expect(fixture.sdma->resume() == Sdma6Status::started, "resume restarts both rings");
    expect(device.log == expected_start(fixture, true, {0x00000000u, 0x00841816u}),
           "resume repeats sdma_v6_0_start without clearing memory");
    expect(device.executions[0] == 2 && device.executions[1] == 2, "ring test reran from WPTR 0");
    expect(fixture.sdma->hw_fini() == Sdma6Status::stopped &&
           fixture.sdma->release() == Sdma6Status::released && fixture.no_leases(),
           "release after hw_fini drops every lease");
    expect(fixture.init() == Sdma6Status::started && fixture.all_leased(),
           "released protocol can start again");
}

void ring_test_timeout() {
    Fixture fixture(true);
    auto& device = *fixture.device;

    device.hang_instance = 0;
    expect(fixture.init() == Sdma6Status::ring_test_timeout, "instance 0 timeout reported");
    const auto report = fixture.sdma->report();
    expect(report.failed_instance == 0 && report.ring_test_polls[0] == sdma6_usec_timeout &&
           device.delay_calls == sdma6_usec_timeout && !report.ring_ready[0] && !report.started,
           "bounded 100000 x 1 us poll");
    bool instance1_touched = false;

    for (const auto& access : device.log) {
        instance1_touched = instance1_touched || access.target == q(1, sem);
    }

    expect(!instance1_touched, "instance 1 not programmed after instance 0 failed");
    expect(fixture.all_leased(), "timeout keeps every lease");
    expect(fixture.sdma->release() == Sdma6Status::quarantined, "release refused while quarantined");
    expect(fixture.sdma->resume() == Sdma6Status::busy, "resume refused while quarantined");
    device.log.clear();
    expect(fixture.sdma->hw_fini() == Sdma6Status::quarantined &&
           (device.at(f32(0)) & 1) != 0 && (device.at(f32(1)) & 1) != 0 &&
           device.log.size() == expected_fini.size(),
           "quarantined hw_fini still halts both engines");
    expect(fixture.sdma->release_after_reset() == Sdma6Status::quarantined && fixture.all_leased(),
           "unconfirmed reset keeps leases");
    device.reset_confirmed = true;
    expect(fixture.sdma->release_after_reset() == Sdma6Status::released && fixture.no_leases(),
           "confirmed reset releases leases");
    expect(fixture.sdma->release_after_reset() == Sdma6Status::busy, "nothing left to release");

    Fixture second(false);
    second.device->hang_instance = 1;
    expect(second.init() == Sdma6Status::ring_test_timeout &&
           second.sdma->report().failed_instance == 1 && second.sdma->report().ring_ready[0] &&
           !second.sdma->report().ring_ready[1], "instance 1 timeout after instance 0 passed");
}

void failure_paths() {
    {
        Fixture fixture(true);
        fixture.device->fail_write = q(1, utcl1_page);
        expect(fixture.init() == Sdma6Status::mmio_failure &&
               fixture.sdma->report().failed_instance == 1 && fixture.all_leased(),
               "failed register store quarantines with leases kept");
        expect(fixture.init() == Sdma6Status::busy, "quarantined hw_init refused");
    }
    {
        Fixture fixture(true);
        fixture.device->all_ones_read = q(0, doorbell_offset);
        expect(fixture.init() == Sdma6Status::mmio_failure &&
               fixture.sdma->report().failed_instance == 0, "all-ones register read quarantines");
    }
    {
        Fixture fixture(true);
        fixture.device->all_ones_read = f32(1);
        expect(fixture.init() == Sdma6Status::mmio_failure && fixture.device->executions[0] == 0,
               "enable failure stops before any ring is programmed");
    }
    {
        Fixture fixture(true);
        fixture.device->doorbell_ok = false;
        expect(fixture.init() == Sdma6Status::doorbell_failure &&
               fixture.sdma->report().failed_instance == 0, "doorbell failure quarantines");
    }
    {
        Fixture fixture(true);
        fixture.device->fail_write = rsmu_data;
        expect(fixture.init() == Sdma6Status::mmio_failure, "NBIO doorbell range failure quarantines");
    }
    {
        Fixture fixture(true);
        fixture.device->fail_write = hdp_flush_index;
        expect(fixture.init() == Sdma6Status::mmio_failure && fixture.device->log.empty() &&
               fixture.all_leased(), "HDP flush failure quarantines before any SDMA store");
    }
    {
        Fixture fixture(true);
        fixture.device->fail_write = q(0, rb_wptr_hi);
        expect(fixture.init() == Sdma6Status::mmio_failure, "WPTR reset failure quarantines");
    }
    {
        Fixture fixture(true);
        fixture.device->fail_write = q(1, cntl);
        fixture.init();
        fixture.device->fail_write = UINT32_MAX;
        expect(fixture.sdma->report().status == Sdma6Status::mmio_failure, "CTXEMPTY failure quarantines");
    }
    {
        Fixture fixture(true);
        fixture.init();
        fixture.device->fail_write = f32(1);
        expect(fixture.sdma->hw_fini() == Sdma6Status::mmio_failure &&
               fixture.sdma->release() == Sdma6Status::quarantined && fixture.all_leased(),
               "failed halt quarantines and pins leases");
    }
}

void memory_validation() {
    {
        Fixture fixture(true);
        Handle small{};
        fixture.manager->allocate(4096, 4096, Access::read_write, true, small);
        expect(fixture.sdma->hw_init(*fixture.manager, small, fixture.ring1, fixture.writeback) ==
               Sdma6Status::invalid_memory && fixture.no_leases() && fixture.device->log.empty(),
               "ring smaller than ring_bytes rejected without hardware access");
        expect(fixture.init() == Sdma6Status::started, "valid memory still accepted afterwards");
    }
    {
        Fixture fixture(true);
        expect(fixture.sdma->hw_init(*fixture.manager, fixture.ring0, fixture.ring0, fixture.writeback) ==
               Sdma6Status::invalid_memory && fixture.no_leases(), "aliased rings rejected");
    }
    {
        Fixture fixture(true);
        Handle read_only{};
        fixture.manager->allocate(rx7900::memory::page_bytes, rx7900::memory::page_bytes,
                                  Access::read, true, read_only);
        expect(fixture.sdma->hw_init(*fixture.manager, fixture.ring0, fixture.ring1, read_only) ==
               Sdma6Status::invalid_memory && fixture.no_leases(), "read-only writeback rejected");
    }
    {
        Fixture fixture(true);
        Handle read_only{};
        fixture.manager->allocate(sdma6_default_ring_bytes, sdma6_ring_alignment, Access::read, true,
                                  read_only);
        expect(fixture.sdma->hw_init(*fixture.manager, fixture.ring0, read_only, fixture.writeback) ==
               Sdma6Status::invalid_memory && fixture.no_leases(), "read-only ring rejected");
    }
    {
        Fixture fixture(true);
        expect(fixture.sdma->hw_init(*fixture.manager, Handle{}, fixture.ring1, fixture.writeback) ==
               Sdma6Status::invalid_memory && fixture.no_leases(), "stale ring handle rejected");
    }
}

// amdgpu_ib_schedule frame of sdma_v6_0_ring_test_ib starting at ring dword
// start (16-dword aligned), hand-derived from sdma_v6_0.c and pkt_open.h.
std::vector<uint32_t> expected_frame(uint32_t instance, uint64_t ib, uint32_t ib_dwords,
                                     uint64_t fence, uint64_t cond, uint32_t seq, bool burst) {
    const uint32_t ref = 0x400u << instance;
    const uint32_t nop2 = burst ? 0x00010000u : 0;
    return {
        // GCR_REQ: GL2_INV|GL2_WB|GLM_INV|GL1_INV|GLV_INV|GLK_INV|GLI_INV(1) << 16.
        17, 0, 0xc3a10000u, 0, 0,
        // COND_EXE on the cond_exec slot, reference 1, count 6 + 2 + 6 + 4 + 2 = 20.
        9, static_cast<uint32_t>(cond), static_cast<uint32_t>(cond >> 32), 1, 20,
        // POLL_REGMEM HDP_FLUSH FUNC 3: DONE << 2, REQ << 2, ref/mask sdma0 << me,
        // RETRY_COUNT 0xfff | INTERVAL 10.
        0x34000008u, hdp_flush_done << 2, hdp_flush_req << 2, ref, ref, 0x0fff000au,
        // (2 - wptr) & 7 = 2 NOPs so INDIRECT ends on 8 dwords.
        nop2, 0,
        // INDIRECT vmid 0, base & ~0x1f, length 8, csa 0.
        4, static_cast<uint32_t>(ib), static_cast<uint32_t>(ib >> 32), ib_dwords, 0, 0,
        // FENCE MTYPE UC, 32-bit seq; TRAP context 0 (AMDGPU_FENCE_FLAG_INT).
        0x00030005u, static_cast<uint32_t>(fence), static_cast<uint32_t>(fence >> 32), seq, 6, 0,
        // amdgpu_ring_commit pad to 16 dwords.
        nop2, 0,
    };
}

std::vector<uint32_t> expected_ib_frame(const Fixture& fixture, uint32_t instance, uint32_t seq,
                                        bool burst) {
    const uint64_t ib = fixture.address(fixture.ib);
    const uint64_t fence = ib + 512 + instance * 96;
    return expected_frame(instance, ib + instance * 256, 8, fence, fence + 32, seq, burst);
}

void ib_test_pass() {
    Fixture fixture(true);
    auto& device = *fixture.device;
    fixture.init();
    device.log.clear();
    const uint64_t ib = fixture.address(fixture.ib);

    expect(fixture.test_ib() == Sdma6Status::started, "IB tests pass on sdma0 and sdma1");
    const auto report = fixture.sdma->report();
    bool frames = true;

    for (uint32_t instance = 0; instance < 2; ++instance) {
        const uint64_t ring = fixture.address(instance == 0 ? fixture.ring0 : fixture.ring1);
        const uint64_t test = ib + 512 + instance * 96 + 64;
        const std::vector<uint32_t> ib_words{2, static_cast<uint32_t>(test),
                                             static_cast<uint32_t>(test >> 32), 0, 0xdeadbeefu, 0, 0, 0};
        frames = frames && fixture.dwords(ring + 64, 32) == expected_ib_frame(fixture, instance, 1, false) &&
                 fixture.dwords(ib + instance * 256, 8) == ib_words &&
                 device.ib_packets[instance].size() == 1 &&
                 device.ib_packets[instance].at(0) ==
                     std::vector<uint32_t>(ib_words.begin(), ib_words.begin() + 5) &&
                 device.opcodes[instance] == std::vector<uint32_t>{0x00030005u, 17, 9, 0x34000008u, 4,
                                                                   0x00030005u, 6} &&
                 device.hdp_refs[instance] == std::vector<uint32_t>{0x400u << instance} &&
                 device.traps[instance] == 1 && fixture.vram->vram_u32(test) == 0xdeadbeefu &&
                 fixture.vram->vram_u32(ib + 512 + instance * 96) == 1 &&
                 fixture.vram->vram_u32(ib + 512 + instance * 96 + 32) == 1 &&
                 report.ib_test[instance] == Sdma6IbTest::passed && report.fence_seq[instance] == 1 &&
                 report.fence_value[instance] == 1 && report.wptr_bytes[instance] == 192 &&
                 report.ring_ready[instance] && !report.ib_disabled[instance] &&
                 report.ib_polls[instance] == 0 && report.ib_mc[instance] == ib + instance * 256 &&
                 report.ib_test_mc[instance] == test && report.fence_mc[instance] == ib + 512 + instance * 96 &&
                 fixture.vram->vram_u32(fixture.address(fixture.writeback) + instance * 96) == 192;
    }

    expect(frames, "exact ring frames, 8-dword WRITE_LINEAR IB, fence seq 1, slots and RPTR 192");
    const std::vector<Access64> expected{mmio(hdp_flush_index, 0), ring_doorbell(0x200, 192),
                                         mmio(hdp_flush_index, 0), ring_doorbell(0x214, 192)};
    expect(device.log == expected, "CPU HDP flush then the 64-bit doorbell per engine; no SDMA register store");
    expect(device.shadow_at_doorbell[0] == 192 && device.shadow_at_doorbell[1] == 192,
           "WPTR poll shadow published before each doorbell");
    expect(!report.ib_buffer_retained && fixture.leases(fixture.ib) == 0 && !report.burst_nop &&
           report.ib_failed_instance == UINT32_MAX && device.delay_calls == 0,
           "IB buffer lease returned; no poll delay needed");

    expect(fixture.test_ib(sdma6_burst_nop_feature_version) == Sdma6Status::started &&
           fixture.sdma->report().burst_nop && fixture.sdma->report().fence_seq[1] == 2 &&
           fixture.dwords(fixture.address(fixture.ring1) + 192, 32) == expected_ib_frame(fixture, 1, 2, true) &&
           fixture.sdma->report().wptr_bytes[1] == 320,
           "feature version 20: burst NOP headers COUNT(1); second run uses fence seq 2 at 192 bytes");
    expect(fixture.sdma->hw_fini() == Sdma6Status::stopped &&
           fixture.sdma->release() == Sdma6Status::released && fixture.no_leases() &&
           fixture.leases(fixture.ib) == 0, "IB-tested rings halt and release every lease");

    Fixture registers(false);
    registers.init();
    expect(registers.test_ib() == Sdma6Status::started &&
           registers.device->executions[0] == 2 && registers.device->executions[1] == 2 &&
           registers.sdma->report().ib_test[1] == Sdma6IbTest::passed,
           "register WPTR path runs both IB tests");
}

void ib_test_refusals() {
    Fixture fixture(true);
    auto& device = *fixture.device;
    expect(fixture.test_ib() == Sdma6Status::busy && fixture.leases(fixture.ib) == 0,
           "test_ib before hw_init refused");
    fixture.init();
    device.log.clear();
    expect(fixture.test_ib(0, 0) == Sdma6Status::invalid_argument &&
           fixture.test_ib(0, sdma6_ib_test_xgmi_timeout_us + 1) == Sdma6Status::invalid_argument,
           "zero or above-XGMI timeout refused");
    expect(fixture.sdma->test_ib(*fixture.manager, fixture.writeback, {0, 1000}) ==
           Sdma6Status::invalid_memory && fixture.leases(fixture.writeback) == 1,
           "IB buffer overlapping the writeback refused");
    expect(fixture.sdma->test_ib(*fixture.manager, fixture.ring0, {0, 1000}) ==
           Sdma6Status::invalid_memory, "IB buffer overlapping a ring refused");
    Handle read_only{};
    fixture.manager->allocate(rx7900::memory::page_bytes, rx7900::memory::page_bytes, Access::read, true,
                              read_only);
    expect(fixture.sdma->test_ib(*fixture.manager, read_only, {0, 1000}) == Sdma6Status::invalid_memory &&
           fixture.leases(read_only) == 0, "GPU read-only IB buffer refused: SDMA writes fence and data");
    expect(fixture.sdma->test_ib(*fixture.manager, Handle{}, {0, 1000}) == Sdma6Status::invalid_memory,
           "stale IB handle refused");
    rx7900::memory::Manager other;
    expect(fixture.sdma->test_ib(other, fixture.ib, {0, 1000}) == Sdma6Status::invalid_memory,
           "IB buffer from another manager refused");
    expect(device.log.empty() && fixture.leases(fixture.ib) == 0 && fixture.sdma->test_ib(
               *fixture.manager, fixture.ib, {0, 1000}) == Sdma6Status::started,
           "refusals touched no hardware; a valid call then passes");
}

void ib_test_failures() {
    Fixture fixture(true);
    auto& device = *fixture.device;
    fixture.init();
    device.hang_instance = 0;
    device.delay_calls = 0;

    expect(fixture.test_ib(0, 40) == Sdma6Status::ib_test_failed, "sdma0 IB timeout fails the IB tests");
    auto report = fixture.sdma->report();
    expect(report.ib_test[0] == Sdma6IbTest::timeout && report.ib_disabled[0] && !report.ring_ready[0] &&
           report.ib_polls[0] == 40 && report.fence_value[0] == 0 && report.fence_seq[0] == 1 &&
           report.ib_test[1] == Sdma6IbTest::passed && report.ring_ready[1] &&
           report.ib_failed_instance == 0 && device.delay_calls == 40,
           "Linux continues after sdma0: sdma1 passes; 40 x 1 us fence polls on sdma0");
    expect(report.ib_buffer_retained && fixture.leases(fixture.ib) == 1 &&
           fixture.test_ib() == Sdma6Status::quarantined, "IB buffer kept; later IB tests refused");
    expect(fixture.sdma->hw_fini() == Sdma6Status::stopped &&
           fixture.sdma->resume() == Sdma6Status::quarantined &&
           fixture.sdma->release() == Sdma6Status::quarantined && fixture.all_leased(),
           "halted, but neither resume nor release while the ring is quarantined");
    expect(fixture.sdma->release_after_reset() == Sdma6Status::quarantined, "unconfirmed reset keeps leases");
    device.reset_confirmed = true;
    expect(fixture.sdma->release_after_reset() == Sdma6Status::released && fixture.no_leases() &&
           fixture.leases(fixture.ib) == 0 &&
           fixture.sdma->report().ib_test[0] == Sdma6IbTest::not_run,
           "confirmed reset releases the rings and the IB buffer");

    Fixture corrupt(true);
    corrupt.init();
    corrupt.device->corrupt_ib_instance = 1;
    expect(corrupt.test_ib() == Sdma6Status::ib_test_failed &&
           corrupt.sdma->report().ib_test[1] == Sdma6IbTest::failed &&
           corrupt.sdma->report().fence_value[1] == 1 && corrupt.sdma->report().ib_disabled[1] &&
           corrupt.sdma->report().ib_test[0] == Sdma6IbTest::passed &&
           corrupt.sdma->report().ib_failed_instance == 1,
           "fence signalled with a wrong destination value: -EINVAL, sdma1 disabled");

    Fixture doorbell(true);
    doorbell.init();
    doorbell.device->doorbell_ok = false;
    expect(doorbell.test_ib() == Sdma6Status::doorbell_failure &&
           doorbell.sdma->report().failed_instance == 0 &&
           doorbell.sdma->report().ib_buffer_retained &&
           doorbell.test_ib() == Sdma6Status::quarantined,
           "failed IB-test doorbell quarantines the protocol with the IB buffer leased");

    Fixture flush(true);
    flush.init();
    flush.device->fail_write = hdp_flush_index;
    expect(flush.test_ib() == Sdma6Status::mmio_failure && flush.sdma->report().ib_buffer_retained,
           "failed CPU HDP flush before the IB-test WPTR quarantines");
}

// FenceManager platform for SDMA submissions plus the atomic WPTR store double.
struct FenceModel final {
    bool held = false;
    unsigned errors = 0;
    uint64_t now = 1000;
    unsigned stores = 0;

    FencePlatform platform() {
        return {this,
                [](void* c) noexcept { auto& p = *static_cast<FenceModel*>(c); p.errors += p.held; p.held = true; },
                [](void* c) noexcept { auto& p = *static_cast<FenceModel*>(c); p.errors += !p.held; p.held = false; },
                [](void* c) noexcept { return static_cast<FenceModel*>(c)->now; },
                [](void*, uint32_t) noexcept {}, [](void*, uint32_t) noexcept {},
                [](void* c, uint32_t, uint64_t timeout) noexcept {
                    static_cast<FenceModel*>(c)->now += timeout;
                    return false;
                }, nullptr, nullptr};
    }

    static bool store(void* context, uint8_t* address, uint64_t value) noexcept {
        ++static_cast<FenceModel*>(context)->stores;
        const rx7900::sdma::CommandBuffer slot(address, 8);
        return slot.write_u32(0, static_cast<uint32_t>(value)) &&
               slot.write_u32(4, static_cast<uint32_t>(value >> 32));
    }
};

struct SubmitFixture final {
    Fixture fixture;
    FenceModel model;
    FenceManager fences;
    Handle user{};
    uint64_t user_mc = 0;
    uint64_t wb = 0;

    explicit SubmitFixture(bool doorbells = true, bool ib_tests = true) : fixture(doorbells) {
        fixture.init();

        if (ib_tests) {
            fixture.test_ib();
        }

        fences.initialize(model.platform());
        fixture.manager->allocate(4 * rx7900::memory::page_bytes, rx7900::memory::page_bytes,
                                  Access::read_write, true, user);
        user_mc = fixture.address(user);
        wb = fixture.address(fixture.writeback);
    }

    Sdma6Status enable() {
        return fixture.sdma->enable_submission(fences, {{&model, FenceModel::store}});
    }

    // WRITE_LINEAR of one dword to user byte dest, padded with NOPs to 8 dwords.
    void write_ib(uint32_t offset, uint32_t dest, uint32_t value) {
        const uint64_t target = user_mc + dest;
        const uint32_t words[8]{2, static_cast<uint32_t>(target), static_cast<uint32_t>(target >> 32),
                                0, value, 0, 0, 0};

        for (uint32_t index = 0; index < 8; ++index) {
            fixture.vram->put_vram_u32(user_mc + offset + index * 4, words[index]);
        }
    }

    Sdma6Status submit(uint32_t instance, uint32_t offset, uint32_t& seq, uint32_t dwords = 8) {
        return fixture.sdma->submit({instance, user, offset, dwords}, seq);
    }

    uint32_t vram(uint64_t address) const { return fixture.vram->vram_u32(address); }
};

void submission_enable() {
    SubmitFixture before(true, false);
    expect(before.enable() == Sdma6Status::unavailable, "no IB-tested ring: nothing to attach");
    SubmitFixture s;
    expect(s.fixture.sdma->enable_submission(s.fences, {{nullptr, nullptr}}) ==
           Sdma6Status::invalid_argument, "doorbell mode needs the atomic WPTR store");
    expect(s.enable() == Sdma6Status::started, "enable after the IB tests");
    const Sdma6Report report = s.fixture.sdma->report();
    bool ok = report.submission_enabled;

    for (uint32_t instance = 0; instance < 2; ++instance) {
        const FenceRingReport fence = s.fences.ring_report(fence_ring_sdma0 + instance);
        ok = ok && report.submit_enabled[instance] && fence.state == FenceRingState::active &&
             fence.engine == FenceEngine::sdma && fence.instance == instance &&
             fence.sync_seq == 1 && fence.retired_position == 192 / 4 &&
             report.submit_fence_mc[instance] == s.wb + 192 + instance * 64 &&
             report.submit_cond_exec_mc[instance] == s.wb + 224 + instance * 64 &&
             s.vram(s.wb + 192 + instance * 64) == 1 && s.vram(s.wb + 224 + instance * 64) == 1;
    }

    expect(ok, "both rings attached; writeback fence slot = IB test seq 1, cond_exec = 1");
    expect(s.enable() == Sdma6Status::busy && s.fixture.test_ib() == Sdma6Status::busy,
           "enable once; no IB tests on fence-managed rings");
    expect(s.fixture.sdma->hw_fini() == Sdma6Status::stopped &&
           s.fixture.sdma->resume() == Sdma6Status::busy &&
           s.fixture.sdma->release() == Sdma6Status::busy,
           "resume (WPTR rewind) and release refused while submission is enabled");
    // Manager allocations are whole pages, so every hw_init writeback already
    // holds sdma6_submission_writeback_bytes; enable_submission still checks it.
}

void submission_frames() {
    SubmitFixture s;
    s.enable();
    auto& device = *s.fixture.device;
    device.log.clear();
    s.write_ib(0, 2048, 0xabcd0001u);
    uint32_t seq = 0;
    expect(s.submit(0, 0, seq) == Sdma6Status::started && seq == 2, "sdma0 submit: fence seq 2");
    const uint64_t ring0 = s.fixture.address(s.fixture.ring0);
    expect(s.fixture.dwords(ring0 + 192, 32) ==
               expected_frame(0, s.user_mc, 8, s.wb + 192, s.wb + 224, 2, false),
           "IB-test frame layout with the caller's IB and the writeback fence/cond_exec slots");
    const Sdma6Report report = s.fixture.sdma->report();
    expect(report.wptr_bytes[0] == 320 && s.model.stores == 1 &&
           device.log == std::vector<Access64>{mmio(hdp_flush_index, 0), ring_doorbell(0x200, 320)} &&
           device.shadow_at_doorbell[0] == 320, "atomic shadow store, HDP flush, 64-bit doorbell");
    expect(s.vram(s.user_mc + 2048) == 0xabcd0001u && s.vram(s.wb + 192) == 2 && device.traps[0] == 2 &&
           s.fences.query(fence_ring_sdma0, 2) == FenceStatus::signalled,
           "engine ran the IB, wrote fence 2 and raised TRAP");
    expect(s.fixture.leases(s.user) == 1 && s.fixture.sdma->retire() == Sdma6Status::started &&
           s.fixture.leases(s.user) == 0, "IB lease returned once signalled");
    // Interrupt-driven: the SDMA1 trap record signals SDMA1's fence.
    s.write_ib(256, 2052, 0xabcd0002u);
    s.submit(1, 256, seq);
    IrqReport irq{};
    IrqVmFaultLog log{};
    ++irq.sdma_trap[1];
    s.fences.process_interrupts({&irq, &log, false});
    expect(s.fences.ring_report(fence_ring_sdma0 + 1).last_seq == 2 &&
           s.fences.ring_report(fence_ring_sdma0 + 1).interrupt_processes == 1,
           "sdma_v6_0_process_trap_irq record processes instance 1");
    // Register WPTR path.
    SubmitFixture r(false);
    expect(r.fixture.sdma->enable_submission(r.fences, {{nullptr, nullptr}}) == Sdma6Status::started,
           "register mode needs no shadow store");
    r.write_ib(0, 2048, 0x77u);
    const unsigned runs = r.fixture.device->executions[1];
    expect(r.submit(1, 0, seq) == Sdma6Status::started && r.fixture.device->executions[1] == runs + 1 &&
           r.vram(r.user_mc + 2048) == 0x77u && r.fixture.device->at(q(1, rb_wptr)) == 320,
           "RB_WPTR/RB_WPTR_HI publish the frame");
    expect(s.model.errors == 0 && r.model.errors == 0, "fence lock balanced");
}

void submission_wrap() {
    SubmitFixture s;
    s.enable();
    uint32_t seq = 0;
    bool ok = true;

    // 32-dword frames on a 2048-dword ring from byte 192: about 2.3 wraps.
    for (uint32_t index = 0; index < 150; ++index) {
        s.write_ib((index % 64) * 32, 4096 + index * 4, 0x9000u + index);
        ok = ok && s.submit(1, (index % 64) * 32, seq) == Sdma6Status::started && seq == 2 + index &&
             s.fences.query(fence_ring_sdma0 + 1, seq) == FenceStatus::signalled;
    }

    bool results = true;

    for (uint32_t index = 0; index < 150; ++index) {
        results = results && s.vram(s.user_mc + 4096 + index * 4) == 0x9000u + index;
    }

    const Sdma6Report report = s.fixture.sdma->report();
    expect(ok && results && report.wptr_bytes[1] == 192 + 150 * 128 && report.submissions[1] == 150 &&
           report.free_dwords[1] == 2048, "ring reused across wraps; every IB ran and fence signalled");
    s.fixture.sdma->retire();
    expect(s.fixture.leases(s.user) == 0, "no lease left");
}

void submission_hang_and_faults() {
    SubmitFixture s;
    s.enable();
    s.fixture.device->hang_instance = 0;
    uint32_t seq = 0;
    bool ok = true;
    s.write_ib(0, 2048, 1);

    for (uint32_t index = 0; index < 4; ++index) {
        ok = ok && s.submit(0, 0, seq) == Sdma6Status::started;
    }

    expect(ok && s.submit(0, 0, seq) == Sdma6Status::ring_full &&
           s.fixture.sdma->report().free_dwords[0] == 2048 - 3 * 32,
           "stalled engine: four fences outstanding, then ring_full");
    s.model.now += 10 * 1000000ULL;
    s.fences.poll();
    expect(s.fences.ring_report(fence_ring_sdma0).hang_reason == FenceHangReason::job_timeout &&
           s.submit(0, 0, seq) == Sdma6Status::ring_hung &&
           s.submit(1, 0, seq) == Sdma6Status::started, "10000 ms SDMA lockup quarantines sdma0 only");
    expect(s.fixture.sdma->disable_submission() == Sdma6Status::ring_hung &&
           s.fixture.leases(s.user) == 4, "disable refused; stalled IB leases kept");
    s.fixture.device->reset_confirmed = true;
    expect(s.fixture.sdma->release_after_reset() == Sdma6Status::released &&
           s.fixture.leases(s.user) == 0 &&
           s.fences.ring_report(fence_ring_sdma0).state == FenceRingState::detached &&
           s.fences.ring_report(fence_ring_sdma0 + 1).state == FenceRingState::detached,
           "confirmed reset detaches both fence rings and returns every IB lease");
    // VMID 0 UTCL2 fault naming SDMA0 (gfxhub CID 13).
    SubmitFixture v;
    v.enable();
    IrqReport irq{};
    IrqVmFaultLog log{};
    log.total = 1;
    log.entries[0].vmid = 0;
    log.entries[0].register_hub = VmHub::gfxhub;
    log.entries[0].status_read = true;
    log.entries[0].status = (13u << 9) | 1u;
    v.fences.process_interrupts({&irq, &log, false});
    v.write_ib(0, 2048, 1);
    expect(v.submit(0, 0, seq) == Sdma6Status::ring_hung && v.submit(1, 0, seq) == Sdma6Status::started,
           "SDMA0 VM fault quarantines sdma0, sdma1 keeps working");
}

void submission_refusals() {
    SubmitFixture s;
    uint32_t seq = 5;
    expect(s.submit(0, 0, seq) == Sdma6Status::unavailable && seq == 0, "submit before enable refused");
    s.enable();
    s.fixture.device->log.clear();
    expect(s.submit(0, 0, seq, 5) == Sdma6Status::invalid_argument &&
           s.submit(0, 0, seq, 0) == Sdma6Status::invalid_argument &&
           s.submit(2, 0, seq) == Sdma6Status::invalid_argument,
           "IB length must be a nonzero multiple of 8; instance bounded");
    expect(s.submit(0, 16, seq) == Sdma6Status::invalid_memory &&
           s.fixture.sdma->submit({0, s.fixture.writeback, 0, 8}, seq) == Sdma6Status::invalid_memory &&
           s.fixture.sdma->submit({0, s.fixture.ring1, 0, 8}, seq) == Sdma6Status::invalid_memory &&
           s.fixture.sdma->submit({0, Handle{}, 0, 8}, seq) == Sdma6Status::invalid_memory,
           "IB alignment, overlap and handle refused");
    expect(s.fixture.device->log.empty() && s.fixture.leases(s.user) == 0 &&
           s.fences.ring_report(fence_ring_sdma0).sync_seq == 1, "refusals touched nothing");
    s.fixture.device->doorbell_ok = false;
    s.write_ib(0, 2048, 1);
    expect(s.submit(0, 0, seq) == Sdma6Status::doorbell_failure &&
           s.fences.ring_report(fence_ring_sdma0).hang_reason == FenceHangReason::submit_failure &&
           s.submit(1, 0, seq) == Sdma6Status::quarantined, "doorbell failure quarantines");
}

void submission_shutdown() {
    SubmitFixture s;
    s.enable();
    uint32_t seq = 0;
    s.write_ib(0, 2048, 1);
    s.submit(0, 0, seq);
    expect(s.fixture.sdma->disable_submission() == Sdma6Status::started &&
           s.fixture.leases(s.user) == 0 && !s.fixture.sdma->report().submission_enabled,
           "idle rings detach and leases return");
    expect(s.fixture.sdma->hw_fini() == Sdma6Status::stopped &&
           s.fixture.sdma->release() == Sdma6Status::released && s.fixture.no_leases(),
           "then the normal halt and release");
}

} // namespace

int main() {
    discovery_validation();
    initialize_arguments();
    doorbell_start();
    register_path_start();
    small_ring();
    fini_resume_release();
    ring_test_timeout();
    failure_paths();
    memory_validation();
    ib_test_pass();
    ib_test_refusals();
    ib_test_failures();
    submission_enable();
    submission_frames();
    submission_wrap();
    submission_hang_and_faults();
    submission_refusals();
    submission_shutdown();
    return checks.finish("sdma6_init_tests");
}

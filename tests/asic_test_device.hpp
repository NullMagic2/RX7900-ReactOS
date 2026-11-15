/*
Module: rx7900.tests.asic_device
Purpose: Share a synthetic Navi31 register/VRAM machine across physical-init tests.
Created: 2026-10-21
Architecture: Test-only model of documented Linux v6.12 handshakes; it is not
hardware evidence and is never linked into a driver.
*/
#pragma once

#include <algorithm>
#include <array>
#include <cstdio>
#include <utility>
#include <vector>

#include "asic_init.hpp"
#include "vram_discovery.hpp"
#include "discovery_test_fixture.hpp"

namespace rx7900_test {

struct Checks final {
    unsigned total = 0;
    unsigned failed = 0;

    void expect(bool condition, const char* contract) {
        ++total;

        if (!condition) {
            ++failed;
            std::fprintf(stderr, "FAIL: %s\n", contract);
        }
    }

    int finish(const char* name) const {
        std::printf("%s: %u checks, %u failures\n", name, total, failed);
        return failed == 0 ? 0 : 1;
    }
};

struct IpRecord final {
    uint16_t hardware_id;
    uint8_t major;
    uint8_t minor;
    std::vector<uint32_t> bases;
    uint8_t revision = 0;
    uint8_t instance = 0;
};

// Version-3 binary with one die and the given IP records, both checksums valid.
inline std::vector<uint8_t> discovery_with(const std::vector<IpRecord>& records) {
    size_t size = 144;

    for (const auto& record : records) {
        size += 8 + record.bases.size() * 4;
    }

    std::vector<uint8_t> bytes(size);
    const rx7900::sdma::CommandBuffer output(bytes.data(), bytes.size());
    const auto u16 = [&](size_t offset, uint16_t value) {
        bytes.at(offset) = static_cast<uint8_t>(value);
        bytes.at(offset + 1) = static_cast<uint8_t>(value >> 8);
    };
    output.write_u32(0, 0x28211407);
    u16(4, 1);
    u16(10, static_cast<uint16_t>(size));
    u16(12, 60);
    u16(16, static_cast<uint16_t>(size - 60));
    output.write_u32(60, 0x53445049);
    u16(64, 3);
    u16(66, static_cast<uint16_t>(size - 60));
    u16(72, 1);
    u16(76, 140);
    u16(142, static_cast<uint16_t>(records.size()));
    size_t cursor = 144;

    for (const auto& record : records) {
        u16(cursor, record.hardware_id);
        bytes.at(cursor + 2) = record.instance;
        bytes.at(cursor + 3) = static_cast<uint8_t>(record.bases.size());
        bytes.at(cursor + 4) = record.major;
        bytes.at(cursor + 5) = record.minor;
        bytes.at(cursor + 6) = record.revision;

        for (size_t index = 0; index < record.bases.size(); ++index) {
            output.write_u32(cursor + 8 + index * 4, record.bases.at(index));
        }

        cursor += 8 + record.bases.size() * 4;
    }

    uint16_t sum = 0;

    for (size_t index = 60; index < size; ++index) {
        sum = static_cast<uint16_t>(sum + bytes.at(index));
    }

    u16(14, sum);
    sum = 0;

    for (size_t index = 10; index < size; ++index) {
        sum = static_cast<uint16_t>(sum + bytes.at(index));
    }

    u16(8, sum);
    return bytes;
}

constexpr uint32_t mp_base = 0x16000;
constexpr uint32_t mmhub_base = 0x1a000;
constexpr uint32_t gc_base = 0x1260;
constexpr uint32_t thm_base = 0x16600;
// NBIF segment 5 deliberately lies beyond the 1 MiB aperture (RSMU fallback).
constexpr uint32_t nbif_bases[6]{0x0, 0x14, 0xd20, 0x10400, 0x241b000, 0x4040000};
constexpr uint32_t aperture_bytes = 0x100000;
constexpr uint64_t fb_base = 0x0000008000000000ULL;
constexpr uint32_t memsize_mib = 24576;
constexpr uint64_t arena_offset = 64ULL << 20;
constexpr uint64_t arena_bytes = 16ULL << 20;

inline std::vector<uint8_t> navi31_discovery() {
    return discovery_with({
        {rx7900::hardware::discovery_mp0_hardware_id, 13, 0, {mp_base}},
        {rx7900::hardware::discovery_nbif_hardware_id, 4, 3,
         {nbif_bases[0], nbif_bases[1], nbif_bases[2], nbif_bases[3], nbif_bases[4], nbif_bases[5]}},
        {rx7900::hardware::discovery_mmhub_hardware_id, 3, 0, {mmhub_base}},
        {rx7900::hardware::discovery_mp1_hardware_id, 13, 0, {mp_base}},
        {rx7900::hardware::discovery_gc_hardware_id, 11, 0, {gc_base, 0xa000}},
        {40, 6, 0, {0x4260}},
        // THM instance0 13.0.3 as the Navi31 board reports it (segment 1 off-BAR).
        {rx7900::hardware::discovery_thm_hardware_id, 13, 0, {thm_base, 0x02400c00}, 3},
        // SDMA0/SDMA1 records carry the GC bases on the board (both instance0 6.0.0).
        {42, 6, 0, {gc_base, 0xa000, 0x1c000, 0x02402c00}},
        {43, 6, 0, {gc_base, 0xa000, 0x1c000, 0x02402c00}},
        // ATHUB 3.0.0, HDP 6.0.0 and LSDMA 6.0.0 with the board's sysfs bases.
        {35, 3, 0, {0x0c00, 0x02408c00}},
        {41, 6, 0, {0x0f20, 0x0240a400}},
        {91, 6, 0, {0x11400, 0x02456800}},
        // DMU (DCE_HWIP) 3.2.0, not harvested: amdgpu_device_has_display_hardware.
        {271, 3, 2, {0x12, 0xc0, 0x34c0, 0x9000, 0x02403c00}},
    });
}

// UMC 8.10 instances 0..nodes-1 with the user's board sysfs bases (UMC n:
// 0x0C004000, 0x0C004400, 0x0C400000, 0x0C404000 plus n * UMC_8_NODE_DIST).
inline std::vector<IpRecord> navi31_umc_records(uint32_t nodes = 6, uint8_t minor = 10) {
    std::vector<IpRecord> records;

    for (uint32_t node = 0; node < nodes; ++node) {
        const uint32_t step = node * 0x800000u;
        records.push_back({rx7900::hardware::discovery_umc_hardware_id, 8, minor,
                           {0x0c004000u + step, 0x0c004400u + step, 0x0c400000u + step, 0x0c404000u + step},
                           0, static_cast<uint8_t>(node)});
    }

    return records;
}

struct PspCommandRecord final {
    uint32_t id;
    uint32_t words[6];
    uint32_t first_payload_word;
    uint32_t frame_offset;
    uint32_t fence;
    bool flushed;
};

struct Event final {
    enum class Kind { psp_boot, ring_create, ring_destroy, psp_command, smu_message, hdp_remap, strap,
                      training_save, training_restore };
    Kind kind;
    uint32_t value;
    uint32_t detail;
};

// Synthetic BAR5 register file plus a CPU-visible model of a local-VRAM arena at
// MC fb_base + arena_offset. PSP/SMU respond synchronously to documented doorbells.
class SimulatedNavi31 final {
public:
    std::vector<uint32_t> registers = std::vector<uint32_t>(aperture_bytes / 4, 0);
    std::vector<uint8_t> vram = std::vector<uint8_t>(arena_bytes, 0xa5);
    std::vector<uint8_t> discovery = navi31_discovery();
    // PSP runtime database: header, two directory entries, boot config and SCPM.
    std::vector<uint8_t> runtime_db = std::vector<uint8_t>(0x220, 0);
    // PSP-to-CPU training data 0x8000 below the VRAM top.
    std::vector<uint8_t> training_data = std::vector<uint8_t>(0x1000, 0x3c);
    bool train_hang = false;
    bool save_ok = true;
    bool restore_ok = true;
    uint32_t training_c2p = 0;
    // VM-hub invalidation engine 17 model: semaphore grant and ACK bits.
    bool hub_semaphore_granted = true;
    uint32_t hub_ack_bits = 0xffff;
    unsigned mmhub_invalidations = 0;
    unsigned gfxhub_invalidations = 0;
    unsigned bus_master_enables = 0;
    std::vector<PspCommandRecord> commands;
    std::vector<Event> events;
    std::vector<uint32_t> smu_messages;
    std::vector<uint32_t> smu_parameters;
    std::vector<uint8_t> uploaded_pptable;
    uint32_t running_features_low = 0x1234;
    uint32_t dpm_levels = 3;
    unsigned mode1_resets = 0;
    bool bootloader_after_reset = true;
    // Minimal SDMA6 engine: NOP and 32-bit FENCE from the ring, 64-bit RPTR writeback.
    uint64_t sdma_rptr[2]{};
    uint32_t sdma_executions[2]{};
    uint32_t sdma_hang_instance = UINT32_MAX;
    std::vector<std::pair<uint32_t, uint64_t>> doorbell_writes;
    uint64_t smu_driver_table = 0;
    std::vector<uint64_t> delays;
    uint32_t smn_index = 0;
    // Every RSMU data write as (SMN byte address, value).
    std::vector<std::pair<uint32_t, uint32_t>> smn_writes;
    uint32_t strap = 0x000000ffu;
    uint32_t mp1_flags = 1;
    uint32_t hdp_flushes = 0;
    uint32_t flushes_at_last_frame = 0;
    uint32_t mm_index_writes = 0;
    uint32_t mm_index_hi_writes = 0;
    uint32_t ifwi_ready_after = 0;
    uint32_t ifwi_reads = 0;
    uint32_t ring_status = 0x80000000u;
    uint32_t psp_response_status = 0;
    uint32_t fail_command = UINT32_MAX;
    uint32_t fail_fw_type = UINT32_MAX;
    uint32_t toc_tmr_bytes = 0x004f1234;
    uint32_t smu_response = 1;
    uint32_t smu_fail_message = UINT32_MAX;
    uint32_t mm_data_fail_after = UINT32_MAX;
    uint64_t ring_mc = 0;
    uint32_t ring_bytes = 0;
    uint32_t write_pointer = 0;
    bool drop_fences = false;
    bool sos_alive = false;
    bool reads_ok = true;
    bool writes_ok = true;
    bool remap_readback_corrupt = false;
    bool reset_confirmed = false;
    unsigned boot_commands = 0;
    // Every MM_DATA store as (VRAM byte position, value) (amdgpu_device_mm_access writes).
    std::vector<std::pair<uint64_t, uint32_t>> vram_writes;
    unsigned power_cycles = 0;

    SimulatedNavi31() {
        const rx7900::sdma::CommandBuffer db(runtime_db.data(), runtime_db.size());
        db.write_u32(0, 0x01000ed5u);
        db.write_u32(4, 2);
        db.write_u32(8, 5);
        db.write_u32(12, 0x00080210u);
        db.write_u32(16, 6);
        db.write_u32(20, 0x00040218u);
        db.write_u32(0x210, 0);
        db.write_u32(0x218, 1);
        rx7900::sdma::CommandBuffer(training_data.data(), training_data.size()).write_u32(0, 0x54534942u);
        power_on_defaults();
    }

    // S3/S4 power loss of the whole card: every register returns to its power-on
    // value (BIOS scratch included, so the VBIOS POST flag is gone), the engines
    // and PSP/SMU firmware lose all state, local VRAM loses its contents
    // (including the P2C training block), and only host system memory behind the
    // GART keeps its bytes. The IFWI and the PSP bootloader come up again.
    void power_cycle() {
        registers.assign(registers.size(), 0);
        std::fill(vram.begin(), vram.end(), static_cast<uint8_t>(0xa5));
        std::fill(training_data.begin(), training_data.end(), static_cast<uint8_t>(0));
        sos_alive = false;
        sdma_rptr[0] = 0;
        sdma_rptr[1] = 0;
        write_pointer = 0;
        smu_driver_table = 0;
        strap = 0x000000ffu;
        smn_index = 0;
        ++power_cycles;
        power_on_defaults();
    }

    // Register values a powered-on, IFWI-booted Navi31 shows before any driver.
    void power_on_defaults() {
        reg(mp_base + 0x63) = 0x80000000u;
        reg(0x16061) = 0x80000001u;
        // GC: RLC autoload complete (RLC_RLCS_BOOTLOAD_STATUS) and a GB_ADDR_CONFIG.
        reg(0xa000 + 0x4e82) = 0x80000000u;
        reg(gc_base + 0x13de) = 0x00000444u;
        reg(nbif_bases[2] + 0xc3) = memsize_mib;
        reg(mmhub_base + 0x8ec) = static_cast<uint32_t>(fb_base >> 24);
        reg(mmhub_base + 0x8ed) = static_cast<uint32_t>((fb_base >> 24) + (memsize_mib >> 4) - 1);
        reg(mmhub_base + 0x8d7) = 0x00000020u;
    }

    uint32_t& reg(uint32_t index) { return registers.at(index); }

    rx7900::hardware::RegisterIo io() noexcept {
        return {this, aperture_bytes, read, write, delay};
    }

    uint8_t* cpu(uint64_t mc) {
        if (mc >= fb_base + arena_offset && mc - fb_base - arena_offset < vram.size()) {
            return vram.data() + (mc - fb_base - arena_offset);
        }

        return gart(mc);
    }

    // Host system memory behind the GART (GTT): a bump allocator over one block.
    static constexpr uint64_t system_bus_base = 0x0000002000000000ULL;
    std::vector<uint8_t> system_ram = std::vector<uint8_t>(8u << 20, 0x5a);
    size_t system_used = 0;
    unsigned gtt_allocations = 0;
    unsigned gtt_releases = 0;
    bool gtt_ok = true;

    uint8_t* system(uint64_t bus) {
        if (bus < system_bus_base || bus - system_bus_base >= system_ram.size()) {
            return nullptr;
        }

        return system_ram.data() + (bus - system_bus_base);
    }

    // MMHUB VMID0 GART walk exactly as programmed: CONTEXT0 enabled, start/end
    // page numbers, page-directory physical base (single-level flat table).
    uint8_t* gart(uint64_t mc) {
        const auto& hub = rx7900::hardware::mmhub_3_0_0_layout;

        if ((reg(mmhub_base + hub.context0_control) & 1u) == 0) {
            return nullptr;
        }

        const uint64_t start = (static_cast<uint64_t>(reg(mmhub_base + hub.context0_start_high)) << 44) |
                               (static_cast<uint64_t>(reg(mmhub_base + hub.context0_start_low)) << 12);
        const uint64_t end = (static_cast<uint64_t>(reg(mmhub_base + hub.context0_end_high)) << 44) |
                             (static_cast<uint64_t>(reg(mmhub_base + hub.context0_end_low)) << 12) | 0xfff;
        const uint64_t base = ((static_cast<uint64_t>(reg(mmhub_base + hub.context0_base_high)) << 32) |
                               reg(mmhub_base + hub.context0_base_low)) & 0x0000fffffffff000ULL;

        if (mc < start || mc > end) {
            return nullptr;
        }

        const uint64_t table_mc = base - (0x20ULL << 24) + fb_base + ((mc - start) >> 12) * 8;

        if (table_mc < fb_base + arena_offset || table_mc + 8 - fb_base - arena_offset > vram.size()) {
            return nullptr;
        }

        uint32_t low = 0;
        uint32_t high = 0;
        const rx7900::ByteView entry(vram.data() + (table_mc - fb_base - arena_offset), 8);
        entry.read_u32(0, low);
        entry.read_u32(4, high);
        const uint64_t pte = (static_cast<uint64_t>(high) << 32) | low;

        if ((pte & 3u) != 3u) {
            return nullptr;
        }

        uint8_t* page = system(pte & 0x0000fffffffff000ULL);
        return page == nullptr ? nullptr : page + (mc & 0xfff);
    }

    static bool gtt_allocate(void* context, uint32_t bytes, uint8_t*& cpu_bytes, uint64_t& bus,
                             void*& token) noexcept {
        auto& self = *static_cast<SimulatedNavi31*>(context);
        const size_t rounded = (static_cast<size_t>(bytes) + 0xfff) & ~static_cast<size_t>(0xfff);

        if (!self.gtt_ok || rounded == 0 || rounded > self.system_ram.size() - self.system_used) {
            return false;
        }

        cpu_bytes = self.system_ram.data() + self.system_used;
        bus = system_bus_base + self.system_used;
        token = cpu_bytes;
        self.system_used += rounded;
        ++self.gtt_allocations;
        return true;
    }

    static bool gtt_release(void* context, void*) noexcept {
        ++static_cast<SimulatedNavi31*>(context)->gtt_releases;
        return true;
    }

    rx7900::hardware::GttPlatform gtt() noexcept { return {this, gtt_allocate, gtt_release}; }

    uint32_t vram_u32(uint64_t mc) {
        uint32_t value = 0;
        const uint8_t* bytes = cpu(mc);

        if (bytes != nullptr) {
            rx7900::ByteView(bytes, 4).read_u32(0, value);
        }

        return value;
    }

    void put_vram_u32(uint64_t mc, uint32_t value) {
        uint8_t* bytes = cpu(mc);

        if (bytes != nullptr) {
            rx7900::sdma::CommandBuffer(bytes, 4).write_u32(0, value);
        }
    }

    static bool read(void* context, uint32_t offset, uint32_t& value) noexcept {
        auto& self = *static_cast<SimulatedNavi31*>(context);
        const uint32_t index = offset / 4;

        if (!self.reads_ok || (offset & 3) != 0 || index >= self.registers.size()) {
            return false;
        }

        if (index == 0x1) {
            if (self.mm_index_writes > self.mm_data_fail_after) {
                return false;
            }

            const uint64_t position = (static_cast<uint64_t>(self.reg(0x6)) << 31) |
                                      (self.reg(0x0) & 0x7fffffffu);
            const uint64_t top = static_cast<uint64_t>(memsize_mib) << 20;
            value = self.vram_byte(self.discovery, top - (64 << 10), position) |
                    self.vram_byte(self.runtime_db, top - 0x100000, position) |
                    self.vram_byte(self.training_data, top - 0x8000, position);
            return true;
        }

        if (index == mmhub_base + 0x751 + 17) {
            value = self.hub_semaphore_granted ? 1u : 0u;
            return true;
        }

        if (index == mmhub_base + 0x775 + 17 || index == gc_base + 0x16bd + 17) {
            value = self.hub_ack_bits;
            return true;
        }

        if (index == 0x16061) {
            ++self.ifwi_reads;
            value = self.ifwi_reads > self.ifwi_ready_after ? self.reg(index) :
                                                            (self.reg(index) & 0x7fffffffu);
            return true;
        }

        if (index == nbif_bases[1] + 1) {
            value = self.smn(self.smn_index);
            return true;
        }

        if (self.remap_readback_corrupt && index == nbif_bases[2] + 0x12d) {
            value = self.reg(index) ^ 4;
            return true;
        }

        value = self.reg(index);
        return true;
    }

    // One little-endian dword of a region placed at start, or zero outside it.
    static uint32_t vram_byte(const std::vector<uint8_t>& region, uint64_t start, uint64_t position) {
        uint32_t value = 0;

        for (uint32_t byte = 0; byte < 4; ++byte) {
            if (position + byte >= start && position + byte - start < region.size()) {
                value |= static_cast<uint32_t>(region.at(position + byte - start)) << (byte * 8);
            }
        }

        return value;
    }

    uint32_t smn(uint32_t address) const {
        if (address == 0x03b10024u) {
            return mp1_flags;
        }

        if (address == (nbif_bases[5] + 0xd102u) * 4) {
            return strap;
        }

        return 0;
    }

    static bool write(void* context, uint32_t offset, uint32_t value) noexcept {
        auto& self = *static_cast<SimulatedNavi31*>(context);
        const uint32_t index = offset / 4;

        if (!self.writes_ok || (offset & 3) != 0 || index >= self.registers.size()) {
            return false;
        }

        self.reg(index) = value;

        if (index == 0x0) {
            ++self.mm_index_writes;
        } else if (index == 0x1) {
            self.vram_writes.emplace_back((static_cast<uint64_t>(self.reg(0x6)) << 31) |
                                              (self.reg(0x0) & 0x7fffffffu), value);
        } else if (index == 0x6) {
            ++self.mm_index_hi_writes;
        } else if (index == nbif_bases[1]) {
            self.smn_index = value;
        } else if (index == nbif_bases[1] + 1) {
            self.smn_writes.emplace_back(self.smn_index, value);
            if (self.smn_index == (nbif_bases[5] + 0xd102u) * 4) {
                self.strap = value;
                self.events.push_back({Event::Kind::strap, value, 0});
            }
        } else if (index == nbif_bases[2] + 0x12e) {
            self.events.push_back({Event::Kind::hdp_remap, value, 0});
        } else if (offset == rx7900::hardware::hdp_remap_byte_offset) {
            ++self.hdp_flushes;
        } else if (index == mp_base + 0x63) {
            self.boot_command(value);
        } else if (index == mp_base + 0x80) {
            self.ring_control(value);
        } else if (index == mp_base + 0x83) {
            self.doorbell(value);
        } else if (index == mp_base + 0x282) {
            self.smu_message(value);
        } else if (index == mmhub_base + 0x763 + 17) {
            ++self.mmhub_invalidations;
        } else if (index == gc_base + 0x16ab + 17) {
            ++self.gfxhub_invalidations;
        }

        return true;
    }

    static void delay(void* context, uint32_t microseconds) noexcept {
        auto& self = *static_cast<SimulatedNavi31*>(context);

        if (self.delays.size() < 4096) {
            self.delays.push_back(microseconds);
        }
    }

    static constexpr uint32_t sdma_queue(uint32_t instance, uint32_t reg) {
        return gc_base + instance * 0x600 + reg;
    }

    void sdma_execute(uint32_t instance, uint64_t wptr) {
        if (instance == sdma_hang_instance || (reg(0xa000 + 0x589a + instance * 0x20) & 1u) != 0 ||
            (reg(sdma_queue(instance, 0x80)) & 1u) == 0) {
            return;
        }

        ++sdma_executions[instance];
        const uint64_t ring = (static_cast<uint64_t>(reg(sdma_queue(instance, 0x81))) << 8) |
                              (static_cast<uint64_t>(reg(sdma_queue(instance, 0x82))) << 40);
        const uint64_t size = 4ULL << ((reg(sdma_queue(instance, 0x80)) >> 1) & 0x1f);
        uint64_t& rptr = sdma_rptr[instance];

        while (rptr < wptr) {
            const uint32_t header = vram_u32(ring + (rptr & (size - 1)));

            const uint32_t op = header & 0xff;
            const auto at = [&](uint64_t index) { return vram_u32(ring + ((rptr + index * 4) & (size - 1))); };

            if (op == 0) {
                rptr += 4;
            } else if (op == 0x11 || op == 0x09) {
                rptr += 20;  // GCR_REQ, COND_EXE
            } else if (op == 0x08) {
                rptr += 24;  // POLL_REGMEM (HDP flush handshake)
            } else if (op == 0x06) {
                rptr += 8;   // TRAP
            } else if (op == 0x04) {
                // INDIRECT: WRITE_LINEAR packets inside the IB.
                const uint64_t ib = (static_cast<uint64_t>(at(2)) << 32) | at(1);
                const uint32_t ib_dwords = at(3);

                for (uint32_t index = 0; index + 4 < ib_dwords + 1 && index < ib_dwords;) {
                    const uint32_t inner = vram_u32(ib + index * 4);

                    if ((inner & 0xff) == 0x02) {
                        const uint64_t target = (static_cast<uint64_t>(vram_u32(ib + (index + 2) * 4)) << 32) |
                                                vram_u32(ib + (index + 1) * 4);
                        put_vram_u32(target, vram_u32(ib + (index + 4) * 4));
                        index += 5 + vram_u32(ib + (index + 3) * 4);
                    } else {
                        ++index;
                    }
                }

                rptr += 24;
            } else if (header == 0x00030005u) {
                const uint64_t address = vram_u32(ring + ((rptr + 4) & (size - 1))) |
                    (static_cast<uint64_t>(vram_u32(ring + ((rptr + 8) & (size - 1)))) << 32);
                put_vram_u32(address, vram_u32(ring + ((rptr + 12) & (size - 1))));
                rptr += 16;
            } else {
                return;
            }
        }

        const uint64_t writeback = (static_cast<uint64_t>(reg(sdma_queue(instance, 0x88))) << 32) |
                                   reg(sdma_queue(instance, 0x89));
        put_vram_u32(writeback, static_cast<uint32_t>(rptr));
        put_vram_u32(writeback + 4, static_cast<uint32_t>(rptr >> 32));
    }

    static bool doorbell32(void* context, uint32_t index, uint32_t value) noexcept {
        auto& self = *static_cast<SimulatedNavi31*>(context);
        self.doorbell_writes.emplace_back(index, value);
        return self.writes_ok;
    }

    static bool doorbell64(void* context, uint32_t index, uint64_t value) noexcept {
        auto& self = *static_cast<SimulatedNavi31*>(context);
        self.doorbell_writes.emplace_back(index, value);

        if (!self.writes_ok) {
            return false;
        }

        for (uint32_t instance = 0; instance < 2; ++instance) {
            if ((self.reg(sdma_queue(instance, 0x92)) & 0x10000000u) != 0 &&
                ((self.reg(sdma_queue(instance, 0xab)) & 0x0ffffffcu) >> 2) == index) {
                self.sdma_execute(instance, value);
            }
        }

        return true;
    }

    rx7900::hardware::DoorbellIo doorbells() noexcept { return {this, 0x80000, doorbell32, doorbell64}; }

    static bool save(void* context) noexcept {
        auto& self = *static_cast<SimulatedNavi31*>(context);
        self.events.push_back({Event::Kind::training_save, 0, 0});
        return self.save_ok;
    }

    static bool restore(void* context) noexcept {
        auto& self = *static_cast<SimulatedNavi31*>(context);
        self.events.push_back({Event::Kind::training_restore, self.hdp_flushes, 0});
        return self.restore_ok;
    }

    rx7900::hardware::PspTrainingIo training_io() noexcept { return {this, save, restore}; }

    static bool bus_master(void* context) noexcept {
        ++static_cast<SimulatedNavi31*>(context)->bus_master_enables;
        return true;
    }

    static bool confirm(void* context) noexcept {
        return static_cast<SimulatedNavi31*>(context)->reset_confirmed;
    }

    void boot_command(uint32_t command) {
        ++boot_commands;
        events.push_back({Event::Kind::psp_boot, command, reg(mp_base + 0x64)});

        // PSP_BL__DRAM_LONG_TRAIN or PSP_BL__DRAM_SHORT_TRAIN.
        if (command == 0x100000 || command == 0x200000) {
            training_c2p = reg(mp_base + 0x64);

            if (train_hang) {
                return;
            }
        }

        reg(mp_base + 0x63) = 0x80000000u;

        if (command == 0x20000) {
            sos_alive = true;
            reg(mp_base + 0x91) = 0x55aa;
            reg(mp_base + 0x7a) = 0x00310035u;
            reg(mp_base + 0x80) = 0x80000000u;
        }
    }

    void ring_control(uint32_t value) {
        if (value == (2u << 16)) {
            ring_mc = (static_cast<uint64_t>(reg(mp_base + 0x86)) << 32) | reg(mp_base + 0x85);
            ring_bytes = reg(mp_base + 0x87);
            events.push_back({Event::Kind::ring_create, ring_bytes, 0});
            reg(mp_base + 0x80) = ring_status;
        } else if (value == 0x30000u) {
            events.push_back({Event::Kind::ring_destroy, 0, 0});
            reg(mp_base + 0x80) = 0x80000000u;
        }
    }

    void doorbell(uint32_t new_pointer) {
        const uint32_t frame = write_pointer % 1024 / 16 * 64;
        write_pointer = new_pointer;
        const uint64_t command = (static_cast<uint64_t>(vram_u32(ring_mc + frame + 4)) << 32) |
                                 vram_u32(ring_mc + frame);
        const uint64_t fence = (static_cast<uint64_t>(vram_u32(ring_mc + frame + 16)) << 32) |
                               vram_u32(ring_mc + frame + 12);
        PspCommandRecord record{};
        record.id = vram_u32(command + 8);

        for (uint32_t index = 0; index < 6; ++index) {
            record.words[index] = vram_u32(command + 28 + index * 4);
        }

        const uint64_t payload = (static_cast<uint64_t>(record.words[1]) << 32) | record.words[0];
        record.first_payload_word = cpu(payload) != nullptr ? vram_u32(payload) : 0;
        record.frame_offset = frame;
        record.fence = vram_u32(ring_mc + frame + 20);
        record.flushed = hdp_flushes > flushes_at_last_frame;
        flushes_at_last_frame = hdp_flushes;
        commands.push_back(record);
        events.push_back({Event::Kind::psp_command, record.id,
                          record.id == 6 ? record.words[3] : 0});
        uint32_t status = psp_response_status;

        if (record.id == fail_command || (record.id == 6 && record.words[3] == fail_fw_type)) {
            status = 0xffff000a;
        }

        put_vram_u32(command + 864, status);
        put_vram_u32(command + 864 + 4, record.id == 4 ? 0x77 : 0);
        put_vram_u32(command + 864 + 16, record.id == 0x20 ? toc_tmr_bytes : 0);

        if (!drop_fences) {
            put_vram_u32(fence, record.fence);
        }
    }

    void smu_message(uint32_t message) {
        const uint32_t parameter = reg(mp_base + 0x292);
        smu_messages.push_back(message);
        smu_parameters.push_back(parameter);
        events.push_back({Event::Kind::smu_message, message, parameter});

        if (message == 0xe) {
            smu_driver_table = (static_cast<uint64_t>(parameter) << 32) | (smu_driver_table & 0xffffffffu);
        } else if (message == 0xf) {
            smu_driver_table = (smu_driver_table & ~0xffffffffULL) | parameter;
        } else if (message == 0x12 && parameter == 1) {
            // Combo PPTable: platform caps, then SkuTable PCIe levels inside PPTable_t.
            put_vram_u32(smu_driver_table + 16, 0x12340000u);
            put_vram_u32(smu_driver_table + 20, 0);
            // software_shutdown_temp (u16 at 29) = 110 C.
            put_vram_u32(smu_driver_table + 28, 0x00006e00u);
            put_vram_u32(smu_driver_table + 832, 0x0000000du);
            put_vram_u32(smu_driver_table + 832 + 1232, 0x06030200u);
            put_vram_u32(smu_driver_table + 832 + 1236, 0x00640606u);
            put_vram_u32(smu_driver_table + 832 + 1240, 0x00c80096u);
        } else if (message == 0x13 && parameter == 0) {
            uploaded_pptable.assign(3972, 0);

            for (uint32_t index = 0; index < 3972; index += 4) {
                rx7900::sdma::CommandBuffer(uploaded_pptable.data(), uploaded_pptable.size())
                    .write_u32(index, vram_u32(smu_driver_table + index));
            }
        }

        if (message == 0x2f) {
            // Mode1Reset: the PSP restarts in its bootloader (no SOS), rings and
            // engines lose their state; VRAM contents and fuses persist.
            sos_alive = false;
            reg(mp_base + 0x91) = 0;
            reg(mp_base + 0x7a) = 0;
            reg(mp_base + 0x80) = 0;
            reg(mp_base + 0x63) = bootloader_after_reset ? 0x80000000u : 0;
            sdma_rptr[0] = 0;
            sdma_rptr[1] = 0;
            ++mode1_resets;
        }

        if (message == smu_fail_message) {
            reg(mp_base + 0x29a) = 0xff;
            return;
        }

        if (message == 0x1f) {
            // GetDpmFreqByIndex: three levels (GFXCLK fine-grained), MHz per clock/level.
            const uint32_t clock = parameter >> 16;
            const uint32_t level = parameter & 0xffff;
            reg(mp_base + 0x292) = level == 0xff ? (dpm_levels | (clock == 0 ? 0x80000000u : 0)) :
                                   1000 * (clock + 1) + 100 * level;
        } else if (message == 0x3) {
            reg(mp_base + 0x292) = 0x3d;
        } else if (message == 0x2) {
            reg(mp_base + 0x292) = 0x004e8300;
        } else if (message == 0xc) {
            reg(mp_base + 0x292) = running_features_low;
        } else if (message == 0xd) {
            reg(mp_base + 0x292) = 0x5;
        }

        reg(mp_base + 0x29a) = smu_response;
    }
};

// Local-VRAM backend: every allocation is CPU-visible inside the simulated arena.
struct VramBackend final {
    SimulatedNavi31* device = nullptr;
    unsigned creates = 0;
    unsigned destroys = 0;

    static rx7900::memory::CreateResult create(void* context, rx7900::sdma::GpuAllocation gpu,
        rx7900::memory::Access, bool, rx7900::memory::Backing& backing) noexcept {
        auto& self = *static_cast<VramBackend*>(context);
        uint8_t* bytes = self.device->cpu(gpu.virtual_address);

        if (bytes == nullptr || self.device->cpu(gpu.virtual_address + gpu.byte_length - 1) == nullptr) {
            return rx7900::memory::CreateResult::no_resources;
        }

        ++self.creates;
        backing = {bytes, bytes, static_cast<size_t>(gpu.byte_length)};
        return rx7900::memory::CreateResult::created;
    }

    static bool destroy(void* context, const rx7900::memory::Backing&,
                        rx7900::sdma::GpuAllocation) noexcept {
        ++static_cast<VramBackend*>(context)->destroys;
        return true;
    }

    rx7900::memory::Backend backend() noexcept { return {this, create, destroy}; }
};

inline rx7900::hardware::VramLocation navi31_vram() {
    const uint64_t bytes = static_cast<uint64_t>(memsize_mib) << 20;
    return {fb_base, fb_base + bytes - 1, bytes, 0x20ULL << 24};
}

} // namespace rx7900_test

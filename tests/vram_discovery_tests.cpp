/*
Module: rx7900.tests.vram_discovery
Purpose: Verify IFWI waiting, MEMSIZE use and indirect VRAM discovery reads.
Created: 2026-10-21
Architecture: Synthetic register machine; no GPU is accessed.
*/
#include <memory>

#include "asic_test_device.hpp"

namespace {

using namespace rx7900::hardware;
using rx7900_test::SimulatedNavi31;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

void acquisition() {
    auto device = std::make_unique<SimulatedNavi31>();
    auto binary = std::make_unique<DiscoveryBinary>();
    VramDiscoveryReport report{};
    const auto status = read_vram_discovery(device->io(), *binary, report);
    const auto bytes = binary->bytes();
    const uint64_t vram = static_cast<uint64_t>(rx7900_test::memsize_mib) << 20;
    bool equal = bytes.size() == discovery_tmr_bytes;

    for (size_t index = 0; equal && index < device->discovery.size(); ++index) {
        uint8_t value = 0;
        equal = bytes.read_u8(index, value) && value == device->discovery.at(index);
    }

    expect(status == VramDiscoveryStatus::acquired && report.status == status,
           "valid IFWI, MEMSIZE and signature acquire discovery");
    expect(report.ifwi_ready && report.ifwi_status == 0x80000001u, "IFWI ready bit observed");
    expect(report.vram_bytes == vram && report.memsize_mib == rx7900_test::memsize_mib,
           "VRAM size comes from RCC_CONFIG_MEMSIZE in MiB");
    expect(report.source_offset == vram - discovery_tmr_offset, "read starts 64 KiB below VRAM top");
    expect(equal, "exact discovery bytes copied through MM_DATA");
    expect(device->mm_index_writes == discovery_tmr_bytes / 4, "one MM_INDEX write per dword");
    expect(device->mm_index_hi_writes == 1 && device->reg(0x6) == ((vram - (64 << 10)) >> 31),
           "MM_INDEX_HI written once with position bits above 31");
    expect((device->reg(0x0) & 0x80000000u) != 0, "MM_INDEX selects the VRAM aperture bit");
    expect(device->delays.empty(), "ready IFWI needs no wait");

    PspRegisters psp;
    expect(validate_psp_discovery(bytes, rx7900_test::aperture_bytes, psp) ==
           DiscoveryStatus::validated && psp.segment0_dword_base() == rx7900_test::mp_base,
           "acquired snapshot feeds the shared checksummed parser");
}

void waiting_and_failures() {
    auto device = std::make_unique<SimulatedNavi31>();
    auto binary = std::make_unique<DiscoveryBinary>();
    VramDiscoveryReport report{};
    device->ifwi_ready_after = 3;
    expect(read_vram_discovery(device->io(), *binary, report) == VramDiscoveryStatus::acquired &&
           report.ifwi_ready && device->delays.size() == 3 && device->delays.at(0) == 1000,
           "IFWI polled with 1 ms waits until ready");

    device = std::make_unique<SimulatedNavi31>();
    device->ifwi_ready_after = UINT32_MAX;
    expect(read_vram_discovery(device->io(), *binary, report) == VramDiscoveryStatus::acquired &&
           !report.ifwi_ready && device->delays.size() == 1000,
           "Linux continues after 1000 IFWI polls and reports not-ready");

    device = std::make_unique<SimulatedNavi31>();
    device->reg(0x16061) = UINT32_MAX;
    device->ifwi_ready_after = 0;
    expect(read_vram_discovery(device->io(), *binary, report) == VramDiscoveryStatus::mmio_failure &&
           binary->bytes().size() == 0, "all-ones IFWI status means a lost device");

    device = std::make_unique<SimulatedNavi31>();
    device->reg(rx7900_test::nbif_bases[2] + 0xc3) = 0;
    expect(read_vram_discovery(device->io(), *binary, report) == VramDiscoveryStatus::no_vram &&
           binary->bytes().size() == 0 && device->mm_index_writes == 0,
           "zero MEMSIZE is not reinterpreted as an APU reservation");

    device = std::make_unique<SimulatedNavi31>();
    device->discovery.at(0) ^= 0xff;
    expect(read_vram_discovery(device->io(), *binary, report) == VramDiscoveryStatus::invalid_binary &&
           binary->bytes().size() == 0, "bad signature leaves no readable snapshot");

    device = std::make_unique<SimulatedNavi31>();
    device->mm_data_fail_after = 7;
    expect(read_vram_discovery(device->io(), *binary, report) == VramDiscoveryStatus::mmio_failure &&
           binary->bytes().size() == 0, "MM_DATA failure aborts the copy");

    device = std::make_unique<SimulatedNavi31>();
    auto io = device->io();
    io.aperture_bytes = 0x58000;
    expect(read_vram_discovery(io, *binary, report) == VramDiscoveryStatus::invalid_argument &&
           device->ifwi_reads == 0, "aperture without C2PMSG_33 is refused before MMIO");
    io = device->io();
    io.read32 = nullptr;
    expect(read_vram_discovery(io, *binary, report) == VramDiscoveryStatus::invalid_argument,
           "missing callbacks are refused");

    device = std::make_unique<SimulatedNavi31>();
    device->discovery.at(10) = 0x01;
    device->discovery.at(11) = 0x30;
    expect(read_vram_discovery(device->io(), *binary, report) == VramDiscoveryStatus::invalid_binary,
           "declared size beyond the 10 KiB reservation is refused");
}

void runtime_database() {
    const uint64_t vram = static_cast<uint64_t>(rx7900_test::memsize_mib) << 20;
    auto device = std::make_unique<SimulatedNavi31>();
    PspRuntimeDatabase database{};
    expect(read_psp_runtime_database(device->io(), vram, database) == VramDiscoveryStatus::acquired &&
           database.present && database.boot_config_valid && database.boot_config == 0 &&
           database.scpm_valid && database.scpm_status == 1,
           "runtime database boot-config and SCPM entries decoded");

    rx7900::sdma::CommandBuffer db(device->runtime_db.data(), device->runtime_db.size());
    db.write_u32(0x210, psp_boot_config_two_stage_training);
    expect(read_psp_runtime_database(device->io(), vram, database) == VramDiscoveryStatus::acquired &&
           database.boot_config == psp_boot_config_two_stage_training,
           "two-stage training bit reported");

    db.write_u32(12, 0x00040210u);
    expect(read_psp_runtime_database(device->io(), vram, database) == VramDiscoveryStatus::acquired &&
           database.present && !database.boot_config_valid && database.scpm_valid,
           "undersized boot-config entry is not found, as in Linux");

    db.write_u32(4, 64);
    expect(read_psp_runtime_database(device->io(), vram, database) == VramDiscoveryStatus::acquired &&
           !database.present, "64 directory entries means no valid database");

    db.write_u32(0, 0);
    expect(read_psp_runtime_database(device->io(), vram, database) == VramDiscoveryStatus::acquired &&
           !database.present && !database.scpm_valid, "missing cookie means no database");

    device->mm_data_fail_after = 0;
    expect(read_psp_runtime_database(device->io(), vram, database) == VramDiscoveryStatus::mmio_failure,
           "indirect read failure reported");
    expect(read_psp_runtime_database(device->io(), 0x80000, database) ==
           VramDiscoveryStatus::invalid_argument, "VRAM smaller than the database offset refused");
}

} // namespace

int main() {
    acquisition();
    waiting_and_failures();
    runtime_database();
    return checks.finish("vram_discovery");
}

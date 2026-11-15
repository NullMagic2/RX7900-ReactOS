/*
Module: rx7900.tests.nbio
Purpose: Verify NBIO discovery, strap/HDP remap, HDP flush, SMN and VRAM placement.
Created: 2026-10-21
Architecture: Synthetic register machine; no GPU is accessed.
*/
#include <memory>
#include <utility>

#include "asic_test_device.hpp"

namespace {

using namespace rx7900::hardware;
using rx7900_test::SimulatedNavi31;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

NbioRegisters registers(const std::vector<uint8_t>& binary) {
    NbioRegisters output;
    validate_nbio_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, output);
    return output;
}

void discovery() {
    const auto binary = rx7900_test::navi31_discovery();
    NbioRegisters output;
    expect(validate_nbio_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                   output) == DiscoveryStatus::validated && output.valid() &&
           output.nbif_base(2) == 0xd20 && output.mmhub_base() == rx7900_test::mmhub_base,
           "NBIF4.3.0 six segments and MMHUB3.0.0 validated");
    expect(validate_nbio_discovery({binary.data(), binary.size()}, 0x7f000, output) ==
           DiscoveryStatus::invalid_argument && !output.valid(),
           "aperture without the HDP remap hole is refused");

    const auto wrong = rx7900_test::discovery_with({
        {discovery_nbif_hardware_id, 4, 2, {0, 0x14, 0xd20, 0x10400, 0x241b000, 0x4040000}},
        {discovery_mmhub_hardware_id, 3, 0, {rx7900_test::mmhub_base}}});
    expect(validate_nbio_discovery({wrong.data(), wrong.size()}, rx7900_test::aperture_bytes,
                                   output) == DiscoveryStatus::unsupported,
           "other NBIF revisions are not assumed compatible");

    const auto short_bases = rx7900_test::discovery_with({
        {discovery_nbif_hardware_id, 4, 3, {0, 0x14, 0xd20}},
        {discovery_mmhub_hardware_id, 3, 0, {rx7900_test::mmhub_base}}});
    expect(validate_nbio_discovery({short_bases.data(), short_bases.size()},
                                   rx7900_test::aperture_bytes, output) == DiscoveryStatus::unsupported,
           "NBIF record with too few segments is refused");

    const auto no_mmhub = rx7900_test::discovery_with({
        {discovery_nbif_hardware_id, 4, 3, {0, 0x14, 0xd20, 0x10400, 0x241b000, 0x4040000}}});
    expect(validate_nbio_discovery({no_mmhub.data(), no_mmhub.size()}, rx7900_test::aperture_bytes,
                                   output) == DiscoveryStatus::invalid_binary,
           "missing MMHUB is refused");
}

void programming() {
    const auto binary = rx7900_test::navi31_discovery();
    auto device = std::make_unique<SimulatedNavi31>();
    Nbio nbio;
    expect(!nbio.flush_hdp(), "flush refused before initialization");
    expect(nbio.initialize(NbioRegisters{}, device->io()) == NbioStatus::unsupported_ip,
           "default register token is refused");
    expect(nbio.initialize(registers(binary), device->io()) == NbioStatus::ready,
           "validated token initializes");
    expect(!nbio.flush_hdp(), "flush refused before HDP remap");
    expect(nbio.program_common() == NbioStatus::ready, "common NBIO programming succeeds");
    expect(device->strap == 0x7fu, "STRAP_NO_SOFT_RESET_DEV0_F2 cleared through RSMU");
    expect(device->reg(0xd20 + 0x12d) == 0x7f000 && device->reg(0xd20 + 0x12e) == 0x7f004,
           "HDP flush registers remapped to the BAR5 hole");
    expect(nbio.flush_hdp() && device->hdp_flushes == 1 && device->reg(0x7f000 / 4) == 0,
           "HDP flush writes zero to the remapped MEM_FLUSH_CNTL");

    device->strap = 0x7f;
    const auto writes_before = device->events.size();
    expect(nbio.program_common() == NbioStatus::ready && device->events.size() == writes_before + 1,
           "already-clear strap is not rewritten");

    uint32_t value = 0;
    expect(nbio.read_smn(0x03b10024u, value) && value == 1 && device->smn_index == 0x03b10024u,
           "SMN read programs index then reads data");

    VramLocation vram{};
    expect(nbio.vram_location(vram) == NbioStatus::ready && vram.mc_start == rx7900_test::fb_base &&
           vram.bytes == (static_cast<uint64_t>(rx7900_test::memsize_mib) << 20) &&
           vram.mc_end == vram.mc_start + vram.bytes - 1 && vram.physical_offset == (0x20ULL << 24),
           "MC VRAM placement decoded from MMHUB and MEMSIZE");

    device->reg(rx7900_test::mmhub_base + 0x8ed) -= 1;
    expect(nbio.vram_location(vram) == NbioStatus::invalid_location && vram.bytes == 0,
           "MEMSIZE beyond the FB window is refused");

    // Doorbell aperture (program_common), soc21 late-init self-ring and GC doorbells.
    expect((device->reg(0xd20 + 0xc0) & 1u) == 1u, "RCC_DOORBELL_APER_EN set by common programming");
    device->reg(0xd20 + 0xc0) = 0xfffffff0u;
    expect(nbio.enable_doorbell_aperture(false) == NbioStatus::ready &&
           device->reg(0xd20 + 0xc0) == 0xfffffff0u &&
           nbio.enable_doorbell_aperture(true) == NbioStatus::ready && device->reg(0xd20 + 0xc0) == 0xfffffff1u,
           "doorbell aperture enable is a read-modify-write of bit 0");
    expect(nbio.enable_doorbell_selfring(0x000000e012340000ULL) == NbioStatus::ready &&
           device->reg(0xd20 + 0xf4) == 0x12340000u && device->reg(0xd20 + 0xf3) == 0xe0u &&
           device->reg(0xd20 + 0xf5) == 0x3u, "self-ring base low, high, then EN|MODE with SIZE 0");
    device->smn_writes.clear();
    expect(nbio.gc_doorbell_init() == NbioStatus::ready && device->smn_writes.size() == 2 &&
           device->smn_writes.at(0) == std::make_pair((0x10400u + 0x4f7640u) * 4, 0x30000007u) &&
           device->smn_writes.at(1) == std::make_pair((0x10400u + 0x4f7643u) * 4, 0x3000000du),
           "S2A doorbell entries 0 and 3 written through RSMU (beyond BAR5)");
    device->writes_ok = false;
    expect(nbio.enable_doorbell_selfring(1) == NbioStatus::mmio_failure &&
           nbio.gc_doorbell_init() == NbioStatus::mmio_failure, "doorbell store failures reported");
    device->writes_ok = true;
    Nbio idle;
    expect(idle.gc_doorbell_init() == NbioStatus::unavailable &&
           idle.enable_doorbell_selfring(1) == NbioStatus::unavailable, "uninitialized doorbell setup refused");

    device = std::make_unique<SimulatedNavi31>();
    device->remap_readback_corrupt = true;
    nbio.initialize(registers(binary), device->io());
    expect(nbio.program_common() == NbioStatus::readback_mismatch && !nbio.flush_hdp(),
           "unconfirmed HDP remap blocks every flush");

    device = std::make_unique<SimulatedNavi31>();
    nbio.initialize(registers(binary), device->io());
    device->writes_ok = false;
    expect(nbio.program_common() == NbioStatus::mmio_failure, "write failure reported");

    auto io = device->io();
    io.aperture_bytes = 0x80000 - 4;
    expect(nbio.initialize(registers(binary), io) == NbioStatus::unavailable,
           "mapping must match the validated aperture");
}

} // namespace

int main() {
    discovery();
    programming();
    return checks.finish("nbio");
}

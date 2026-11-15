/*
Module: rx7900.tests.pci_caps
Purpose: Verify capability walks, Linux v6.12 amdgpu PCIe/ReBAR/restore policy and
upstream path discovery against the user's RX 7900 topology.
Created: 2026-10-22
Architecture: Exercises src/common/pci_caps exactly as the native adapter links it.
Fixture provenance: the first 64 bytes of each function are the user's read-only
sysfs config bytes (2026-10-22). Capability bodies are reconstructed, because
sysfs exposes only 64 bytes to the unprivileged user: pointers follow the live
headers and the Linux 6.12 nbio_4_3_0_offset.h BIF_CFG_DEV0_EPF0 layout; link
fields follow sysfs max/current link speed and width; ReBAR masks follow sysfs
resource0_resize (0xff00) and resource2_resize (0x1fe); the boot log shows the
firmware-assigned 256 MiB BAR0 and 2 MiB BAR2 before amdgpu resized them.
*/
#include <algorithm>
#include <array>
#include <cstdio>
#include <map>
#include <tuple>
#include <vector>

#include "pci_caps.hpp"

namespace {

using rx7900::ByteView;
using rx7900::CapStatus;

unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* contract) {
    ++checks;

    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", contract);
    }
}

using Image = std::array<uint8_t, rx7900::pci_extended_config_bytes>;

void put8(Image& image, size_t offset, uint8_t value) { image.at(offset) = value; }

void put16(Image& image, size_t offset, uint16_t value) {
    put8(image, offset, static_cast<uint8_t>(value));
    put8(image, offset + 1, static_cast<uint8_t>(value >> 8));
}

void put32(Image& image, size_t offset, uint32_t value) {
    put16(image, offset, static_cast<uint16_t>(value));
    put16(image, offset + 2, static_cast<uint16_t>(value >> 16));
}

void put_header(Image& image, const std::array<uint8_t, 64>& header) {
    std::copy(header.begin(), header.end(), image.begin());
}

void put_cap(Image& image, uint8_t offset, uint8_t id, uint8_t next) {
    put8(image, offset, id);
    put8(image, offset + 1u, next);
}

void put_ext(Image& image, uint16_t offset, uint16_t id, uint8_t version, uint16_t next) {
    put32(image, offset, static_cast<uint32_t>(id) | (static_cast<uint32_t>(version) << 16) |
                             (static_cast<uint32_t>(next) << 20));
}

// PCIe v2 capability: flags, link capabilities/status and LNKCAP2.
void put_express(Image& image, uint8_t offset, uint8_t next, uint8_t type, uint8_t max_speed,
                 uint8_t max_width, uint8_t speed, uint8_t width, uint32_t lnkcap2,
                 bool slot = false) {
    put_cap(image, offset, rx7900::pci_cap_id_express, next);
    put16(image, offset + 2u, static_cast<uint16_t>(2u | (type << 4) | (slot ? 0x100u : 0u)));
    put32(image, offset + 4u, 0x00008fc0);
    put16(image, offset + 8u, 0x2930);
    put32(image, offset + 0x0cu, static_cast<uint32_t>(max_speed) | (static_cast<uint32_t>(max_width) << 4) |
                                     0x00400c00u);
    put16(image, offset + 0x10u, 0x0040);
    put16(image, offset + 0x12u, static_cast<uint16_t>(speed | (width << 4)));
    put32(image, offset + 0x24u, 0x0009181f);
    put16(image, offset + 0x28u, 0x0400);
    put32(image, offset + 0x2cu, lnkcap2);
    put16(image, offset + 0x30u, 0x0004);
}

// Live sysfs config bytes 0x00-0x3f, 2026-10-22.
constexpr std::array<uint8_t, 64> gpu_header{
    0x02, 0x10, 0x4c, 0x74, 0x07, 0x04, 0x10, 0x00, 0xc8, 0x00, 0x00, 0x03, 0x10, 0x00, 0x80, 0x00,
    0x0c, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00,
    0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 0xa0, 0xdf, 0x00, 0x00, 0x00, 0x00, 0xa2, 0x1d, 0x1e, 0x47,
    0x00, 0x00, 0xb0, 0xdf, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x01, 0x00, 0x00};
constexpr std::array<uint8_t, 64> downstream_header{
    0x02, 0x10, 0x79, 0x14, 0x07, 0x04, 0x10, 0x00, 0x10, 0x00, 0x04, 0x06, 0x10, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x03, 0x03, 0x00, 0xf1, 0xf1, 0x00, 0x00,
    0xa0, 0xdf, 0xb0, 0xdf, 0x01, 0x00, 0x11, 0x00, 0x28, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x01, 0x1a, 0x00};
constexpr std::array<uint8_t, 64> upstream_header{
    0x02, 0x10, 0x78, 0x14, 0x07, 0x00, 0x10, 0x00, 0x10, 0x00, 0x04, 0x06, 0x10, 0x00, 0x01, 0x00,
    0x00, 0x00, 0xc0, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x00, 0xf1, 0xf1, 0x00, 0x00,
    0xa0, 0xdf, 0xb0, 0xdf, 0x01, 0x00, 0x11, 0x00, 0x28, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x01, 0x1a, 0x00};
constexpr std::array<uint8_t, 64> root_port_header{
    0x22, 0x10, 0xed, 0x14, 0x07, 0x04, 0x10, 0x00, 0x00, 0x00, 0x04, 0x06, 0x10, 0x00, 0x81, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x00, 0xf1, 0xf1, 0x00, 0x00,
    0xa0, 0xdf, 0xc0, 0xdf, 0x01, 0x00, 0x11, 0x00, 0x28, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x1a, 0x00};

constexpr uint8_t gen4 = 4;
constexpr uint32_t gen1_to_4_vector = 0x1e;

// GPU 0000:03:00.0: legacy endpoint, 16 GT/s x16 capable and trained to its
// own switch at 16 GT/s x16 (sysfs max/current link of 03:00.0).
Image gpu_image(bool msi_enabled = false) {
    Image image{};
    put_header(image, gpu_header);
    put_cap(image, 0x48, rx7900::pci_cap_id_vendor, 0x50);
    put_cap(image, 0x50, rx7900::pci_cap_id_power_management, 0x64);
    put16(image, 0x52, 0x7e03);
    put16(image, 0x54, 0x0008);
    put_express(image, 0x64, 0xa0, rx7900::pcie_type_legacy_endpoint, gen4, 16, gen4, 16,
                gen1_to_4_vector);
    put_cap(image, 0xa0, rx7900::pci_cap_id_msi, 0x00);
    put16(image, 0xa2, static_cast<uint16_t>(0x0080u | (msi_enabled ? 1u : 0u)));
    put32(image, 0xa4, msi_enabled ? 0xfee00000u : 0u);
    put32(image, 0xa8, 0);
    put16(image, 0xac, msi_enabled ? 0x0022 : 0);
    // Extended list from the BIF_CFG_DEV0_EPF0 register layout.
    put_ext(image, 0x100, rx7900::pci_ext_cap_id_vendor, 1, 0x150);
    put_ext(image, 0x150, rx7900::pci_ext_cap_id_aer, 2, 0x200);
    put_ext(image, 0x200, rx7900::pci_ext_cap_id_rebar, 1, 0x240);
    put32(image, 0x204, 0xff00u << 4);           // BAR0: 256 MiB .. 32 GiB
    put32(image, 0x208, 0x00000840);             // BAR0, NBAR 2, size 8 (256 MiB)
    put32(image, 0x20c, 0x1feu << 4);            // BAR2: 2 MiB .. 256 MiB
    put32(image, 0x210, 0x00000102);             // BAR2, size 1 (2 MiB)
    put_ext(image, 0x240, 0x04, 1, 0x270);
    put_ext(image, 0x270, 0x19, 1, 0x2a0);
    put_ext(image, 0x2a0, 0x0d, 1, 0x2d0);
    put_ext(image, 0x2d0, 0x1b, 1, 0x320);
    put_ext(image, 0x320, rx7900::pci_ext_cap_id_ltr, 1, 0x410);
    put32(image, 0x324, 0x10031003);
    put_ext(image, 0x410, 0x26, 1, 0x440);
    put_ext(image, 0x440, 0x27, 1, 0x000);
    return image;
}

Image downstream_image() {
    Image image{};
    put_header(image, downstream_header);
    put_express(image, 0x50, 0x00, rx7900::pcie_type_downstream, gen4, 16, gen4, 16, gen1_to_4_vector);
    return image;
}

// 01:00.0: x16 capable, trained x8 to the root port.
Image upstream_image() {
    Image image{};
    put_header(image, upstream_header);
    put_express(image, 0x48, 0x00, rx7900::pcie_type_upstream, gen4, 16, gen4, 8, gen1_to_4_vector);
    return image;
}

// 00:01.1: Gen4 root port, maximum and current x8.
Image root_port_image() {
    Image image{};
    put_header(image, root_port_header);
    put_express(image, 0x50, 0x00, rx7900::pcie_type_root_port, gen4, 8, gen4, 8, gen1_to_4_vector, true);
    return image;
}

ByteView legacy(const Image& image) { return ByteView(image.data(), rx7900::pci_config_bytes); }
ByteView extended(const Image& image) { return ByteView(image.data(), image.size()); }

rx7900::UpstreamChain machine_chain() {
    rx7900::UpstreamChain chain{};
    const Image images[] = {downstream_image(), upstream_image(), root_port_image()};
    const rx7900::PciLocation locations[] = {{0, 2, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 1}};

    for (size_t index = 0; index < 3; ++index) {
        chain.bridges[index].location = locations[index];
        std::copy(images[index].begin(), images[index].begin() + rx7900::pci_config_bytes,
                  chain.bridges[index].bytes);
    }

    chain.count = 3;
    return chain;
}

void set_bridge(rx7900::UpstreamChain& chain, size_t index, const Image& image) {
    std::copy(image.begin(), image.begin() + rx7900::pci_config_bytes, chain.bridges[index].bytes);
}

void check_capability_walk() {
    const Image gpu = gpu_image();
    rx7900::CapabilityList list{};
    expect(rx7900::walk_capabilities(legacy(gpu), list) == CapStatus::success && list.count == 4,
           "GPU capability list has vendor, PM, PCIe and MSI entries");
    expect(list.entries[0].offset == 0x48 && list.entries[1].id == 1 && list.entries[2].offset == 0x64 &&
               list.entries[3].id == rx7900::pci_cap_id_msi,
           "capability entries keep their live pointers");

    uint8_t offset = 0;
    expect(rx7900::find_capability(legacy(gpu), rx7900::pci_cap_id_msix, offset) == CapStatus::absent,
           "GPU function lists no MSI-X");
    expect(rx7900::walk_capabilities(ByteView(gpu.data(), 255), list) == CapStatus::truncated_input,
           "capability walk needs the full legacy image");

    Image looping = gpu;
    put_cap(looping, 0xa0, rx7900::pci_cap_id_msi, 0x48);
    expect(rx7900::walk_capabilities(legacy(looping), list) == CapStatus::malformed && list.count == 0,
           "a cyclic capability list is malformed and publishes nothing");

    Image unaligned = gpu;
    put8(unaligned, 0x34, 0x4b);
    expect(rx7900::walk_capabilities(legacy(unaligned), list) == CapStatus::success &&
               list.entries[0].offset == 0x48,
           "low pointer bits are ignored like Linux");

    Image no_list = gpu;
    put16(no_list, rx7900::pci_status_offset, 0);
    expect(rx7900::walk_capabilities(legacy(no_list), list) == CapStatus::success && list.count == 0,
           "Status.CapList clear means no capabilities");

    Image low = gpu;
    put8(low, 0x34, 0x3c);
    expect(rx7900::walk_capabilities(legacy(low), list) == CapStatus::success && list.count == 0,
           "pointers below 0x40 end the list");

    Image end_id = gpu;
    put_cap(end_id, 0x50, 0xff, 0x64);
    expect(rx7900::walk_capabilities(legacy(end_id), list) == CapStatus::success && list.count == 1,
           "ID 0xFF ends the list");
}

void check_extended() {
    const Image gpu = gpu_image();
    rx7900::ExtendedConfigProbe probe{};
    probe.identity = 0x744c1002;

    for (auto& head : probe.block_heads) {
        head = probe.identity;
    }

    expect(!rx7900::extended_config_reachable(probe),
           "CF8/CFC aliasing of every 0x100 block is unreachable extended space");
    probe.block_heads[0] = UINT32_MAX;
    expect(!rx7900::extended_config_reachable(probe), "all-ones at 0x100 is unreachable");
    probe.block_heads[0] = 0x1501000b;
    expect(rx7900::extended_config_reachable(probe), "a real extended header is reachable");

    rx7900::ExtCapabilityList list{};
    expect(rx7900::walk_ext_capabilities(legacy(gpu), list) == CapStatus::extended_unavailable,
           "a 256-byte image has no extended capabilities");
    expect(rx7900::walk_ext_capabilities(extended(gpu), list) == CapStatus::success && list.count == 10 &&
               list.entries[2].offset == 0x200 && list.entries[2].id == rx7900::pci_ext_cap_id_rebar,
           "extended list follows the EPF0 layout");

    Image looping = gpu;
    put_ext(looping, 0x440, 0x27, 1, 0x100);
    uint16_t offset = 0;
    expect(rx7900::walk_ext_capabilities(extended(looping), list) == CapStatus::malformed,
           "cyclic extended list is malformed");
    expect(rx7900::find_ext_capability(extended(looping), 0x99, offset) == CapStatus::malformed,
           "cyclic extended search is bounded");

    Image empty = gpu;
    put32(empty, 0x100, 0);
    expect(rx7900::find_ext_capability(extended(empty), rx7900::pci_ext_cap_id_rebar, offset) ==
               CapStatus::absent,
           "zero header at 0x100 means no extended capabilities");
}

void check_express_decode() {
    const Image gpu = gpu_image();
    rx7900::PcieCapability express{};
    expect(rx7900::decode_pcie(legacy(gpu), express) == CapStatus::success && express.present &&
               express.port_type == rx7900::pcie_type_legacy_endpoint && express.link_implemented &&
               !express.slot_implemented && !express.root_implemented && express.link2_implemented,
           "GPU is a v2 PCIe legacy endpoint with link registers");
    expect(rx7900::pcie_speed_cap(express) == rx7900::PcieSpeed::gt16_0 &&
               rx7900::pcie_width_cap(express) == 16 &&
               rx7900::pcie_link_speed(express) == rx7900::PcieSpeed::gt16_0 &&
               rx7900::pcie_link_width(express) == 16,
           "GPU link capability and status decode to 16 GT/s x16");
    expect(rx7900::pcie_bandwidth_mbps(rx7900::PcieSpeed::gt16_0, 8) == 126024,
           "Linux reports 126.024 Gb/s for 16 GT/s x8");

    Image v1 = gpu;
    put16(v1, 0x66, 0x0011);
    expect(rx7900::decode_pcie(legacy(v1), express) == CapStatus::success && express.link_capabilities2 == 0 &&
               rx7900::pcie_speed_cap(express) == rx7900::PcieSpeed::unknown,
           "v1 capability has no LNKCAP2 and LNKCAP SLS 16 GT/s is unknown to Linux");
    put32(v1, 0x70, 0x00000102);
    rx7900::decode_pcie(legacy(v1), express);
    expect(rx7900::pcie_speed_cap(express) == rx7900::PcieSpeed::gt5_0,
           "v1 LNKCAP SLS 5 GT/s is decoded");

    Image late = gpu;
    put_cap(late, 0xa0, rx7900::pci_cap_id_msi, 0xe0);
    put_cap(late, 0xe0, rx7900::pci_cap_id_express, 0x00);
    put_cap(late, 0x64, rx7900::pci_cap_id_vendor, 0xa0);
    put16(late, 0xe2, 0x0012);
    expect(rx7900::decode_pcie(extended(late), express) == CapStatus::malformed,
           "PCIe registers crossing 0xFF are malformed even with an extended image");

    Image none = gpu;
    put8(none, 0x64, rx7900::pci_cap_id_vendor);
    expect(rx7900::decode_pcie(legacy(none), express) == CapStatus::absent && !express.present,
           "absent PCIe capability leaves zeroed registers");

    rx7900::PowerManagementCapability power{};
    expect(rx7900::decode_power_management(legacy(gpu), power) == CapStatus::success &&
               power.offset == 0x50 && power.power_state == 0 && power.control_status == 0x0008,
           "PM capability reports D0");
}

void check_msi_decode() {
    rx7900::MsiCapability msi{};
    const Image disabled = gpu_image(false);
    expect(rx7900::decode_msi(legacy(disabled), msi) == CapStatus::success && msi.present &&
               !msi.enabled && msi.address64 && !msi.per_vector_mask && msi.data_offset == 0xac &&
               msi.vectors_capable == 1,
           "GPU MSI is 64-bit, single vector, unmasked and disabled under line interrupts");
    const Image enabled = gpu_image(true);
    expect(rx7900::decode_msi(legacy(enabled), msi) == CapStatus::success && msi.enabled &&
               msi.address_low == 0xfee00000u && msi.data == 0x22,
           "enabled MSI message is decoded");

    Image reserved = disabled;
    put16(reserved, 0xa2, 0x008c);
    expect(rx7900::decode_msi(legacy(reserved), msi) == CapStatus::malformed,
           "reserved Multiple Message Capable encoding is malformed");

    Image masked = disabled;
    put16(masked, 0xa2, 0x0180);
    put32(masked, 0xb0, 0x1);
    expect(rx7900::decode_msi(legacy(masked), msi) == CapStatus::success && msi.per_vector_mask &&
               msi.mask_offset == 0xb0 && msi.mask == 1,
           "per-vector mask register follows the 64-bit data word");

    Image msix = disabled;
    put_cap(msix, 0xa0, rx7900::pci_cap_id_msi, 0xc0);
    put_cap(msix, 0xc0, rx7900::pci_cap_id_msix, 0x00);
    put16(msix, 0xc2, 0x0003);
    put32(msix, 0xc4, 0x00042005);
    put32(msix, 0xc8, 0x00043005);
    rx7900::MsixCapability decoded{};
    expect(rx7900::decode_msix(legacy(msix), decoded) == CapStatus::success && decoded.table_entries == 4 &&
               decoded.table_bar == 5 && decoded.table_offset == 0x42000 && !decoded.enabled,
           "MSI-X table location is decoded");
    put32(msix, 0xc4, 0x00042006);
    expect(rx7900::decode_msix(legacy(msix), decoded) == CapStatus::malformed,
           "MSI-X BIR above 5 is malformed");
}

void check_rebar() {
    const Image gpu = gpu_image();
    rx7900::ResizableBarCapability rebar{};
    expect(rx7900::decode_resizable_bar(legacy(gpu), rebar) == CapStatus::extended_unavailable && !rebar.present,
           "ReBAR cannot be read without extended configuration space");
    expect(rx7900::decode_resizable_bar(extended(gpu), rebar) == CapStatus::success && rebar.count == 2 &&
               rebar.offset == 0x200,
           "GPU exposes two resizable BARs at 0x200");

    rx7900::ResizableBar bar{};
    expect(rx7900::find_resizable_bar(rebar, 0, bar) == CapStatus::success && bar.supported_sizes == 0xff00 &&
               bar.current_size == 8 && bar.control_offset == 0x208 &&
               rx7900::rebar_size_to_bytes(bar.current_size) == (256ull << 20),
           "BAR0 matches sysfs resource0_resize and the firmware 256 MiB size");
    expect(rx7900::find_resizable_bar(rebar, 2, bar) == CapStatus::success && bar.supported_sizes == 0x1fe &&
               rx7900::rebar_size_to_bytes(bar.current_size) == (2ull << 20),
           "BAR2 matches sysfs resource2_resize and the 2 MiB doorbell BAR");
    expect(rx7900::find_resizable_bar(rebar, 5, bar) == CapStatus::absent, "BAR5 is not resizable");

    Image zero = gpu;
    put32(zero, 0x208, 0x00000800);
    expect(rx7900::decode_resizable_bar(extended(zero), rebar) == CapStatus::malformed, "NBAR 0 is malformed");
    Image duplicate = gpu;
    put32(duplicate, 0x210, 0x00000100);
    expect(rx7900::decode_resizable_bar(extended(duplicate), rebar) == CapStatus::malformed,
           "duplicate ReBAR BAR index is malformed");

    Image pulse = gpu;
    put16(pulse, rx7900::pci_device_offset, 0x731f);
    put32(pulse, 0x204, 0x700u << 4);
    rx7900::decode_resizable_bar(extended(pulse), rebar);
    expect(rebar.bars[0].supported_sizes == 0x3f00, "Linux Sapphire RX 5600 XT quirk is applied");

    expect(rx7900::rebar_bytes_to_size(24ull << 30) == 15 && rx7900::rebar_bytes_to_size(25753026560ull) == 15 &&
               rx7900::rebar_bytes_to_size(1) == 0 && rx7900::rebar_bytes_to_size(0) == 0xff &&
               rx7900::rebar_size_to_bytes(15) == (32ull << 30) && rx7900::rebar_size_to_bytes(44) == 0,
           "ReBAR size conversions follow pci_rebar_bytes_to_size");
}

void check_fb_bar_policy() {
    rx7900::FbBarInputs linux_host{24ull << 30, 256ull << 20, 0xff00, true, false, true, false};
    rx7900::FbBarPlan plan{};
    expect(rx7900::plan_fb_bar_resize(linux_host, plan) == CapStatus::success &&
               plan.action == rx7900::FbBarAction::resize && plan.target_size == 15,
           "Linux resizes this card's 256 MiB BAR0 to 32 GiB");

    rx7900::FbBarInputs inputs = linux_host;
    inputs.physical_address_64bit = false;
    rx7900::plan_fb_bar_resize(inputs, plan);
    expect(plan.action == rx7900::FbBarAction::keep_32bit_physical, "32-bit physical addressing keeps BAR0");

    inputs = linux_host;
    inputs.firmware_config_preserved = true;
    rx7900::plan_fb_bar_resize(inputs, plan);
    expect(plan.action == rx7900::FbBarAction::keep_config_preserved,
           "preserved firmware assignment keeps BAR0 (ReactOS PnP model)");

    inputs = linux_host;
    inputs.bar0_supported_sizes = 0;
    rx7900::plan_fb_bar_resize(inputs, plan);
    expect(plan.action == rx7900::FbBarAction::keep_not_resizable, "unreadable ReBAR keeps BAR0");

    inputs = linux_host;
    inputs.root_window_above_4g = false;
    rx7900::plan_fb_bar_resize(inputs, plan);
    expect(plan.action == rx7900::FbBarAction::keep_no_root_window, "no 64-bit root window keeps BAR0");

    inputs = linux_host;
    inputs.bar0_bytes = 32ull << 30;
    rx7900::plan_fb_bar_resize(inputs, plan);
    expect(plan.action == rx7900::FbBarAction::keep_large_enough, "firmware large BAR is kept");

    inputs = linux_host;
    inputs.sriov_vf = true;
    rx7900::plan_fb_bar_resize(inputs, plan);
    expect(plan.action == rx7900::FbBarAction::keep_sriov_vf, "SR-IOV VF keeps BAR0");

    inputs = linux_host;
    inputs.real_vram_bytes = 128ull << 20;
    inputs.bar0_bytes = 64ull << 20;
    rx7900::plan_fb_bar_resize(inputs, plan);
    expect(plan.action == rx7900::FbBarAction::keep_size_unsupported && plan.target_size == 7,
           "an unoffered target size is -EINVAL");

    inputs = linux_host;
    inputs.real_vram_bytes = 0;
    expect(rx7900::plan_fb_bar_resize(inputs, plan) == CapStatus::malformed, "zero VRAM has no plan");
}

void check_pcie_info() {
    const Image gpu = gpu_image();
    const rx7900::UpstreamChain chain = machine_chain();
    rx7900::PcieInfoReport report{};
    expect(rx7900::derive_pcie_info(legacy(gpu), chain, rx7900::linux_default_pcie_policy, report) ==
               CapStatus::success,
           "PCIe info derives for the real topology");
    expect(report.partner_index == 2 && report.platform_speed == rx7900::PcieSpeed::gt16_0 &&
               report.platform_width == 8 && report.device_speed_cap == rx7900::PcieSpeed::gt16_0,
           "partner skips both AMD switch ports and uses the root port's Gen4 x8 capability");
    expect(report.masks.gen_mask == 0x000f000f && report.masks.mlw_mask == 0x000f0000,
           "CAIL masks are ASIC+platform Gen1-4 and platform x1..x8");

    rx7900::PcieSmuLink link{};
    expect(rx7900::pcie_smu_link(legacy(gpu), chain, rx7900::linux_default_pcie_policy, link) ==
               CapStatus::success && link.gen == 3 && link.lane == 4,
           "SMU pair is gen 3 (Gen4) lane 4 (x8), matching Linux pp_dpm_pcie");

    rx7900::PcieCapability own{};
    rx7900::decode_pcie(legacy(gpu), own);
    expect(rx7900::pcie_link_width(own) == 16,
           "the GPU's own link status reports x16, which the old conversion used");

    rx7900::PcieInfoPolicy intel = rx7900::linux_default_pcie_policy;
    intel.dynamic_switching = false;
    expect(rx7900::derive_pcie_info(legacy(gpu), chain, intel, report) == CapStatus::success &&
               report.partner_index == 2 && report.platform_width == 8 &&
               report.platform_speed == rx7900::PcieSpeed::gt16_0 &&
               rx7900::encode_smu_pcie(report.masks).lane == 4,
           "pcie_bandwidth_available keeps the last x8 minimum (root port)");

    rx7900::UpstreamChain root{};
    expect(rx7900::derive_pcie_info(legacy(gpu), root, rx7900::linux_default_pcie_policy, report) ==
               CapStatus::success && report.root_bus &&
               report.masks.gen_mask == rx7900::amdgpu_default_pcie_gen_mask &&
               report.masks.mlw_mask == rx7900::amdgpu_default_pcie_mlw_mask,
           "root-bus devices get the amdgpu default masks");
    link = rx7900::encode_smu_pcie(report.masks);
    expect(link.gen == 1 && link.lane == 6, "default masks encode Gen2 x16");

    rx7900::PcieInfoPolicy passthrough = rx7900::linux_default_pcie_policy;
    passthrough.passthrough = true;
    rx7900::derive_pcie_info(legacy(gpu), root, passthrough, report);
    expect(!report.root_bus && report.partner_index == -1 &&
               report.masks.gen_mask == (0x0000000fu | rx7900::cail_pcie_link_speed_gen1 |
                                         rx7900::cail_pcie_link_speed_gen2) &&
               report.masks.mlw_mask == rx7900::amdgpu_default_pcie_mlw_mask,
           "pass-through root bus derives with unknown platform caps");

    rx7900::PcieInfoPolicy overrides{0x00040004, 0x00020000, true, false};
    rx7900::derive_pcie_info(legacy(gpu), chain, overrides, report);
    expect(report.masks.gen_mask == 0x00040004 && report.masks.mlw_mask == 0x00020000 &&
               report.partner_index == -1,
           "module-parameter overrides return before the partner walk");
    link = rx7900::encode_smu_pcie(report.masks);
    expect(link.gen == 2 && link.lane == 2, "override masks encode Gen3 x2");

    rx7900::UpstreamChain only_amd = chain;
    only_amd.count = 2;
    rx7900::derive_pcie_info(legacy(gpu), only_amd, rx7900::linux_default_pcie_policy, report);
    expect(report.partner_index == -1 && report.platform_width == rx7900::pcie_width_unknown &&
               report.masks.mlw_mask == rx7900::amdgpu_default_pcie_mlw_mask &&
               (report.masks.gen_mask & 0xffff0000u) == 0x00030000u,
           "a chain of only AMD switch ports leaves the platform unknown");

    rx7900::UpstreamChain conventional = chain;
    Image plain = root_port_image();
    put8(plain, 0x50, rx7900::pci_cap_id_vendor);
    set_bridge(conventional, 2, plain);
    rx7900::derive_pcie_info(legacy(gpu), conventional, rx7900::linux_default_pcie_policy, report);
    expect(report.partner_index == 2 && report.platform_speed == rx7900::PcieSpeed::unknown &&
               report.platform_width == rx7900::pcie_width_unknown,
           "a partner without PCIe reads zero link capabilities");

    rx7900::PcieInfoPolicy intel_conventional = intel;
    rx7900::derive_pcie_info(legacy(gpu), conventional, intel_conventional, report);
    expect(report.partner_index == 2 && report.platform_width == 0 && report.masks.mlw_mask == 0 &&
               rx7900::encode_smu_pcie(report.masks).lane == 0,
           "bandwidth walk through a non-PCIe bridge yields width 0 and no lane mask, like Linux");

    rx7900::UpstreamChain odd = chain;
    Image odd_root = root_port_image();
    put32(odd_root, 0x5c, 0x00400c34);
    set_bridge(odd, 2, odd_root);
    rx7900::derive_pcie_info(legacy(gpu), odd, rx7900::linux_default_pcie_policy, report);
    expect(report.platform_width == 3 && report.masks.mlw_mask == 0 &&
               rx7900::encode_smu_pcie(report.masks).lane == 0,
           "a reserved partner width leaves the lane mask empty (smu13 then refuses lane 0)");

    rx7900::UpstreamChain gen5 = chain;
    Image gen5_root = root_port_image();
    put32(gen5_root, 0x7c, 0x3e);
    set_bridge(gen5, 2, gen5_root);
    Image gen6_gpu = gpu;
    put32(gen6_gpu, 0x90, 0x7e);
    rx7900::derive_pcie_info(legacy(gen6_gpu), gen5, rx7900::linux_default_pcie_policy, report);
    expect(report.masks.gen_mask == (0x00000001u | 0x001f0000u) &&
               rx7900::encode_smu_pcie(report.masks).gen == 3,
           "64 GT/s ASIC falls into Linux's Gen1-only branch; Gen5 platform still encodes gen 3");

    rx7900::UpstreamChain bad = chain;
    set_bridge(bad, 1, gpu);
    expect(rx7900::derive_pcie_info(legacy(gpu), bad, rx7900::linux_default_pcie_policy, report) ==
               CapStatus::malformed,
           "a type-0 header in the upstream chain is malformed");
    expect(rx7900::derive_pcie_info(legacy(downstream_image()), chain, rx7900::linux_default_pcie_policy,
                                    report) == CapStatus::malformed,
           "the device image must be a type-0 header");
    expect(rx7900::derive_pcie_info(ByteView(gpu.data(), 64), chain, rx7900::linux_default_pcie_policy,
                                    report) == CapStatus::truncated_input,
           "a 64-byte header is not enough to walk capabilities");

    rx7900::UpstreamChain looping = chain;
    Image loop_root = root_port_image();
    put_cap(loop_root, 0x50, rx7900::pci_cap_id_vendor, 0x50);
    set_bridge(looping, 2, loop_root);
    expect(rx7900::pcie_smu_link(legacy(gpu), looping, rx7900::linux_default_pcie_policy, link) ==
               CapStatus::malformed && link.gen == 0 && link.lane == 0,
           "a malformed partner list fails closed with zero SMU fields");

    rx7900::UpstreamChain overfull = chain;
    overfull.count = rx7900::pci_upstream_capacity + 1;
    expect(rx7900::derive_pcie_info(legacy(gpu), overfull, rx7900::linux_default_pcie_policy, report) ==
               CapStatus::capacity_exceeded,
           "chain count beyond capacity is rejected");
}

// Simulated segment-0 configuration space for upstream path discovery.
struct FakeBus final {
    std::map<std::tuple<uint8_t, uint8_t, uint8_t>, Image> functions;
    unsigned reads = 0;
};

bool fake_read(void* context, rx7900::PciLocation location, uint16_t offset, uint32_t& value) noexcept {
    auto& bus = *static_cast<FakeBus*>(context);
    ++bus.reads;
    const auto found = bus.functions.find({location.bus, location.slot, location.function});

    if (found == bus.functions.end()) {
        return false;
    }

    return ByteView(found->second.data(), rx7900::pci_config_bytes).read_u32(offset, value);
}

Image function_header(uint16_t vendor, uint16_t device, uint8_t type, uint8_t secondary = 0,
                      uint8_t subordinate = 0) {
    Image image{};
    put16(image, 0, vendor);
    put16(image, 2, device);
    put8(image, rx7900::pci_header_type_offset, type);

    if ((type & 0x7f) == 1) {
        put8(image, 0x19, secondary);
        put8(image, 0x1a, subordinate);
    }

    return image;
}

// Bus 0 as listed in sysfs (decimal secondary/subordinate converted).
FakeBus machine_bus() {
    FakeBus bus{};
    bus.functions[{0, 0, 0}] = function_header(0x1022, 0x14e8, 0x80);
    bus.functions[{0, 0, 2}] = function_header(0x1022, 0x14e9, 0x80);
    bus.functions[{0, 1, 0}] = function_header(0x1022, 0x14ea, 0x80);
    bus.functions[{0, 1, 1}] = function_header(0x1022, 0x14ed, 0x81, 1, 3);
    bus.functions[{0, 2, 0}] = function_header(0x1022, 0x14ea, 0x80);
    bus.functions[{0, 2, 1}] = function_header(0x1022, 0x14ee, 0x81, 4, 16);
    bus.functions[{0, 8, 0}] = function_header(0x1022, 0x14ea, 0x80);
    bus.functions[{0, 8, 1}] = function_header(0x1022, 0x14eb, 0x81, 17, 17);
    bus.functions[{0, 8, 2}] = function_header(0x1022, 0x14eb, 0x81, 18, 18);
    bus.functions[{0, 8, 3}] = function_header(0x1022, 0x14eb, 0x81, 19, 19);
    bus.functions[{0, 0x18, 0}] = function_header(0x1022, 0x14f0, 0x80);
    bus.functions[{1, 0, 0}] = function_header(0x1002, 0x1478, 0x01, 2, 3);
    bus.functions[{2, 0, 0}] = function_header(0x1002, 0x1479, 0x01, 3, 3);
    bus.functions[{3, 0, 0}] = function_header(0x1002, 0x744c, 0x80);
    bus.functions[{3, 0, 1}] = function_header(0x1002, 0xab30, 0x00);
    // A function behind a non-multifunction slot must not be enumerated.
    bus.functions[{0, 0x14, 3}] = function_header(0x1022, 0x790e, 0x81, 3, 3);
    return bus;
}

void check_upstream_path() {
    FakeBus bus = machine_bus();
    rx7900::UpstreamPath path{};
    const rx7900::ConfigDwordReader reader{&bus, fake_read};
    expect(rx7900::resolve_upstream_path(reader, {0, 3, 0, 0}, path) == CapStatus::success &&
               path.count == 3 && path.bridges[0].bus == 2 && path.bridges[1].bus == 1 &&
               path.bridges[2].bus == 0 && path.bridges[2].slot == 1 && path.bridges[2].function == 1,
           "upstream path is 02:00.0, 01:00.0, 00:01.1 (nearest first)");
    expect(rx7900::resolve_upstream_path(reader, {0, 0, 1, 1}, path) == CapStatus::success && path.count == 0,
           "a bus-0 function sits on the root bus");
    expect(rx7900::resolve_upstream_path(reader, {0, 0x40, 0, 0}, path) == CapStatus::absent,
           "a bus no bus-0 bridge forwards is reported unresolved");

    FakeBus ambiguous = machine_bus();
    ambiguous.functions[{0, 3, 0}] = function_header(0x1022, 0x14ea, 0x81, 2, 3);
    const rx7900::ConfigDwordReader ambiguous_reader{&ambiguous, fake_read};
    expect(rx7900::resolve_upstream_path(ambiguous_reader, {0, 3, 0, 0}, path) == CapStatus::malformed,
           "two bridges forwarding the same bus are malformed");

    FakeBus stuck = machine_bus();
    stuck.functions[{1, 0, 0}] = function_header(0x1002, 0x1478, 0x01, 1, 3);
    const rx7900::ConfigDwordReader stuck_reader{&stuck, fake_read};
    expect(rx7900::resolve_upstream_path(stuck_reader, {0, 3, 0, 0}, path) == CapStatus::malformed,
           "a bridge whose secondary does not advance is malformed");

    FakeBus orphan = machine_bus();
    orphan.functions.erase({2, 0, 0});
    const rx7900::ConfigDwordReader orphan_reader{&orphan, fake_read};
    expect(rx7900::resolve_upstream_path(orphan_reader, {0, 3, 0, 0}, path) == CapStatus::malformed,
           "a claimed bus without a forwarding child bridge is malformed");

    FakeBus deep{};
    deep.functions[{0, 0, 0}] = function_header(0x1022, 1, 0x01, 1, 20);

    for (uint8_t level = 1; level < 20; ++level) {
        deep.functions[{level, 0, 0}] = function_header(0x1002, 2, 0x01, static_cast<uint8_t>(level + 1), 20);
    }

    const rx7900::ConfigDwordReader deep_reader{&deep, fake_read};
    expect(rx7900::resolve_upstream_path(deep_reader, {0, 20, 0, 0}, path) == CapStatus::capacity_exceeded,
           "more than eight levels exceed the fixed chain capacity");
    expect(rx7900::resolve_upstream_path({nullptr, nullptr}, {0, 3, 0, 0}, path) == CapStatus::malformed,
           "a missing reader is rejected");
}

void check_save_restore() {
    const Image gpu = gpu_image();
    rx7900::PciSavedState state{};
    rx7900::RestorePlan plan{};
    expect(rx7900::save_pci_state(legacy(gpu), state) == CapStatus::success && state.valid && !state.extended &&
               state.header[4] == 0x0000000c && state.express.present,
           "legacy save captures the header and PCIe controls");
    expect(rx7900::plan_pci_restore(state, plan) == CapStatus::success && plan.count == 20,
           "legacy restore has four PCIe controls and sixteen header dwords");
    expect(plan.writes[0].offset == 0x6c && plan.writes[0].width == 2 && plan.writes[0].force &&
               plan.writes[1].offset == 0x74 && plan.writes[2].offset == 0x8c && plan.writes[3].offset == 0x94,
           "PCIe DEVCTL, LNKCTL, DEVCTL2, LNKCTL2 precede the header");
    expect(plan.writes[4].offset == 0x3c && !plan.writes[4].force && plan.writes[9].offset == 0x28 &&
               plan.writes[10].offset == 0x24 && plan.writes[10].retries == 10 &&
               plan.writes[15].offset == 0x10 && plan.writes[15].retries == 10 &&
               plan.writes[16].offset == 0x0c && plan.writes[16].retries == 0 && plan.writes[19].offset == 0,
           "type-0 header restores 15..10, BARs 9..4 with retries, then 3..0");
    expect(plan.writes[18].offset == rx7900::pci_command_offset && plan.writes[18].value == 0x00100407,
           "command/status dword is restored after the BARs");

    rx7900::PciSavedState full{};
    expect(rx7900::save_pci_state(extended(gpu_image(true)), full) == CapStatus::success && full.extended &&
               full.ltr.present && full.rebar.count == 2 && full.msi.enabled,
           "extended save captures LTR, ReBAR and enabled MSI");
    expect(rx7900::plan_pci_restore(full, plan) == CapStatus::success && plan.count == 28,
           "extended restore adds LTR, two ReBAR controls and five MSI writes");
    expect(plan.writes[0].offset == 0x324 && plan.writes[0].value == 0x10031003 &&
               plan.writes[5].offset == 0x208 && plan.writes[5].value == 0x00000840 &&
               plan.writes[6].offset == 0x210,
           "LTR is first and ReBAR controls follow the PCIe controls");
    expect(plan.writes[23].offset == 0xa2 && plan.writes[23].value == 0x0080 && plan.writes[24].offset == 0xa4 &&
               plan.writes[25].offset == 0xa8 && plan.writes[26].offset == 0xac && plan.writes[26].width == 2 &&
               plan.writes[27].offset == 0xa2 && plan.writes[27].value == 0x0081,
           "MSI restore disables, writes the message, then re-enables");

    Image bridge = root_port_image();
    rx7900::PciSavedState bridge_state{};
    rx7900::save_pci_state(legacy(bridge), bridge_state);
    rx7900::plan_pci_restore(bridge_state, plan);
    expect(plan.count == 22 && plan.writes[0].offset == 0x58 && plan.writes[2].offset == 0x68 &&
               plan.writes[3].offset == 0x6c,
           "root port restores DEVCTL, LNKCTL, SLTCTL, RTCTL, DEVCTL2, LNKCTL2");
    expect(plan.writes[6].offset == 0x3c && plan.writes[10].offset == 0x2c && plan.writes[10].force &&
               plan.writes[12].offset == 0x24 && plan.writes[12].force && plan.writes[13].offset == 0x20 &&
               !plan.writes[13].force && plan.writes[21].offset == 0,
           "type-1 header restores 15..12, forces 11..9, then 8..0");

    Image cardbus = gpu;
    put8(cardbus, rx7900::pci_header_type_offset, 0x02);
    expect(rx7900::save_pci_state(legacy(cardbus), state) == CapStatus::malformed && !state.valid,
           "CardBus headers are not saved");
    rx7900::PciSavedState empty{};
    expect(rx7900::plan_pci_restore(empty, plan) == CapStatus::malformed, "unsaved state has no restore plan");
}

} // namespace

int main() {
    check_capability_walk();
    check_extended();
    check_express_decode();
    check_msi_decode();
    check_rebar();
    check_fb_bar_policy();
    check_pcie_info();
    check_upstream_path();
    check_save_restore();
    std::printf("%u checks; %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

/*
Module: rx7900.tests.vram_reservations
Purpose: Verify the Linux VRAM reservation map, pre-OS scanout sizing and driver-region placement.
Created: 2026-10-22
Architecture: Pure inputs built field by field plus the shared synthetic register
machine for the DCN reads; no VBIOS or VRAM content is embedded.
*/
#include <memory>

#include "asic_test_device.hpp"
#include "vram_reservations.hpp"

namespace {

using namespace rx7900::hardware;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

constexpr uint64_t mib = 1ULL << 20;
// The user's board: RCC_CONFIG_MEMSIZE 24560 MiB (Linux log), BAR0 256 MiB on ReactOS.
constexpr uint64_t board_vram = 24560 * mib;
constexpr uint64_t window = 256 * mib;

VramReservationInputs navi31(uint32_t fb_bytes) {
    VramReservationInputs inputs{};
    inputs.vram_bytes = board_vram;
    inputs.visible_vram_bytes = window;
    inputs.capability = atom_capability_two_stage_training | atom_capability_firmware_posted;
    inputs.reserved_fb_bytes = 28 * 1024 * 1024;
    inputs.display_hardware = true;
    inputs.vbios_fb_bytes = fb_bytes;
    return inputs;
}

const VramReservation* find(const VramReservationMap& map, VramReservationKind kind) {
    for (uint32_t index = 0; index < map.count; ++index) {
        if (map.entries[index].kind == kind) {
            return &map.entries[index];
        }
    }

    return nullptr;
}

bool is(const VramReservationMap& map, VramReservationKind kind, uint64_t offset, uint64_t bytes,
        bool lifetime) {
    const VramReservation* entry = find(map, kind);
    return entry != nullptr && entry->offset == offset && entry->bytes == bytes && entry->lifetime == lifetime;
}

void scanout_size() {
    expect(vbios_fb_bytes(1, 0, 0) == 9 * mib, "D1VGA_MODE_ENABLE: AMDGPU_VBIOS_VGA_ALLOCATION");
    expect(vbios_fb_bytes(0, (1080u << 16) | 1920u, 1920u) == 8294400u &&
           vbios_fb_bytes(0xfffffffeu, 0xc000ffffu | (2160u << 16), 0xffffc000u | 3840u) == 33177600u,
           "viewport height times pitch times 4; only the 14-bit fields count");

    std::unique_ptr<rx7900_test::SimulatedNavi31> device = std::make_unique<rx7900_test::SimulatedNavi31>();
    const auto binary = rx7900_test::navi31_discovery();
    DcnFbRegisters dcn;
    expect(validate_dcn_fb_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, dcn) ==
               DiscoveryStatus::validated && dcn.segment(1) == 0xc0 && dcn.segment(2) == 0x34c0,
           "DMU 3.2.0 segments 1 and 2 validated");
    device->reg(0xc0 + 0x0c) = 0;
    device->reg(0x34c0 + 0x5ea) = (1080u << 16) | 1920u;
    device->reg(0x34c0 + 0x607) = 2048;
    uint32_t bytes = 0;
    expect(read_vbios_fb_bytes(device->io(), dcn, bytes) && bytes == 1080u * 2048u * 4u,
           "HUBP0 viewport height and HUBPREQ0 pitch read through BAR5");
    device->reg(0xc0 + 0x0c) = 1;
    expect(read_vbios_fb_bytes(device->io(), dcn, bytes) && bytes == 9 * mib, "VGA mode read");
    device->reg(0xc0 + 0x0c) = 0xffffffffu;
    bytes = 5;
    expect(!read_vbios_fb_bytes(device->io(), dcn, bytes) && bytes == 5, "all-ones register refused");

    const auto no_dmu = rx7900_test::discovery_with({{discovery_mmhub_hardware_id, 3, 0, {0x1a000}}});
    expect(validate_dcn_fb_discovery({no_dmu.data(), no_dmu.size()}, rx7900_test::aperture_bytes, dcn) !=
               DiscoveryStatus::validated && !dcn.valid() && !read_vbios_fb_bytes(device->io(), dcn, bytes),
           "missing DMU refused");
    expect(validate_dcn_fb_discovery({binary.data(), binary.size()}, 0x3000, dcn) ==
               DiscoveryStatus::invalid_argument, "DCN registers outside BAR5 refused");
}

void navi31_map() {
    VramReservationMap map{};
    expect(build_vram_reservation_map(navi31(8294400u), map) == VramReservationStatus::built && map.count == 4,
           "1080p pre-OS scanout, two-stage training: four entries");
    expect(is(map, VramReservationKind::training_c2p, board_vram - 29 * mib, 4096, true) &&
           map.c2p_offset == board_vram - 29 * mib && map.p2c_offset == board_vram - 0x8000 &&
           map.training_reserved,
           "C2P at ALIGN(vram - reserve - 1 MiB, 1 MiB), P2C 32 KiB below the top");
    expect(is(map, VramReservationKind::firmware_top, board_vram - 28 * mib, 28 * mib, true) &&
           map.top_reserve_bytes == 28 * mib, "fw_reserved_memory at the top of VRAM");
    expect(is(map, VramReservationKind::stolen_vga, 0, 8294400u, true) && map.stolen_extended_bytes == 0 &&
           find(map, VramReservationKind::stolen_extended) == nullptr,
           "stolen VGA covers the pre-OS buffer below 9 MiB, no extended part");
    expect(is(map, VramReservationKind::training_encroached, 0, 32 * mib, false) &&
           map.atom_scratch_bytes == 20 * 1024, "BIST range listed as transient; default ATOM scratch");

    expect(build_vram_reservation_map(navi31(33177600u), map) == VramReservationStatus::built &&
           is(map, VramReservationKind::stolen_vga, 0, 9 * mib, true) &&
           is(map, VramReservationKind::stolen_extended, 9 * mib, 33177600u - 9 * mib, true),
           "4K pre-OS buffer: 9 MiB stolen VGA plus the extended remainder");

    VramReservationInputs plain = navi31(8294400u);
    plain.capability = atom_capability_firmware_posted;
    plain.display_hardware = false;
    plain.reserved_fb_bytes = atom_default_reserved_fb_bytes;
    expect(build_vram_reservation_map(plain, map) == VramReservationStatus::built && map.count == 1 &&
           is(map, VramReservationKind::firmware_top, board_vram - 64 * 1024, 64 * 1024, true),
           "no training, no display: only the 64 KiB discovery TMR");

    VramReservationInputs tiny = navi31(0);
    tiny.vram_bytes = 64 * mib;
    tiny.visible_vram_bytes = 64 * mib;
    tiny.vbios_fb_bytes = static_cast<uint32_t>(57 * mib);
    tiny.capability = 0;
    tiny.reserved_fb_bytes = 1 * mib;
    expect(build_vram_reservation_map(tiny, map) == VramReservationStatus::built && map.stolen_vga_bytes == 0 &&
           find(map, VramReservationKind::stolen_vga) == nullptr,
           "a pre-OS buffer leaving under 8 MiB is dropped");
}

void firmware_usage_and_refusals() {
    VramReservationInputs inputs = navi31(8294400u);
    inputs.usage.layout = AtomVramUsageLayout::v2_1;
    inputs.usage.firmware_start_kb = 0x80000000u | (80u * 1024u);
    inputs.usage.firmware_kb = 1024;
    VramReservationMap map{};
    expect(build_vram_reservation_map(inputs, map) == VramReservationStatus::built &&
           is(map, VramReservationKind::firmware_usage, 80 * mib, 1 * mib, true) && !map.usage_beyond_visible,
           "v2.1 SR-IOV message reservation becomes fw_vram_usage");

    inputs.usage.firmware_start_kb = 0x80000000u | 5u;
    inputs.usage.firmware_kb = 2;
    expect(build_vram_reservation_map(inputs, map) == VramReservationStatus::overlapping,
           "a firmware range overlapping stolen VGA memory fails as create_at would");
    inputs.vbios_fb_bytes = 0;
    expect(build_vram_reservation_map(inputs, map) == VramReservationStatus::built &&
           is(map, VramReservationKind::firmware_usage, 4096, 4096, true),
           "create_at page rounding: offset down, size up");
    inputs.usage.firmware_kb = 4;
    expect(build_vram_reservation_map(inputs, map) == VramReservationStatus::built &&
           is(map, VramReservationKind::firmware_usage, 4096, 8192, true),
           "the unrounded request end stays covered");

    inputs.usage.firmware_start_kb = 0x80000000u | (24600u * 1024u);
    expect(build_vram_reservation_map(inputs, map) == VramReservationStatus::out_of_range && map.count == 0,
           "a reservation outside VRAM fails; output cleared");
    inputs.usage.firmware_start_kb = 0x80000000u | (24550u * 1024u);
    inputs.usage.firmware_kb = 1024;
    expect(build_vram_reservation_map(inputs, map) == VramReservationStatus::overlapping,
           "a firmware range inside the top reservation overlaps it");

    VramReservationInputs large = navi31(0);
    large.reserved_fb_bytes = 0xffffffffu;
    large.vram_bytes = 2 * mib;
    large.visible_vram_bytes = 2 * mib;
    expect(build_vram_reservation_map(large, map) == VramReservationStatus::out_of_range,
           "reserve_size above VRAM refused");
    VramReservationInputs huge_fb = navi31(0);
    huge_fb.vram_bytes = 16 * mib;
    huge_fb.visible_vram_bytes = 16 * mib;
    huge_fb.vbios_fb_bytes = static_cast<uint32_t>(32 * mib);
    expect(build_vram_reservation_map(huge_fb, map) == VramReservationStatus::out_of_range,
           "pre-OS buffer above VRAM refused");
    VramReservationInputs zero = navi31(0);
    zero.visible_vram_bytes = board_vram + 1;
    expect(build_vram_reservation_map(zero, map) == VramReservationStatus::invalid_argument,
           "visible VRAM above VRAM refused");
}

void placement() {
    // asic_bringup: arena (32 MiB) plus training backup (32 MiB) contiguous from 64 MiB.
    const VramPlacementRequest request{64 * mib, 64 * mib, 2 * mib, window};
    VramReservationMap map{};
    uint64_t offset = 0;
    build_vram_reservation_map(navi31(33177600u), map);
    expect(place_vram_region(map, request, offset) == VramPlacementStatus::preferred && offset == 64 * mib &&
           !vram_range_reserved(map, offset, request.bytes),
           "4K pre-OS buffer and training: the fixed 64 MiB placement is clear");

    // 8K pre-OS scanout: 7680 x 4320 x 4 bytes (126.5625 MiB) overlaps 64 MiB.
    build_vram_reservation_map(navi31(7680u * 4320u * 4u), map);
    expect(vram_range_reserved(map, 64 * mib, 64 * mib) &&
           place_vram_region(map, request, offset) == VramPlacementStatus::relocated && offset == 128 * mib &&
           !vram_range_reserved(map, offset, request.bytes),
           "an overlapping pre-OS buffer moves the region to the next 2 MiB boundary above it");

    VramReservationInputs crowded = navi31(0);
    crowded.vbios_fb_bytes = static_cast<uint32_t>(200 * mib);
    build_vram_reservation_map(crowded, map);
    offset = 7;
    expect(place_vram_region(map, request, offset) == VramPlacementStatus::no_space && offset == 7,
           "no clear range inside the BAR0 window: refused, offset untouched");

    VramReservationInputs usage = navi31(8294400u);
    usage.usage.layout = AtomVramUsageLayout::v2_1;
    usage.usage.firmware_start_kb = 0x80000000u | (80u * 1024u + 4u);
    usage.usage.firmware_kb = 1024;
    build_vram_reservation_map(usage, map);
    expect(place_vram_region(map, request, offset) == VramPlacementStatus::relocated && offset == 82 * mib,
           "a firmware range inside the arena pushes it past the range end");

    expect(place_vram_region(map, {64 * mib + 4096, 64 * mib, 2 * mib, window}, offset) ==
               VramPlacementStatus::invalid_argument &&
           place_vram_region(map, {64 * mib, 64 * mib, 3 * mib, window}, offset) ==
               VramPlacementStatus::invalid_argument &&
           place_vram_region(map, {64 * mib, 64 * mib, 2 * mib, board_vram + 1}, offset) ==
               VramPlacementStatus::invalid_argument &&
           place_vram_region(map, {64 * mib, 0, 2 * mib, window}, offset) == VramPlacementStatus::invalid_argument &&
           place_vram_region(VramReservationMap{}, request, offset) == VramPlacementStatus::invalid_argument,
           "misaligned, non-power-of-two, oversized and empty requests refused");
    expect(vram_range_reserved(map, 0, 1) && vram_range_reserved(map, UINT64_MAX, 2) &&
           !vram_range_reserved(map, 40 * mib, 4096) && vram_range_reserved(map, 40 * mib, 0),
           "overlap queries: reserved, wrapping and empty ranges count as reserved");
}

} // namespace

int main() {
    scanout_size();
    navi31_map();
    firmware_usage_and_refusals();
    placement();
    return checks.finish("vram_reservations");
}

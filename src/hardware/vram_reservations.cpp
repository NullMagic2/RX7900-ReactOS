/*
Module: rx7900.hardware.vram_reservations
Purpose: Derive Linux's VRAM reservations from ATOM, DCN and VRAM size and place driver regions.
Created: 2026-10-22
Architecture: Allocation-free arithmetic over fixed-capacity entries; three DCN
register reads through RegisterIo; no VRAM is touched.
Copyright 2016-2022 Advanced Micro Devices, Inc. Adapted Linux v6.12 amdgpu_ttm.c,
amdgpu_gmc.c, gmc_v11_0.c, amdgpu_object.c (amdgpu_bo_create_kernel_at page
rounding), amdgpu.h, amdgpu_psp.h and dcn_3_2_0_offset.h/_sh_mask.h.
Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#include "vram_reservations.hpp"

namespace rx7900::hardware {
namespace {

constexpr uint64_t page_bytes = 4096;
// dcn_3_2_0_offset.h: regD1VGA_CONTROL (BASE_IDX 1), regHUBP0_DCSURF_PRI_VIEWPORT_DIMENSION
// and regHUBPREQ0_DCSURF_SURFACE_PITCH (BASE_IDX 2).
constexpr uint32_t d1vga_control = 0x000c;
constexpr uint32_t hubp0_viewport_dimension = 0x05ea;
constexpr uint32_t hubpreq0_surface_pitch = 0x0607;
// dcn_3_2_0_sh_mask.h fields.
constexpr uint32_t d1vga_mode_enable = 0x00000001u;
constexpr uint32_t viewport_height_mask = 0x3fff0000u;
constexpr uint32_t viewport_height_shift = 16;
constexpr uint32_t surface_pitch_mask = 0x00003fffu;

bool overlap(uint64_t a, uint64_t a_bytes, uint64_t b, uint64_t b_bytes) noexcept {
    return a_bytes != 0 && b_bytes != 0 && a < b + b_bytes && b < a + a_bytes;
}

// amdgpu_bo_create_kernel_at: offset &= PAGE_MASK, size = ALIGN(size, PAGE_SIZE);
// a zero size creates nothing. The entry also covers the unrounded end so an
// unaligned firmware request is never partly outside the map.
bool add(VramReservationMap& map, VramReservationKind kind, uint64_t offset, uint64_t bytes,
         bool lifetime) noexcept {
    if (bytes == 0) {
        return true;
    }

    if (map.count >= VramReservationMap::capacity || offset > UINT64_MAX - bytes ||
        offset + bytes > UINT64_MAX - (page_bytes - 1) || bytes > UINT64_MAX - (page_bytes - 1)) {
        return false;
    }

    const uint64_t start = offset & ~(page_bytes - 1);
    const uint64_t linux_end = start + ((bytes + page_bytes - 1) & ~(page_bytes - 1));
    const uint64_t request_end = (offset + bytes + page_bytes - 1) & ~(page_bytes - 1);
    map.entries[map.count++] = {kind, start, (linux_end > request_end ? linux_end : request_end) - start,
                                lifetime};
    return true;
}

} // namespace

uint32_t vbios_fb_bytes(uint32_t control, uint32_t viewport, uint32_t pitch) noexcept {
    if ((control & d1vga_mode_enable) != 0) {
        return static_cast<uint32_t>(vbios_vga_allocation_bytes);
    }

    // Both fields are 14 bits: the product stays below 2^30.
    return ((viewport & viewport_height_mask) >> viewport_height_shift) *
           (pitch & surface_pitch_mask) * 4u;
}

DiscoveryStatus validate_dcn_fb_discovery(ByteView binary, uint32_t aperture,
                                          DcnFbRegisters& output) noexcept {
    output = {};

    if (aperture == 0 || (aperture & 3u) != 0) {
        return DiscoveryStatus::invalid_argument;
    }

    DiscoveredIp dmu;
    const auto status = discover_ip(binary, {discovery_dmu_hardware_id, 0, 3, 2, 0, 3}, dmu);

    if (status != DiscoveryStatus::validated) {
        return status;
    }

    uint32_t offset = 0;

    if (!discovery_register_offset(dmu.segment(1), d1vga_control, aperture, offset) ||
        !discovery_register_offset(dmu.segment(2), hubp0_viewport_dimension, aperture, offset) ||
        !discovery_register_offset(dmu.segment(2), hubpreq0_surface_pitch, aperture, offset)) {
        return DiscoveryStatus::invalid_argument;
    }

    output.dmu_ = dmu;
    output.aperture_bytes_ = aperture;
    return DiscoveryStatus::validated;
}

bool read_vbios_fb_bytes(const RegisterIo& io, const DcnFbRegisters& registers,
                         uint32_t& bytes) noexcept {
    uint32_t control = 0;
    uint32_t viewport = 0;
    uint32_t pitch = 0;

    if (!registers.valid() || io.aperture_bytes != registers.aperture_bytes() ||
        !read_ip_register(io, registers.segment(1), d1vga_control, control) ||
        control == UINT32_MAX) {
        return false;
    }

    // Linux reads the HUBP registers only when VGA mode is off.
    if ((control & d1vga_mode_enable) == 0 &&
        (!read_ip_register(io, registers.segment(2), hubp0_viewport_dimension, viewport) ||
         !read_ip_register(io, registers.segment(2), hubpreq0_surface_pitch, pitch) ||
         viewport == UINT32_MAX || pitch == UINT32_MAX)) {
        return false;
    }

    bytes = vbios_fb_bytes(control, viewport, pitch);
    return true;
}

VramReservationStatus build_vram_reservation_map(const VramReservationInputs& inputs,
                                                 VramReservationMap& output) noexcept {
    output = {};

    if (inputs.vram_bytes == 0 || inputs.visible_vram_bytes == 0 ||
        inputs.visible_vram_bytes > inputs.vram_bytes || inputs.reserved_fb_bytes == 0) {
        return VramReservationStatus::invalid_argument;
    }

    VramReservationMap map{};
    const uint64_t vram = inputs.vram_bytes;
    map.vram_bytes = vram;

    // amdgpu_atomfirmware_allocate_fb_scratch (bare metal).
    const AtomFbScratch scratch = atom_allocate_fb_scratch(inputs.usage);
    map.atom_scratch_bytes = scratch.scratch_bytes;

    // amdgpu_gmc_get_vbios_allocations: Navi31 keeps no stolen VGA memory by
    // ASIC quirk (keep_stolen_vga_memory false) and has no stolen_reserved range.
    uint64_t size = inputs.display_hardware ? inputs.vbios_fb_bytes : 0;

    if (size > vram) {
        // Linux's unsigned difference would keep it and then fail create_at.
        return VramReservationStatus::out_of_range;
    }

    if (vram - size < vbios_minimum_remaining_bytes) {
        size = 0;
    }

    map.stolen_vga_bytes = size > vbios_vga_allocation_bytes ? vbios_vga_allocation_bytes : size;
    map.stolen_extended_bytes = size - map.stolen_vga_bytes;
    map.usage_beyond_visible = scratch.firmware_usage_bytes > inputs.visible_vram_bytes ||
                               scratch.driver_usage_bytes > inputs.visible_vram_bytes;

    // amdgpu_ttm_init order: fw_vram_usage, drv_vram_usage, TMR, stolen memory.
    bool ok = add(map, VramReservationKind::firmware_usage, scratch.firmware_usage_offset,
                  scratch.firmware_usage_bytes, true) &&
              add(map, VramReservationKind::driver_usage, scratch.driver_usage_offset,
                  scratch.driver_usage_bytes, true);

    // amdgpu_ttm_reserve_tmr: reserve_size from firmwareinfo (DISCOVERY_TMR_OFFSET
    // fallback already applied by parse_atom_firmware_info).
    const uint64_t reserve = inputs.reserved_fb_bytes;

    if (!ok || reserve > vram) {
        return ok ? VramReservationStatus::out_of_range : VramReservationStatus::invalid_argument;
    }

    map.top_reserve_bytes = reserve;

    if ((inputs.capability & atom_capability_two_stage_training) != 0) {
        // amdgpu_ttm_training_data_block_init: ALIGN(mc_vram_size - reserve - SZ_1M, SZ_1M).
        if (vram - reserve < vram_training_alignment) {
            return VramReservationStatus::out_of_range;
        }

        const uint64_t below = vram - reserve - vram_training_alignment;
        map.c2p_offset = (below + vram_training_alignment - 1) & ~(vram_training_alignment - 1);
        map.p2c_offset = vram - psp_training_p2c_offset;
        map.training_reserved = true;
        ok = add(map, VramReservationKind::training_c2p, map.c2p_offset, psp_training_data_bytes, true);
    }

    ok = ok && add(map, VramReservationKind::firmware_top, vram - reserve, reserve, true) &&
         add(map, VramReservationKind::stolen_vga, 0, map.stolen_vga_bytes, true) &&
         add(map, VramReservationKind::stolen_extended, map.stolen_vga_bytes,
             map.stolen_extended_bytes, true);

    // psp_mem_training: BIST overwrites the bottom of VRAM while it runs.
    if (ok && map.training_reserved) {
        ok = add(map, VramReservationKind::training_encroached, 0, psp_training_encroached_bytes, false);
    }

    if (!ok) {
        return VramReservationStatus::invalid_argument;
    }

    for (uint32_t index = 0; index < map.count; ++index) {
        const VramReservation& entry = map.entries[index];

        if (entry.offset > vram || entry.bytes > vram - entry.offset) {
            return VramReservationStatus::out_of_range;
        }

        for (uint32_t other = index + 1; other < map.count; ++other) {
            const VramReservation& second = map.entries[other];

            if (entry.lifetime && second.lifetime &&
                overlap(entry.offset, entry.bytes, second.offset, second.bytes)) {
                return VramReservationStatus::overlapping;
            }
        }
    }

    output = map;
    return VramReservationStatus::built;
}

bool vram_range_reserved(const VramReservationMap& map, uint64_t offset, uint64_t bytes) noexcept {
    if (bytes == 0 || offset > UINT64_MAX - bytes) {
        return true;
    }

    for (uint32_t index = 0; index < map.count && index < VramReservationMap::capacity; ++index) {
        if (overlap(offset, bytes, map.entries[index].offset, map.entries[index].bytes)) {
            return true;
        }
    }

    return false;
}

VramPlacementStatus place_vram_region(const VramReservationMap& map,
                                      const VramPlacementRequest& request,
                                      uint64_t& offset) noexcept {
    const uint64_t alignment = request.alignment;

    if (map.vram_bytes == 0 || request.bytes == 0 || alignment == 0 ||
        (alignment & (alignment - 1)) != 0 || (request.preferred_offset & (alignment - 1)) != 0 ||
        request.limit > map.vram_bytes || request.bytes > request.limit) {
        return VramPlacementStatus::invalid_argument;
    }

    uint64_t candidate = request.preferred_offset;
    bool moved = false;

    // Each step jumps past at least one reservation, so the loop ends after at
    // most capacity + 1 rounds.
    for (uint32_t round = 0; round <= VramReservationMap::capacity; ++round) {
        if (candidate > request.limit - request.bytes) {
            return VramPlacementStatus::no_space;
        }

        uint64_t next = candidate;

        for (uint32_t index = 0; index < map.count && index < VramReservationMap::capacity; ++index) {
            const VramReservation& entry = map.entries[index];

            if (overlap(candidate, request.bytes, entry.offset, entry.bytes)) {
                const uint64_t end = entry.offset + entry.bytes;
                const uint64_t aligned = end > UINT64_MAX - (alignment - 1) ? UINT64_MAX :
                                         (end + alignment - 1) & ~(alignment - 1);
                next = aligned > next ? aligned : next;
            }
        }

        if (next == candidate) {
            if (vram_range_reserved(map, candidate, request.bytes)) {
                return VramPlacementStatus::no_space;
            }

            offset = candidate;
            return moved ? VramPlacementStatus::relocated : VramPlacementStatus::preferred;
        }

        candidate = next;
        moved = true;
    }

    return VramPlacementStatus::no_space;
}

} // namespace rx7900::hardware

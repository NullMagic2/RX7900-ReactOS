/*
Module: rx7900.hardware.vram_reservations
Purpose: Build the Linux v6.12 VRAM reservation map of a Navi31 and place driver regions outside it.
Created: 2026-10-22
Architecture: Pure decisions over atom_firmware results and DCN3.2 pre-OS scanout
registers read through the shared RegisterIo; the native adapter (asic_bringup)
consumes the placement before it maps any VRAM.
Copyright 2016-2022 Advanced Micro Devices, Inc. Adapted Linux v6.12 amdgpu_ttm.c
(amdgpu_ttm_init reservations, amdgpu_ttm_reserve_tmr, training data block),
amdgpu_gmc.c (amdgpu_gmc_get_vbios_allocations), gmc_v11_0.c
(gmc_v11_0_get_vbios_fb_size), amdgpu.h, amdgpu_psp.h, amdgpu_discovery.h and
dcn_3_2_0_offset.h/_sh_mask.h. Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#pragma once

#include "atom_firmware.hpp"
#include "psp_training.hpp"

namespace rx7900::hardware {

// amdgpu.h AMDGPU_VBIOS_VGA_ALLOCATION: 8 MiB VGA emulator plus 1 MiB FB.
constexpr uint64_t vbios_vga_allocation_bytes = 9ULL << 20;
// amdgpu_gmc_get_vbios_allocations drops the pre-OS buffer below this remainder.
constexpr uint64_t vbios_minimum_remaining_bytes = 8ULL << 20;
// amdgpu_ttm_training_data_block_init aligns the C2P block to SZ_1M; the block
// sizes and the BIST range are psp_training.hpp's (amdgpu_psp.h) constants.
constexpr uint64_t vram_training_alignment = 1ULL << 20;
// soc15_hw_ip.h DMU_HWID (DCE_HWIP); GC 11.0.0 boards carry DCN 3.2.0.
constexpr uint16_t discovery_dmu_hardware_id = 271;

enum class VramReservationKind : uint32_t {
    // mman.stolen_vga_memory [0, stolen_vga_size) and stolen_extended_memory.
    stolen_vga,
    stolen_extended,
    // mman.fw_vram_usage / drv_vram_usage from vram_usagebyfirmware.
    firmware_usage,
    driver_usage,
    // mman.fw_reserved_memory: [real_vram_size - reserve_size, real_vram_size), the
    // PSP/IP-discovery top-of-VRAM region (discovery binary at top - 64 KiB, P2C
    // training data at top - 32 KiB inside it).
    firmware_top,
    // psp mem_train_ctx.c2p_bo (4 KiB), reserved only with two-stage training.
    training_c2p,
    // Not a TTM reservation: PSP BIST long training overwrites [0, 32 MiB) while
    // it runs. Driver data placed there would be destroyed (Linux saves it to
    // system memory around psp_mem_training); listed so placement avoids it.
    training_encroached
};

struct VramReservation final {
    VramReservationKind kind;
    // VRAM byte offset (MC address minus vram_start), not an MC or bus address.
    uint64_t offset;
    uint64_t bytes;
    // Held for the driver lifetime (true) or only during PSP training (false).
    bool lifetime;
};

// Inputs mirror the adev fields Linux uses. vram_bytes: gmc.real_vram_size
// (== mc_vram_size, RCC_CONFIG_MEMSIZE << 20). visible_vram_bytes: gmc.visible_vram_size
// (min of BAR0 and VRAM). capability and reserved_fb_bytes: AtomFirmwareInfo
// (reserved_fb_bytes already holds the DISCOVERY_TMR_OFFSET fallback). usage:
// parse_atom_vram_usage (layout none when absent). display_hardware:
// amdgpu_device_has_display_hardware (DMU discovered, not harvested).
// vbios_fb_bytes: gmc_v11_0_get_vbios_fb_size (read_vbios_fb_bytes).
struct VramReservationInputs final {
    uint64_t vram_bytes;
    uint64_t visible_vram_bytes;
    uint32_t capability;
    uint32_t reserved_fb_bytes;
    AtomVramUsage usage;
    bool display_hardware;
    uint32_t vbios_fb_bytes;
};

enum class VramReservationStatus {
    built,
    invalid_argument,
    // A reservation lies outside VRAM (Linux amdgpu_bo_create_kernel_at fails
    // and amdgpu_ttm_init aborts).
    out_of_range,
    // Two lifetime reservations overlap (Linux would fail the second create_at).
    overlapping
};

struct VramReservationMap final {
    static constexpr size_t capacity = 7;
    VramReservation entries[capacity]{};
    uint32_t count = 0;
    uint64_t vram_bytes = 0;
    // amdgpu_gmc_get_vbios_allocations results.
    uint64_t stolen_vga_bytes = 0;
    uint64_t stolen_extended_bytes = 0;
    // amdgpu_ttm_reserve_tmr: reserve_size and the training data block layout.
    uint64_t top_reserve_bytes = 0;
    bool training_reserved = false;
    uint64_t c2p_offset = 0;
    uint64_t p2c_offset = 0;
    // ATOM interpreter scratch in system memory (amdgpu_atomfirmware_allocate_fb_scratch).
    uint32_t atom_scratch_bytes = 0;
    // fw/drv usage requests Linux skips because they exceed visible VRAM
    // (amdgpu_ttm_fw/drv_reserve_vram_init); still listed here (fail closed).
    bool usage_beyond_visible = false;
};

// Pure. Linux v6.12 bare-metal Navi31 order: amdgpu_atomfirmware_allocate_fb_scratch,
// amdgpu_gmc_get_vbios_allocations (keep_stolen_vga_memory false, no stolen_reserved
// for this ASIC), then amdgpu_ttm_init: fw_vram_usage, drv_vram_usage,
// amdgpu_ttm_reserve_tmr (C2P block first when ATOM_FIRMWARE_CAP_ENABLE_2STAGE_BIST_TRAINING,
// then the top region) and the stolen VGA/extended regions. Empty regions are
// omitted. Output is cleared first and complete only on built.
VramReservationStatus build_vram_reservation_map(const VramReservationInputs& inputs,
                                                 VramReservationMap& output) noexcept;

// True when [offset, offset + bytes) intersects any entry (or wraps).
bool vram_range_reserved(const VramReservationMap& map, uint64_t offset, uint64_t bytes) noexcept;

// gmc_v11_0_get_vbios_fb_size on raw register values: D1VGA_CONTROL.D1VGA_MODE_ENABLE
// gives AMDGPU_VBIOS_VGA_ALLOCATION, otherwise HUBP0 PRI_VIEWPORT_HEIGHT times
// HUBPREQ0 DCSURF_SURFACE_PITCH (pixels) times 4 bytes.
uint32_t vbios_fb_bytes(uint32_t d1vga_control, uint32_t viewport_dimension,
                        uint32_t surface_pitch) noexcept;

// Only validate_dcn_fb_discovery creates a valid token: unique DMU instance0 3.2.0
// with segments 1 (D1VGA_CONTROL) and 2 (HUBP0/HUBPREQ0) whose registers fit BAR5.
class DcnFbRegisters final {
public:
    DcnFbRegisters() noexcept = default;
    bool valid() const noexcept { return dmu_.valid() && aperture_bytes_ != 0; }
    uint32_t segment(size_t index) const noexcept { return dmu_.segment(index); }
    uint32_t aperture_bytes() const noexcept { return aperture_bytes_; }

private:
    friend DiscoveryStatus validate_dcn_fb_discovery(ByteView, uint32_t, DcnFbRegisters&) noexcept;
    DiscoveredIp dmu_{};
    uint32_t aperture_bytes_ = 0;
};

DiscoveryStatus validate_dcn_fb_discovery(ByteView binary, uint32_t register_aperture_bytes,
                                          DcnFbRegisters& output) noexcept;

// PASSIVE_LEVEL, exclusive owner, before any display programming. Three
// read-only register reads; false on an access failure or an all-ones value
// (device not responding), bytes unchanged.
bool read_vbios_fb_bytes(const RegisterIo& io, const DcnFbRegisters& registers,
                         uint32_t& bytes) noexcept;

// A driver-owned VRAM region the native adapter maps through BAR0. offset is
// tried first; limit is the end of the CPU-reachable window (visible VRAM).
struct VramPlacementRequest final {
    uint64_t preferred_offset;
    uint64_t bytes;
    uint64_t alignment;
    uint64_t limit;
};

enum class VramPlacementStatus { preferred, relocated, invalid_argument, no_space };

// Placement policy: keep preferred_offset when [offset, offset + bytes) meets no
// entry (lifetime or training); otherwise the lowest alignment-multiple offset
// above preferred_offset that is clear of every entry and ends at or below
// limit. Never moves below preferred_offset, so the floor the caller chose (here
// above the BIST-encroached range and typical pre-OS scanout) is kept. offset is
// written only for preferred/relocated; the result is re-verified against every
// entry before it is returned.
VramPlacementStatus place_vram_region(const VramReservationMap& map,
                                      const VramPlacementRequest& request,
                                      uint64_t& offset) noexcept;

} // namespace rx7900::hardware

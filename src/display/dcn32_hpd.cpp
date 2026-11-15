/*
Module: rx7900.display.dcn32_hpd
Purpose: Bounded read-only DCN3.2 HPD sense access.
Created: 2026-10-22
Architecture: DCN3.2 segment2 offsets from pinned Linux v6.12; native BAR5
ownership and exclusive serialization are supplied outside this module.
Copyright 2012-2023 Advanced Micro Devices, Inc.
Register provenance and notice: licenses/AMDGPU-DISPLAY.txt.
*/
#include "dcn32_hpd.hpp"

namespace rx7900::display {
namespace {

// dcn_3_2_0_offset.h: HPD0..4_DC_HPD_INT_STATUS, BASE_IDX=2.
constexpr uint32_t hpd0_int_status = 0x1f14u;
constexpr uint32_t hpd_stride = 8u;
constexpr uint8_t hpd_count = 5;
// dcn_3_2_0_sh_mask.h: DC_HPD_SENSE, bit 1. Bit 0 is IRQ status and must
// never be acknowledged by this read-only path.
constexpr uint32_t sense_mask = 0x2u;

} // namespace

HpdResult read_dcn32_hpd(RegisterIo io, const Dcn32RegisterSpace& space,
                         uint8_t verified_pin) noexcept {
    if (io.read == nullptr || !space.discovery_verified ||
        space.dmu_major != 3 || space.dmu_minor != 2 ||
        space.dmu_revision != 0 || verified_pin >= hpd_count) {
        return HpdResult::unavailable;
    }

    const uint64_t dword = static_cast<uint64_t>(space.segment2_dword_base) +
                           hpd0_int_status + static_cast<uint64_t>(verified_pin) * hpd_stride;
    const uint64_t offset = dword * sizeof(uint32_t);
    if (offset > UINT32_MAX || offset > space.aperture_bytes ||
        sizeof(uint32_t) > space.aperture_bytes - offset) {
        return HpdResult::unavailable;
    }

    uint32_t value = 0;
    if (!io.read(io.context, static_cast<uint32_t>(offset), value) ||
        value == UINT32_MAX) {
        return HpdResult::read_failure;
    }

    return (value & sense_mask) != 0 ? HpdResult::connected : HpdResult::disconnected;
}

} // namespace rx7900::display

/*
Module: rx7900.display.dcn32_otg
Purpose: Bounded actual DCN3.2 disabled-OTG timing register writes and readback.
Created: 2026-10-21
Architecture: Reuses prepare_optc_timing; native caller supplies owned register IO.
Copyright 2012-15, 2022 Advanced Micro Devices, Inc.
Pinned Linux offsets, masks and programming source: licenses/AMDGPU-DISPLAY.txt.
*/
#include "dcn32_otg.hpp"

namespace rx7900::display {
namespace {

// Exact generated OTG0 DWORD offsets, BASE_IDX=2. OTG1/2/3 banks are +0x80.
constexpr uint32_t instance_stride = 0x80;
constexpr uint32_t instance_count = 4;
constexpr uint32_t field_mask = 0x7fff;
constexpr uint32_t pair_mask = 0x7fff7fff;
constexpr uint32_t otg_control = 0x1b41;
constexpr uint32_t otg_clock = 0x1b85;
constexpr uint32_t otg_h_division = 0x1b2e;
constexpr uint32_t otg_interlace = 0x1b44;
constexpr uint32_t otg_stereo = 0x1b54;
constexpr uint32_t otg_update_lock = 0x1b5a;
constexpr uint32_t otg_master_lock = 0x1b8a;
constexpr uint32_t otg_v_total_control = 0x1b33;
constexpr uint32_t otg_double_buffer = 0x1b5b;

struct FieldWrite final {
    uint32_t dword_index;
    uint32_t mask;
    uint32_t bits;
    uint32_t byte_offset = 0;
    uint32_t value = 0;
};

bool byte_offset(const Dcn32RegisterSpace& space, uint32_t instance,
                 uint32_t index, uint32_t& output) noexcept {
    // All source terms are u32; widen before adding/scaling to avoid both the
    // DWORD-base and byte-offset wraparound classes. Inclusive final DWORD fits.
    const uint64_t offset = (static_cast<uint64_t>(space.segment2_dword_base) + index +
                             static_cast<uint64_t>(instance) * instance_stride) * 4u;

    if (offset > UINT32_MAX || offset > space.aperture_bytes ||
        sizeof(uint32_t) > space.aperture_bytes - offset) {
        return false;
    }

    output = static_cast<uint32_t>(offset);
    return true;
}

OtgResult safe_state(RegisterIo io, const Dcn32RegisterSpace& space,
                     uint32_t instance, bool check_pending) noexcept {
    struct Guard final {
        uint32_t index;
        uint32_t mask;
        uint32_t expected;
    };

    const Guard guards[]{
        {otg_control, 0x00010001u, 0}, // Requested and current master-enable.
        {otg_clock, 0x00010111u, 0x00000101u}, // Enabled/on, not reset/busy.
        {otg_h_division, 0x00030103u, 0}, // No manual/current/requested division.
        {otg_interlace, 0x00000001u, 0},
        {otg_stereo, 0x01000000u, 0},
        {otg_update_lock, 0x00000001u, 0},
        {otg_master_lock, 0x00000101u, 0},
        {otg_v_total_control, 0x0000000fu, 0}
    };

    for (const Guard& guard : guards) {
        uint32_t offset = 0;
        uint32_t value = 0;

        if (!byte_offset(space, instance, guard.index, offset)) {
            return OtgResult::unavailable;
        }

        if (!io.read(io.context, offset, value)) {
            return OtgResult::read_failure;
        }

        if ((value & guard.mask) != guard.expected) {
            return OtgResult::unsafe_hardware_state;
        }
    }

    if (check_pending) {
        uint32_t offset = 0;
        uint32_t value = 0;

        if (!byte_offset(space, instance, otg_double_buffer, offset)) {
            return OtgResult::unavailable;
        }

        if (!io.read(io.context, offset, value)) {
            return OtgResult::read_failure;
        }

        if ((value & 0x030006f1u) != 0) {
            return OtgResult::unsafe_hardware_state;
        }
    }

    return OtgResult::disabled_fields_verified; // Guard check only; no write.
}

} // namespace

OtgResult Dcn32Otg::program_disabled(
    const Timing& timing, const DisabledOtgPrerequisites& prerequisites) noexcept {
    if (quarantined_) {
        return OtgResult::quarantined;
    }

    OptcTiming fields{};

    if (prepare_optc_timing(timing, fields) != Status::success) {
        return OtgResult::invalid_timing;
    }

    if (!prerequisites.resources_and_initialization_owned ||
        !prerequisites.display_clocks_verified ||
        !prerequisites.exclusive_register_lock_held ||
        !prerequisites.vtg_pipe_link_quiesced || !prerequisites.single_pipe_rgb_no_odm ||
        !space_.discovery_verified || space_.dmu_major != 3 || space_.dmu_minor != 2 ||
        space_.dmu_revision != 0 || instance_ >= instance_count ||
        io_.read == nullptr || io_.write == nullptr) {
        return OtgResult::unavailable;
    }

    FieldWrite writes[]{
        {0x1b2a, field_mask, fields.h_total_minus_one},
        {0x1b2c, pair_mask, fields.h_sync_end << 16},
        {0x1b2b, pair_mask, fields.h_blank_start | (fields.h_blank_end << 16)},
        {0x1b2d, 1, fields.h_sync_negative},
        {0x1b2f, field_mask, fields.v_total_minus_one},
        {0x1b30, field_mask, fields.v_total_minus_one},
        {0x1b31, field_mask, fields.v_total_minus_one},
        {0x1b37, pair_mask, fields.v_sync_end << 16},
        {0x1b36, pair_mask, fields.v_blank_start | (fields.v_blank_end << 16)},
        {0x1b38, 1, fields.v_sync_negative}
    };

    // Validate all offsets first, then read every current field before any write.
    for (FieldWrite& write : writes) {
        if (!byte_offset(space_, instance_, write.dword_index, write.byte_offset)) {
            return OtgResult::unavailable;
        }
    }

    OtgResult checked = safe_state(io_, space_, instance_, true);

    if (checked != OtgResult::disabled_fields_verified) {
        return checked;
    }

    for (FieldWrite& write : writes) {
        uint32_t current = 0;

        if (!io_.read(io_.context, write.byte_offset, current)) {
            return OtgResult::read_failure;
        }

        if (write.dword_index == 0x1b2d && (current & 0x00010000u) != 0) {
            return OtgResult::unsafe_hardware_state; // Composite sync unsupported.
        }

        write.value = (current & ~write.mask) | write.bits;
    }

    bool wrote = false;

    for (const FieldWrite& write : writes) {
        checked = safe_state(io_, space_, instance_, false);

        if (checked != OtgResult::disabled_fields_verified) {
            quarantined_ = wrote;
            return wrote ? OtgResult::quarantined : checked;
        }

        // A failing write callback may have partially reached the device. The
        // readback also flushes posted writes through the native MMIO boundary.
        wrote = true;

        if (!io_.write(io_.context, write.byte_offset, write.value)) {
            quarantined_ = true;
            return OtgResult::quarantined;
        }

        uint32_t readback = 0;

        if (!io_.read(io_.context, write.byte_offset, readback) ||
            (readback & write.mask) != write.bits) {
            quarantined_ = true;
            return OtgResult::quarantined;
        }
    }

    // Recheck all fields after the complete batch: a transient successful
    // individual readback cannot hide a later field loss or external change.
    for (const FieldWrite& write : writes) {
        uint32_t readback = 0;

        if (!io_.read(io_.context, write.byte_offset, readback) ||
            (readback & write.mask) != write.bits) {
            quarantined_ = true;
            return OtgResult::quarantined;
        }
    }

    checked = safe_state(io_, space_, instance_, false);

    if (checked != OtgResult::disabled_fields_verified) {
        quarantined_ = true;
        return OtgResult::quarantined;
    }

    return OtgResult::disabled_fields_verified;
}

} // namespace rx7900::display

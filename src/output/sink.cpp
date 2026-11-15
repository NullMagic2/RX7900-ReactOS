/*
Module: rx7900.output.sink
Purpose: Checked Linux-derived EDID block, CTA and detailed timing decoding.
Created: 2026-10-21
Architecture: Reuses common ByteView and display timing/catalog validation.
Adapted from Linux v6.12 drm_edid.c and drm_edid.h; permission and exact pinned
sources are preserved in licenses/AMDGPU-OUTPUT.txt. No Linux runtime is linked.
Copyright (c) 2006 Luc Verhaegen (quirks list)
Copyright (c) 2007-2008 Intel Corporation
Copyright 2010 Red Hat, Inc.
Copyright (C) 2006 Dennis Munsie
*/
#include "sink.hpp"

namespace rx7900::output {
namespace {

constexpr uint32_t detailed_bytes = 18;
constexpr uint32_t base_detailed_offset = 54;
constexpr uint32_t base_detailed_count = 4;
constexpr uint8_t cta_extension_tag = 0x02;
constexpr uint8_t cta_extended_data_tag = 7;
constexpr uint8_t cta_hf_eeodb_tag = 0x78;
constexpr uint32_t rgb8_wire_bits_per_pixel = 24;
static_assert(base_detailed_count + (maximum_edid_blocks - 1u) * 6u <=
              display::mode_capacity, "Every accepted EDID DTD fits the shared catalog");

// All offsets are bounded by a previously verified complete block. The common
// checked reader remains the only raw byte access, including for these reads.
uint8_t byte_at(ByteView input, size_t offset) noexcept {
    uint8_t value = 0;
    input.read_u8(offset, value);
    return value;
}

Status validate_block(ByteView input, size_t base) noexcept {
    if (!input.contains(base, edid_block_bytes)) {
        return Status::truncated_input;
    }

    uint32_t sum = 0;
    bool any_nonzero = false;

    for (uint32_t index = 0; index < edid_block_bytes; ++index) {
        const uint8_t value = byte_at(input, base + index);
        sum += value;
        any_nonzero = any_nonzero || value != 0;
    }

    if (!any_nonzero) {
        return Status::malformed_edid;
    }

    return (sum & 0xffu) == 0 ? Status::success : Status::checksum_mismatch;
}

void collect_detailed(ByteView input, size_t offset, bool preferred,
                      SinkSnapshot& snapshot) noexcept {
    uint16_t pixel_clock = 0;

    if (!input.read_u16(offset, pixel_clock) || !input.contains(offset, detailed_bytes)) {
        ++snapshot.rejected_detailed_modes;
        return;
    }

    if (pixel_clock == 0) {
        return; // Monitor descriptors and padding are not pixel timings.
    }

    const uint8_t flags = byte_at(input, offset + 17);

    // Borderless progressive, separate digital sync only. Reject both stereo
    // selector bits and its low selector bit, rather than interpreting it as RGB.
    if ((flags & 0xe1u) != 0 || (flags & 0x18u) != 0x18u ||
        byte_at(input, offset + 15) != 0 || byte_at(input, offset + 16) != 0) {
        ++snapshot.rejected_detailed_modes;
        return;
    }

    const uint32_t h_high = byte_at(input, offset + 4);
    const uint32_t v_high = byte_at(input, offset + 7);
    const uint32_t sync_high = byte_at(input, offset + 11);
    const uint32_t v_sync = byte_at(input, offset + 10);
    const uint32_t h_active = byte_at(input, offset + 2) | ((h_high & 0xf0u) << 4);
    const uint32_t v_active = byte_at(input, offset + 5) | ((v_high & 0xf0u) << 4);
    const uint32_t h_blank = byte_at(input, offset + 3) | ((h_high & 0x0fu) << 8);
    const uint32_t v_blank = byte_at(input, offset + 6) | ((v_high & 0x0fu) << 8);

    // All decoded fields are at most 12 bits; totals and clock scaling fit u32.
    const display::Timing timing{
        static_cast<uint32_t>(pixel_clock) * 10u,
        h_active, h_active + h_blank,
        byte_at(input, offset + 8) | ((sync_high & 0xc0u) << 2),
        byte_at(input, offset + 9) | ((sync_high & 0x30u) << 4),
        v_active, v_active + v_blank,
        (v_sync >> 4) | ((sync_high & 0x0cu) << 2),
        (v_sync & 0x0fu) | ((sync_high & 0x03u) << 4),
        ((flags & 0x02u) != 0 ? display::positive_hsync : 0u) |
        ((flags & 0x04u) != 0 ? display::positive_vsync : 0u)
    };

    if (h_active < 64 || v_active < 64) {
        ++snapshot.rejected_detailed_modes;
        return;
    }

    uint32_t index = 0;
    const display::Status appended = snapshot.detailed_modes.append(timing, index);

    if (appended == display::Status::success) {
        if (preferred) {
            snapshot.preferred_mode = index;
        }
    } else if (appended != display::Status::duplicate_mode) {
        ++snapshot.rejected_detailed_modes;
    }
}

Status collect_cta(ByteView input, size_t base, SinkSnapshot& snapshot) noexcept {
    const uint8_t revision = byte_at(input, base + 1);

    if (revision == 0 || revision > 3) {
        ++snapshot.unsupported_extensions;
        return Status::success;
    }

    const uint32_t detailed_start = byte_at(input, base + 2);

    if (detailed_start == 0) {
        return Status::success; // No data block collection or detailed timings.
    }

    if (detailed_start < 4 || detailed_start > 127) {
        return Status::malformed_edid;
    }

    if (revision >= 3) {
        uint32_t offset = 4;

        while (offset < detailed_start) {
            const uint8_t header = byte_at(input, base + offset);
            const uint32_t payload_length = header & 0x1fu;
            const uint32_t remaining = detailed_start - offset - 1u;

            if (payload_length > remaining) {
                return Status::malformed_edid;
            }

            if ((header >> 5) == cta_extended_data_tag && payload_length != 0 &&
                byte_at(input, base + offset + 1u) == cta_hf_eeodb_tag) {
                return Status::unsupported_edid; // Cannot trust base block count.
            }

            offset += 1u + payload_length;
        }
    }

    // Match Linux cea_for_each_detailed_block: checksum byte is never a DTD byte.
    for (uint32_t offset = detailed_start; detailed_bytes <= 127u - offset;
         offset += detailed_bytes) {
        collect_detailed(input, base + offset, false, snapshot);
    }

    return Status::success;
}

} // namespace

Status parse_edid(ByteView input, SinkSnapshot& output) noexcept {
    if (!input.contains(0, edid_block_bytes)) {
        return Status::truncated_input;
    }

    constexpr uint8_t header[8]{0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00};

    for (uint32_t index = 0; index < sizeof(header); ++index) {
        if (byte_at(input, index) != header[index]) {
            return Status::malformed_edid;
        }
    }

    const uint8_t revision = byte_at(input, 19);

    if (byte_at(input, 18) != 1 || (revision != 3 && revision != 4)) {
        return Status::unsupported_edid;
    }

    const uint32_t blocks = static_cast<uint32_t>(byte_at(input, 126)) + 1u;

    if (blocks > maximum_edid_blocks) {
        return Status::unsupported_edid;
    }

    const uint32_t required_bytes = blocks * edid_block_bytes;

    if (input.size() < required_bytes) {
        return Status::truncated_input;
    }

    if (input.size() != required_bytes) {
        return Status::malformed_edid;
    }

    for (uint32_t block = 0; block < blocks; ++block) {
        const Status status = validate_block(input, block * edid_block_bytes);

        if (status != Status::success) {
            return status;
        }
    }

    SinkSnapshot candidate{};
    candidate.block_count = blocks;
    const uint8_t video_input = byte_at(input, 20);
    candidate.digital_input = (video_input & 0x80u) != 0;

    if (candidate.digital_input && revision == 4) {
        const uint32_t depth = (video_input >> 4) & 0x07u;

        if (depth == 7) {
            return Status::unsupported_edid;
        }

        candidate.bits_per_channel = depth == 0 ? 0 : 2u * depth + 4u;
    }

    const bool base_preferred = revision == 4 || (byte_at(input, 24) & 0x02u) != 0;

    for (uint32_t index = 0; index < base_detailed_count; ++index) {
        collect_detailed(input, base_detailed_offset + index * detailed_bytes,
                         base_preferred && index == 0, candidate);
    }

    for (uint32_t block = 1; block < blocks; ++block) {
        const size_t base = block * edid_block_bytes;

        if (byte_at(input, base) != cta_extension_tag) {
            ++candidate.unsupported_extensions;
            continue;
        }

        const Status status = collect_cta(input, base, candidate);

        if (status != Status::success) {
            return status;
        }
    }

    output = candidate;
    return Status::success;
}

Status prepare_output_mode(const SinkSnapshot& sink, uint32_t sink_mode_index,
                           const LinkCapabilities& link,
                           display::Timing& output) noexcept {
    display::Timing timing{};

    if (sink.detailed_modes.get(sink_mode_index, timing) != display::Status::success) {
        return Status::mode_not_found;
    }

    if (link.transport != Transport::hdmi_tmds &&
        link.transport != Transport::display_port_sst) {
        return Status::capabilities_unavailable;
    }

    if (link.maximum_pixel_clock_khz == 0 || link.usable_payload_bits_per_second == 0 ||
        !link.rgb_8bpc_verified) {
        return Status::capabilities_unavailable;
    }

    // u32 kHz times 1000 times 24 fits u64. Full pixel rate is conservative for
    // packetized links; the backend still accounts for transport-specific rules.
    const uint64_t required_payload = static_cast<uint64_t>(timing.pixel_clock_khz) *
                                      1000u * rgb8_wire_bits_per_pixel;

    if (!sink.digital_input || (sink.bits_per_channel != 0 && sink.bits_per_channel < 8) ||
        timing.pixel_clock_khz > link.maximum_pixel_clock_khz ||
        required_payload > link.usable_payload_bits_per_second) {
        return Status::unsupported_output;
    }

    output = timing;
    return Status::success;
}

} // namespace rx7900::output

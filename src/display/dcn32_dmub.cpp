/*
Module: rx7900.display.dcn32_dmub
Purpose: Bounded DMCUB region layout, start-up, status and inbox1 command ring.
Created: 2026-10-24
Architecture: Exact v6.12 dcn_3_2_0_offset.h/_sh_mask.h DMCUB values (BASE_IDX
2) and dmub_srv.c / dmub_cmd.h constants; sequences of the files in the header.
Copyright 2019-2023 Advanced Micro Devices, Inc.
Linux notice and provenance: licenses/AMDGPU-DISPLAY.txt.
*/
#include "dcn32_dmub.hpp"

namespace rx7900::display {
namespace {

constexpr uint32_t cc_dc_pipe_dis = 0x00cau;
constexpr uint32_t region5_offset = 0x0198u;
constexpr uint32_t region5_offset_high = 0x0199u;
constexpr uint32_t region5_top = 0x01a2u;
constexpr uint32_t cw_base_address[4]{0x01a8u, 0x01a9u, 0x01aau, 0x01abu};   // CW3..CW6
constexpr uint32_t cw_top_address[4]{0x01b0u, 0x01b1u, 0x01b2u, 0x01b3u};
constexpr uint32_t cw_offset[4]{0x01bbu, 0x01bdu, 0x01bfu, 0x01c1u};
constexpr uint32_t cw_offset_high[4]{0x01bcu, 0x01beu, 0x01c0u, 0x01c2u};
constexpr uint32_t inbox1_base = 0x01d4u;
constexpr uint32_t inbox1_size = 0x01d5u;
constexpr uint32_t inbox1_wptr = 0x01d6u;
constexpr uint32_t inbox1_rptr = 0x01d7u;
constexpr uint32_t outbox0_base = 0x01d8u;
constexpr uint32_t outbox0_size = 0x01d9u;
constexpr uint32_t outbox0_wptr = 0x01dau;
constexpr uint32_t outbox0_rptr = 0x01dbu;
constexpr uint32_t outbox1_base = 0x01dcu;
constexpr uint32_t outbox1_size = 0x01ddu;
constexpr uint32_t outbox1_wptr = 0x01deu;
constexpr uint32_t outbox1_rptr = 0x01dfu;
constexpr uint32_t scratch0 = 0x01e3u;
constexpr uint32_t scratch7 = 0x01eau;
constexpr uint32_t scratch14 = 0x01f1u;
constexpr uint32_t scratch15 = 0x01f2u;
constexpr uint32_t dmcub_cntl = 0x01f6u;
constexpr uint32_t gpint_datain1 = 0x01f8u;
constexpr uint32_t dmcub_cntl2 = 0x0200u;
constexpr uint32_t mmhubbub_soft_reset = 0x0342u;

constexpr uint32_t dmcub_supported = 0x00010000u;
constexpr uint32_t top_address_mask = 0x1fffffffu;
constexpr uint32_t window_enable = 0x80000000u;
constexpr uint32_t dmcub_enable = 0x00010000u;
constexpr uint32_t traceport_enable = 0x00080000u;
constexpr uint32_t soft_reset = 0x00000001u;
constexpr uint32_t dmuif_soft_reset = 0x00000100u;
constexpr uint32_t skip_phy_init_panel_sequence = 0x00000020u;
constexpr uint32_t psp_version_mask = 0x001100ffu;

// dmub_srv.c / dmub_cmd.h.
constexpr uint32_t cw3_base = 0x63000000u;
constexpr uint32_t cw4_base = 0x64000000u;
constexpr uint32_t cw5_base = 0x65000000u;
constexpr uint32_t cw6_base = 0x66000000u;
constexpr uint32_t region5_base = 0xa0000000u;
constexpr uint32_t trace_entry_offset = 16u;
constexpr uint32_t stack_bytes = 128u * 1024u + 512u * 1024u;
constexpr uint32_t mailbox_bytes = 2u * dmub_ring_bytes;
constexpr uint32_t default_fw_state_bytes = 64u * 1024u;
constexpr uint32_t default_trace_bytes = 64u * 1024u;
constexpr uint32_t scratch_bytes = 1024u;
constexpr uint32_t shared_state_bytes = 1024u;
constexpr uint32_t psp_header_bytes = 0x100u;
constexpr uint32_t psp_footer_bytes = 0x100u;
constexpr uint32_t meta_magic = 0x444d5542u;
constexpr uint32_t meta_legacy_offset = 0x24u;
constexpr uint32_t meta_union_bytes = 64u;
constexpr uint32_t boot_dal_fw = 0x1u;
constexpr uint32_t boot_mailbox_ready = 0x2u;
constexpr uint32_t gpint_stop_fw = (1u << 28) | (2u << 16);
constexpr uint32_t gpint_stop_response = 0xdeaddeadu;
constexpr uint32_t gpint_polls = 30u;
constexpr uint32_t auto_load_timeout_us = 100000u;
constexpr uint32_t auto_load_step_us = 100u;
// Each window is at most the 29-bit DMCUB aperture.
constexpr uint32_t maximum_window_bytes = 0x01000000u;

uint64_t align(uint64_t value, uint64_t alignment) noexcept {
    return (value + alignment - 1u) / alignment * alignment;
}

uint32_t le32(ByteView view, size_t offset) noexcept {
    uint32_t value = 0;
    view.read_u32(offset, value);
    return value;
}

// dmub_get_fw_meta_info_from_blob: meta union ends offset bytes before the blob end.
bool meta_at(ByteView file, size_t blob, size_t blob_bytes, size_t offset,
             uint32_t& fw_region, uint32_t& trace, uint32_t& version) noexcept {
    if (blob_bytes < meta_union_bytes + offset) {
        return false;
    }
    const size_t at = blob + blob_bytes - offset - meta_union_bytes;
    if (!file.contains(at, meta_union_bytes) || le32(file, at) != meta_magic) {
        return false;
    }
    fw_region = le32(file, at + 4);
    trace = le32(file, at + 8);
    version = le32(file, at + 12);
    return true;
}

} // namespace

DmubResult dmub_calc_layout(ByteView firmware, uint32_t vbios_bytes, DmubLayout& layout) noexcept {
    hardware::FirmwareImage image{};
    if (hardware::validate_firmware(firmware, hardware::FirmwareKind::dmcub_3_2, image) !=
            hardware::FirmwareStatus::success ||
        image.part_count != 1 || vbios_bytes == 0 || vbios_bytes > maximum_window_bytes) {
        return DmubResult::invalid_firmware;
    }
    const uint32_t inst_const = image.parts[0].bytes;
    const uint64_t bss_offset = static_cast<uint64_t>(image.payload_offset) + inst_const;
    if (inst_const <= psp_header_bytes + psp_footer_bytes || image.dmcub_bss_bytes > maximum_window_bytes ||
        bss_offset > firmware.size() ||
        !firmware.contains(static_cast<size_t>(bss_offset), image.dmcub_bss_bytes)) {
        return DmubResult::invalid_firmware;
    }

    DmubLayout candidate{};
    candidate.inst_const_bytes = inst_const - psp_header_bytes - psp_footer_bytes;
    candidate.bss_data_bytes = image.dmcub_bss_bytes;
    candidate.bss_data_file_offset = static_cast<uint32_t>(bss_offset);
    candidate.fw_version = image.version;

    uint32_t fw_state = default_fw_state_bytes;
    uint32_t trace = default_trace_bytes;
    uint32_t region = 0;
    uint32_t trace_meta = 0;
    uint32_t version = 0;
    bool found = false;
    if (candidate.bss_data_bytes != 0) {
        found = meta_at(firmware, candidate.bss_data_file_offset, candidate.bss_data_bytes,
                        meta_legacy_offset, region, trace_meta, version);
    } else {
        const size_t inst = static_cast<size_t>(image.payload_offset) + psp_header_bytes;
        for (size_t offset = 0; offset < 16 && !found; ++offset) {
            found = meta_at(firmware, inst, candidate.inst_const_bytes, offset, region,
                            trace_meta, version);
        }
    }
    if (found) {
        if (region == 0 || region > maximum_window_bytes || trace_meta == 0 ||
            trace_meta > maximum_window_bytes) {
            return DmubResult::invalid_firmware;
        }
        fw_state = region;
        trace = trace_meta;
        candidate.metadata_found = true;
        if (candidate.fw_version == 0) {
            candidate.fw_version = version;
        }
    }

    const uint32_t sizes[dmub_window_count]{
        candidate.inst_const_bytes, stack_bytes, candidate.bss_data_bytes, vbios_bytes,
        mailbox_bytes, trace, fw_state, scratch_bytes, shared_state_bytes};
    uint64_t top = 0;
    for (uint32_t index = 0; index < dmub_window_count; ++index) {
        const uint64_t base = align(top, 256);
        top = base + align(sizes[index], 64);
        if (top > UINT32_MAX) {
            return DmubResult::invalid_firmware;
        }
        candidate.windows[index] = {static_cast<uint32_t>(base), static_cast<uint32_t>(top)};
    }
    const uint64_t total = align(top, 4096);
    if (total > UINT32_MAX) {
        return DmubResult::invalid_firmware;
    }
    candidate.framebuffer_bytes = static_cast<uint32_t>(total);
    layout = candidate;
    return DmubResult::success;
}

bool dmub_held_in_reset(const hardware::RegisterIo& io, const Dcn32RegisterSpace& space,
                        bool& held) noexcept {
    uint32_t value = 0;
    if (!hardware::register_io_valid(io) || !space.discovery_verified ||
        !hardware::read_ip_register(io, space.segment2_dword_base, dmcub_cntl2, value) ||
        value == UINT32_MAX) {
        return false;
    }
    held = (value & soft_reset) != 0;
    return true;
}

void dmub_set_header(uint8_t (&command)[dmub_command_bytes], uint8_t type, uint8_t sub_type,
                     uint8_t payload_bytes) noexcept {
    command[0] = type;
    command[1] = sub_type;
    command[2] = 0;
    command[3] = static_cast<uint8_t>(payload_bytes & 0x3fu);
}

void dmub_build_fams_setup(bool should_manage_pstate, uint8_t (&command)[dmub_command_bytes]) noexcept {
    for (uint8_t& byte : command) {
        byte = 0;
    }
    dmub_set_header(command, dmub_cmd_fw_assisted_mclk_switch, dmub_cmd_fams_setup_fw_ctrl,
                    dmub_fams_config_payload_bytes);
    // config_data.fams_enabled is the first payload byte.
    command[4] = should_manage_pstate ? 1u : 0u;
}

bool Dcn32Dmub::read(uint32_t reg, uint32_t& value) noexcept {
    return hardware::read_ip_register(io_, space_.segment2_dword_base, reg, value) &&
           value != UINT32_MAX;
}

bool Dcn32Dmub::write(uint32_t reg, uint32_t value) noexcept {
    return hardware::write_ip_register(io_, space_.segment2_dword_base, reg, value);
}

bool Dcn32Dmub::update(uint32_t reg, uint32_t field, uint32_t value) noexcept {
    uint32_t current = 0;
    return read(reg, current) && write(reg, (current & ~field) | (value & field));
}

DmubResult Dcn32Dmub::fail() noexcept {
    quarantined_ = true;
    ready_ = false;
    return DmubResult::quarantined;
}

// dmub_dcn32_reset.
bool Dcn32Dmub::reset() noexcept {
    uint32_t control2 = 0;
    if (!read(dmcub_cntl2, control2)) {
        return false;
    }
    if ((control2 & soft_reset) == 0) {
        if (!write(gpint_datain1, gpint_stop_fw)) {
            return false;
        }
        const uint32_t acknowledged = gpint_stop_fw & 0x0fffffffu;
        for (uint32_t poll = 0; poll < gpint_polls; ++poll) {
            uint32_t value = 0;
            if (read(gpint_datain1, value) && value == acknowledged) {
                break;
            }
        }
        for (uint32_t poll = 0; poll < gpint_polls; ++poll) {
            uint32_t value = 0;
            if (read(scratch7, value) && value == gpint_stop_response) {
                break;
            }
        }
    }
    return update(dmcub_cntl2, soft_reset, soft_reset) && update(dmcub_cntl, dmcub_enable, 0) &&
           update(mmhubbub_soft_reset, dmuif_soft_reset, dmuif_soft_reset) &&
           write(inbox1_rptr, 0) && write(inbox1_wptr, 0) && write(outbox1_rptr, 0) &&
           write(outbox1_wptr, 0) && write(outbox0_rptr, 0) && write(outbox0_wptr, 0) &&
           write(scratch0, 0) && write(gpint_datain1, 0);
}

bool Dcn32Dmub::prepare_memory(ByteView firmware, ByteView vbios) noexcept {
    const DmubRegion& bss = layout_.windows[static_cast<uint32_t>(DmubWindow::bss_data)];
    const DmubRegion& bios = layout_.windows[static_cast<uint32_t>(DmubWindow::vbios)];
    if (layout_.bss_data_bytes != 0) {
        ByteView data(nullptr, 0);
        if (!firmware.subview(layout_.bss_data_file_offset, layout_.bss_data_bytes, data) ||
            !memory_.write(memory_.context, bss.base, data)) {
            return false;
        }
    }
    if (!memory_.write(memory_.context, bios.base, vbios)) {
        return false;
    }
    const DmubWindow cleared[]{DmubWindow::mailbox, DmubWindow::trace_buffer, DmubWindow::fw_state,
                               DmubWindow::shared_state};
    for (DmubWindow window : cleared) {
        const DmubRegion& region = layout_.windows[static_cast<uint32_t>(window)];
        if (!memory_.fill_zero(memory_.context, region.base, region.top - region.base)) {
            return false;
        }
    }
    return memory_.flush(memory_.context);
}

bool Dcn32Dmub::program_windows() noexcept {
    const DmubWindow windows[4]{DmubWindow::vbios, DmubWindow::mailbox, DmubWindow::trace_buffer,
                                DmubWindow::fw_state};
    const uint32_t bases[4]{cw3_base, cw4_base, cw5_base, cw6_base};
    for (uint32_t index = 0; index < 4; ++index) {
        const DmubRegion& region = layout_.windows[static_cast<uint32_t>(windows[index])];
        const uint64_t offset = memory_.gpu_address + region.base;
        const uint32_t base = bases[index];
        const uint32_t top = base + (region.top - region.base);
        if (!write(cw_offset[index], static_cast<uint32_t>(offset)) ||
            !write(cw_offset_high[index], static_cast<uint32_t>(offset >> 32)) ||
            !write(cw_base_address[index], base) ||
            !write(cw_top_address[index], (top & top_address_mask) | window_enable)) {
            return false;
        }
        if (windows[index] == DmubWindow::trace_buffer &&
            (!write(region5_offset, static_cast<uint32_t>(offset)) ||
             !write(region5_offset_high, static_cast<uint32_t>(offset >> 32)) ||
             !write(region5_top, ((top - base - 1u) & top_address_mask) | window_enable))) {
            return false;
        }
    }
    const DmubRegion& trace = layout_.windows[static_cast<uint32_t>(DmubWindow::trace_buffer)];
    const DmubRegion& mail = layout_.windows[static_cast<uint32_t>(DmubWindow::mailbox)];
    const uint32_t outbox1 = cw4_base + dmub_ring_bytes;
    // setup_outbox0, setup_mailbox(inbox1), setup_out_mailbox(outbox1).
    return write(outbox0_base, region5_base + trace_entry_offset) &&
           write(outbox0_size, (trace.top - trace.base) - trace_entry_offset) &&
           write(inbox1_base, cw4_base) && write(inbox1_size, dmub_ring_bytes) &&
           write(outbox1_base, outbox1) &&
           write(outbox1_size, cw4_base + (mail.top - mail.base) - outbox1);
}

DmubResult Dcn32Dmub::start(const hardware::RegisterIo& io, const Dcn32RegisterSpace& space,
                            ByteView firmware, ByteView vbios, const DmubMemory& memory) noexcept {
    if (ready_ || quarantined_ || !hardware::register_io_valid(io) || !space.discovery_verified ||
        space.dmu_major != 3 || space.dmu_minor != 2 || space.dmu_revision != 0 ||
        memory.write == nullptr || memory.read == nullptr || memory.fill_zero == nullptr ||
        memory.flush == nullptr || (memory.gpu_address & 0xfffu) != 0 || vbios.size() == 0 ||
        vbios.size() > maximum_window_bytes) {
        return DmubResult::invalid_argument;
    }
    DmubLayout layout{};
    const DmubResult computed = dmub_calc_layout(firmware, static_cast<uint32_t>(vbios.size()), layout);
    if (computed != DmubResult::success) {
        return computed;
    }
    if (memory.bytes < layout.framebuffer_bytes) {
        return DmubResult::invalid_argument;
    }
    io_ = io;
    space_ = space;
    memory_ = memory;
    layout_ = layout;

    uint32_t pipe_disable = 0;
    if (!read(cc_dc_pipe_dis, pipe_disable)) {
        return DmubResult::not_ready;
    }
    if ((pipe_disable & dmcub_supported) == 0) {
        return DmubResult::unsupported;
    }
    // From here every failure may have changed DMCUB or its VRAM.
    if (!prepare_memory(firmware, vbios) || !reset()) {
        return fail();
    }
    wptr_ = 0;
    rptr_ = 0;
    uint32_t options = 0;
    // enable_dmub_boot_options (z10 not disabled), skip_dmub_panel_power_sequence(false),
    // then dmub_dcn32_reset_release with psp_version 0 (no DMCU on DCN 3.2).
    if (!program_windows() || !write(scratch14, 0) || !read(scratch14, options) ||
        !write(scratch14, options & ~skip_phy_init_panel_sequence) ||
        !update(mmhubbub_soft_reset, dmuif_soft_reset, 0) ||
        !write(scratch15, 0u & psp_version_mask) ||
        !update(dmcub_cntl, dmcub_enable | traceport_enable, dmcub_enable | traceport_enable) ||
        !update(dmcub_cntl2, soft_reset, 0)) {
        return fail();
    }
    for (uint32_t waited = 0; waited <= auto_load_timeout_us; waited += auto_load_step_us) {
        uint32_t boot = 0;
        if (read(scratch0, boot) &&
            (boot & (boot_dal_fw | boot_mailbox_ready)) == (boot_dal_fw | boot_mailbox_ready)) {
            ready_ = true;
            return DmubResult::success;
        }
        io_.delay_microseconds(io_.context, auto_load_step_us);
    }
    // Running but not proven booted: keep it quarantined, do not use the ring.
    fail();
    return DmubResult::timeout;
}

DmubResult Dcn32Dmub::stop() noexcept {
    if (!hardware::register_io_valid(io_)) {
        return DmubResult::not_ready;
    }
    ready_ = false;
    if (!reset()) {
        return fail();
    }
    wptr_ = 0;
    rptr_ = 0;
    // A proven reset clears an earlier quarantine of the ring (dmub_srv_hw_reset).
    quarantined_ = false;
    return DmubResult::success;
}

bool Dcn32Dmub::status(DmubStatus& output) noexcept {
    uint32_t control = 0;
    uint32_t control2 = 0;
    DmubStatus value{};
    if (!hardware::register_io_valid(io_) || !read(scratch0, value.boot_status) ||
        !read(inbox1_rptr, value.inbox_rptr) || !read(inbox1_wptr, value.inbox_wptr) ||
        !read(dmcub_cntl, control) || !read(dmcub_cntl2, control2)) {
        return false;
    }
    value.enabled = (control & dmcub_enable) != 0;
    value.soft_reset = (control2 & soft_reset) != 0;
    output = value;
    return true;
}

DmubResult Dcn32Dmub::execute(uint8_t (&command)[dmub_command_bytes], bool reply,
                              uint32_t timeout_us) noexcept {
    if (!ready()) {
        return quarantined_ ? DmubResult::quarantined : DmubResult::not_ready;
    }
    if (timeout_us == 0 || timeout_us > 1000000u) {
        return DmubResult::invalid_argument;
    }
    uint32_t rptr = 0;
    if (!read(inbox1_rptr, rptr) || rptr >= dmub_ring_bytes || (rptr % dmub_command_bytes) != 0) {
        return fail();
    }
    rptr_ = rptr;
    const uint32_t used = wptr_ >= rptr_ ? wptr_ - rptr_ : dmub_ring_bytes - (rptr_ - wptr_);
    if (used == dmub_ring_bytes - dmub_command_bytes) {
        return DmubResult::queue_full;
    }
    const uint32_t ring = layout_.windows[static_cast<uint32_t>(DmubWindow::mailbox)].base;
    const uint32_t slot = wptr_;
    if (!memory_.write(memory_.context, ring + slot, ByteView(command, dmub_command_bytes)) ||
        !memory_.flush(memory_.context)) {
        return fail();
    }
    wptr_ = (wptr_ + dmub_command_bytes) % dmub_ring_bytes;
    if (!write(inbox1_wptr, wptr_)) {
        return fail();
    }
    for (uint32_t waited = 0; waited <= timeout_us; ++waited) {
        if (!read(inbox1_rptr, rptr) || rptr > dmub_ring_bytes) {
            return fail();
        }
        if (rptr == wptr_) {
            rptr_ = rptr;
            if (reply && !memory_.read(memory_.context, ring + slot, command, dmub_command_bytes)) {
                return fail();
            }
            return DmubResult::success;
        }
        io_.delay_microseconds(io_.context, 1);
    }
    // A command was published and never consumed: DMCUB state is unknown.
    fail();
    return DmubResult::timeout;
}

} // namespace rx7900::display

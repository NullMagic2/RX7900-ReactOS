/*
Module: rx7900.hardware.psp_ring
Purpose: Encode PSP GPCOM frames/commands and observe real fence completion.
Created: 2026-10-21
Architecture: Bounded KM ring protocol; native owner supplies MMIO and VRAM.
Copyright 2016, 2017, 2020 Advanced Micro Devices, Inc. Adapted Linux v6.12
amdgpu_psp.c/.h, psp_v13_0.c, psp_gfx_if.h, amdgpu_gmc.c (vram_mc2pa) and
mp_13_0_0_offset.h. Notice and exact provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#include "psp_ring.hpp"

namespace rx7900::hardware {
namespace {

constexpr uint32_t c2pmsg_64 = 0x80;
constexpr uint32_t c2pmsg_67 = 0x83;
constexpr uint32_t c2pmsg_69 = 0x85;
constexpr uint32_t c2pmsg_70 = 0x86;
constexpr uint32_t c2pmsg_71 = 0x87;
constexpr uint32_t response_flag = 0x80000000u;
constexpr uint32_t ring_status_mask = 0x8000ffffu;
constexpr uint32_t ring_type_km = 2;
constexpr uint32_t destroy_rings = 0x00030000u;
constexpr uint32_t handshake_delay_microseconds = 20000;
constexpr uint32_t maximum_poll_iterations = 1000000;
constexpr uint32_t maximum_poll_microseconds = 1000;

constexpr uint32_t command_load_ta = 0x01;
constexpr uint32_t command_unload_ta = 0x02;
constexpr uint32_t command_load_asd = 0x04;
constexpr uint32_t command_setup_tmr = 0x05;
constexpr uint32_t command_load_ip_fw = 0x06;
constexpr uint32_t command_destroy_tmr = 0x07;
constexpr uint32_t command_load_toc = 0x20;
constexpr uint32_t command_autoload_rlc = 0x21;
constexpr uint32_t command_boot_config = 0x22;
constexpr uint32_t boot_config_set_command = 1;
constexpr uint32_t boot_config_get_command = 2;
constexpr uint32_t fw_type_register_list = 67;
constexpr uint32_t tmr_flag_virt_phy_addr = 0x2;

constexpr uint32_t command_offset = psp_ring_bytes;
constexpr uint32_t fence_offset = psp_ring_bytes + psp_command_bytes;
constexpr uint32_t frame_bytes = 64;
constexpr uint32_t frame_dwords = frame_bytes / 4;
constexpr uint32_t ring_dwords = psp_ring_bytes / 4;
constexpr uint32_t command_id_offset = 8;
constexpr uint32_t command_union_offset = 28;
constexpr uint32_t response_offset = 864;
constexpr uint32_t response_boot_config_offset = 64;

uint32_t low(uint64_t value) noexcept { return static_cast<uint32_t>(value); }
uint32_t high(uint64_t value) noexcept { return static_cast<uint32_t>(value >> 32); }

} // namespace

bool PspRing::read(uint32_t reg, uint32_t& value) noexcept {
    return read_ip_register(io_, registers_.segment0_dword_base(), reg, value) &&
           value != UINT32_MAX;
}

bool PspRing::write(uint32_t reg, uint32_t value) noexcept {
    return write_ip_register(io_, registers_.segment0_dword_base(), reg, value);
}

PspRingStatus PspRing::wait_mailbox(uint32_t value, uint32_t mask) noexcept {
    for (uint32_t iteration = 0; iteration < policy_.register_poll_iterations; ++iteration) {
        if (!read(c2pmsg_64, report_.raw_mailbox)) {
            return PspRingStatus::mmio_failure;
        }

        if ((report_.raw_mailbox & mask) == value) {
            return PspRingStatus::ready;
        }

        // A set response flag with a nonzero low status is a reported failure.
        if (mask == ring_status_mask && (report_.raw_mailbox & response_flag) != 0) {
            return PspRingStatus::unexpected_response;
        }

        io_.delay_microseconds(io_.context, 1);
    }

    return PspRingStatus::timeout;
}

PspRingStatus PspRing::quarantine(PspRingStatus reason) noexcept {
    phase_ = Phase::faulted;
    report_.status = reason;
    return reason;
}

PspRingStatus PspRing::initialize(PspRegisters registers, RegisterIo io, Nbio& nbio,
                                  VramLocation vram, PspRingPolicy policy,
                                  PspRingRecovery recovery) noexcept {
    if (phase_ == Phase::live || phase_ == Phase::faulted || report_.memory_retained ||
        report_.tmr_retained) {
        return PspRingStatus::busy;
    }

    phase_ = Phase::uninitialized;
    report_ = {};

    if (!registers.valid()) {
        report_.status = PspRingStatus::unsupported_ip;
        return report_.status;
    }

    if (!register_io_valid(io) || io.aperture_bytes != registers.aperture_bytes() ||
        vram.bytes == 0 || vram.mc_end < vram.mc_start ||
        policy.register_poll_iterations == 0 ||
        policy.register_poll_iterations > maximum_poll_iterations ||
        policy.fence_poll_iterations == 0 || policy.fence_poll_iterations > maximum_poll_iterations ||
        policy.fence_poll_microseconds == 0 ||
        policy.fence_poll_microseconds > maximum_poll_microseconds) {
        report_.status = PspRingStatus::unavailable;
        return report_.status;
    }

    registers_ = registers;
    io_ = io;
    nbio_ = &nbio;
    vram_ = vram;
    policy_ = policy;
    recovery_ = recovery;
    phase_ = Phase::ready;
    report_.status = PspRingStatus::ready;
    return report_.status;
}

bool PspRing::retain(memory::Handle handle, uint32_t minimum_bytes, uint64_t alignment,
                     bool cpu_visible, memory::AllocationInfo& output) noexcept {
    // Local VRAM MC range only: PSP receives these addresses directly.
    return manager_ != nullptr && retain_local_vram(*manager_, handle, vram_, minimum_bytes,
                                                    alignment, cpu_visible, output);
}

void PspRing::release_leases(bool include_tmr) noexcept {
    if (manager_ == nullptr) {
        return;
    }

    if (report_.memory_retained) {
        manager_->unretain(control_handle_);
        manager_->unretain(firmware_handle_);
        control_handle_ = {};
        firmware_handle_ = {};
        control_ = {};
        firmware_ = {};
        report_.memory_retained = false;
    }

    if (include_tmr && report_.tmr_retained) {
        manager_->unretain(tmr_handle_);
        tmr_handle_ = {};
        report_.tmr_retained = false;
        report_.tmr_mc_address = 0;
        report_.tmr_bytes = 0;
    }

    if (!report_.memory_retained && !report_.tmr_retained) {
        manager_ = nullptr;
    }
}

PspRingStatus PspRing::create(memory::Manager& manager, memory::Handle control,
                              memory::Handle firmware) noexcept {
    if (phase_ != Phase::ready || report_.memory_retained) {
        return phase_ == Phase::faulted ? PspRingStatus::quarantined : PspRingStatus::busy;
    }

    if (manager_ != nullptr && manager_ != &manager) {
        return PspRingStatus::busy;
    }

    manager_ = &manager;
    memory::AllocationInfo control_info{};
    memory::AllocationInfo firmware_info{};

    if (!retain(control, psp_control_bytes, psp_ring_bytes, true, control_info)) {
        release_leases(false);
        report_.status = PspRingStatus::invalid_memory;
        return report_.status;
    }

    if (!retain(firmware, psp_firmware_buffer_bytes, psp_firmware_buffer_bytes, true,
                firmware_info)) {
        manager.unretain(control);
        release_leases(false);
        report_.status = PspRingStatus::invalid_memory;
        return report_.status;
    }

    control_handle_ = control;
    firmware_handle_ = firmware;
    control_ = control_info;
    firmware_ = firmware_info;
    report_.memory_retained = true;
    const sdma::CommandBuffer bytes(control_.backing.cpu_bytes, psp_control_bytes);

    for (uint32_t offset = 0; offset < psp_control_bytes; offset += sizeof(uint32_t)) {
        bytes.write_u32(offset, 0);
    }

    // Nothing is published to PSP before C2PMSG_69; earlier failures release.
    if (!nbio_->flush_hdp()) {
        release_leases(false);
        report_.status = PspRingStatus::mmio_failure;
        return report_.status;
    }

    const auto idle = wait_mailbox(response_flag, response_flag);

    if (idle != PspRingStatus::ready) {
        release_leases(false);
        report_.status = idle;
        return report_.status;
    }

    const uint64_t ring = control_.gpu.virtual_address;

    if (!write(c2pmsg_69, low(ring)) || !write(c2pmsg_70, high(ring)) ||
        !write(c2pmsg_71, psp_ring_bytes) || !write(c2pmsg_64, ring_type_km << 16)) {
        return quarantine(PspRingStatus::mmio_failure);
    }

    io_.delay_microseconds(io_.context, handshake_delay_microseconds);
    const auto created = wait_mailbox(response_flag, ring_status_mask);

    if (created != PspRingStatus::ready) {
        return quarantine(created);
    }

    fence_value_ = 0;
    report_.ring_live = true;
    phase_ = Phase::live;
    report_.status = PspRingStatus::ring_created;
    return report_.status;
}

bool PspRing::stage(ByteView payload) noexcept {
    if (payload.size() == 0 || payload.size() > psp_firmware_buffer_bytes) {
        return false;
    }

    const sdma::CommandBuffer destination(firmware_.backing.cpu_bytes, psp_firmware_buffer_bytes);

    // psp_copy_fw: the whole buffer is cleared and the payload copied in front.
    for (size_t offset = 0; offset < psp_firmware_buffer_bytes; offset += sizeof(uint32_t)) {
        uint32_t value = 0;

        for (size_t byte = 0; byte < sizeof(value) && offset + byte < payload.size(); ++byte) {
            uint8_t part = 0;
            payload.read_u8(offset + byte, part);
            value |= static_cast<uint32_t>(part) << (byte * 8);
        }

        if (!destination.write_u32(offset, value)) {
            return false;
        }
    }

    return true;
}

PspRingStatus PspRing::submit(const Command& command) noexcept {
    if (phase_ != Phase::live) {
        return phase_ == Phase::faulted ? PspRingStatus::quarantined : PspRingStatus::busy;
    }

    const sdma::CommandBuffer control(control_.backing.cpu_bytes, psp_control_bytes);
    const sdma::CommandBuffer buffer = control.subbuffer(command_offset, psp_command_bytes);

    for (uint32_t offset = 0; offset < psp_command_bytes; offset += sizeof(uint32_t)) {
        buffer.write_u32(offset, 0);
    }

    buffer.write_u32(command_id_offset, command.id);

    for (uint32_t index = 0; index < command.word_count; ++index) {
        buffer.write_u32(command_union_offset + index * 4, command.words[index]);
    }

    report_.last_command = command.id;
    report_.last_fw_type = command.fw_type;
    report_.response_status = 0;
    report_.response_session = 0;
    report_.response_tmr_bytes = 0;
    report_.response_fw_address = 0;
    report_.response_boot_config = 0;

    if (++fence_value_ == 0) {
        fence_value_ = 1;
    }

    report_.last_fence = fence_value_;

    if (!read(c2pmsg_67, report_.write_pointer)) {
        return quarantine(PspRingStatus::mmio_failure);
    }

    // Linux maps any multiple of the ring size to frame zero and rejects frames
    // outside the ring; a write pointer inside a frame is also refused here.
    const uint32_t pointer = report_.write_pointer % ring_dwords;

    if (report_.write_pointer >= ring_dwords || pointer % frame_dwords != 0) {
        return quarantine(PspRingStatus::unexpected_response);
    }

    const uint32_t frame = pointer / frame_dwords * frame_bytes;
    const uint64_t command_mc = control_.gpu.virtual_address + command_offset;
    const uint64_t fence_mc = control_.gpu.virtual_address + fence_offset;

    for (uint32_t offset = 0; offset < frame_bytes; offset += sizeof(uint32_t)) {
        control.write_u32(frame + offset, 0);
    }

    control.write_u32(frame + 0, low(command_mc));
    control.write_u32(frame + 4, high(command_mc));
    control.write_u32(frame + 12, low(fence_mc));
    control.write_u32(frame + 16, high(fence_mc));
    control.write_u32(frame + 20, fence_value_);

    if (!nbio_->flush_hdp() ||
        !write(c2pmsg_67, (pointer + frame_dwords) % ring_dwords)) {
        return quarantine(PspRingStatus::mmio_failure);
    }

    // HDP6 has no read invalidate. The opaque delay callback separates each
    // uncached reload of the fence word from the previous one.
    const ByteView shared(control_.backing.cpu_bytes, psp_control_bytes);
    bool fenced = false;

    for (uint32_t iteration = 0; iteration < policy_.fence_poll_iterations; ++iteration) {
        uint32_t fence = 0;

        if (shared.read_u32(fence_offset, fence) && fence == fence_value_) {
            fenced = true;
            break;
        }

        io_.delay_microseconds(io_.context, policy_.fence_poll_microseconds);
    }

    if (!fenced) {
        return quarantine(PspRingStatus::timeout);
    }

    const size_t response = command_offset + response_offset;
    uint32_t address_low = 0;
    uint32_t address_high = 0;
    shared.read_u32(response, report_.response_status);
    shared.read_u32(response + 4, report_.response_session);
    shared.read_u32(response + 8, address_low);
    shared.read_u32(response + 12, address_high);
    shared.read_u32(response + 16, report_.response_tmr_bytes);
    shared.read_u32(response + response_boot_config_offset, report_.response_boot_config);
    report_.response_fw_address = (static_cast<uint64_t>(address_high) << 32) | address_low;
    ++report_.completed_commands;
    report_.status = report_.response_status == 0 ? PspRingStatus::completed :
                                                    PspRingStatus::command_failed;
    return report_.status;
}

PspRingStatus PspRing::load_toc(ByteView toc, uint32_t& tmr_bytes) noexcept {
    if (phase_ != Phase::live) {
        return phase_ == Phase::faulted ? PspRingStatus::quarantined : PspRingStatus::busy;
    }

    if (!stage(toc)) {
        report_.status = PspRingStatus::invalid_payload;
        return report_.status;
    }

    const uint64_t address = firmware_address();
    const auto status = submit({command_load_toc,
        {low(address), high(address), static_cast<uint32_t>(toc.size())}, 3, 0});

    if (status != PspRingStatus::completed) {
        return status;
    }

    if (report_.response_tmr_bytes == 0) {
        report_.status = PspRingStatus::unexpected_response;
        return report_.status;
    }

    tmr_bytes = report_.response_tmr_bytes;
    return status;
}

PspRingStatus PspRing::setup_tmr(memory::Handle tmr, uint32_t bytes) noexcept {
    if (phase_ != Phase::live || report_.tmr_retained) {
        return phase_ == Phase::faulted ? PspRingStatus::quarantined : PspRingStatus::busy;
    }

    memory::AllocationInfo allocation{};

    if (bytes == 0 || (bytes & (static_cast<uint32_t>(memory::page_bytes) - 1)) != 0 ||
        !retain(tmr, bytes, psp_tmr_alignment, false, allocation)) {
        report_.status = PspRingStatus::invalid_memory;
        return report_.status;
    }

    tmr_handle_ = tmr;
    report_.tmr_retained = true;
    report_.tmr_mc_address = allocation.gpu.virtual_address;
    report_.tmr_bytes = bytes;
    // amdgpu_gmc_vram_mc2pa: MC offset into VRAM plus MMHUB FB_OFFSET.
    const uint64_t physical = allocation.gpu.virtual_address - vram_.mc_start +
                              vram_.physical_offset;
    const uint64_t address = allocation.gpu.virtual_address;
    return submit({command_setup_tmr, {low(address), high(address), bytes,
        tmr_flag_virt_phy_addr, low(physical), high(physical)}, 6, 0});
}

PspRingStatus PspRing::load_ip_firmware(ByteView payload, uint32_t psp_type) noexcept {
    if (phase_ != Phase::live) {
        return phase_ == Phase::faulted ? PspRingStatus::quarantined : PspRingStatus::busy;
    }

    if (psp_type == 0 || !stage(payload)) {
        report_.status = PspRingStatus::invalid_payload;
        return report_.status;
    }

    const uint64_t address = firmware_address();
    return submit({command_load_ip_fw, {low(address), high(address),
        static_cast<uint32_t>(payload.size()), psp_type}, 4, psp_type});
}

PspRingStatus PspRing::autoload_rlc() noexcept {
    return submit({command_autoload_rlc, {}, 0, 0});
}

PspRingStatus PspRing::load_trusted_app(uint32_t load_type, ByteView binary,
                                         uint64_t shared_mc, uint32_t shared_bytes,
                                         uint32_t& session) noexcept {
    if (phase_ != Phase::live) {
        return phase_ == Phase::faulted ? PspRingStatus::quarantined : PspRingStatus::busy;
    }

    if (!stage(binary)) {
        report_.status = PspRingStatus::invalid_payload;
        return report_.status;
    }

    // psp_prep_ta_load_cmd_buf: app address/len, then the shared command buffer.
    const uint64_t address = firmware_address();
    const auto status = submit({load_type, {low(address), high(address),
        static_cast<uint32_t>(binary.size()), low(shared_mc), high(shared_mc), shared_bytes},
        6, 0});

    if (status == PspRingStatus::completed) {
        session = report_.response_session;
    }

    return status;
}

PspRingStatus PspRing::load_asd(ByteView asd, uint32_t& session) noexcept {
    return load_trusted_app(command_load_asd, asd, 0, 0, session);
}

PspRingStatus PspRing::load_ta(ByteView ta, uint64_t shared_mc, uint32_t shared_bytes,
                               uint32_t& session) noexcept {
    if (phase_ != Phase::live) {
        return phase_ == Phase::faulted ? PspRingStatus::quarantined : PspRingStatus::busy;
    }

    if (shared_bytes == 0 || (shared_mc & (psp_ta_shared_alignment - 1)) != 0 ||
        shared_mc < vram_.mc_start || shared_mc > vram_.mc_end ||
        shared_bytes - 1 > vram_.mc_end - shared_mc) {
        report_.status = PspRingStatus::invalid_memory;
        return report_.status;
    }

    return load_trusted_app(command_load_ta, ta, shared_mc, shared_bytes, session);
}

PspRingStatus PspRing::boot_config_get(uint32_t& raw_boot_config) noexcept {
    const auto status = submit({command_boot_config, {0, boot_config_get_command}, 2, 0});

    if (status == PspRingStatus::completed || status == PspRingStatus::command_failed) {
        raw_boot_config = report_.response_boot_config;
    }

    return status;
}

PspRingStatus PspRing::boot_config_set(uint32_t boot_config) noexcept {
    return submit({command_boot_config,
        {0, boot_config_set_command, boot_config, boot_config}, 4, 0});
}

PspRingStatus PspRing::load_register_list(ByteView register_list) noexcept {
    return load_ip_firmware(register_list, fw_type_register_list);
}

PspRingStatus PspRing::unload_ta(uint32_t session) noexcept {
    return submit({command_unload_ta, {session}, 1, 0});
}

PspRingStatus PspRing::destroy_tmr() noexcept {
    return submit({command_destroy_tmr, {}, 0, 0});
}

PspRingStatus PspRing::stop() noexcept {
    if (phase_ != Phase::live) {
        return phase_ == Phase::faulted ? PspRingStatus::quarantined : PspRingStatus::busy;
    }

    if (!write(c2pmsg_64, destroy_rings)) {
        return quarantine(PspRingStatus::mmio_failure);
    }

    io_.delay_microseconds(io_.context, handshake_delay_microseconds);
    const auto status = wait_mailbox(response_flag, response_flag);

    if (status != PspRingStatus::ready) {
        return quarantine(status);
    }

    release_leases(false);
    report_.ring_live = false;
    phase_ = Phase::stopped;
    report_.status = PspRingStatus::stopped;
    return report_.status;
}

PspRingStatus PspRing::release_after_reset() noexcept {
    if (!report_.memory_retained && !report_.tmr_retained && phase_ != Phase::faulted) {
        return PspRingStatus::busy;
    }

    if (recovery_.confirm_reset_quiescence == nullptr ||
        !recovery_.confirm_reset_quiescence(recovery_.context)) {
        return PspRingStatus::quarantined;
    }

    release_leases(true);
    report_.ring_live = false;
    phase_ = Phase::uninitialized;
    report_.status = PspRingStatus::released;
    return report_.status;
}

} // namespace rx7900::hardware

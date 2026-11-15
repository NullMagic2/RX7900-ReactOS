/*
Module: rx7900.hardware.psp_boot
Purpose: Stage bounded PSP payloads and execute real PSP13.0.0 mailbox commands.
Created: 2026-10-21
Architecture: Retains uncertain resources; no production MMIO backend is fabricated.
Copyright 2012, 2016, 2020 Advanced Micro Devices, Inc.
Adapted Linux v6.12 psp_v13_0.c/amdgpu_psp.c/amdgpu_psp.h and
mp_13_0_2_offset.h. Notice and exact provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#include "psp_boot.hpp"

namespace rx7900::hardware {
namespace {

constexpr uint32_t mailbox_command = 0x63;
constexpr uint32_t mailbox_address = 0x64;
constexpr uint32_t secure_os_version = 0x7a;
constexpr uint32_t secure_os_life = 0x91;
constexpr uint32_t mailbox_ready = 0x80000000;
constexpr uint32_t maximum_poll_iterations = 1000000;

bool fits(PspRegisters registers, uint32_t reg) noexcept {
    uint32_t offset = 0;
    return registers.valid() && discovery_register_offset(registers.segment0_dword_base(),
        reg, registers.aperture_bytes(), offset);
}

const FirmwarePart* part(const FirmwareImage& image, uint32_t identifier) noexcept {
    for (uint32_t index = 0; index < image.part_count; ++index) {
        if (image.parts[index].identifier == identifier) {
            return &image.parts[index];
        }
    }

    return nullptr;
}

} // namespace

DiscoveryStatus validate_psp_discovery(ByteView binary, uint32_t aperture,
                                     PspRegisters& output) noexcept {
    output = {};

    if (aperture == 0 || (aperture & 3) != 0) {
        return DiscoveryStatus::invalid_argument;
    }

    DiscoveredIp ip;
    const auto status = discover_ip(binary, {discovery_mp0_hardware_id, 0, 13, 0, 0, 1}, ip);

    if (status != DiscoveryStatus::validated) {
        return status;
    }

    uint32_t offset = 0;

    if (!discovery_register_offset(ip.segment(0), secure_os_life, aperture, offset)) {
        return DiscoveryStatus::invalid_argument;
    }

    output.ip_ = ip;
    output.aperture_bytes_ = aperture;
    return DiscoveryStatus::validated;
}

bool PspBoot::read(uint32_t reg, uint32_t& value) noexcept {
    if (!fits(registers_, reg)) {
        return false;
    }

    const uint64_t offset = (static_cast<uint64_t>(registers_.segment0_dword_base()) + reg) * 4;
    return io_.read32(io_.context, static_cast<uint32_t>(offset), value) && value != UINT32_MAX;
}

bool PspBoot::write(uint32_t reg, uint32_t value) noexcept {
    if (!fits(registers_, reg)) {
        return false;
    }

    const uint64_t offset = (static_cast<uint64_t>(registers_.segment0_dword_base()) + reg) * 4;
    return io_.write32(io_.context, static_cast<uint32_t>(offset), value);
}

PspBootStatus PspBoot::ready(uint32_t iterations) noexcept {
    for (uint32_t iteration = 0; iteration < iterations; ++iteration) {
        if (!read(mailbox_command, report_.raw_mailbox)) {
            return PspBootStatus::mmio_failure;
        }

        if (report_.raw_mailbox == mailbox_ready) {
            return PspBootStatus::bootloader_ready;
        }

        // The exact all-other-bits-zero contract is known for this generation.
        // Nonzero error details are not invented from PSP13.0.6+ conventions.
        if ((report_.raw_mailbox & mailbox_ready) != 0) {
            return PspBootStatus::unexpected_response;
        }

        io_.delay_microseconds(io_.context, 1);
    }

    return PspBootStatus::timeout;
}

PspBootStatus PspBoot::initialize(PspRegisters registers, PspBootIo io,
                                  PspWaitPolicy policy) noexcept {
    if (report_.staging_retained || phase_ == Phase::pending || phase_ == Phase::faulted) {
        return PspBootStatus::busy;
    }

    phase_ = Phase::uninitialized;
    report_ = {};

    if (!registers.valid()) {
        report_.status = PspBootStatus::unsupported_ip;
        return report_.status;
    }

    if (io.read32 == nullptr || io.write32 == nullptr || io.publish_staging == nullptr ||
        io.delay_microseconds == nullptr || (registers.aperture_bytes() & 3) != 0 ||
        !fits(registers, secure_os_life) || policy.ready_poll_iterations == 0 ||
        policy.ready_poll_iterations > maximum_poll_iterations ||
        policy.completion_poll_iterations == 0 ||
        policy.completion_poll_iterations > maximum_poll_iterations) {
        report_.status = PspBootStatus::unavailable;
        return report_.status;
    }

    registers_ = registers;
    io_ = io;
    policy_ = policy;

    if (!read(secure_os_life, report_.sos_life)) {
        report_.status = PspBootStatus::mmio_failure;
        return report_.status;
    }

    if (report_.sos_life != 0) {
        if (!read(secure_os_version, report_.sos_version)) {
            report_.status = PspBootStatus::mmio_failure;
            return report_.status;
        }

        phase_ = Phase::running;
        report_.status = PspBootStatus::already_running;
        return report_.status;
    }

    report_.status = ready(policy.ready_poll_iterations);

    if (report_.status == PspBootStatus::bootloader_ready) {
        phase_ = Phase::ready;
    }

    return report_.status;
}

bool PspBoot::build_plan(const FirmwareImage& image) noexcept {
    step_count_ = 0;
    next_step_ = 0;

    // PSP13.0.0's pinned function table has no IPKEYMGR bootloader hook.
    // Refuse a newer packaging requirement instead of silently skipping it.
    if (part(image, 11) != nullptr) {
        return false;
    }

    const struct { PspBootStep kind; uint32_t gate; uint32_t payload; uint32_t command; } order[]{
        {PspBootStep::key_database, 3, 3, 0x80000},
        // Pinned psp_v13_0_bootloader_load_spl passes psp->kdb, not psp->spl.
        {PspBootStep::spl_table, 5, 3, 0x10000000},
        {PspBootStep::system_driver, 2, 2, 0x10000},
        {PspBootStep::soc_driver, 7, 7, 0xb0000},
        {PspBootStep::interface_driver, 8, 8, 0xd0000},
        {PspBootStep::debug_driver, 9, 9, 0xc0000},
        {PspBootStep::ras_driver, 10, 10, 0xe0000},
        {PspBootStep::secure_os, 1, 1, 0x20000}
    };

    for (const auto& candidate : order) {
        if (part(image, candidate.gate) == nullptr) {
            continue;
        }

        const auto* payload = part(image, candidate.payload);

        if (payload == nullptr || payload->bytes > psp_staging_bytes || payload->bytes == 0) {
            return false;
        }

        steps_[step_count_++] = {candidate.kind, *payload, candidate.command};
    }

    return step_count_ >= 2 && steps_[step_count_ - 1].kind == PspBootStep::secure_os;
}

PspBootStatus PspBoot::prepare(ByteView firmware, memory::Manager& manager,
                               memory::Handle staging, PspMcAddress address) noexcept {
    if (phase_ == Phase::running) {
        return PspBootStatus::already_running;
    }

    if (phase_ != Phase::ready || report_.staging_retained) {
        return PspBootStatus::busy;
    }

    FirmwareImage image{};

    if (validate_firmware(firmware, FirmwareKind::psp_13_0, image) != FirmwareStatus::success ||
        !build_plan(image)) {
        report_.status = PspBootStatus::invalid_firmware;
        return report_.status;
    }

    memory::AllocationInfo allocation{};
    memory::Slice slice{staging, 0};
    sdma::GpuRange range{};

    if (manager.describe(staging, allocation) != memory::Status::success ||
        manager.resolve(slice, psp_staging_bytes, memory::Access::read_write, range) !=
            memory::Status::success || allocation.backing.cpu_bytes == nullptr ||
        allocation.backing.cpu_byte_count < psp_staging_bytes ||
        address.value != allocation.gpu.virtual_address ||
        (address.value & (psp_staging_bytes - 1)) != 0 ||
        manager.retain(staging, memory::Access::read_write) != memory::Status::success) {
        report_.status = PspBootStatus::invalid_staging;
        return report_.status;
    }

    manager_ = &manager;
    staging_ = staging;
    allocation_ = allocation;
    address_ = address;
    firmware_ = firmware;
    report_.staging_retained = true;
    report_.completed_steps = 0;
    report_.command_attempted = false;
    phase_ = Phase::prepared;
    report_.status = PspBootStatus::prepared;
    return report_.status;
}

bool PspBoot::stage(const Step& step) noexcept {
    ByteView payload(nullptr, 0);

    if (!firmware_.subview(step.payload.offset, step.payload.bytes, payload)) {
        return false;
    }

    sdma::CommandBuffer destination(allocation_.backing.cpu_bytes, psp_staging_bytes);

    for (size_t offset = 0; offset < psp_staging_bytes; offset += sizeof(uint32_t)) {
        uint32_t value = 0;

        if (offset < payload.size() && !payload.read_u32(offset, value)) {
            return false;
        }

        if (!destination.write_u32(offset, value)) {
            return false;
        }
    }

    return true;
}

PspBootStatus PspBoot::quarantine(PspBootStatus reason) noexcept {
    phase_ = Phase::faulted;
    report_.status = reason;
    return reason;
}

PspBootStatus PspBoot::submit_next() noexcept {
    if ((phase_ != Phase::prepared && phase_ != Phase::between_steps) ||
        !report_.staging_retained || next_step_ >= step_count_) {
        return phase_ == Phase::faulted ? PspBootStatus::quarantined : PspBootStatus::busy;
    }

    if (!read(secure_os_life, report_.sos_life)) {
        return quarantine(PspBootStatus::mmio_failure);
    }

    if (report_.sos_life != 0) {
        return quarantine(PspBootStatus::unexpected_response);
    }

    const auto idle_status = ready(policy_.ready_poll_iterations);

    if (idle_status != PspBootStatus::bootloader_ready) {
        return quarantine(idle_status);
    }

    const auto& step = steps_[next_step_];
    report_.step = step.kind;
    report_.payload_identifier = step.payload.identifier;
    report_.command_attempted = false;

    if (!stage(step) || !io_.publish_staging(io_.context, address_, psp_staging_bytes)) {
        return quarantine(PspBootStatus::invalid_staging);
    }

    const uint32_t encoded_address = static_cast<uint32_t>(address_.value >> 20);
    uint32_t observed = 0;

    if (!write(mailbox_address, encoded_address) || !read(mailbox_address, observed) ||
        observed != encoded_address) {
        return quarantine(PspBootStatus::mmio_failure);
    }

    report_.command_attempted = true;
    sos_delay_done_ = false;

    if (!write(mailbox_command, step.command)) {
        return quarantine(PspBootStatus::mmio_failure);
    }

    phase_ = Phase::pending;
    report_.status = PspBootStatus::submitted;
    return report_.status;
}

bool PspBoot::release_lease() noexcept {
    if (!report_.staging_retained || manager_ == nullptr) {
        return false;
    }

    const auto status = manager_->unretain(staging_);
    report_.staging_retained = false;
    manager_ = nullptr;
    staging_ = {};
    allocation_ = {};
    firmware_ = ByteView(nullptr, 0);
    return status == memory::Status::success;
}

PspBootStatus PspBoot::wait() noexcept {
    if (phase_ != Phase::pending || next_step_ >= step_count_) {
        return phase_ == Phase::faulted ? PspBootStatus::quarantined : PspBootStatus::busy;
    }

    const bool sos = steps_[next_step_].kind == PspBootStep::secure_os;

    if (sos && !sos_delay_done_) {
        io_.delay_microseconds(io_.context, 20000);
        sos_delay_done_ = true;
    }

    if (!sos) {
        const auto completion = ready(policy_.completion_poll_iterations);

        if (completion != PspBootStatus::bootloader_ready) {
            return quarantine(completion);
        }
    } else {
        bool alive = false;

        for (uint32_t iteration = 0; iteration < policy_.completion_poll_iterations; ++iteration) {
            if (!read(secure_os_life, report_.sos_life)) {
                return quarantine(PspBootStatus::mmio_failure);
            }

            if (report_.sos_life != 0) {
                alive = true;
                break;
            }

            if (!read(mailbox_command, report_.raw_mailbox)) {
                return quarantine(PspBootStatus::mmio_failure);
            }

            if ((report_.raw_mailbox & mailbox_ready) != 0 && report_.raw_mailbox != mailbox_ready) {
                return quarantine(PspBootStatus::unexpected_response);
            }

            io_.delay_microseconds(io_.context, 1);
        }

        if (!alive) {
            return quarantine(PspBootStatus::timeout);
        }

        if (!read(secure_os_version, report_.sos_version)) {
            return quarantine(PspBootStatus::mmio_failure);
        }
    }

    ++next_step_;
    ++report_.completed_steps;

    if (sos) {
        phase_ = Phase::running;

        if (!release_lease()) {
            return quarantine(PspBootStatus::quarantined);
        }

        report_.status = PspBootStatus::secure_os_running;
    } else {
        phase_ = Phase::between_steps;
        report_.status = PspBootStatus::component_completed;
    }

    return report_.status;
}

PspBootStatus PspBoot::release_idle_staging() noexcept {
    if ((phase_ != Phase::prepared && phase_ != Phase::between_steps) ||
        !report_.staging_retained) {
        return phase_ == Phase::faulted ? PspBootStatus::quarantined : PspBootStatus::busy;
    }

    if (!read(secure_os_life, report_.sos_life)) {
        return quarantine(PspBootStatus::mmio_failure);
    }

    if (report_.sos_life != 0) {
        return quarantine(PspBootStatus::unexpected_response);
    }

    const auto status = ready(policy_.ready_poll_iterations);

    if (status != PspBootStatus::bootloader_ready) {
        return quarantine(status);
    }

    if (!release_lease()) {
        return quarantine(PspBootStatus::quarantined);
    }

    phase_ = Phase::ready;
    report_.status = PspBootStatus::staging_released;
    return report_.status;
}

PspBootStatus PspBoot::release_after_reset() noexcept {
    if (!report_.staging_retained && phase_ != Phase::faulted) {
        return PspBootStatus::busy;
    }

    if (io_.confirm_reset_quiescence == nullptr ||
        !io_.confirm_reset_quiescence(io_.context)) {
        return PspBootStatus::quarantined;
    }

    if (report_.staging_retained && !release_lease()) {
        return quarantine(PspBootStatus::quarantined);
    }

    phase_ = Phase::uninitialized;
    report_.status = PspBootStatus::staging_released;
    return report_.status;
}

} // namespace rx7900::hardware

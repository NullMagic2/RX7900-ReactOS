/*
Module: rx7900.hardware.psp_boot
Purpose: Execute the bounded PSP13.0.0 bootloader/SOS mailbox protocol.
Created: 2026-10-21
Architecture: Native MMIO and retained PSP-visible staging; no ASIC reset or firmware ring.
*/
#pragma once

#include "firmware.hpp"
#include "discovery.hpp"
#include "manager.hpp"

namespace rx7900::hardware {

constexpr size_t psp_staging_bytes = 1024 * 1024;

enum class PspBootStatus {
    bootloader_ready,
    prepared,
    submitted,
    component_completed,
    secure_os_running,
    already_running,
    staging_released,
    unavailable,
    unsupported_ip,
    invalid_firmware,
    invalid_staging,
    busy,
    timeout,
    mmio_failure,
    unexpected_response,
    quarantined
};

enum class PspBootStep {
    key_database,
    spl_table,
    system_driver,
    soc_driver,
    interface_driver,
    debug_driver,
    ras_driver,
    secure_os
};

// Only validate_psp_discovery can create a valid token. Its copied segment0 is
// an MP0 instance0 dword base, distinct from GC bases, BAR addresses and CPU
// pointers. Default construction fails closed; there is no PCI-ID inference.
class PspRegisters final {
public:
    PspRegisters() noexcept = default;
    bool valid() const noexcept { return ip_.valid(); }
    uint32_t segment0_dword_base() const noexcept { return ip_.segment(0); }
    uint32_t aperture_bytes() const noexcept { return aperture_bytes_; }

private:
    friend DiscoveryStatus validate_psp_discovery(ByteView, uint32_t, PspRegisters&) noexcept;
    DiscoveredIp ip_{};
    uint32_t aperture_bytes_ = 0;
};

// Checked signatures/checksums/die/IP extents and a unique MP0 instance0 IP13.0.0
// are mandatory. Mailbox35/36/58/81 must fit the assigned BAR aperture. Copies
// validated bases, clearing output on failure; borrows no bytes after return.
// Native acquisition and device binding of binary/BAR remain owner obligations.
DiscoveryStatus validate_psp_discovery(ByteView binary, uint32_t register_aperture_bytes,
                                     PspRegisters& output) noexcept;

struct PspMcAddress final { uint64_t value; };

struct PspBootIo final {
    void* context;
    bool (*read32)(void*, uint32_t byte_offset, uint32_t&) noexcept;
    // True confirms an ordered native MMIO store; false may have reached the
    // device, so command-write failure always quarantines the staging lease.
    bool (*write32)(void*, uint32_t byte_offset, uint32_t) noexcept;
    // Complete CPU/HDP publication into the PSP-visible MC/GTT address domain.
    // A GFXHUB-only GPUVA mapping or raw DMA common buffer is insufficient.
    bool (*publish_staging)(void*, PspMcAddress, size_t byte_count) noexcept;
    // Native callback must wait at least the requested duration. SOS has a
    // mandatory 20 ms settling interval; bounded register polls wait 1 us.
    void (*delay_microseconds)(void*, uint32_t) noexcept;
    // Optional error recovery: true only after the native reset/power owner has
    // established that PSP cannot access this staging allocation. Never infer
    // this from READY or a planned reset after an ambiguous command/error.
    bool (*confirm_reset_quiescence)(void*) noexcept;
};

struct PspWaitPolicy final {
    uint32_t ready_poll_iterations;
    uint32_t completion_poll_iterations;
};

struct PspBootReport final {
    PspBootStatus status = PspBootStatus::unavailable;
    PspBootStep step = PspBootStep::key_database;
    uint32_t payload_identifier = 0;
    uint32_t completed_steps = 0;
    uint32_t raw_mailbox = 0;
    uint32_t sos_life = 0;
    uint32_t sos_version = 0;
    bool staging_retained = false;
    bool command_attempted = false;
};

// Serialized PASSIVE_LEVEL operations by the exclusive native device owner.
// Native MMIO, power/reset exclusion, manager and immutable firmware owner outlive
// an active operation. No ISR/DPC, allocation, exceptions or C++ runtime required.
//
// Linux boot order is KDB, optional SPL command, SYS, optional SOC/INTF/DBG/RAS,
// then SOS. The pinned PSP13 implementation supplies KDB bytes for the SPL command;
// the report exposes payload_identifier=3 for that step. TOC/RL/runtime IP firmware
// require later PSP ring commands and are not claimed loaded by this protocol.
//
// A completed mailbox or SOS-life transition is a hardware observation, not a
// software signature check or proof of whole-device firmware authentication.
// No authenticated flag or successful no-hardware-access fallback is provided.
class PspBoot final {
public:
    PspBoot() noexcept = default;
    PspBoot(const PspBoot&) = delete;
    PspBoot& operator=(const PspBoot&) = delete;

    PspBootStatus initialize(PspRegisters registers, PspBootIo io,
                              PspWaitPolicy policy) noexcept;

    // staging is a CPU-visible read/write allocation >=1 MiB, GPU/MC address
    // aligned1 MiB, owned and PSP-accessible through native MC/GTT setup. address
    // must match the allocation's native-established MC/GPUVA; a DMA bus address
    // or PCI BAR cannot substitute for it. Retains one manager lease until final
    // observed SOS completion, confirmed idle release or confirmed reset recovery.
    // Firmware bytes remain immutable/alive through the complete boot sequence.
    PspBootStatus prepare(ByteView firmware, memory::Manager& manager,
                           memory::Handle staging, PspMcAddress address) noexcept;
    PspBootStatus submit_next() noexcept;
    // Waits only for the submitted command, bounded by policy. On timeout or
    // unexpected status, ownership remains quarantined until native reset proof.
    PspBootStatus wait() noexcept;

    // Only between commands; confirms exact mailbox READY and no running SOS.
    // Releases staging, not firmware already accepted by PSP. No implicit undo.
    PspBootStatus release_idle_staging() noexcept;
    // A failed manager/backend destroy remains manager-owned quarantine even
    // after this class relinquishes its lease; native cleanup must retry it.
    PspBootStatus release_after_reset() noexcept;
    PspBootReport report() const noexcept { return report_; }

private:
    struct Step final {
        PspBootStep kind{};
        FirmwarePart payload{};
        uint32_t command = 0;
    };

    enum class Phase { uninitialized, ready, prepared, pending, between_steps, running, faulted };

    bool read(uint32_t reg, uint32_t& value) noexcept;
    bool write(uint32_t reg, uint32_t value) noexcept;
    PspBootStatus ready(uint32_t iterations) noexcept;
    PspBootStatus quarantine(PspBootStatus reason) noexcept;
    bool release_lease() noexcept;
    bool stage(const Step& step) noexcept;
    bool build_plan(const FirmwareImage& image) noexcept;

    PspRegisters registers_{};
    PspBootIo io_{};
    PspWaitPolicy policy_{};
    PspBootReport report_{};
    Phase phase_ = Phase::uninitialized;
    ByteView firmware_{nullptr, 0};
    memory::Manager* manager_ = nullptr;
    memory::Handle staging_{};
    memory::AllocationInfo allocation_{};
    PspMcAddress address_{};
    Step steps_[8]{};
    size_t step_count_ = 0;
    size_t next_step_ = 0;
    bool sos_delay_done_ = false;
};

} // namespace rx7900::hardware

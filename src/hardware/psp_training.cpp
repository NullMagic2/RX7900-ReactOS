/*
Module: rx7900.hardware.psp_training
Purpose: Sequence PSP13 BIST memory training (cold-boot long, resume short) and its data.
Created: 2026-10-21
Architecture: Bounded mailbox protocol; native callbacks preserve encroached VRAM.
Copyright 2016, 2020 Advanced Micro Devices, Inc. Adapted Linux v6.12 psp_v13_0.c
(memory_training, memory_training_send_msg), amdgpu_psp.c/.h and amdgpu_ttm.c
(training_data_block_init). Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#include "psp_training.hpp"
#include "vram_discovery.hpp"

namespace rx7900::hardware {
namespace {

constexpr uint32_t c2pmsg_35 = 0x63;
constexpr uint32_t c2pmsg_36 = 0x64;
constexpr uint32_t c2pmsg_81 = 0x91;
// PSP_BL__DRAM_LONG_TRAIN / PSP_BL__DRAM_SHORT_TRAIN (amdgpu_psp.h).
constexpr uint32_t long_train = 0x100000;
constexpr uint32_t short_train = 0x200000;
constexpr uint32_t ready_flag = 0x80000000u;
// MEM_TRAIN_SEND_MSG_TIMEOUT_US with 1 us register polls.
constexpr uint32_t training_poll_limit = 3000000;
constexpr uint64_t one_mib = 0x100000;

PspTrainingStatus finish(PspTrainingReport& report, PspTrainingStatus status) noexcept {
    report.status = status;
    return status;
}

// amdgpu_ttm_training_data_block_init: C2P at ALIGN(vram - reserve - 1 MiB, 1 MiB),
// P2C 0x8000 below the top; everything above the encroached range.
bool training_layout(uint64_t vram_bytes, uint32_t reserved_top_bytes,
                     PspTrainingReport& report) noexcept {
    if (vram_bytes < psp_training_encroached_bytes + reserved_top_bytes + 2 * one_mib ||
        reserved_top_bytes > vram_bytes) {
        return false;
    }

    report.c2p_offset = (vram_bytes - reserved_top_bytes - one_mib + one_mib - 1) & ~(one_mib - 1);
    report.p2c_offset = vram_bytes - psp_training_p2c_offset;
    return report.c2p_offset + psp_training_data_bytes <= report.p2c_offset &&
           (report.c2p_offset >> 20) <= UINT32_MAX;
}

// Common prefix of psp_v13_0_memory_training: the SOS sign of life, then the
// 16-byte P2C header (amdgpu_device_vram_access). completed means "train now".
PspTrainingStatus training_prologue(PspRegisters registers, const RegisterIo& io,
                                    uint64_t vram_bytes, uint32_t reserved_top_bytes,
                                    PspTrainingReport& report) noexcept {
    if (!registers.valid() || !register_io_valid(io) ||
        io.aperture_bytes != registers.aperture_bytes()) {
        return PspTrainingStatus::unavailable;
    }

    if (!training_layout(vram_bytes, reserved_top_bytes, report)) {
        return PspTrainingStatus::invalid_layout;
    }

    uint32_t sos_life = 0;

    if (!read_ip_register(io, registers.segment0_dword_base(), c2pmsg_81, sos_life) ||
        sos_life == UINT32_MAX) {
        return PspTrainingStatus::mmio_failure;
    }

    if (sos_life != 0) {
        return PspTrainingStatus::skipped_sos_alive;
    }

    uint8_t header[16]{};
    uint32_t window_high = UINT32_MAX;

    if (!read_vram_block(io, report.p2c_offset, header, sizeof(header), window_high)) {
        return PspTrainingStatus::mmio_failure;
    }

    for (uint32_t index = 0; index < 4; ++index) {
        ByteView(header, sizeof(header)).read_u32(index * 4, report.p2c_header[index]);
    }

    return PspTrainingStatus::completed;
}

// psp_v13_0_memory_training_send_msg: C2PMSG_36 = C2P >> 20, C2PMSG_35 = message,
// then up to MEM_TRAIN_SEND_MSG_TIMEOUT_US of C2PMSG_35 bit 31 polls.
PspTrainingStatus send_training_message(PspRegisters registers, const RegisterIo& io,
                                        uint32_t message, PspTrainingReport& report) noexcept {
    const uint32_t base = registers.segment0_dword_base();

    if (!write_ip_register(io, base, c2pmsg_36, static_cast<uint32_t>(report.c2p_offset >> 20)) ||
        !write_ip_register(io, base, c2pmsg_35, message)) {
        return PspTrainingStatus::mmio_failure;
    }

    report.message_sent = true;
    report.message = message;
    report.polls = 0;

    for (; report.polls < training_poll_limit; ++report.polls) {
        if (!read_ip_register(io, base, c2pmsg_35, report.mailbox) || report.mailbox == UINT32_MAX) {
            return PspTrainingStatus::mmio_failure;
        }

        if ((report.mailbox & ready_flag) != 0) {
            return PspTrainingStatus::completed;
        }

        io.delay_microseconds(io.context, 1);
    }

    return PspTrainingStatus::timeout;
}

// The SEND_LONG_MSG branch: the BIST range is saved, trained over, restored and
// published with an HDP flush. A timeout returns without restoring (as Linux),
// because training may still own the encroached range.
PspTrainingStatus long_training(PspRegisters registers, const RegisterIo& io, Nbio& nbio,
                                PspTrainingIo training, PspTrainingReport& report) noexcept {
    if (!training.save_encroached(training.context)) {
        return PspTrainingStatus::save_failed;
    }

    const PspTrainingStatus sent = send_training_message(registers, io, long_train, report);

    if (sent != PspTrainingStatus::completed) {
        return sent;
    }

    if (!training.restore_encroached(training.context) || !nbio.flush_hdp()) {
        return PspTrainingStatus::restore_failed;
    }

    return PspTrainingStatus::completed;
}

// PSP_MEM_TRAIN_SAVE: amdgpu_device_vram_access(p2c, sys_cache, train_data_size).
bool save_training_data(const RegisterIo& io, const PspTrainingReport& report,
                        PspTrainingData& data) noexcept {
    uint32_t window_high = UINT32_MAX;
    data.valid = read_vram_block(io, report.p2c_offset, data.bytes, psp_training_data_bytes,
                                 window_high);
    return data.valid;
}

uint32_t cache_dword(const PspTrainingData& data, uint32_t index) noexcept {
    uint32_t value = 0;
    ByteView(data.bytes, psp_training_data_bytes).read_u32(index * 4, value);
    return value;
}

} // namespace

PspTrainingStatus run_psp_long_training(PspRegisters registers, const RegisterIo& io, Nbio& nbio,
                                        uint64_t vram_bytes, uint32_t reserved_top_bytes,
                                        PspTrainingIo training, PspTrainingData& data,
                                        PspTrainingReport& report) noexcept {
    report = {};
    data.valid = false;

    if (training.save_encroached == nullptr || training.restore_encroached == nullptr) {
        return finish(report, PspTrainingStatus::unavailable);
    }

    const PspTrainingStatus prologue = training_prologue(registers, io, vram_bytes,
                                                         reserved_top_bytes, report);

    if (prologue != PspTrainingStatus::completed) {
        return finish(report, prologue);
    }

    // COLD_BOOT is SEND_LONG_MSG, which always implies SAVE; the empty CPU cache
    // never requests RESTORE or a short message.
    report.operations = psp_training_op_long | psp_training_op_save;
    const PspTrainingStatus trained = long_training(registers, io, nbio, training, report);

    if (trained != PspTrainingStatus::completed) {
        return finish(report, trained);
    }

    if (!save_training_data(io, report, data)) {
        return finish(report, PspTrainingStatus::readback_failed);
    }

    return finish(report, PspTrainingStatus::completed);
}

PspTrainingStatus run_psp_resume_training(PspRegisters registers, const RegisterIo& io, Nbio& nbio,
                                          uint64_t vram_bytes, uint32_t reserved_top_bytes,
                                          PspTrainingIo training, PspTrainingData& data,
                                          PspTrainingReport& report) noexcept {
    report = {};
    const PspTrainingStatus prologue = training_prologue(registers, io, vram_bytes,
                                                         reserved_top_bytes, report);

    if (prologue != PspTrainingStatus::completed) {
        return finish(report, prologue);
    }

    // The ops escalation of psp_v13_0_memory_training, in Linux order.
    uint32_t ops = psp_training_op_short | psp_training_op_restore;
    const uint32_t cache0 = cache_dword(data, 0);
    const uint32_t cache3 = cache_dword(data, 3);

    if ((ops & psp_training_op_restore) != 0 && cache0 != psp_training_signature) {
        ops |= psp_training_op_save;
    }

    if (report.p2c_header[0] == psp_training_signature &&
        !(cache0 == psp_training_signature && cache3 == report.p2c_header[3])) {
        ops |= psp_training_op_save;
    }

    if ((ops & psp_training_op_save) != 0 && report.p2c_header[0] != psp_training_signature) {
        ops |= psp_training_op_long;
    }

    if ((ops & psp_training_op_long) != 0) {
        ops = (ops & ~psp_training_op_short) | psp_training_op_save;
    }

    report.operations = ops;

    if ((ops & psp_training_op_long) != 0) {
        // The encroached save/restore callbacks are required before any write.
        if (training.save_encroached == nullptr || training.restore_encroached == nullptr) {
            return finish(report, PspTrainingStatus::unavailable);
        }

        const PspTrainingStatus trained = long_training(registers, io, nbio, training, report);

        if (trained != PspTrainingStatus::completed) {
            return finish(report, trained);
        }
    }

    if ((ops & psp_training_op_save) != 0 && !save_training_data(io, report, data)) {
        return finish(report, PspTrainingStatus::readback_failed);
    }

    if ((ops & psp_training_op_restore) != 0) {
        uint32_t window_high = UINT32_MAX;

        if (!write_vram_block(io, report.c2p_offset, ByteView(data.bytes, psp_training_data_bytes),
                              window_high)) {
            return finish(report, PspTrainingStatus::mmio_failure);
        }
    }

    // amdgpu_force_long_training defaults to 0: the short training message.
    if ((ops & psp_training_op_short) != 0) {
        const PspTrainingStatus sent = send_training_message(registers, io, short_train, report);

        if (sent != PspTrainingStatus::completed) {
            return finish(report, sent);
        }
    }

    return finish(report, PspTrainingStatus::completed);
}

} // namespace rx7900::hardware

/*
Module: rx7900.hardware.psp_training
Purpose: Run PSP13 two-stage DRAM training (cold-boot long, resume short) before SOS load.
Created: 2026-10-21
Architecture: PspRegisters/RegisterIo/Nbio reuse; native callbacks own the
bottom-of-VRAM save and restore because only they can map that range.
*/
#pragma once

#include "psp_boot.hpp"
#include "nbio.hpp"

namespace rx7900::hardware {

// Linux amdgpu_psp.h: BIST training overwrites the lowest 32 MiB of VRAM.
constexpr uint64_t psp_training_encroached_bytes = 0x2000000;
constexpr uint32_t psp_training_data_bytes = 0x1000;
constexpr uint32_t psp_training_p2c_offset = 0x8000;
constexpr uint32_t psp_training_signature = 0x54534942;
// enum psp_memory_training_ops: COLD_BOOT is SEND_LONG_MSG, RESUME is SEND_SHORT_MSG.
constexpr uint32_t psp_training_op_long = 0x1;
constexpr uint32_t psp_training_op_save = 0x2;
constexpr uint32_t psp_training_op_restore = 0x4;
constexpr uint32_t psp_training_op_short = 0x8;

enum class PspTrainingStatus {
    completed,
    skipped_sos_alive,
    unavailable,
    invalid_layout,
    save_failed,
    mmio_failure,
    timeout,
    restore_failed,
    readback_failed
};

struct PspTrainingIo final {
    void* context;
    // Copy VRAM [0, psp_training_encroached_bytes) to storage BIST cannot reach,
    // and copy it back. True only after every byte was copied; restore must
    // complete CPU stores before returning so the following HDP flush covers them.
    bool (*save_encroached)(void*) noexcept;
    bool (*restore_encroached)(void*) noexcept;
};

// Copied PSP-to-CPU training data (Linux mem_train_ctx.sys_cache): saved after a
// long training (or a resume SAVE) and written back to the C2P block on resume.
// valid only records that a SAVE filled it; the decisions read the bytes as
// Linux reads its zero-initialized sys_cache.
struct PspTrainingData final {
    uint8_t bytes[psp_training_data_bytes]{};
    bool valid = false;
};

struct PspTrainingReport final {
    PspTrainingStatus status = PspTrainingStatus::unavailable;
    uint64_t c2p_offset = 0;
    uint64_t p2c_offset = 0;
    uint32_t p2c_header[4]{};
    uint32_t mailbox = 0;
    uint32_t polls = 0;
    bool message_sent = false;
    // psp_memory_training_ops actually executed (resume path; cold boot is long|save).
    uint32_t operations = 0;
    // The PSP_BL__DRAM_* message sent last (0 when none).
    uint32_t message = 0;
};

// PASSIVE_LEVEL, exclusive owner, PSP bootloader idle and SOS not yet loaded.
// Layout follows amdgpu_ttm_training_data_block_init: C2P data 1 MiB aligned below
// the reserved top region, P2C data 0x8000 below the VRAM top. Sends
// PSP_BL__DRAM_LONG_TRAIN with C2PMSG_36 = C2P >> 20 and waits up to the Linux 3 s
// budget for C2PMSG_35 bit 31. A timeout returns without restoring (as Linux),
// because training may still own the encroached range.
PspTrainingStatus run_psp_long_training(PspRegisters registers, const RegisterIo& io, Nbio& nbio,
                                        uint64_t vram_bytes, uint32_t reserved_top_bytes,
                                        PspTrainingIo training, PspTrainingData& data,
                                        PspTrainingReport& report) noexcept;

// psp_v13_0_memory_training(PSP_MEM_TRAIN_RESUME) from psp_resume, same layout and
// preconditions as the long training: SOS alive skips; otherwise the P2C header
// is read and the Linux operation escalation applied: SHORT implies RESTORE; a
// cache without signature adds SAVE; a P2C header with signature that the cache
// does not match (dword 3) adds SAVE; SAVE without a P2C signature adds LONG,
// which replaces SHORT. LONG runs exactly as cold boot (encroached save, long
// message, restore, HDP flush) and needs the training callbacks (unavailable
// without them, before any write); SAVE copies psp_training_data_bytes of P2C
// into data; RESTORE writes data to C2P (amdgpu_device_vram_access); SHORT sends
// PSP_BL__DRAM_SHORT_TRAIN with C2PMSG_36 = C2P >> 20 and waits the same 3 s.
// report.operations records the executed set.
PspTrainingStatus run_psp_resume_training(PspRegisters registers, const RegisterIo& io, Nbio& nbio,
                                          uint64_t vram_bytes, uint32_t reserved_top_bytes,
                                          PspTrainingIo training, PspTrainingData& data,
                                          PspTrainingReport& report) noexcept;

} // namespace rx7900::hardware

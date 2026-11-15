/*
 * Copyright 2008 Advanced Micro Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Author: Stanislaw Skowronek
 */
/*
Module: rx7900.hardware.atom_interpreter
Purpose: Execute VBIOS ATOM command tables (ASIC_Init re-POST) with strict bounds.
Created: 2026-10-24 (adaptation; the upstream notice above is preserved)
Architecture: C++17 port of Linux v6.12 drivers/gpu/drm/amd/amdgpu/atom.c with
atom.h, atom-bits.h and atom-names.h (pinned with SHA-256 in
work/upstream-gpuinit/SHA256SUMS). Register access goes only through the shared
RegisterIo contract with the discovery_register_offset aperture check that every
other AsicInit protocol uses; all interpreter state lives in this device-owned
object, never on the kernel stack. Used by AsicInit::repost.
*/
#pragma once

#include "register_io.hpp"
#include "atom_bios.hpp"

namespace rx7900::hardware {

// atom.h opcode range: 1..ATOM_OP_CNT-1 execute, ATOM_OP_EOT ends a table.
constexpr uint8_t atom_opcode_count = 127;
constexpr uint8_t atom_opcode_eot = 91;
// atom.h ATOM_ARG_* operand kinds (attribute bits 2:0 select the source kind).
enum class AtomArgument : uint8_t { reg = 0, ps = 1, ws = 2, fb = 3, id = 4, imm = 5, pll = 6, mc = 7 };
// atom.h ATOM_WS_* special workspace indices.
constexpr uint8_t atom_ws_quotient = 0x40;
constexpr uint8_t atom_ws_remainder = 0x41;
constexpr uint8_t atom_ws_dataptr = 0x42;
constexpr uint8_t atom_ws_shift = 0x43;
constexpr uint8_t atom_ws_or_mask = 0x44;
constexpr uint8_t atom_ws_and_mask = 0x45;
constexpr uint8_t atom_ws_fb_window = 0x46;
constexpr uint8_t atom_ws_attributes = 0x47;
constexpr uint8_t atom_ws_regptr = 0x48;
// atom.h ATOM_IO_* modes: memory-mapped, PCI, SYSIO, or 0x80 | indirect port.
constexpr uint32_t atom_io_mm = 0;
constexpr uint32_t atom_io_pci = 1;
constexpr uint32_t atom_io_sysio = 2;
constexpr uint32_t atom_io_iio = 0x80;
// atom_master_list_of_command_functions_v2_1 asic_init (ATOM_CMD_INIT).
constexpr uint8_t atom_command_asic_init = 0;

// Fixed device-owned capacities. Linux allocates WS per call (kcalloc) and has
// no depth limit; here frames, workspaces, parameter space and the FB scratch
// are bounded arrays inside this object.
constexpr uint32_t atom_frame_capacity = 16;
// A u8 WS index addresses at most 256 dwords per frame.
constexpr uint32_t atom_workspace_dwords = 256;
// Parameter space shared by the root table and every nested call.
constexpr uint32_t atom_parameter_dwords = 64;
// FB scratch (amdgpu_atomfirmware_allocate_fb_scratch, 20 KiB default);
// larger firmware requests are clamped and reported.
constexpr uint32_t atom_scratch_capacity_bytes = 64u * 1024u;
// atom_index_iio: one u16 program slot per u8 program identifier.
constexpr uint32_t atom_iio_slots = 256;

// Failures that execute reports. completed is the only success. quarantined:
// an earlier execution found the ROM program structurally unsafe; the
// interpreter refuses every later execution until bind is called again.
enum class AtomStatus : uint8_t {
    completed,
    invalid_argument,
    invalid_rom,
    absent_table,
    unsupported_revision,
    // Code or data outside its table or the ROM image, an IIO program outside
    // its table, or a decoded instruction that does not fit its table.
    malformed_table,
    step_budget,
    delay_budget,
    call_depth,
    // A REG/IIO register index outside the owned register aperture.
    register_range,
    // The RegisterIo callback reported a failed or uncertain access.
    io_failure,
    // A condition Linux only logs (listed by AtomFault), with stop_on_tolerated.
    tolerated_fault,
    quarantined
};

// Conditions Linux v6.12 logs with pr_info/DRM_ERROR and then continues. By
// default this port continues with the identical result (counted in the
// report); AtomPolicy::stop_on_tolerated turns each into an abort instead.
enum class AtomFault : uint8_t {
    none,
    // PS/WS index at or beyond the frame's parameter/workspace size: reads give
    // 0xCDCDCDCD (masked), writes are dropped.
    ps_index,
    ws_index,
    // FB scratch index outside the scratch: reads 0, writes dropped.
    fb_range,
    // REG access while SET_PCI_PORT / SET_SYS_IO_PORT is selected: reads 0,
    // writes dropped ("PCI/SYSIO registers are not implemented").
    pci_port,
    sysio_port,
    // REG access through an indirect port with no IIO program: reads 0.
    iio_undefined,
    // IIO opcode that atom_iio_execute does not implement: returns 0.
    iio_opcode,
    // SWITCH case list entry that is neither a case nor the end marker:
    // execution continues at that byte.
    bad_case,
    // REPEAT, SAVE_REG, RESTORE_REG: Linux prints "unimplemented!" and
    // continues at the next byte.
    unimplemented_opcode,
    // Opcode 0 or >= ATOM_OP_CNT: Linux leaves the table and returns 0.
    invalid_opcode,
    // CALL_TABLE whose caller declares more PS than remains: the callee gets an
    // empty parameter space (Linux would pass a pointer beyond its buffer).
    ps_window
};

struct AtomPolicy final {
    // Instructions (and IIO operations) per execute, nested calls included.
    // Linux aborts a loop that keeps jumping to one target for 20 s.
    uint32_t max_steps;
    // Nested CALL_TABLE depth including the root table, <= atom_frame_capacity.
    uint32_t max_call_depth;
    // Total DELAY_MILLISEC/DELAY_MICROSEC time per execute (ATOM_CMD_TIMEOUT_SEC).
    uint64_t max_delay_us;
    bool stop_on_tolerated;
};

constexpr AtomPolicy atom_default_policy{4000000u, atom_frame_capacity, 20000000u, false};

// Decoded operation classes; the opcode (and Linux opcode_table argument)
// selects the destination kind, jump condition, port or delay unit.
enum class AtomOperation : uint8_t {
    invalid, move, and_op, or_op, shift_left, shift_right, mul, div, add, sub,
    set_port, set_reg_block, set_fb_base, compare, switch_op, jump, test, delay,
    call_table, repeat, clear, nop, eot, mask, post_card, beep, save_reg,
    restore_reg, set_data_block, xor_op, shl, shr, debug, process_ds, mul32, div32
};

// atom.c ATOM_COND_* (jump), ATOM_PORT_* (set_port) and ATOM_UNIT_* (delay).
constexpr uint8_t atom_cond_above = 0;
constexpr uint8_t atom_cond_above_or_equal = 1;
constexpr uint8_t atom_cond_always = 2;
constexpr uint8_t atom_cond_below = 3;
constexpr uint8_t atom_cond_below_or_equal = 4;
constexpr uint8_t atom_cond_equal = 5;
constexpr uint8_t atom_cond_not_equal = 6;
constexpr uint8_t atom_port_ati = 0;
constexpr uint8_t atom_port_pci = 1;
constexpr uint8_t atom_port_sysio = 2;
constexpr uint8_t atom_unit_microsec = 0;
constexpr uint8_t atom_unit_millisec = 1;

// Code bytes [base, base + bytes) of one command table inside the ROM.
struct AtomCodeRange final {
    uint32_t base = 0;
    uint32_t bytes = 0;
};

struct AtomInstruction final {
    // ROM byte offset of the opcode and the instruction length in bytes.
    uint32_t offset = 0;
    uint32_t length = 0;
    uint8_t opcode = 0;
    AtomOperation operation = AtomOperation::invalid;
    // Linux opcode_table argument: destination AtomArgument, jump condition,
    // port kind or delay unit; zero for the other operations.
    uint8_t argument = 0;
    // Attribute byte for operations that have one, else zero.
    uint8_t attribute = 0;
    bool has_attribute = false;
    // First fixed operand: REG/ID index, table or data-block index, jump
    // target, delay count, port, register block, PROCESS_DS size, POST byte.
    uint32_t operand = 0;
    // SWITCH: number of case entries and whether the list stopped at a byte
    // that is neither a case nor the end marker (execution resumes there).
    uint32_t cases = 0;
    bool bad_case = false;
};

enum class AtomDecodeStatus : uint8_t { success, invalid_argument, truncated };

// Any IRQL; borrows rom only for the call. Decodes the instruction at offset,
// which must lie inside range (itself inside rom), with every operand byte
// inside range. Opcodes outside 1..126 decode as AtomOperation::invalid with
// length 1. Output is unchanged on failure.
AtomDecodeStatus atom_decode_instruction(ByteView rom, AtomCodeRange range, uint32_t offset,
                                         AtomInstruction& output) noexcept;

struct AtomExecReport final {
    AtomStatus status = AtomStatus::invalid_argument;
    uint8_t root_table = 0;
    // Where an abort happened (table index, ROM offset and opcode byte).
    uint8_t failed_table = 0;
    uint32_t failed_offset = 0;
    uint8_t failed_opcode = 0;
    uint32_t steps = 0;
    uint32_t calls = 0;
    uint32_t max_depth = 0;
    uint64_t delay_us = 0;
    uint32_t register_reads = 0;
    uint32_t register_writes = 0;
    uint32_t iio_operations = 0;
    // PLL/MC operands: amdgpu cail_pll_* / cail_mc_* read 0 and drop writes.
    uint32_t inert_pll_mc = 0;
    uint32_t tolerated_faults = 0;
    AtomFault first_fault = AtomFault::none;
};

// Optional diagnostic hook called before each instruction executes, with the
// nesting depth (1 = root). Must not call back into the interpreter.
struct AtomObserver final {
    void* context;
    void (*instruction)(void*, const AtomInstruction&, uint32_t depth) noexcept;
};

// Construct once in nonpaged device storage (it is about 85 KiB); trivially
// destructible. Serialized PASSIVE_LEVEL use by the exclusive device owner:
// DELAY_MILLISEC waits through RegisterIo::delay_microseconds. bind borrows
// the ROM image until the next bind or unbind; it must stay resident and
// immutable for that time (owners rebind before each use).
class AtomInterpreter final {
public:
    AtomInterpreter() noexcept = default;
    AtomInterpreter(const AtomInterpreter&) = delete;
    AtomInterpreter& operator=(const AtomInterpreter&) = delete;

    // amdgpu_atom_parse: BIOS 0xAA55 magic, " 761295520" ATI magic at 0x30,
    // the "ATOM" ROM header (validate_atom_bios), command and data master
    // tables, then atom_index_iio over the IndirectIOAccess data table. An IIO
    // program that does not end inside its table stops indexing there. scratch_bytes
    // (atom_allocate_fb_scratch) is clamped to atom_scratch_capacity_bytes.
    // Rebinding identical ROM content (size and FNV-1a 64 fingerprint) with the
    // same scratch size only re-points the view: FB scratch, the persistent
    // atom_context state and any quarantine survive, as Linux keeps one
    // atom_context per device. Other content starts from a zeroed scratch and
    // no quarantine. Fails with invalid_rom, leaving the interpreter unbound.
    AtomStatus bind(ByteView rom, uint32_t scratch_bytes) noexcept;
    void unbind() noexcept;
    bool bound() const noexcept { return bound_; }
    bool quarantined() const noexcept { return quarantined_; }
    uint32_t scratch_bytes() const noexcept { return scratch_dwords_ * 4u; }
    uint32_t iio_programs() const noexcept { return iio_programs_; }
    bool iio_truncated() const noexcept { return iio_truncated_; }

    // amdgpu_atom_parse_cmd_header plus the table extent: success only for a
    // nonzero master entry whose common header lies inside the ROM.
    AtomTableStatus command_table(uint8_t index, AtomTable& output) const noexcept;

    // Parameter space for the next execute (dword values, little-endian as
    // ATOM sees them). Index checks fail closed.
    bool set_parameter(uint32_t index, uint32_t value) noexcept;
    uint32_t parameter(uint32_t index) const noexcept;
    void clear_parameters() noexcept;

    // amdgpu_atom_execute_table: resets data block, register block, FB window,
    // IO mode and the quotient/remainder, then runs table with the first
    // parameter_dwords parameters. Every opcode of the v6.12 interpreter is
    // implemented with its exact operand semantics; deviations are bounds:
    // code fetches and jumps stay inside the executing table, ROM data reads
    // inside the image, registers inside io.aperture_bytes (Linux would fall
    // back to indirect PCIe access), plus the policy step/delay/depth budgets.
    // A structural failure (malformed_table, register_range, call_depth)
    // quarantines. Hardware writes made before an abort are not undone.
    AtomStatus execute(const RegisterIo& io, uint8_t table, uint32_t parameter_dwords,
                       AtomPolicy policy, AtomObserver observer = {nullptr, nullptr}) noexcept;

    // amdgpu_atomfirmware_asic_init: ASIC_Init must be command table format 2,
    // content >= 1 (else unsupported_revision), then runs with
    // asic_init_ps_allocation_v2_1 built by atom_asic_init_parameters.
    AtomStatus asic_init(const RegisterIo& io, uint32_t bootup_sclk_10khz, uint32_t bootup_mclk_10khz,
                         bool fb_reset, AtomPolicy policy) noexcept;

    AtomExecReport report() const noexcept { return report_; }

private:
    struct Frame final {
        uint32_t base = 0;
        uint32_t bytes = 0;
        uint32_t pc = 0;
        uint32_t ws_size = 0;
        uint32_t ps_offset = 0;
        uint32_t ps_dwords = 0;
        uint32_t ps_shift = 0;
        uint8_t table = 0;
    };

    bool fail(AtomStatus status) noexcept;
    bool tolerate(AtomFault fault) noexcept;
    bool push_frame(uint8_t table, uint32_t ps_offset, uint32_t ps_dwords) noexcept;
    bool step(const RegisterIo& io, AtomObserver observer) noexcept;
    bool code_u8(uint32_t& cursor, uint8_t& value) noexcept;
    bool code_u16(uint32_t& cursor, uint16_t& value) noexcept;
    bool code_u32(uint32_t& cursor, uint32_t& value) noexcept;
    bool code_direct(uint8_t align, uint32_t& cursor, uint32_t& value) noexcept;
    bool read_register(const RegisterIo& io, uint32_t index, uint32_t& value) noexcept;
    bool write_register(const RegisterIo& io, uint32_t index, uint32_t value) noexcept;
    bool iio_execute(const RegisterIo& io, uint32_t program, uint32_t index, uint32_t data,
                     uint32_t& result) noexcept;
    bool get_src_int(const RegisterIo& io, uint8_t attr, uint32_t& cursor, uint32_t* saved,
                     uint32_t& value) noexcept;
    bool get_src(const RegisterIo& io, uint8_t attr, uint32_t& cursor, uint32_t& value) noexcept;
    bool get_dst(const RegisterIo& io, uint8_t arg, uint8_t attr, uint32_t& cursor, uint32_t* saved,
                 uint32_t& value) noexcept;
    bool skip_dst(uint8_t arg, uint8_t attr, uint32_t& cursor) noexcept;
    bool put_dst(const RegisterIo& io, uint8_t arg, uint8_t attr, uint32_t cursor, uint32_t value,
                 uint32_t saved) noexcept;
    bool arithmetic(const RegisterIo& io, const AtomInstruction& instruction, uint32_t& cursor) noexcept;
    bool shift(const RegisterIo& io, const AtomInstruction& instruction, uint32_t& cursor) noexcept;
    bool divmul(const RegisterIo& io, const AtomInstruction& instruction, uint32_t& cursor) noexcept;
    bool flow(const AtomInstruction& instruction, uint32_t& cursor, bool& transferred) noexcept;
    bool switch_op(const RegisterIo& io, uint8_t attr, uint32_t& cursor, bool& transferred) noexcept;
    bool call(uint8_t table, uint32_t& cursor, bool& transferred) noexcept;
    bool data_block(uint8_t index) noexcept;
    bool delay(const RegisterIo& io, uint8_t unit, uint8_t count) noexcept;
    void index_iio() noexcept;
    void clear_iio() noexcept;

    ByteView rom_{nullptr, 0};
    uint32_t command_master_ = 0;
    uint32_t command_master_bytes_ = 0;
    uint32_t data_master_ = 0;
    uint32_t data_master_bytes_ = 0;
    uint16_t iio_[atom_iio_slots]{};
    // IndirectIOAccess table extent; every IIO program byte lies inside it.
    uint32_t iio_base_ = 0;
    uint32_t iio_end_ = 0;
    uint32_t iio_programs_ = 0;
    bool iio_truncated_ = false;
    bool bound_ = false;
    bool quarantined_ = false;
    uint64_t fingerprint_ = 0;
    AtomPolicy policy_ = atom_default_policy;
    AtomExecReport report_{};
    // The instruction being executed, recorded in the report on an abort.
    uint8_t current_table_ = 0;
    uint32_t current_offset_ = 0;
    uint8_t current_opcode_ = 0;
    // atom_context state shared by every frame (Linux keeps it in ctx->ctx).
    uint32_t data_block_ = 0;
    uint32_t reg_block_ = 0;
    uint32_t fb_base_ = 0;
    uint32_t quotient_ = 0;
    uint32_t remainder_ = 0;
    uint32_t io_attr_ = 0;
    uint32_t shift_ = 0;
    uint32_t io_mode_ = atom_io_mm;
    bool cs_equal_ = false;
    bool cs_above_ = false;
    Frame frames_[atom_frame_capacity]{};
    uint32_t depth_ = 0;
    uint32_t parameters_[atom_parameter_dwords]{};
    uint32_t workspace_[atom_frame_capacity][atom_workspace_dwords]{};
    uint32_t scratch_[atom_scratch_capacity_bytes / 4]{};
    uint32_t scratch_dwords_ = 0;
};

} // namespace rx7900::hardware

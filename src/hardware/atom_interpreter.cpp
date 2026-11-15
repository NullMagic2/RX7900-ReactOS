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
Purpose: Bounded ATOM bytecode decoder and interpreter (Linux v6.12 atom.c semantics).
Created: 2026-10-24 (adaptation; the upstream notice above is preserved)
Architecture: Iterative frames in device-owned arrays replace atom.c recursion;
every ROM, table, parameter, workspace, scratch and register index is checked.
Adapted from atom.c (opcode_table, atom_get_src_int, atom_put_dst, atom_op_*,
atom_iio_execute, atom_index_iio, amdgpu_atom_parse, amdgpu_atom_execute_table)
and amdgpu_atomfirmware.c (amdgpu_atomfirmware_asic_init).
Shift counts follow the x86 target (count modulo 32) where atom.c shifts by a
bytecode-controlled amount, which is undefined in C for 32 or more.
*/
#include "atom_interpreter.hpp"
#include "atom_firmware.hpp"

namespace rx7900::hardware {
namespace {

// atom.h ROM layout.
constexpr uint32_t atom_ati_magic_offset = 0x30;
constexpr char atom_ati_magic[] = " 761295520";
constexpr uint32_t atom_rom_cmd_pointer = 0x1e;
constexpr uint32_t atom_rom_data_pointer = 0x20;
constexpr uint32_t atom_data_iio_pointer = 0x32;
constexpr uint32_t atom_master_entries = 4;
constexpr uint32_t atom_ct_ws = 4;
constexpr uint32_t atom_ct_ps = 5;
constexpr uint8_t atom_ct_ps_mask = 0x7f;
constexpr uint32_t atom_ct_code = 6;
constexpr uint8_t atom_case_magic = 0x63;
constexpr uint16_t atom_case_end = 0x5a5a;
constexpr uint32_t atom_uninitialized = 0xcdcdcdcdu;

// atom.h ATOM_SRC_* alignments and the atom.c translation tables.
constexpr uint8_t atom_src_dword = 0;
constexpr uint8_t atom_src_byte0 = 4;
constexpr uint32_t atom_arg_mask[8] = {0xffffffffu, 0xffffu, 0xffff00u, 0xffff0000u,
                                       0xffu, 0xff00u, 0xff0000u, 0xff000000u};
constexpr uint32_t atom_arg_shift[8] = {0, 0, 8, 16, 0, 8, 16, 24};
constexpr uint8_t atom_dst_to_src[8][4] = {
    {0, 0, 0, 0}, {1, 2, 3, 0}, {1, 2, 3, 0}, {1, 2, 3, 0},
    {4, 5, 6, 7}, {4, 5, 6, 7}, {4, 5, 6, 7}, {4, 5, 6, 7},
};
constexpr uint8_t atom_def_dst[8] = {0, 0, 1, 2, 0, 1, 2, 3};

// atom.h ATOM_IIO_* and atom.c atom_iio_len.
constexpr uint8_t atom_iio_nop = 0;
constexpr uint8_t atom_iio_start = 1;
constexpr uint8_t atom_iio_read = 2;
constexpr uint8_t atom_iio_write = 3;
constexpr uint8_t atom_iio_clear = 4;
constexpr uint8_t atom_iio_set = 5;
constexpr uint8_t atom_iio_move_index = 6;
constexpr uint8_t atom_iio_move_attr = 7;
constexpr uint8_t atom_iio_move_data = 8;
constexpr uint8_t atom_iio_end = 9;
constexpr uint8_t atom_iio_len[10] = {1, 2, 3, 3, 3, 3, 4, 4, 4, 3};

struct OpcodeEntry final {
    AtomOperation operation;
    uint8_t argument;
};

// opcode_table destination kinds per six-opcode group: REG, PS, WS, FB, PLL, MC.
constexpr uint8_t group_argument[6] = {0, 1, 2, 3, 6, 7};
// opcode_table jump conditions for opcodes 67..73.
constexpr uint8_t jump_condition[7] = {atom_cond_always, atom_cond_equal, atom_cond_below,
                                       atom_cond_above, atom_cond_below_or_equal,
                                       atom_cond_above_or_equal, atom_cond_not_equal};

struct OpcodeGroup final {
    uint8_t first;
    AtomOperation operation;
};

constexpr OpcodeGroup six_groups[] = {
    {1, AtomOperation::move}, {7, AtomOperation::and_op}, {13, AtomOperation::or_op},
    {19, AtomOperation::shift_left}, {25, AtomOperation::shift_right}, {31, AtomOperation::mul},
    {37, AtomOperation::div}, {43, AtomOperation::add}, {49, AtomOperation::sub},
    {60, AtomOperation::compare}, {74, AtomOperation::test}, {84, AtomOperation::clear},
    {92, AtomOperation::mask}, {103, AtomOperation::xor_op}, {109, AtomOperation::shl},
    {115, AtomOperation::shr},
};

// Linux v6.12 opcode_table[ATOM_OP_CNT] as (operation, argument).
OpcodeEntry opcode_entry(uint8_t opcode) noexcept {
    for (const auto& group : six_groups) {
        if (opcode >= group.first && opcode < group.first + 6) {
            return {group.operation, group_argument[opcode - group.first]};
        }
    }

    if (opcode >= 55 && opcode <= 57) {
        return {AtomOperation::set_port, static_cast<uint8_t>(opcode - 55)};
    }

    if (opcode >= 67 && opcode <= 73) {
        return {AtomOperation::jump, jump_condition[opcode - 67]};
    }

    switch (opcode) {
    case 58: return {AtomOperation::set_reg_block, 0};
    case 59: return {AtomOperation::set_fb_base, 0};
    case 66: return {AtomOperation::switch_op, 0};
    case 80: return {AtomOperation::delay, atom_unit_millisec};
    case 81: return {AtomOperation::delay, atom_unit_microsec};
    case 82: return {AtomOperation::call_table, 0};
    case 83: return {AtomOperation::repeat, 0};
    case 90: return {AtomOperation::nop, 0};
    case 91: return {AtomOperation::eot, 0};
    case 98: return {AtomOperation::post_card, 0};
    case 99: return {AtomOperation::beep, 0};
    case 100: return {AtomOperation::save_reg, 0};
    case 101: return {AtomOperation::restore_reg, 0};
    case 102: return {AtomOperation::set_data_block, 0};
    case 121: return {AtomOperation::debug, 0};
    case 122: return {AtomOperation::process_ds, 0};
    case 123: return {AtomOperation::mul32, 1};
    case 124: return {AtomOperation::mul32, 2};
    case 125: return {AtomOperation::div32, 1};
    case 126: return {AtomOperation::div32, 2};
    default: return {AtomOperation::invalid, 0};
    }
}

uint32_t shl32(uint32_t value, uint32_t count) noexcept {
    return value << (count & 31u);
}

uint32_t shr32(uint32_t value, uint32_t count) noexcept {
    return value >> (count & 31u);
}

// 0xFFFFFFFF >> (32 - width) in atom_iio_execute.
uint32_t iio_field(uint8_t width) noexcept {
    return shr32(0xffffffffu, 32u - width);
}

// Immediate and direct operand widths for an ATOM_SRC_* alignment.
uint32_t direct_bytes(uint8_t align) noexcept {
    return align == atom_src_dword ? 4u : align < atom_src_byte0 ? 2u : 1u;
}

// atom_skip_src_int operand widths.
uint32_t operand_bytes(uint8_t argument, uint8_t align) noexcept {
    switch (static_cast<AtomArgument>(argument & 7u)) {
    case AtomArgument::reg:
    case AtomArgument::id:
        return 2;
    case AtomArgument::imm:
        return direct_bytes(align);
    default:
        return 1;
    }
}

uint32_t source_bytes(uint8_t attr) noexcept {
    return operand_bytes(attr & 7u, (attr >> 3) & 7u);
}

// Destinations are never ID or IMM (opcode_table), so the width is the kind's.
uint32_t destination_bytes(uint8_t argument) noexcept {
    return operand_bytes(argument, atom_src_dword);
}

bool in_range(AtomCodeRange range, uint32_t offset, uint32_t bytes) noexcept {
    return offset >= range.base && offset - range.base <= range.bytes &&
           bytes <= range.bytes - (offset - range.base);
}

// SWITCH case list from position, as atom_op_switch walks it.
AtomDecodeStatus decode_cases(ByteView rom, AtomCodeRange range, uint8_t attr, uint32_t& position,
                              AtomInstruction& instruction) noexcept {
    const uint32_t case_bytes = 1u + direct_bytes((attr >> 3) & 7u) + 2u;

    for (;;) {
        uint16_t marker = 0;
        uint8_t magic = 0;

        if (!in_range(range, position, 2) || !rom.read_u16(position, marker)) {
            return AtomDecodeStatus::truncated;
        }

        if (marker == atom_case_end) {
            position += 2;
            return AtomDecodeStatus::success;
        }

        rom.read_u8(position, magic);

        if (magic != atom_case_magic) {
            instruction.bad_case = true;
            return AtomDecodeStatus::success;
        }

        if (!in_range(range, position, case_bytes)) {
            return AtomDecodeStatus::truncated;
        }

        position += case_bytes;
        ++instruction.cases;
    }
}

bool is_structural(AtomStatus status) noexcept {
    return status == AtomStatus::malformed_table || status == AtomStatus::register_range ||
           status == AtomStatus::call_depth;
}

} // namespace

AtomDecodeStatus atom_decode_instruction(ByteView rom, AtomCodeRange range, uint32_t offset,
                                         AtomInstruction& output) noexcept {
    if (range.bytes == 0 || !rom.contains(range.base, range.bytes) || !in_range(range, offset, 1)) {
        return AtomDecodeStatus::invalid_argument;
    }

    AtomInstruction instruction{};
    uint8_t opcode = 0;
    rom.read_u8(offset, opcode);
    const OpcodeEntry entry = opcode_entry(opcode);
    instruction.offset = offset;
    instruction.opcode = opcode;
    instruction.operation = entry.operation;
    instruction.argument = entry.argument;
    uint32_t position = offset + 1;
    const auto attribute = [&]() {
        if (!in_range(range, position, 1)) {
            return false;
        }

        rom.read_u8(position, instruction.attribute);
        instruction.has_attribute = true;
        ++position;
        return true;
    };
    const auto fixed = [&](uint32_t bytes) {
        if (!in_range(range, position, bytes)) {
            return false;
        }

        if (bytes == 1) {
            uint8_t value = 0;
            rom.read_u8(position, value);
            instruction.operand = value;
        } else if (bytes == 2) {
            uint16_t value = 0;
            rom.read_u16(position, value);
            instruction.operand = value;
        }

        position += bytes;
        return true;
    };
    // A destination at the current position: its index is the first operand.
    const auto destination = [&]() { return fixed(destination_bytes(instruction.argument)); };
    const auto skip = [&](uint32_t bytes) {
        if (!in_range(range, position, bytes)) {
            return false;
        }

        position += bytes;
        return true;
    };
    bool fits = true;

    switch (entry.operation) {
    case AtomOperation::move:
    case AtomOperation::and_op:
    case AtomOperation::or_op:
    case AtomOperation::mul:
    case AtomOperation::div:
    case AtomOperation::add:
    case AtomOperation::sub:
    case AtomOperation::compare:
    case AtomOperation::test:
    case AtomOperation::xor_op:
    case AtomOperation::shl:
    case AtomOperation::shr:
    case AtomOperation::mul32:
    case AtomOperation::div32:
        fits = attribute() && destination() && skip(source_bytes(instruction.attribute));
        break;
    case AtomOperation::shift_left:
    case AtomOperation::shift_right:
        fits = attribute() && destination() && skip(1);
        break;
    case AtomOperation::clear:
        fits = attribute() && destination();
        break;
    case AtomOperation::mask:
        fits = attribute() && destination() &&
               skip(direct_bytes((instruction.attribute >> 3) & 7u)) &&
               skip(source_bytes(instruction.attribute));
        break;
    case AtomOperation::set_port:
        fits = fixed(entry.argument == atom_port_ati ? 2u : 1u);
        break;
    case AtomOperation::set_reg_block:
    case AtomOperation::jump:
        fits = fixed(2);
        break;
    case AtomOperation::set_fb_base:
        fits = attribute() && skip(source_bytes(instruction.attribute));
        break;
    case AtomOperation::switch_op:
        if (!attribute() || !skip(source_bytes(instruction.attribute))) {
            fits = false;
            break;
        }

        if (decode_cases(rom, range, instruction.attribute, position, instruction) !=
            AtomDecodeStatus::success) {
            return AtomDecodeStatus::truncated;
        }

        break;
    case AtomOperation::delay:
    case AtomOperation::call_table:
    case AtomOperation::post_card:
    case AtomOperation::set_data_block:
    case AtomOperation::debug:
        fits = fixed(1);
        break;
    case AtomOperation::process_ds:
        fits = fixed(2) && skip(instruction.operand);
        break;
    default:
        // REPEAT, NOP, EOT, BEEP, SAVE_REG, RESTORE_REG and invalid opcodes
        // consume only the opcode byte.
        break;
    }

    if (!fits) {
        return AtomDecodeStatus::truncated;
    }

    instruction.length = position - offset;
    output = instruction;
    return AtomDecodeStatus::success;
}

void AtomInterpreter::unbind() noexcept {
    rom_ = ByteView(nullptr, 0);
    command_master_ = 0;
    command_master_bytes_ = 0;
    data_master_ = 0;
    data_master_bytes_ = 0;
    clear_iio();
    bound_ = false;
    quarantined_ = false;
    fingerprint_ = 0;
    depth_ = 0;
    scratch_dwords_ = 0;
}

void AtomInterpreter::clear_iio() noexcept {
    for (auto& slot : iio_) {
        slot = 0;
    }

    iio_programs_ = 0;
    iio_base_ = 0;
    iio_end_ = 0;
    iio_truncated_ = false;
}

AtomStatus AtomInterpreter::bind(ByteView rom, uint32_t scratch_bytes) noexcept {
    AtomHeader header{};

    // validate_atom_bios checks the 0xAA55 BIOS magic and the ATOM header magic.
    if (validate_atom_bios(rom, header) != ParseStatus::success || header.reversed_signature ||
        !rom.contains(atom_ati_magic_offset, sizeof(atom_ati_magic) - 1)) {
        unbind();
        return AtomStatus::invalid_rom;
    }

    for (uint32_t index = 0; index + 1 < sizeof(atom_ati_magic); ++index) {
        uint8_t byte = 0;
        rom.read_u8(atom_ati_magic_offset + index, byte);

        if (byte != static_cast<uint8_t>(atom_ati_magic[index])) {
            unbind();
            return AtomStatus::invalid_rom;
        }
    }

    uint16_t command = 0;
    uint16_t data = 0;
    AtomTable command_table{};
    AtomTable data_table{};

    if (!rom.read_u16(header.offset + size_t{atom_rom_cmd_pointer}, command) ||
        !rom.read_u16(header.offset + size_t{atom_rom_data_pointer}, data) || command == 0 || data == 0 ||
        atom_table_at(rom, command, command_table) != AtomTableStatus::success ||
        atom_table_at(rom, data, data_table) != AtomTableStatus::success) {
        unbind();
        return AtomStatus::invalid_rom;
    }

    // FNV-1a 64 over the whole image identifies the content across rebinds.
    uint64_t fingerprint = 0xcbf29ce484222325ULL;

    for (size_t index = 0; index < rom.size(); ++index) {
        uint8_t byte = 0;
        rom.read_u8(index, byte);
        fingerprint = (fingerprint ^ byte) * 0x00000100000001b3ULL;
    }

    const uint32_t bytes = scratch_bytes < atom_scratch_capacity_bytes ? scratch_bytes :
                                                                         atom_scratch_capacity_bytes;
    const bool same = bound_ && fingerprint == fingerprint_ && rom.size() == rom_.size() &&
                      bytes / 4u == scratch_dwords_;
    rom_ = rom;
    command_master_ = command;
    command_master_bytes_ = static_cast<uint32_t>(command_table.bytes);
    data_master_ = data;
    data_master_bytes_ = static_cast<uint32_t>(data_table.bytes);
    clear_iio();
    index_iio();
    bound_ = true;

    if (same) {
        return AtomStatus::completed;
    }

    fingerprint_ = fingerprint;
    quarantined_ = false;
    scratch_dwords_ = bytes / 4u;

    for (auto& dword : scratch_) {
        dword = 0;
    }

    data_block_ = 0;
    reg_block_ = 0;
    fb_base_ = 0;
    quotient_ = 0;
    remainder_ = 0;
    io_attr_ = 0;
    shift_ = 0;
    io_mode_ = atom_io_mm;
    cs_equal_ = false;
    cs_above_ = false;
    clear_parameters();
    return AtomStatus::completed;
}

// atom_index_iio over the IndirectIOAccess data table (master entry at 0x32):
// START, program id, body until END (atom_iio_len widths), END plus two bytes.
// Only programs whose body ends inside the table are indexed.
void AtomInterpreter::index_iio() noexcept {
    uint16_t entry = 0;
    AtomTable table{};

    if (atom_data_iio_pointer + 2u > data_master_bytes_ ||
        !rom_.read_u16(data_master_ + size_t{atom_data_iio_pointer}, entry) || entry == 0) {
        return;
    }

    if (atom_table_at(rom_, entry, table) != AtomTableStatus::success) {
        iio_truncated_ = true;
        return;
    }

    iio_base_ = entry;
    iio_end_ = static_cast<uint32_t>(table.base + table.bytes);
    uint32_t position = iio_base_ + atom_master_entries;

    for (;;) {
        uint8_t opcode = 0;
        uint8_t identifier = 0;

        if (position >= iio_end_ || !rom_.read_u8(position, opcode) || opcode != atom_iio_start) {
            return;
        }

        if (position + 2u > iio_end_) {
            iio_truncated_ = true;
            return;
        }

        rom_.read_u8(position + 1u, identifier);
        const uint32_t program = position + 2u;
        uint32_t cursor = program;

        for (;;) {
            if (cursor >= iio_end_) {
                iio_truncated_ = true;
                return;
            }

            rom_.read_u8(cursor, opcode);

            if (opcode == atom_iio_end) {
                break;
            }

            if (opcode > atom_iio_end) {
                iio_truncated_ = true;
                return;
            }

            cursor += atom_iio_len[opcode];
        }

        if (cursor + atom_iio_len[atom_iio_end] > iio_end_ || program > UINT16_MAX) {
            iio_truncated_ = true;
            return;
        }

        if (iio_[identifier] == 0) {
            ++iio_programs_;
        }

        iio_[identifier] = static_cast<uint16_t>(program);
        position = cursor + atom_iio_len[atom_iio_end];
    }
}

AtomTableStatus AtomInterpreter::command_table(uint8_t index, AtomTable& output) const noexcept {
    const uint32_t entry_offset = atom_master_entries + 2u * index;
    uint16_t entry = 0;

    if (!bound_) {
        return AtomTableStatus::invalid_table;
    }

    if (entry_offset + 2u > command_master_bytes_ ||
        !rom_.read_u16(command_master_ + size_t{entry_offset}, entry)) {
        return AtomTableStatus::invalid_table;
    }

    if (entry == 0) {
        return AtomTableStatus::absent;
    }

    return atom_table_at(rom_, entry, output);
}

bool AtomInterpreter::set_parameter(uint32_t index, uint32_t value) noexcept {
    if (index >= atom_parameter_dwords) {
        return false;
    }

    parameters_[index] = value;
    return true;
}

uint32_t AtomInterpreter::parameter(uint32_t index) const noexcept {
    return index < atom_parameter_dwords ? parameters_[index] : 0;
}

void AtomInterpreter::clear_parameters() noexcept {
    for (auto& dword : parameters_) {
        dword = 0;
    }
}

bool AtomInterpreter::fail(AtomStatus status) noexcept {
    if (report_.status == AtomStatus::completed) {
        report_.status = status;
        report_.failed_table = current_table_;
        report_.failed_offset = current_offset_;
        report_.failed_opcode = current_opcode_;

        if (is_structural(status)) {
            quarantined_ = true;
        }
    }

    return false;
}

bool AtomInterpreter::tolerate(AtomFault fault) noexcept {
    ++report_.tolerated_faults;

    if (report_.first_fault == AtomFault::none) {
        report_.first_fault = fault;
    }

    return !policy_.stop_on_tolerated || fail(AtomStatus::tolerated_fault);
}

bool AtomInterpreter::push_frame(uint8_t table, uint32_t ps_offset, uint32_t ps_dwords) noexcept {
    AtomTable header{};
    uint8_t ws = 0;
    uint8_t ps = 0;

    if (depth_ >= policy_.max_call_depth) {
        return fail(AtomStatus::call_depth);
    }

    const AtomTableStatus found = command_table(table, header);

    if (found == AtomTableStatus::absent) {
        return fail(AtomStatus::absent_table);
    }

    if (found != AtomTableStatus::success || header.bytes <= atom_ct_code) {
        return fail(AtomStatus::malformed_table);
    }

    rom_.read_u8(header.base + atom_ct_ws, ws);
    rom_.read_u8(header.base + atom_ct_ps, ps);
    Frame& frame = frames_[depth_];
    frame.base = static_cast<uint32_t>(header.base);
    frame.bytes = static_cast<uint32_t>(header.bytes);
    frame.pc = frame.base + atom_ct_code;
    frame.ws_size = ws;
    frame.ps_offset = ps_offset;
    frame.ps_dwords = ps_dwords;
    frame.ps_shift = static_cast<uint32_t>(ps & atom_ct_ps_mask) / 4u;
    frame.table = table;

    // kcalloc: every call starts with a zeroed workspace.
    for (uint32_t index = 0; index < frame.ws_size; ++index) {
        workspace_[depth_][index] = 0;
    }

    ++depth_;

    if (depth_ > report_.max_depth) {
        report_.max_depth = depth_;
    }

    return true;
}

bool AtomInterpreter::code_u8(uint32_t& cursor, uint8_t& value) noexcept {
    const Frame& frame = frames_[depth_ - 1];

    if (!in_range({frame.base, frame.bytes}, cursor, 1) || !rom_.read_u8(cursor, value)) {
        return fail(AtomStatus::malformed_table);
    }

    cursor += 1;
    return true;
}

bool AtomInterpreter::code_u16(uint32_t& cursor, uint16_t& value) noexcept {
    const Frame& frame = frames_[depth_ - 1];

    if (!in_range({frame.base, frame.bytes}, cursor, 2) || !rom_.read_u16(cursor, value)) {
        return fail(AtomStatus::malformed_table);
    }

    cursor += 2;
    return true;
}

bool AtomInterpreter::code_u32(uint32_t& cursor, uint32_t& value) noexcept {
    const Frame& frame = frames_[depth_ - 1];

    if (!in_range({frame.base, frame.bytes}, cursor, 4) || !rom_.read_u32(cursor, value)) {
        return fail(AtomStatus::malformed_table);
    }

    cursor += 4;
    return true;
}

// atom_get_src_direct.
bool AtomInterpreter::code_direct(uint8_t align, uint32_t& cursor, uint32_t& value) noexcept {
    const uint32_t bytes = direct_bytes(align);

    if (bytes == 4) {
        return code_u32(cursor, value);
    }

    if (bytes == 2) {
        uint16_t word = 0;

        if (!code_u16(cursor, word)) {
            return false;
        }

        value = word;
        return true;
    }

    uint8_t byte = 0;

    if (!code_u8(cursor, byte)) {
        return false;
    }

    value = byte;
    return true;
}

// cail_reg_read/cail_reg_write (WREG32/RREG32 on a dword index), limited to the
// owned aperture exactly as read_ip_register/write_ip_register check it.
bool AtomInterpreter::read_register(const RegisterIo& io, uint32_t index, uint32_t& value) noexcept {
    uint32_t offset = 0;

    if (!discovery_register_offset(0, index, io.aperture_bytes, offset)) {
        return fail(AtomStatus::register_range);
    }

    if (!io.read32(io.context, offset, value)) {
        return fail(AtomStatus::io_failure);
    }

    ++report_.register_reads;
    return true;
}

bool AtomInterpreter::write_register(const RegisterIo& io, uint32_t index, uint32_t value) noexcept {
    uint32_t offset = 0;

    if (!discovery_register_offset(0, index, io.aperture_bytes, offset)) {
        return fail(AtomStatus::register_range);
    }

    if (!io.write32(io.context, offset, value)) {
        return fail(AtomStatus::io_failure);
    }

    ++report_.register_writes;
    return true;
}

// atom_iio_execute: program bytes stay inside the indexed IndirectIOAccess table.
bool AtomInterpreter::iio_execute(const RegisterIo& io, uint32_t program, uint32_t index, uint32_t data,
                                  uint32_t& result) noexcept {
    uint32_t temp = atom_uninitialized;
    uint32_t base = program;
    const auto byte = [&](uint32_t at) {
        uint8_t value = 0;
        rom_.read_u8(at, value);
        return value;
    };
    const auto insert = [&](uint32_t source) {
        const uint32_t field = iio_field(byte(base + 1));
        temp &= ~shl32(field, byte(base + 3));
        temp |= shl32(shr32(source, byte(base + 2)) & field, byte(base + 3));
    };

    for (;;) {
        uint8_t opcode = 0;

        if (report_.steps >= policy_.max_steps) {
            return fail(AtomStatus::step_budget);
        }

        ++report_.steps;
        ++report_.iio_operations;

        if (base < iio_base_ || base >= iio_end_ || !rom_.read_u8(base, opcode)) {
            return fail(AtomStatus::malformed_table);
        }

        const uint32_t width = opcode <= atom_iio_end ? atom_iio_len[opcode] : 1u;

        if (width > iio_end_ - base) {
            return fail(AtomStatus::malformed_table);
        }

        uint16_t reg = 0;

        switch (opcode) {
        case atom_iio_nop:
            break;
        case atom_iio_read:
            rom_.read_u16(base + 1u, reg);

            if (!read_register(io, reg, temp)) {
                return false;
            }

            break;
        case atom_iio_write:
            rom_.read_u16(base + 1u, reg);

            if (!write_register(io, reg, temp)) {
                return false;
            }

            break;
        case atom_iio_clear:
            temp &= ~shl32(iio_field(byte(base + 1)), byte(base + 2));
            break;
        case atom_iio_set:
            temp |= shl32(iio_field(byte(base + 1)), byte(base + 2));
            break;
        case atom_iio_move_index:
            insert(index);
            break;
        case atom_iio_move_data:
            insert(data);
            break;
        case atom_iio_move_attr:
            insert(io_attr_);
            break;
        case atom_iio_end:
            result = temp;
            return true;
        default:
            // Unknown IIO opcode (START inside a body included): returns 0.
            result = 0;
            return tolerate(AtomFault::iio_opcode);
        }

        base += width;
    }
}

bool AtomInterpreter::get_src_int(const RegisterIo& io, uint8_t attr, uint32_t& cursor, uint32_t* saved,
                                  uint32_t& value) noexcept {
    const uint8_t align = (attr >> 3) & 7u;
    const Frame& frame = frames_[depth_ - 1];
    uint32_t result = atom_uninitialized;
    uint8_t index8 = 0;
    uint16_t index16 = 0;

    switch (static_cast<AtomArgument>(attr & 7u)) {
    case AtomArgument::reg: {
        if (!code_u16(cursor, index16)) {
            return false;
        }

        const uint32_t index = index16 + reg_block_;

        if (io_mode_ == atom_io_mm) {
            if (!read_register(io, index, result)) {
                return false;
            }

            break;
        }

        // The PCI, SYSIO and undefined-IIO paths return 0 before saved/masking.
        value = 0;

        if (io_mode_ == atom_io_pci) {
            return tolerate(AtomFault::pci_port);
        }

        if (io_mode_ == atom_io_sysio) {
            return tolerate(AtomFault::sysio_port);
        }

        const uint16_t program = (io_mode_ & atom_io_iio) != 0 ? iio_[io_mode_ & 0x7fu] : 0;

        if (program == 0) {
            return tolerate(AtomFault::iio_undefined);
        }

        if (!iio_execute(io, program, index, 0, result)) {
            return false;
        }

        break;
    }
    case AtomArgument::ps:
        if (!code_u8(cursor, index8)) {
            return false;
        }

        if (index8 < frame.ps_dwords) {
            result = parameters_[frame.ps_offset + index8];
        } else if (!tolerate(AtomFault::ps_index)) {
            return false;
        }

        break;
    case AtomArgument::ws:
        if (!code_u8(cursor, index8)) {
            return false;
        }

        switch (index8) {
        case atom_ws_quotient: result = quotient_; break;
        case atom_ws_remainder: result = remainder_; break;
        case atom_ws_dataptr: result = data_block_; break;
        case atom_ws_shift: result = shift_; break;
        case atom_ws_or_mask: result = shl32(1u, shift_); break;
        case atom_ws_and_mask: result = ~shl32(1u, shift_); break;
        case atom_ws_fb_window: result = fb_base_; break;
        case atom_ws_attributes: result = io_attr_; break;
        case atom_ws_regptr: result = reg_block_; break;
        default:
            if (index8 < frame.ws_size) {
                result = workspace_[depth_ - 1][index8];
            } else if (!tolerate(AtomFault::ws_index)) {
                return false;
            }
        }

        break;
    case AtomArgument::id: {
        if (!code_u16(cursor, index16)) {
            return false;
        }

        // U32(idx + data_block): a data read anywhere in the ROM image only.
        if (!rom_.read_u32(size_t{index16} + data_block_, result)) {
            return fail(AtomStatus::malformed_table);
        }

        break;
    }
    case AtomArgument::fb: {
        if (!code_u8(cursor, index8)) {
            return false;
        }

        const uint32_t index = fb_base_ / 4u + index8;

        if (index < scratch_dwords_) {
            result = scratch_[index];
        } else {
            result = 0;

            if (!tolerate(AtomFault::fb_range)) {
                return false;
            }
        }

        break;
    }
    case AtomArgument::imm:
        // Immediates return unmasked and never set saved.
        return code_direct(align, cursor, value);
    case AtomArgument::pll:
    case AtomArgument::mc:
        if (!code_u8(cursor, index8)) {
            return false;
        }

        // cail_pll_read / cail_mc_read return 0 on amdgpu.
        result = 0;
        ++report_.inert_pll_mc;
        break;
    }

    if (saved != nullptr) {
        *saved = result;
    }

    value = (result & atom_arg_mask[align]) >> atom_arg_shift[align];
    return true;
}

bool AtomInterpreter::get_src(const RegisterIo& io, uint8_t attr, uint32_t& cursor, uint32_t& value) noexcept {
    return get_src_int(io, attr, cursor, nullptr, value);
}

bool AtomInterpreter::get_dst(const RegisterIo& io, uint8_t arg, uint8_t attr, uint32_t& cursor,
                              uint32_t* saved, uint32_t& value) noexcept {
    const uint8_t align = atom_dst_to_src[(attr >> 3) & 7u][(attr >> 6) & 3u];
    return get_src_int(io, static_cast<uint8_t>(arg | (align << 3)), cursor, saved, value);
}

bool AtomInterpreter::skip_dst(uint8_t arg, uint8_t attr, uint32_t& cursor) noexcept {
    const Frame& frame = frames_[depth_ - 1];
    const uint8_t align = atom_dst_to_src[(attr >> 3) & 7u][(attr >> 6) & 3u];
    const uint32_t bytes = operand_bytes(arg, align);

    if (!in_range({frame.base, frame.bytes}, cursor, bytes)) {
        return fail(AtomStatus::malformed_table);
    }

    cursor += bytes;
    return true;
}

bool AtomInterpreter::put_dst(const RegisterIo& io, uint8_t arg, uint8_t attr, uint32_t cursor,
                              uint32_t value, uint32_t saved) noexcept {
    const uint8_t align = atom_dst_to_src[(attr >> 3) & 7u][(attr >> 6) & 3u];
    const Frame& frame = frames_[depth_ - 1];
    const uint32_t merged = ((value << atom_arg_shift[align]) & atom_arg_mask[align]) |
                            (saved & ~atom_arg_mask[align]);
    uint8_t index8 = 0;
    uint16_t index16 = 0;

    switch (static_cast<AtomArgument>(arg & 7u)) {
    case AtomArgument::reg: {
        if (!code_u16(cursor, index16)) {
            return false;
        }

        const uint32_t index = index16 + reg_block_;

        if (io_mode_ == atom_io_mm) {
            // Register 0 (MM_INDEX) takes a byte address.
            return write_register(io, index, index == 0 ? merged << 2 : merged);
        }

        if (io_mode_ == atom_io_pci) {
            return tolerate(AtomFault::pci_port);
        }

        if (io_mode_ == atom_io_sysio) {
            return tolerate(AtomFault::sysio_port);
        }

        // Writes select iio[io_mode & 0xFF] (the 0x80 bit included), reads
        // iio[io_mode & 0x7F], exactly as atom.c indexes them.
        const uint16_t program = (io_mode_ & atom_io_iio) != 0 ? iio_[io_mode_ & 0xffu] : 0;
        uint32_t ignored = 0;

        if (program == 0) {
            return tolerate(AtomFault::iio_undefined);
        }

        return iio_execute(io, program, index, merged, ignored);
    }
    case AtomArgument::ps:
        if (!code_u8(cursor, index8)) {
            return false;
        }

        if (index8 >= frame.ps_dwords) {
            return tolerate(AtomFault::ps_index);
        }

        parameters_[frame.ps_offset + index8] = merged;
        return true;
    case AtomArgument::ws:
        if (!code_u8(cursor, index8)) {
            return false;
        }

        switch (index8) {
        case atom_ws_quotient: quotient_ = merged; return true;
        case atom_ws_remainder: remainder_ = merged; return true;
        case atom_ws_dataptr: data_block_ = merged & 0xffffu; return true;
        case atom_ws_shift: shift_ = merged & 0xffu; return true;
        case atom_ws_or_mask:
        case atom_ws_and_mask: return true;
        case atom_ws_fb_window: fb_base_ = merged; return true;
        case atom_ws_attributes: io_attr_ = merged & 0xffffu; return true;
        case atom_ws_regptr: reg_block_ = merged & 0xffffu; return true;
        default:
            if (index8 >= frame.ws_size) {
                return tolerate(AtomFault::ws_index);
            }

            workspace_[depth_ - 1][index8] = merged;
            return true;
        }
    case AtomArgument::fb: {
        if (!code_u8(cursor, index8)) {
            return false;
        }

        const uint32_t index = fb_base_ / 4u + index8;

        if (index >= scratch_dwords_) {
            return tolerate(AtomFault::fb_range);
        }

        scratch_[index] = merged;
        return true;
    }
    case AtomArgument::pll:
    case AtomArgument::mc:
        if (!code_u8(cursor, index8)) {
            return false;
        }

        // cail_pll_write / cail_mc_write do nothing on amdgpu.
        ++report_.inert_pll_mc;
        return true;
    default:
        // No opcode has an ID or IMM destination.
        return fail(AtomStatus::malformed_table);
    }
}

// MOVE, AND, OR, ADD, SUB, XOR, MASK, CLEAR, COMPARE and TEST.
bool AtomInterpreter::arithmetic(const RegisterIo& io, const AtomInstruction& instruction,
                                 uint32_t& cursor) noexcept {
    const uint8_t arg = instruction.argument;
    uint8_t attr = instruction.attribute;
    uint32_t saved = atom_uninitialized;
    uint32_t dst = 0;
    uint32_t src = 0;
    uint32_t mask = 0;
    const uint32_t dptr = cursor;

    switch (instruction.operation) {
    case AtomOperation::move:
        if (((attr >> 3) & 7u) != atom_src_dword) {
            if (!get_dst(io, arg, attr, cursor, &saved, dst)) {
                return false;
            }
        } else if (!skip_dst(arg, attr, cursor)) {
            return false;
        }

        return get_src(io, attr, cursor, src) && put_dst(io, arg, attr, dptr, src, saved);
    case AtomOperation::clear:
        attr &= 0x38u;
        attr = static_cast<uint8_t>(attr | (atom_def_dst[attr >> 3] << 6));
        return get_dst(io, arg, attr, cursor, &saved, dst) && put_dst(io, arg, attr, dptr, 0, saved);
    case AtomOperation::compare:
    case AtomOperation::test:
        if (!get_dst(io, arg, attr, cursor, nullptr, dst) || !get_src(io, attr, cursor, src)) {
            return false;
        }

        if (instruction.operation == AtomOperation::compare) {
            cs_equal_ = dst == src;
            cs_above_ = dst > src;
        } else {
            cs_equal_ = (dst & src) == 0;
        }

        return true;
    case AtomOperation::mask:
        if (!get_dst(io, arg, attr, cursor, &saved, dst) ||
            !code_direct((attr >> 3) & 7u, cursor, mask) || !get_src(io, attr, cursor, src)) {
            return false;
        }

        return put_dst(io, arg, attr, dptr, (dst & mask) | src, saved);
    default:
        break;
    }

    if (!get_dst(io, arg, attr, cursor, &saved, dst) || !get_src(io, attr, cursor, src)) {
        return false;
    }

    switch (instruction.operation) {
    case AtomOperation::and_op: dst &= src; break;
    case AtomOperation::or_op: dst |= src; break;
    case AtomOperation::add: dst += src; break;
    case AtomOperation::sub: dst -= src; break;
    default: dst ^= src; break;
    }

    return put_dst(io, arg, attr, dptr, dst, saved);
}

// SHIFT_LEFT/SHIFT_RIGHT (byte count, default destination alignment) and
// SHL/SHR (source count applied to the whole saved destination).
bool AtomInterpreter::shift(const RegisterIo& io, const AtomInstruction& instruction,
                            uint32_t& cursor) noexcept {
    const uint8_t arg = instruction.argument;
    uint8_t attr = instruction.attribute;
    uint32_t saved = atom_uninitialized;
    uint32_t dst = 0;
    uint32_t count = 0;
    const uint32_t dptr = cursor;
    const bool left = instruction.operation == AtomOperation::shift_left ||
                      instruction.operation == AtomOperation::shl;

    if (instruction.operation == AtomOperation::shift_left ||
        instruction.operation == AtomOperation::shift_right) {
        attr &= 0x38u;
        attr = static_cast<uint8_t>(attr | (atom_def_dst[attr >> 3] << 6));

        if (!get_dst(io, arg, attr, cursor, &saved, dst) || !code_direct(atom_src_byte0, cursor, count)) {
            return false;
        }

        dst = left ? shl32(dst, count) : shr32(dst, count);
        return put_dst(io, arg, attr, dptr, dst, saved);
    }

    const uint8_t align = atom_dst_to_src[(attr >> 3) & 7u][(attr >> 6) & 3u];

    if (!get_dst(io, arg, attr, cursor, &saved, dst) || !get_src(io, attr, cursor, count)) {
        return false;
    }

    // The u8 shift variable keeps the low byte of the source.
    count &= 0xffu;
    dst = left ? shl32(saved, count) : shr32(saved, count);
    dst = (dst & atom_arg_mask[align]) >> atom_arg_shift[align];
    return put_dst(io, arg, attr, dptr, dst, saved);
}

// MUL, DIV, MUL32 and DIV32 write only the quotient/remainder registers.
bool AtomInterpreter::divmul(const RegisterIo& io, const AtomInstruction& instruction,
                             uint32_t& cursor) noexcept {
    uint32_t dst = 0;
    uint32_t src = 0;

    if (!get_dst(io, instruction.argument, instruction.attribute, cursor, nullptr, dst) ||
        !get_src(io, instruction.attribute, cursor, src)) {
        return false;
    }

    switch (instruction.operation) {
    case AtomOperation::mul:
        quotient_ = dst * src;
        break;
    case AtomOperation::div:
        quotient_ = src != 0 ? dst / src : 0;
        remainder_ = src != 0 ? dst % src : 0;
        break;
    case AtomOperation::mul32: {
        const uint64_t product = static_cast<uint64_t>(dst) * src;
        quotient_ = static_cast<uint32_t>(product);
        remainder_ = static_cast<uint32_t>(product >> 32);
        break;
    }
    default: {
        // do_div of (remainder:dst) by src; both halves take the quotient.
        if (src == 0) {
            quotient_ = 0;
            remainder_ = 0;
            break;
        }

        const uint64_t quotient = ((static_cast<uint64_t>(remainder_) << 32) | dst) / src;
        quotient_ = static_cast<uint32_t>(quotient);
        remainder_ = static_cast<uint32_t>(quotient >> 32);
        break;
    }
    }

    return true;
}

bool AtomInterpreter::switch_op(const RegisterIo& io, uint8_t attr, uint32_t& cursor,
                                bool& transferred) noexcept {
    Frame& frame = frames_[depth_ - 1];
    uint32_t src = 0;

    if (!get_src(io, attr, cursor, src)) {
        return false;
    }

    for (;;) {
        uint16_t marker = 0;
        uint32_t probe = cursor;

        if (!code_u16(probe, marker)) {
            return false;
        }

        if (marker == atom_case_end) {
            cursor = probe;
            return true;
        }

        uint8_t magic = 0;
        probe = cursor;
        code_u8(probe, magic);

        if (magic != atom_case_magic) {
            // "Bad case": execution continues at this byte.
            return tolerate(AtomFault::bad_case);
        }

        cursor = probe;
        uint32_t value = 0;
        uint16_t target = 0;

        if (!get_src(io, static_cast<uint8_t>((attr & 0x38u) | static_cast<uint8_t>(AtomArgument::imm)),
                     cursor, value) || !code_u16(cursor, target)) {
            return false;
        }

        if (value == src) {
            if (target >= frame.bytes) {
                return fail(AtomStatus::malformed_table);
            }

            frame.pc = frame.base + target;
            transferred = true;
            return true;
        }
    }
}

// atom_op_calltable: the callee's PS starts ps_shift dwords into the caller's.
bool AtomInterpreter::call(uint8_t table, uint32_t& cursor, bool& transferred) noexcept {
    const uint32_t entry_offset = atom_master_entries + 2u * table;
    uint16_t entry = 0;

    if (entry_offset + 2u > command_master_bytes_ ||
        !rom_.read_u16(command_master_ + size_t{entry_offset}, entry)) {
        return fail(AtomStatus::malformed_table);
    }

    if (entry == 0) {
        return true;
    }

    Frame& caller = frames_[depth_ - 1];
    uint32_t offset = caller.ps_offset + caller.ps_dwords;
    uint32_t dwords = 0;

    if (caller.ps_shift <= caller.ps_dwords) {
        offset = caller.ps_offset + caller.ps_shift;
        dwords = caller.ps_dwords - caller.ps_shift;
    } else if (!tolerate(AtomFault::ps_window)) {
        return false;
    }

    caller.pc = cursor;
    ++report_.calls;

    if (!push_frame(table, offset, dwords)) {
        return false;
    }

    transferred = true;
    return true;
}

// atom_op_setdatablock: 0 clears, 255 selects the executing table itself.
bool AtomInterpreter::data_block(uint8_t index) noexcept {
    if (index == 0) {
        data_block_ = 0;
        return true;
    }

    if (index == 255) {
        data_block_ = frames_[depth_ - 1].base & 0xffffu;
        return true;
    }

    const uint32_t entry_offset = atom_master_entries + 2u * index;
    uint16_t entry = 0;

    if (entry_offset + 2u > data_master_bytes_ ||
        !rom_.read_u16(data_master_ + size_t{entry_offset}, entry)) {
        return fail(AtomStatus::malformed_table);
    }

    data_block_ = entry;
    return true;
}

// atom_op_delay: msleep/mdelay(count) or udelay(count) through the owner's wait.
bool AtomInterpreter::delay(const RegisterIo& io, uint8_t unit, uint8_t count) noexcept {
    const uint64_t microseconds = unit == atom_unit_millisec ? uint64_t{count} * 1000u : uint64_t{count};

    if (microseconds > policy_.max_delay_us - report_.delay_us) {
        return fail(AtomStatus::delay_budget);
    }

    report_.delay_us += microseconds;

    if (microseconds != 0) {
        io.delay_microseconds(io.context, static_cast<uint32_t>(microseconds));
    }

    return true;
}

// Control, block-selection and no-operation instructions.
bool AtomInterpreter::flow(const AtomInstruction& instruction, uint32_t& cursor, bool& transferred) noexcept {
    Frame& frame = frames_[depth_ - 1];
    uint8_t byte = 0;
    uint16_t word = 0;

    switch (instruction.operation) {
    case AtomOperation::set_port:
        if (instruction.argument == atom_port_ati) {
            if (!code_u16(cursor, word)) {
                return false;
            }

            io_mode_ = word == 0 ? atom_io_mm : (atom_io_iio | word);
            return true;
        }

        io_mode_ = instruction.argument == atom_port_pci ? atom_io_pci : atom_io_sysio;
        return code_u8(cursor, byte);
    case AtomOperation::set_reg_block:
        if (!code_u16(cursor, word)) {
            return false;
        }

        reg_block_ = word;
        return true;
    case AtomOperation::jump: {
        if (!code_u16(cursor, word)) {
            return false;
        }

        bool taken = false;

        switch (instruction.argument) {
        case atom_cond_above: taken = cs_above_; break;
        case atom_cond_above_or_equal: taken = cs_above_ || cs_equal_; break;
        case atom_cond_always: taken = true; break;
        case atom_cond_below: taken = !(cs_above_ || cs_equal_); break;
        case atom_cond_below_or_equal: taken = !cs_above_; break;
        case atom_cond_equal: taken = cs_equal_; break;
        default: taken = !cs_equal_; break;
        }

        if (taken) {
            if (word >= frame.bytes) {
                return fail(AtomStatus::malformed_table);
            }

            frame.pc = frame.base + word;
            transferred = true;
        }

        return true;
    }
    case AtomOperation::call_table:
        return code_u8(cursor, byte) && call(byte, cursor, transferred);
    case AtomOperation::set_data_block:
        return code_u8(cursor, byte) && data_block(byte);
    case AtomOperation::post_card:
    case AtomOperation::debug:
        return code_u8(cursor, byte);
    case AtomOperation::process_ds:
        if (!code_u16(cursor, word)) {
            return false;
        }

        if (!in_range({frame.base, frame.bytes}, cursor, word)) {
            return fail(AtomStatus::malformed_table);
        }

        cursor += word;
        return true;
    case AtomOperation::repeat:
    case AtomOperation::save_reg:
    case AtomOperation::restore_reg:
        return tolerate(AtomFault::unimplemented_opcode);
    case AtomOperation::eot:
        --depth_;
        transferred = true;
        return true;
    case AtomOperation::invalid:
        // The opcode loop breaks out of the table and returns success.
        if (!tolerate(AtomFault::invalid_opcode)) {
            return false;
        }

        --depth_;
        transferred = true;
        return true;
    default:
        // NOP and BEEP.
        return true;
    }
}

bool AtomInterpreter::step(const RegisterIo& io, AtomObserver observer) noexcept {
    Frame& frame = frames_[depth_ - 1];
    AtomInstruction instruction{};
    current_table_ = frame.table;
    current_offset_ = frame.pc;
    current_opcode_ = 0;
    rom_.read_u8(frame.pc, current_opcode_);

    if (report_.steps >= policy_.max_steps) {
        return fail(AtomStatus::step_budget);
    }

    ++report_.steps;

    if (atom_decode_instruction(rom_, {frame.base, frame.bytes}, frame.pc, instruction) !=
        AtomDecodeStatus::success) {
        return fail(AtomStatus::malformed_table);
    }

    if (observer.instruction != nullptr) {
        observer.instruction(observer.context, instruction, depth_);
    }

    uint32_t cursor = frame.pc + 1;
    bool transferred = false;
    bool executed = false;

    switch (instruction.operation) {
    case AtomOperation::move:
    case AtomOperation::and_op:
    case AtomOperation::or_op:
    case AtomOperation::add:
    case AtomOperation::sub:
    case AtomOperation::xor_op:
    case AtomOperation::mask:
    case AtomOperation::clear:
    case AtomOperation::compare:
    case AtomOperation::test:
        executed = code_u8(cursor, instruction.attribute) && arithmetic(io, instruction, cursor);
        break;
    case AtomOperation::shift_left:
    case AtomOperation::shift_right:
    case AtomOperation::shl:
    case AtomOperation::shr:
        executed = code_u8(cursor, instruction.attribute) && shift(io, instruction, cursor);
        break;
    case AtomOperation::mul:
    case AtomOperation::div:
    case AtomOperation::mul32:
    case AtomOperation::div32:
        executed = code_u8(cursor, instruction.attribute) && divmul(io, instruction, cursor);
        break;
    case AtomOperation::set_fb_base:
        executed = code_u8(cursor, instruction.attribute) && get_src(io, instruction.attribute, cursor, fb_base_);
        break;
    case AtomOperation::switch_op:
        executed = code_u8(cursor, instruction.attribute) &&
                   switch_op(io, instruction.attribute, cursor, transferred);
        break;
    case AtomOperation::delay: {
        uint8_t count = 0;
        executed = code_u8(cursor, count) && delay(io, instruction.argument, count);
        break;
    }
    default:
        executed = flow(instruction, cursor, transferred);
        break;
    }

    if (!executed) {
        return false;
    }

    // The executed operand stream must match the decoded instruction exactly.
    if (!transferred) {
        if (cursor != instruction.offset + instruction.length) {
            return fail(AtomStatus::malformed_table);
        }

        frame.pc = cursor;
    }

    return true;
}

AtomStatus AtomInterpreter::execute(const RegisterIo& io, uint8_t table, uint32_t parameter_dwords,
                                    AtomPolicy policy, AtomObserver observer) noexcept {
    report_ = {};
    report_.root_table = table;

    if (!bound_ || !register_io_valid(io) || parameter_dwords > atom_parameter_dwords ||
        policy.max_call_depth == 0 || policy.max_call_depth > atom_frame_capacity) {
        report_.status = AtomStatus::invalid_argument;
        return report_.status;
    }

    if (quarantined_) {
        report_.status = AtomStatus::quarantined;
        return report_.status;
    }

    report_.status = AtomStatus::completed;
    policy_ = policy;
    current_table_ = table;
    current_offset_ = 0;
    current_opcode_ = 0;
    // amdgpu_atom_execute_table resets these before every top-level table.
    data_block_ = 0;
    reg_block_ = 0;
    fb_base_ = 0;
    io_mode_ = atom_io_mm;
    quotient_ = 0;
    remainder_ = 0;
    depth_ = 0;

    bool running = push_frame(table, 0, parameter_dwords);

    while (running && depth_ != 0) {
        running = step(io, observer);
    }

    depth_ = 0;
    return report_.status;
}

AtomStatus AtomInterpreter::asic_init(const RegisterIo& io, uint32_t bootup_sclk_10khz,
                                      uint32_t bootup_mclk_10khz, bool fb_reset, AtomPolicy policy) noexcept {
    AtomTable table{};
    uint8_t format = 0;
    uint8_t content = 0;
    report_ = {};
    report_.root_table = atom_command_asic_init;

    if (!bound_) {
        report_.status = AtomStatus::invalid_argument;
        return report_.status;
    }

    const AtomTableStatus found = command_table(atom_command_asic_init, table);

    if (found != AtomTableStatus::success) {
        report_.status = found == AtomTableStatus::absent ? AtomStatus::absent_table : AtomStatus::invalid_rom;
        return report_.status;
    }

    // amdgpu_atom_parse_cmd_header reads the raw revision bytes.
    rom_.read_u8(table.base + 2, format);
    rom_.read_u8(table.base + 3, content);

    if (format != 2 || content < 1) {
        report_.status = AtomStatus::unsupported_revision;
        return report_.status;
    }

    const AtomAsicInitParameters ps = atom_asic_init_parameters(bootup_sclk_10khz, bootup_mclk_10khz, fb_reset);
    clear_parameters();
    set_parameter(0, ps.engine);
    set_parameter(1, ps.memory);
    return execute(io, atom_command_asic_init, atom_asic_init_parameter_dwords, policy);
}

} // namespace rx7900::hardware

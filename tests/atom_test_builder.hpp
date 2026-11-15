/*
Module: rx7900.tests.atom_test_builder
Purpose: Assemble tiny synthetic ATOM VBIOS images by encoder calls for interpreter tests.
Created: 2026-10-24
Architecture: Test-only encoder of the Linux v6.12 atom.c / atom.h / atom-bits.h
byte formats (ROM header, master tables, command and data tables, IIO programs),
written from the pinned upstream copies in work/upstream-gpuinit. It embeds no
binary fixture: every image is built from named calls and stays a few hundred
bytes. It is never linked into a driver and does not reuse the interpreter.
*/
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <vector>

namespace rx7900_test::atom {

// atom.h ATOM_ARG_* operand kinds.
enum class Arg : uint8_t { reg = 0, ps = 1, ws = 2, fb = 3, id = 4, imm = 5, pll = 6, mc = 7 };

// atom.h ATOM_SRC_* alignments.
constexpr uint8_t src_dword = 0;
constexpr uint8_t src_word0 = 1;
constexpr uint8_t src_word8 = 2;
constexpr uint8_t src_word16 = 3;
constexpr uint8_t src_byte0 = 4;
constexpr uint8_t src_byte8 = 5;
constexpr uint8_t src_byte16 = 6;
constexpr uint8_t src_byte24 = 7;

// atom.h ATOM_WS_* special workspace indices.
constexpr uint8_t ws_quotient = 0x40;
constexpr uint8_t ws_remainder = 0x41;
constexpr uint8_t ws_dataptr = 0x42;
constexpr uint8_t ws_shift = 0x43;
constexpr uint8_t ws_or_mask = 0x44;
constexpr uint8_t ws_and_mask = 0x45;
constexpr uint8_t ws_fb_window = 0x46;
constexpr uint8_t ws_attributes = 0x47;
constexpr uint8_t ws_regptr = 0x48;

// atom.c ATOM_COND_* in opcode_table order (opcodes 67..73).
enum class Cond : uint8_t { always, equal, below, above, below_or_equal, above_or_equal, not_equal };

// Linux v6.12 opcode_table numbers. Six-entry groups take the destination kind
// as an offset in the order REG, PS, WS, FB, PLL, MC.
namespace op {
constexpr uint8_t move = 1;
constexpr uint8_t and_ = 7;
constexpr uint8_t or_ = 13;
constexpr uint8_t shift_left = 19;
constexpr uint8_t shift_right = 25;
constexpr uint8_t mul = 31;
constexpr uint8_t div = 37;
constexpr uint8_t add = 43;
constexpr uint8_t sub = 49;
constexpr uint8_t set_port_ati = 55;
constexpr uint8_t set_port_pci = 56;
constexpr uint8_t set_port_sysio = 57;
constexpr uint8_t set_reg_block = 58;
constexpr uint8_t set_fb_base = 59;
constexpr uint8_t compare = 60;
constexpr uint8_t switch_ = 66;
constexpr uint8_t jump = 67;
constexpr uint8_t test = 74;
constexpr uint8_t delay_millisec = 80;
constexpr uint8_t delay_microsec = 81;
constexpr uint8_t call_table = 82;
constexpr uint8_t repeat = 83;
constexpr uint8_t clear = 84;
constexpr uint8_t nop = 90;
constexpr uint8_t eot = 91;
constexpr uint8_t mask = 92;
constexpr uint8_t post_card = 98;
constexpr uint8_t beep = 99;
constexpr uint8_t save_reg = 100;
constexpr uint8_t restore_reg = 101;
constexpr uint8_t set_data_block = 102;
constexpr uint8_t xor_ = 103;
constexpr uint8_t shl = 109;
constexpr uint8_t shr = 115;
constexpr uint8_t debug = 121;
constexpr uint8_t process_ds = 122;
constexpr uint8_t mul32_ps = 123;
constexpr uint8_t mul32_ws = 124;
constexpr uint8_t div32_ps = 125;
constexpr uint8_t div32_ws = 126;
} // namespace op

// atom.h ATOM_IIO_* opcodes.
namespace iio {
constexpr uint8_t nop = 0;
constexpr uint8_t start = 1;
constexpr uint8_t read = 2;
constexpr uint8_t write = 3;
constexpr uint8_t clear = 4;
constexpr uint8_t set = 5;
constexpr uint8_t move_index = 6;
constexpr uint8_t move_attr = 7;
constexpr uint8_t move_data = 8;
constexpr uint8_t end = 9;
} // namespace iio

[[noreturn]] inline void builder_error(const char* message) {
    std::fprintf(stderr, "atom_test_builder: %s\n", message);
    std::abort();
}

// One operand. As a source, align is its ATOM_SRC_* field (attribute bits 5:3);
// as a destination, sel is the destination selector (attribute bits 7:6) that
// atom_dst_to_src combines with the source alignment. Clear and the byte-count
// shifts take the destination alignment from align instead.
struct Operand final {
    Arg arg = Arg::imm;
    uint32_t value = 0;
    uint8_t align = src_dword;
    uint8_t sel = 0;

    Operand aligned(uint8_t alignment) const {
        Operand copy = *this;
        copy.align = alignment;
        return copy;
    }

    Operand selected(uint8_t selector) const {
        Operand copy = *this;
        copy.sel = selector;
        return copy;
    }
};

inline Operand reg(uint16_t index) { return {Arg::reg, index, src_dword, 0}; }
inline Operand ps(uint8_t index) { return {Arg::ps, index, src_dword, 0}; }
inline Operand ws(uint8_t index) { return {Arg::ws, index, src_dword, 0}; }
inline Operand fb(uint8_t index) { return {Arg::fb, index, src_dword, 0}; }
inline Operand pll(uint8_t index) { return {Arg::pll, index, src_dword, 0}; }
inline Operand mc(uint8_t index) { return {Arg::mc, index, src_dword, 0}; }
inline Operand id(uint16_t index) { return {Arg::id, index, src_dword, 0}; }
inline Operand imm32(uint32_t value) { return {Arg::imm, value, src_dword, 0}; }
inline Operand imm16(uint16_t value, uint8_t alignment = src_word0) { return {Arg::imm, value, alignment, 0}; }
inline Operand imm8(uint8_t value, uint8_t alignment = src_byte0) { return {Arg::imm, value, alignment, 0}; }

// atom_get_src_direct / atom_skip_src_int immediate widths.
inline uint32_t direct_bytes(uint8_t alignment) {
    return alignment == src_dword ? 4u : alignment < src_byte0 ? 2u : 1u;
}

inline uint32_t operand_bytes(Arg arg, uint8_t alignment) {
    switch (arg) {
    case Arg::reg:
    case Arg::id:
        return 2;
    case Arg::imm:
        return direct_bytes(alignment);
    default:
        return 1;
    }
}

// Offset of a destination kind inside a six-opcode group.
inline uint8_t group_offset(Arg arg) {
    switch (arg) {
    case Arg::reg: return 0;
    case Arg::ps: return 1;
    case Arg::ws: return 2;
    case Arg::fb: return 3;
    case Arg::pll: return 4;
    case Arg::mc: return 5;
    default: builder_error("ID and IMM are never destinations");
    }
}

inline void put_le(std::vector<uint8_t>& bytes, uint32_t value, uint32_t width) {
    for (uint32_t index = 0; index < width; ++index) {
        bytes.push_back(static_cast<uint8_t>(value >> (8 * index)));
    }
}

inline void set_u16(std::vector<uint8_t>& bytes, size_t offset, uint16_t value) {
    bytes.at(offset) = static_cast<uint8_t>(value);
    bytes.at(offset + 1) = static_cast<uint8_t>(value >> 8);
}

inline void set_u32(std::vector<uint8_t>& bytes, size_t offset, uint32_t value) {
    set_u16(bytes, offset, static_cast<uint16_t>(value));
    set_u16(bytes, offset + 2, static_cast<uint16_t>(value >> 16));
}

inline uint16_t get_u16(const std::vector<uint8_t>& bytes, size_t offset) {
    return static_cast<uint16_t>(bytes.at(offset) | (bytes.at(offset + 1) << 8));
}

// atom.h ATOM_CT_CODE_PTR: code follows the common header, WS and PS bytes.
constexpr uint32_t command_code_offset = 6;

// Bytecode of one command table. Jump and case targets are labels resolved
// against the table start (ctx->start), as atom_op_jump and atom_op_switch add.
class Code final {
public:
    struct Label final {
        size_t id;
    };

    // Each emitted instruction (code-relative offset, length, opcode) so tests can
    // cross-check the interpreter's decoder and aim mutations at operands.
    struct Emitted final {
        uint32_t offset;
        uint32_t length;
        uint8_t opcode;
        // Code-relative offset of a jump or case target word, else UINT32_MAX.
        uint32_t target_at;
    };

    Label label() {
        labels_.push_back(SIZE_MAX);
        return {labels_.size() - 1};
    }

    void place(Label label) { labels_.at(label.id) = bytes_.size(); }

    // MOVE/AND/OR/ADD/SUB/XOR/MUL/DIV/COMPARE/TEST/SHL/SHR: opcode group base
    // plus the destination kind, attribute, destination index, source operand.
    void group(uint8_t first, Operand dst, Operand src) {
        begin(static_cast<uint8_t>(first + group_offset(dst.arg)));
        attribute(src, dst.sel);
        index(dst);
        source(src);
        end();
    }

    void move(Operand dst, Operand src) { group(op::move, dst, src); }
    void and_(Operand dst, Operand src) { group(op::and_, dst, src); }
    void or_(Operand dst, Operand src) { group(op::or_, dst, src); }
    void add(Operand dst, Operand src) { group(op::add, dst, src); }
    void sub(Operand dst, Operand src) { group(op::sub, dst, src); }
    void xor_(Operand dst, Operand src) { group(op::xor_, dst, src); }
    void mul(Operand dst, Operand src) { group(op::mul, dst, src); }
    void div(Operand dst, Operand src) { group(op::div, dst, src); }
    void compare(Operand dst, Operand src) { group(op::compare, dst, src); }
    void test(Operand dst, Operand src) { group(op::test, dst, src); }
    void shl(Operand dst, Operand src) { group(op::shl, dst, src); }
    void shr(Operand dst, Operand src) { group(op::shr, dst, src); }

    // MUL32/DIV32 exist only for PS and WS destinations.
    void mul32(Operand dst, Operand src) { wide(dst.arg == Arg::ps ? op::mul32_ps : op::mul32_ws, dst, src); }
    void div32(Operand dst, Operand src) { wide(dst.arg == Arg::ps ? op::div32_ps : op::div32_ws, dst, src); }

    // MASK: destination, a direct mask of the source width, then the source.
    void mask(Operand dst, uint32_t mask_value, Operand src) {
        begin(static_cast<uint8_t>(op::mask + group_offset(dst.arg)));
        attribute(src, dst.sel);
        index(dst);
        put_le(bytes_, mask_value, direct_bytes(src.align));
        source(src);
        end();
    }

    // CLEAR, SHIFT_LEFT and SHIFT_RIGHT keep only attribute bits 5:3 (dst.align).
    void clear(Operand dst) {
        begin(static_cast<uint8_t>(op::clear + group_offset(dst.arg)));
        bytes_.push_back(static_cast<uint8_t>(dst.align << 3));
        index(dst);
        end();
    }

    void shift_left(Operand dst, uint8_t count) { byte_shift(op::shift_left, dst, count); }
    void shift_right(Operand dst, uint8_t count) { byte_shift(op::shift_right, dst, count); }

    void set_port_ati(uint16_t port) { fixed(op::set_port_ati, port, 2); }
    void set_port_pci(uint8_t value = 0) { fixed(op::set_port_pci, value, 1); }
    void set_port_sysio(uint8_t value = 0) { fixed(op::set_port_sysio, value, 1); }
    void set_reg_block(uint16_t block) { fixed(op::set_reg_block, block, 2); }
    void set_data_block(uint8_t index) { fixed(op::set_data_block, index, 1); }
    void delay_ms(uint8_t count) { fixed(op::delay_millisec, count, 1); }
    void delay_us(uint8_t count) { fixed(op::delay_microsec, count, 1); }
    void call_table(uint8_t index) { fixed(op::call_table, index, 1); }
    void post_card(uint8_t value) { fixed(op::post_card, value, 1); }
    void debug(uint8_t value) { fixed(op::debug, value, 1); }

    void set_fb_base(Operand src) {
        begin(op::set_fb_base);
        attribute(src, 0);
        source(src);
        end();
    }

    void jump(Cond cond, Label target) {
        begin(static_cast<uint8_t>(op::jump + static_cast<uint8_t>(cond)));
        target_word(target);
        end();
    }

    // A jump to a raw table-relative target (for out-of-table tests).
    void jump_raw(Cond cond, uint16_t target) {
        begin(static_cast<uint8_t>(op::jump + static_cast<uint8_t>(cond)));
        emitted_.back().target_at = static_cast<uint32_t>(bytes_.size());
        put_le(bytes_, target, 2);
        end();
    }

    struct Case final {
        uint32_t value;
        Label target;
    };

    // SWITCH: source, then CASE_MAGIC + immediate of the source width + target
    // per case, then CASE_END.
    void switch_(Operand src, const std::vector<Case>& cases) {
        begin(op::switch_);
        attribute(src, 0);
        source(src);

        for (const auto& item : cases) {
            bytes_.push_back(0x63);
            put_le(bytes_, item.value, direct_bytes(src.align));
            target_word(item.target);
        }

        put_le(bytes_, 0x5a5a, 2);
        end();
    }

    // PROCESS_DS: a u16 byte count and that many skipped bytes.
    void process_ds(const std::vector<uint8_t>& payload) {
        begin(op::process_ds);
        put_le(bytes_, static_cast<uint32_t>(payload.size()), 2);
        bytes_.insert(bytes_.end(), payload.begin(), payload.end());
        end();
    }

    void nop() { single(op::nop); }
    void eot() { single(op::eot); }
    void beep() { single(op::beep); }
    void repeat() { single(op::repeat); }
    void save_reg() { single(op::save_reg); }
    void restore_reg() { single(op::restore_reg); }

    // Arbitrary bytes recorded as one instruction (invalid opcodes, truncation).
    void raw(const std::vector<uint8_t>& bytes) {
        begin(bytes.empty() ? 0 : bytes.front());
        bytes_.insert(bytes_.end(), bytes.begin() + (bytes.empty() ? 0 : 1), bytes.end());
        end();
    }

    size_t size() const { return bytes_.size(); }
    const std::vector<Emitted>& emitted() const { return emitted_; }

    // Code bytes with every label target resolved to a table-relative offset.
    std::vector<uint8_t> resolved() const {
        std::vector<uint8_t> output = bytes_;

        for (const auto& fixup : fixups_) {
            const size_t at = labels_.at(fixup.second);

            if (at == SIZE_MAX) {
                builder_error("label used but never placed");
            }

            set_u16(output, fixup.first, static_cast<uint16_t>(command_code_offset + at));
        }

        return output;
    }

private:
    void begin(uint8_t opcode) {
        emitted_.push_back({static_cast<uint32_t>(bytes_.size()), 0, opcode, UINT32_MAX});
        bytes_.push_back(opcode);
    }

    void end() { emitted_.back().length = static_cast<uint32_t>(bytes_.size()) - emitted_.back().offset; }

    void single(uint8_t opcode) {
        begin(opcode);
        end();
    }

    void fixed(uint8_t opcode, uint32_t value, uint32_t width) {
        begin(opcode);
        put_le(bytes_, value, width);
        end();
    }

    void attribute(Operand src, uint8_t selector) {
        bytes_.push_back(static_cast<uint8_t>(static_cast<uint8_t>(src.arg) | ((src.align & 7u) << 3) |
                                              ((selector & 3u) << 6)));
    }

    // Destination index width: REG 2 bytes, every other destination kind 1.
    void index(Operand dst) { put_le(bytes_, dst.value, dst.arg == Arg::reg ? 2u : 1u); }

    void source(Operand src) { put_le(bytes_, src.value, operand_bytes(src.arg, src.align)); }

    void wide(uint8_t opcode, Operand dst, Operand src) {
        if (dst.arg != Arg::ps && dst.arg != Arg::ws) {
            builder_error("MUL32/DIV32 take a PS or WS destination");
        }

        begin(opcode);
        attribute(src, dst.sel);
        index(dst);
        source(src);
        end();
    }

    void byte_shift(uint8_t first, Operand dst, uint8_t count) {
        begin(static_cast<uint8_t>(first + group_offset(dst.arg)));
        bytes_.push_back(static_cast<uint8_t>(dst.align << 3));
        index(dst);
        bytes_.push_back(count);
        end();
    }

    void target_word(Label target) {
        emitted_.back().target_at = static_cast<uint32_t>(bytes_.size());
        fixups_.push_back({bytes_.size(), target.id});
        put_le(bytes_, 0, 2);
    }

    std::vector<uint8_t> bytes_;
    std::vector<size_t> labels_;
    std::vector<std::pair<size_t, size_t>> fixups_;
    std::vector<Emitted> emitted_;
};

// One IndirectIOAccess program: START id, body, END plus its two trailing bytes
// (atom_iio_len and atom_index_iio's base += 3).
class IioProgram final {
public:
    void nop() { bytes.push_back(iio::nop); }
    void read(uint16_t reg_index) { op16(iio::read, reg_index); }
    void write(uint16_t reg_index) { op16(iio::write, reg_index); }
    void clear(uint8_t width, uint8_t shift) { bytes.insert(bytes.end(), {iio::clear, width, shift}); }
    void set(uint8_t width, uint8_t shift) { bytes.insert(bytes.end(), {iio::set, width, shift}); }
    void move_index(uint8_t width, uint8_t from, uint8_t to) { bytes.insert(bytes.end(), {iio::move_index, width, from, to}); }
    void move_attr(uint8_t width, uint8_t from, uint8_t to) { bytes.insert(bytes.end(), {iio::move_attr, width, from, to}); }
    void move_data(uint8_t width, uint8_t from, uint8_t to) { bytes.insert(bytes.end(), {iio::move_data, width, from, to}); }
    void raw(uint8_t byte) { bytes.push_back(byte); }

    std::vector<uint8_t> bytes;

private:
    void op16(uint8_t opcode, uint16_t value) {
        bytes.push_back(opcode);
        put_le(bytes, value, 2);
    }
};

// firmware_info v3.x fields amdgpu reads (atom_firmware.cpp offsets).
struct FirmwareInfo final {
    uint8_t content_revision = 4;
    uint32_t bootup_sclk_10khz = 0;
    uint32_t bootup_mclk_10khz = 0;
    uint32_t capability = 0;
    uint32_t bios_scratch_reg = 0;
};

// Data master indices (atom_master_list_of_data_tables_v2_1).
constexpr uint8_t data_firmwareinfo = 4;
constexpr uint8_t data_vram_usage = 11;
// ATOM_DATA_IIO_PTR 0x32 = 4 + 2 * 23.
constexpr uint8_t data_indirect_io = 23;

// The built image and where each structure landed (for targeted mutations).
struct Image final {
    std::vector<uint8_t> bytes;
    uint32_t rom_header = 0;
    uint32_t command_master = 0;
    uint32_t data_master = 0;
    uint32_t iio_table = 0;
    // Present command table index -> table offset, and its emitted instructions.
    std::map<uint8_t, uint32_t> commands;
    std::map<uint8_t, std::vector<Code::Emitted>> instructions;
    std::map<uint8_t, uint32_t> data;
    // IIO program id -> offset of its first body byte (after START id).
    std::map<uint8_t, uint32_t> iio_programs;
};

// Fixed layout: 0xAA55 and " 761295520" at 0x30, ROM header 2.2 at 0x60 (0x48
// points at it, "ATOM" at +4, command master at +0x1E, data master at +0x20),
// then the command master, data master, command tables, data tables and the
// IIO table, followed by optional tail padding.
class ImageBuilder final {
public:
    struct CommandTable final {
        uint8_t format = 1;
        uint8_t content = 1;
        // ATOM_CT_WS_PTR in dwords and ATOM_CT_PS_PTR in bytes (bit 7 masked).
        uint8_t ws_dwords = 0;
        uint8_t ps_bytes = 0;
        Code code;
    };

    CommandTable& command(uint8_t index, uint8_t ws_dwords = 0, uint8_t ps_bytes = 0) {
        CommandTable& table = commands_[index];
        table.ws_dwords = ws_dwords;
        table.ps_bytes = ps_bytes;
        return table;
    }

    Code& code(uint8_t index) { return commands_.at(index).code; }

    // A data table: payload follows the four-byte common header.
    void data(uint8_t index, uint8_t format, uint8_t content, const std::vector<uint8_t>& payload) {
        data_[index] = DataTable{format, content, payload};
    }

    void firmware_info(const FirmwareInfo& info) {
        // v3.4/v3.5 need fw_reserved_size_in_kb at 84; 100 bytes cover v3.1..v3.5.
        std::vector<uint8_t> payload(100 - 4, 0);
        set_u32(payload, 8 - 4, info.bootup_sclk_10khz);
        set_u32(payload, 12 - 4, info.bootup_mclk_10khz);
        set_u32(payload, 16 - 4, info.capability);
        set_u32(payload, 24 - 4, info.bios_scratch_reg);
        data(data_firmwareinfo, 3, info.content_revision, payload);
    }

    // Programs are laid out in first-use order; a repeated identifier keeps one slot.
    IioProgram& iio_program(uint8_t identifier) {
        if (iio_.find(identifier) == iio_.end()) {
            iio_order_.push_back(identifier);
        }

        return iio_[identifier];
    }

    // Entries in the command master (default: highest table index + 1).
    void command_entries(uint32_t count) { command_entries_ = count; }
    // Pattern bytes appended after the last table (ID reads past the tables).
    void tail_padding(uint32_t bytes) { tail_padding_ = bytes; }

    Image build() const {
        constexpr uint32_t rom_header = 0x60;
        constexpr uint32_t rom_header_bytes = 0x30;
        constexpr uint32_t data_entries = 35;
        Image image{};
        std::vector<uint8_t>& bytes = image.bytes;
        bytes.assign(rom_header + rom_header_bytes, 0);
        set_u16(bytes, 0, 0xaa55);
        const char magic[] = " 761295520";

        for (size_t index = 0; index + 1 < sizeof(magic); ++index) {
            bytes.at(0x30 + index) = static_cast<uint8_t>(magic[index]);
        }

        set_u16(bytes, 0x48, rom_header);
        header(bytes, rom_header, rom_header_bytes, 2, 2);
        bytes.at(rom_header + 4) = 'A';
        bytes.at(rom_header + 5) = 'T';
        bytes.at(rom_header + 6) = 'O';
        bytes.at(rom_header + 7) = 'M';
        image.rom_header = rom_header;

        uint32_t entries = command_entries_;

        if (entries == 0) {
            entries = commands_.empty() ? 1u : static_cast<uint32_t>(commands_.rbegin()->first) + 1u;
        }

        image.command_master = append_table(bytes, 4 + 2 * entries, 1, 1);
        image.data_master = append_table(bytes, 4 + 2 * data_entries, 2, 1);
        set_u16(bytes, rom_header + 0x1e, static_cast<uint16_t>(image.command_master));
        set_u16(bytes, rom_header + 0x20, static_cast<uint16_t>(image.data_master));

        for (const auto& entry : commands_) {
            const CommandTable& table = entry.second;
            const std::vector<uint8_t> code = table.code.resolved();
            const uint32_t base = append_table(bytes, static_cast<uint32_t>(command_code_offset + code.size()),
                                               table.format, table.content);
            bytes.at(base + 4) = table.ws_dwords;
            bytes.at(base + 5) = table.ps_bytes;

            for (size_t index = 0; index < code.size(); ++index) {
                bytes.at(base + command_code_offset + index) = code.at(index);
            }

            if (entry.first < entries) {
                set_u16(bytes, image.command_master + 4 + 2u * entry.first, static_cast<uint16_t>(base));
            }

            image.commands[entry.first] = base;
            image.instructions[entry.first] = table.code.emitted();
        }

        for (const auto& entry : data_) {
            const uint32_t base = append_table(bytes, static_cast<uint32_t>(4 + entry.second.payload.size()),
                                               entry.second.format, entry.second.content);

            for (size_t index = 0; index < entry.second.payload.size(); ++index) {
                bytes.at(base + 4 + index) = entry.second.payload.at(index);
            }

            set_u16(bytes, image.data_master + 4 + 2u * entry.first, static_cast<uint16_t>(base));
            image.data[entry.first] = base;
        }

        if (!iio_order_.empty()) {
            uint32_t total = 4;

            for (const uint8_t identifier : iio_order_) {
                total += 2 + static_cast<uint32_t>(iio_.at(identifier).bytes.size()) + 3;
            }

            const uint32_t base = append_table(bytes, total, 1, 1);
            uint32_t cursor = base + 4;

            for (const uint8_t identifier : iio_order_) {
                const auto& body = iio_.at(identifier).bytes;
                bytes.at(cursor) = iio::start;
                bytes.at(cursor + 1) = identifier;
                image.iio_programs[identifier] = cursor + 2;
                cursor += 2;

                for (const uint8_t byte : body) {
                    bytes.at(cursor++) = byte;
                }

                bytes.at(cursor) = iio::end;
                cursor += 3;
            }

            set_u16(bytes, image.data_master + 4 + 2u * data_indirect_io, static_cast<uint16_t>(base));
            image.iio_table = base;
        }

        for (uint32_t index = 0; index < tail_padding_; ++index) {
            bytes.push_back(static_cast<uint8_t>(0xa0 + index * 7));
        }

        if (bytes.size() > 0xffff) {
            builder_error("image exceeds the 16-bit ATOM offset range");
        }

        return image;
    }

private:
    struct DataTable final {
        uint8_t format;
        uint8_t content;
        std::vector<uint8_t> payload;
    };

    static void header(std::vector<uint8_t>& bytes, uint32_t offset, uint32_t size, uint8_t format,
                       uint8_t content) {
        set_u16(bytes, offset, static_cast<uint16_t>(size));
        bytes.at(offset + 2) = format;
        bytes.at(offset + 3) = content;
    }

    static uint32_t append_table(std::vector<uint8_t>& bytes, uint32_t size, uint8_t format, uint8_t content) {
        const uint32_t base = static_cast<uint32_t>(bytes.size());
        bytes.resize(bytes.size() + size, 0);
        header(bytes, base, size, format, content);
        return base;
    }

    std::map<uint8_t, CommandTable> commands_;
    std::map<uint8_t, DataTable> data_;
    std::map<uint8_t, IioProgram> iio_;
    std::vector<uint8_t> iio_order_;
    uint32_t command_entries_ = 0;
    uint32_t tail_padding_ = 0;
};

} // namespace rx7900_test::atom

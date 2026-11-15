/*
Module: rx7900.tests.atom_interpreter
Purpose: Verify the bounded ATOM interpreter against Linux v6.12 atom.c semantics and bounds.
Created: 2026-10-24
Architecture: Every image comes from the programmatic encoder (atom_test_builder.hpp).
Per-opcode semantics, a randomized cross-check against the independent reference
model (atom_reference_model.hpp), deterministic mutation fuzzing, budgets and
quarantine. Images live in exactly sized allocations so ASan flags any read past
them. Synthetic only: no VBIOS bytes and no hardware.
*/
#include <algorithm>
#include <array>
#include <cstdio>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "asic_test_device.hpp"
#include "atom_interpreter.hpp"
#include "atom_firmware.hpp"
#include "atom_reference_model.hpp"

namespace {

using namespace rx7900::hardware;
using rx7900::AtomTable;
using rx7900::AtomTableStatus;
namespace atom = rx7900_test::atom;
using atom::Access;
using atom::Code;
using atom::Cond;
using atom::Image;
using atom::ImageBuilder;
using atom::Operand;
using atom::fb;
using atom::id;
using atom::imm16;
using atom::imm32;
using atom::imm8;
using atom::mc;
using atom::pll;
using atom::ps;
using atom::reg;
using atom::ws;
using atom::ref_arg_mask;
using atom::ref_arg_shift;
using atom::ref_dst_to_src;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

// The register aperture every test bus exposes (dword registers).
constexpr uint32_t bus_dwords = 1024;

// Register file behind RegisterIo: records every access and every delay, and
// counts any access outside the aperture (the interpreter must never issue one).
struct Bus final {
    explicit Bus(uint32_t dwords = bus_dwords) : registers(dwords, 0) {}

    std::vector<uint32_t> registers;
    std::vector<Access> trace;
    std::vector<uint32_t> delays;
    uint64_t delay_total = 0;
    uint32_t violations = 0;
    uint32_t reads = 0;
    uint32_t writes = 0;
    // Zero-based access number that reports an I/O failure.
    uint32_t fail_at = UINT32_MAX;
    uint32_t accesses = 0;

    bool admit(uint32_t offset) {
        if ((offset & 3u) != 0 || offset / 4u >= registers.size()) {
            ++violations;
            return false;
        }

        return accesses++ != fail_at;
    }

    static bool read(void* context, uint32_t offset, uint32_t& value) noexcept {
        auto& self = *static_cast<Bus*>(context);

        if (!self.admit(offset)) {
            return false;
        }

        value = self.registers[offset / 4u];
        ++self.reads;
        self.trace.push_back({false, offset / 4u, value});
        return true;
    }

    static bool write(void* context, uint32_t offset, uint32_t value) noexcept {
        auto& self = *static_cast<Bus*>(context);

        if (!self.admit(offset)) {
            return false;
        }

        self.registers[offset / 4u] = value;
        ++self.writes;
        self.trace.push_back({true, offset / 4u, value});
        return true;
    }

    static void delay(void* context, uint32_t microseconds) noexcept {
        auto& self = *static_cast<Bus*>(context);
        self.delays.push_back(microseconds);
        self.delay_total += microseconds;
    }

    RegisterIo io() noexcept { return {this, static_cast<uint32_t>(registers.size() * 4u), read, write, delay}; }

    std::vector<Access> writes_only() const {
        std::vector<Access> output;

        for (const auto& access : trace) {
            if (access.write) {
                output.push_back(access);
            }
        }

        return output;
    }
};

// The image copied into an allocation of exactly its (possibly truncated) size.
class ExactImage final {
public:
    explicit ExactImage(const std::vector<uint8_t>& bytes, size_t size = SIZE_MAX)
        : size_(std::min(size, bytes.size())), data_(new uint8_t[size_ == 0 ? 1 : size_]) {
        std::copy(bytes.begin(), bytes.begin() + static_cast<long>(size_), data_.get());
    }

    rx7900::ByteView view() const { return {data_.get(), size_}; }
    size_t size() const { return size_; }

private:
    size_t size_;
    std::unique_ptr<uint8_t[]> data_;
};

constexpr uint32_t default_scratch = 256;

struct Machine final {
    explicit Machine(const Image& built, uint32_t scratch = default_scratch) : image(built), rom(built.bytes) {
        bound = atom->bind(rom.view(), scratch);
    }

    Image image;
    ExactImage rom;
    std::unique_ptr<AtomInterpreter> atom = std::make_unique<AtomInterpreter>();
    Bus bus;
    AtomStatus bound = AtomStatus::invalid_rom;

    AtomStatus run(uint8_t table = 0, const std::vector<uint32_t>& params = {}, uint32_t ps_dwords = 16,
                   AtomPolicy policy = atom_default_policy, AtomObserver observer = {nullptr, nullptr}) {
        atom->clear_parameters();

        for (uint32_t index = 0; index < params.size(); ++index) {
            atom->set_parameter(index, params.at(index));
        }

        return atom->execute(bus.io(), table, ps_dwords, policy, observer);
    }

    uint32_t ps(uint32_t index) const { return atom->parameter(index); }
    AtomExecReport report() const { return atom->report(); }
};

// One root table (index 0) emitted by fill and terminated with EOT.
template <typename Fill>
Image single(uint8_t ws_dwords, uint8_t ps_bytes, Fill fill) {
    ImageBuilder builder;
    auto& table = builder.command(0, ws_dwords, ps_bytes);
    fill(table.code);
    table.code.eot();
    return builder.build();
}

AtomPolicy policy_with(uint32_t steps, uint32_t depth, uint64_t delay_us, bool stop = false) {
    return {steps, depth, delay_us, stop};
}

// ---------------------------------------------------------------------------
// Decoder: the opcode_table mapping, written independently from atom.c.
// ---------------------------------------------------------------------------

struct ExpectedOp final {
    AtomOperation operation;
    uint8_t argument;
};

ExpectedOp expected_op(uint8_t opcode) {
    struct Group {
        uint8_t first;
        AtomOperation operation;
    };
    static const Group groups[] = {
        {1, AtomOperation::move},       {7, AtomOperation::and_op},      {13, AtomOperation::or_op},
        {19, AtomOperation::shift_left}, {25, AtomOperation::shift_right}, {31, AtomOperation::mul},
        {37, AtomOperation::div},       {43, AtomOperation::add},        {49, AtomOperation::sub},
        {60, AtomOperation::compare},   {74, AtomOperation::test},       {84, AtomOperation::clear},
        {92, AtomOperation::mask},      {103, AtomOperation::xor_op},    {109, AtomOperation::shl},
        {115, AtomOperation::shr},
    };
    // ATOM_ARG_REG, PS, WS, FB, PLL, MC.
    static const uint8_t kinds[6] = {0, 1, 2, 3, 6, 7};
    // JUMP, JUMP_EQUAL, JUMP_BELOW, JUMP_ABOVE, JUMP_BELOW_OR_EQUAL,
    // JUMP_ABOVE_OR_EQUAL, JUMP_NOT_EQUAL as ATOM_COND_* values.
    static const uint8_t conditions[7] = {2, 5, 3, 0, 4, 1, 6};

    for (const auto& group : groups) {
        if (opcode >= group.first && opcode < group.first + 6) {
            return {group.operation, kinds[opcode - group.first]};
        }
    }

    if (opcode >= 55 && opcode <= 57) {
        return {AtomOperation::set_port, static_cast<uint8_t>(opcode - 55)};
    }

    if (opcode >= 67 && opcode <= 73) {
        return {AtomOperation::jump, conditions[opcode - 67]};
    }

    switch (opcode) {
    case 58: return {AtomOperation::set_reg_block, 0};
    case 59: return {AtomOperation::set_fb_base, 0};
    case 66: return {AtomOperation::switch_op, 0};
    case 80: return {AtomOperation::delay, 1};
    case 81: return {AtomOperation::delay, 0};
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

// Every emitted instruction of every table decodes at its builder offset with
// the builder's length and opcode, and the expected operation.
bool decode_matches(const Image& image, uint32_t& mismatches) {
    const rx7900::ByteView rom(image.bytes.data(), image.bytes.size());
    bool all = true;

    for (const auto& entry : image.instructions) {
        const uint32_t base = image.commands.at(entry.first);
        const AtomCodeRange range{base, atom::get_u16(image.bytes, base)};

        for (const auto& emitted : entry.second) {
            AtomInstruction decoded{};
            const uint32_t at = base + atom::command_code_offset + emitted.offset;
            const bool ok = atom_decode_instruction(rom, range, at, decoded) == AtomDecodeStatus::success &&
                            decoded.offset == at && decoded.length == emitted.length &&
                            decoded.opcode == emitted.opcode &&
                            decoded.operation == expected_op(emitted.opcode).operation;

            if (!ok) {
                ++mismatches;
                all = false;
            }
        }
    }

    return all;
}

void decoder() {
    uint32_t mapped = 0;
    uint32_t invalid_ones = 0;

    for (uint32_t opcode = 0; opcode < 256; ++opcode) {
        // Opcode, attribute 0 (REG source), zero operands, then CASE_END.
        std::vector<uint8_t> bytes(32, 0);
        bytes.at(0) = static_cast<uint8_t>(opcode);
        bytes.at(4) = 0x5a;
        bytes.at(5) = 0x5a;
        const rx7900::ByteView rom(bytes.data(), bytes.size());
        AtomInstruction decoded{};
        const auto status = atom_decode_instruction(rom, {0, 32}, 0, decoded);
        const ExpectedOp wanted = expected_op(static_cast<uint8_t>(opcode));

        if (status == AtomDecodeStatus::success && decoded.operation == wanted.operation &&
            decoded.argument == wanted.argument) {
            ++mapped;
        }

        if (wanted.operation == AtomOperation::invalid && decoded.length == 1) {
            ++invalid_ones;
        }
    }

    expect(mapped == 256, "all 256 opcode bytes decode to the atom.c opcode_table operation and argument");
    expect(invalid_ones == 130, "opcode 0 and 127..255 decode as invalid with length 1");

    const std::vector<uint8_t> tiny{atom::op::move, 0x05};
    AtomInstruction decoded{};
    expect(atom_decode_instruction({tiny.data(), tiny.size()}, {0, 2}, 0, decoded) == AtomDecodeStatus::truncated &&
               atom_decode_instruction({tiny.data(), tiny.size()}, {0, 3}, 0, decoded) ==
                   AtomDecodeStatus::invalid_argument &&
               atom_decode_instruction({tiny.data(), tiny.size()}, {0, 2}, 2, decoded) ==
                   AtomDecodeStatus::invalid_argument &&
               atom_decode_instruction({tiny.data(), tiny.size()}, {0, 0}, 0, decoded) ==
                   AtomDecodeStatus::invalid_argument,
           "decoder refuses truncated operands, ranges outside the ROM and offsets outside the range");

    // Every encoder form decodes to the builder's own boundaries.
    ImageBuilder builder;
    auto& code = builder.command(0, 4, 0).code;
    const Code::Label label = code.label();
    code.move(reg(0x10), imm32(1));
    code.move(ps(1).selected(2), reg(2).aligned(atom::src_byte8));
    code.and_(ws(1), id(4));
    code.or_(fb(1), imm16(3, atom::src_word16));
    code.add(pll(1), imm8(4, atom::src_byte24));
    code.sub(mc(2), ps(3));
    code.xor_(reg(5), ws(0x40));
    code.mask(ps(2), 0xff00ff00u, imm32(7));
    code.mask(ws(2), 0x0ff0u, imm16(7));
    code.clear(fb(3).aligned(atom::src_word8));
    code.shift_left(reg(1).aligned(atom::src_byte16), 3);
    code.shift_right(ps(1), 3);
    code.shl(ws(1), imm8(2));
    code.shr(reg(3).selected(1), ps(1).aligned(atom::src_word0));
    code.mul(ps(1), imm32(3));
    code.div(reg(3), imm16(3));
    code.mul32(ps(1), ws(0));
    code.div32(ws(1), imm8(3));
    code.compare(fb(1), mc(1));
    code.test(pll(1), imm32(9));
    code.set_port_ati(0x1234);
    code.set_port_pci(1);
    code.set_port_sysio(2);
    code.set_reg_block(0x100);
    code.set_fb_base(ps(1).aligned(atom::src_word0));
    code.set_data_block(4);
    code.jump(Cond::above_or_equal, label);
    code.switch_(ps(1).aligned(atom::src_word8), {{1, label}, {2, label}});
    code.switch_(imm32(2), {{1, label}});
    code.delay_ms(1);
    code.delay_us(2);
    code.call_table(9);
    code.post_card(1);
    code.debug(2);
    code.process_ds({1, 2, 3});
    code.repeat();
    code.save_reg();
    code.restore_reg();
    code.beep();
    code.nop();
    code.place(label);
    code.eot();
    uint32_t mismatches = 0;
    expect(decode_matches(builder.build(), mismatches) && mismatches == 0,
           "every encoder form (all operand kinds and alignments) decodes to its emitted length");
}

// ---------------------------------------------------------------------------
// Operand kinds as sources and destinations.
// ---------------------------------------------------------------------------

void sources() {
    ImageBuilder builder;
    builder.data(1, 1, 1, {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88});
    auto& code = builder.command(0, 2, 0).code;
    code.set_data_block(1);
    code.move(ps(0), reg(0x10));
    code.move(ps(1), ps(8));
    code.move(ws(0), imm32(0x0badf00du));
    code.move(ps(2), ws(0));
    code.move(fb(3), imm32(0x600dcafeu));
    code.move(ps(3), fb(3));
    // ID index 4 past the data block: its first payload dword.
    code.move(ps(4), id(4));
    code.move(ps(5), imm32(0x89abcdefu));
    // Word and byte immediates merge into the read destination (atom_op_move).
    code.move(ps(6), imm16(0xbeef));
    code.move(ps(7), imm8(0x5a));
    code.move(ps(9), pll(3));
    code.move(ps(10), mc(4));
    code.set_data_block(0);
    code.move(ps(11), id(0));
    code.eot();
    Machine m(builder.build());
    m.bus.registers.at(0x10) = 0x11223344u;
    std::vector<uint32_t> params(16, 0);
    params.at(6) = 0x12345678u;
    params.at(7) = 0x9abcdef0u;
    params.at(8) = 0xa1b2c3d4u;
    params.at(9) = 0xffffffffu;
    params.at(10) = 0xffffffffu;
    expect(m.bound == AtomStatus::completed && m.run(0, params) == AtomStatus::completed, "source program runs");
    expect(m.ps(0) == 0x11223344u && m.ps(1) == 0xa1b2c3d4u && m.ps(2) == 0x0badf00du && m.ps(3) == 0x600dcafeu,
           "REG, PS, WS and FB sources");
    expect(m.ps(4) == 0x44332211u && m.ps(11) == 0x0000aa55u,
           "ID reads U32(idx + data_block): a data table, and the ROM itself at data block 0");
    expect(m.ps(5) == 0x89abcdefu && m.ps(6) == 0x1234beefu && m.ps(7) == 0x9abcde5au,
           "IMM dword, word0 and byte0 immediates (narrow moves keep the other destination bits)");
    expect(m.ps(9) == 0 && m.ps(10) == 0 && m.report().inert_pll_mc == 2,
           "PLL and MC sources read 0 (cail_pll_read / cail_mc_read) and are counted");
    expect(m.bus.trace == std::vector<Access>{{false, 0x10, 0x11223344u}}, "only the REG source touched the bus");
}

void destinations() {
    const Image image = single(2, 0, [](Code& code) {
        code.move(reg(0x20), imm32(0xcafef00du));
        code.move(ps(0), imm32(1));
        code.move(ws(1), imm32(2));
        code.move(ps(1), ws(1));
        code.move(fb(5), imm32(3));
        code.move(ps(2), fb(5));
        code.move(pll(1), imm32(4));
        code.move(mc(2), imm32(5));
        // Register 0 (MM_INDEX) takes a byte address: atom_put_dst writes val << 2.
        code.move(reg(0), imm32(0x10));
    });
    Machine m(image);
    expect(m.run() == AtomStatus::completed && m.ps(0) == 1 && m.ps(1) == 2 && m.ps(2) == 3 &&
               m.report().inert_pll_mc == 2,
           "PS, WS and FB destinations; PLL and MC writes are dropped and counted");
    expect(m.bus.trace == std::vector<Access>{{true, 0x20, 0xcafef00du}, {true, 0, 0x40}},
           "dword MOVE skips reading its destination; REG 0 is written as a byte address");

    // Every read-modify-write family on REG, PS, WS and FB destinations.
    struct Family {
        uint8_t first;
        uint32_t (*apply)(uint32_t, uint32_t);
    };
    const Family families[] = {
        {atom::op::and_, [](uint32_t d, uint32_t s) { return d & s; }},
        {atom::op::or_, [](uint32_t d, uint32_t s) { return d | s; }},
        {atom::op::add, [](uint32_t d, uint32_t s) { return d + s; }},
        {atom::op::sub, [](uint32_t d, uint32_t s) { return d - s; }},
        {atom::op::xor_, [](uint32_t d, uint32_t s) { return d ^ s; }},
    };
    const uint32_t d = 0xfff0f00fu;
    const uint32_t s = 0x0ff0ff11u;
    uint32_t good = 0;

    for (const auto& family : families) {
        const Image rmw = single(1, 0, [&](Code& code) {
            code.move(ws(0), imm32(d));
            code.move(fb(0), imm32(d));
            code.group(family.first, reg(0x30), imm32(s));
            code.group(family.first, ps(0), imm32(s));
            code.group(family.first, ws(0), imm32(s));
            code.group(family.first, fb(0), imm32(s));
            code.group(family.first, pll(0), imm32(s));
            code.group(family.first, mc(0), imm32(s));
            code.move(ps(1), ws(0));
            code.move(ps(2), fb(0));
        });
        Machine machine(rmw);
        machine.bus.registers.at(0x30) = d;
        const uint32_t want = family.apply(d, s);

        if (machine.run(0, {d}) == AtomStatus::completed && machine.bus.registers.at(0x30) == want &&
            machine.ps(0) == want && machine.ps(1) == want && machine.ps(2) == want &&
            machine.report().inert_pll_mc == 4 &&
            machine.bus.trace == std::vector<Access>{{false, 0x30, d}, {true, 0x30, want}}) {
            ++good;
        }
    }

    expect(good == 5, "AND/OR/ADD/SUB/XOR on REG, PS, WS, FB (and inert PLL/MC) destinations");

    const Image wrap = single(0, 0, [](Code& code) {
        code.add(ps(0), imm32(2));
        code.sub(ps(1), imm32(2));
        code.mask(ps(2), 0xff00ff00u, imm32(0x00110011u));
        code.mask(ps(3), 0x00f0u, imm16(0x0003));
    });
    Machine w(wrap);
    expect(w.run(0, {0xffffffffu, 1, 0x12345678u, 0xabcd1234u}) == AtomStatus::completed && w.ps(0) == 1 &&
               w.ps(1) == 0xffffffffu && w.ps(2) == 0x12115611u && w.ps(3) == 0xabcd0033u,
           "ADD/SUB wrap modulo 2^32; MASK is (dst & mask) | src at the source width");
}

// ---------------------------------------------------------------------------
// Alignment: source masks, atom_dst_to_src, atom_def_dst.
// ---------------------------------------------------------------------------

uint32_t merge(uint32_t value, uint32_t saved, int align) {
    return ((value << ref_arg_shift[align]) & ref_arg_mask[align]) | (saved & ~ref_arg_mask[align]);
}

uint32_t field_of(uint32_t value, int align) { return (value & ref_arg_mask[align]) >> ref_arg_shift[align]; }

void alignment() {
    const uint32_t source = 0x11223344u;
    const uint32_t expect_fields[8] = {0x11223344u, 0x3344u, 0x2233u, 0x1122u, 0x44u, 0x33u, 0x22u, 0x11u};
    uint32_t fields = 0;

    for (uint8_t align = 0; align < 8; ++align) {
        if (field_of(source, align) == expect_fields[align]) {
            ++fields;
        }
    }

    expect(fields == 8, "ATOM_SRC_* field extraction table (DWORD, WORD0/8/16, BYTE0/8/16/24)");

    // MOVE and ADD: every source alignment x destination selector.
    uint32_t move_ok = 0;
    uint32_t add_ok = 0;
    uint32_t reg_ok = 0;

    for (uint8_t align = 0; align < 8; ++align) {
        const Image image = single(0, 0, [&](Code& code) {
            for (uint8_t sel = 0; sel < 4; ++sel) {
                code.move(ps(sel).selected(sel), reg(0x10).aligned(align));
                code.add(ps(static_cast<uint8_t>(4 + sel)).selected(sel), reg(0x10).aligned(align));
                code.move(reg(static_cast<uint16_t>(0x20 + sel)).selected(sel), reg(0x10).aligned(align));
            }
        });
        Machine m(image);
        m.bus.registers.at(0x10) = source;
        std::vector<uint32_t> params(8);

        for (uint32_t index = 0; index < 8; ++index) {
            params.at(index) = 0xa5c3e1f0u ^ (index * 0x01010101u);
        }

        for (uint32_t sel = 0; sel < 4; ++sel) {
            m.bus.registers.at(0x20 + sel) = 0x5a5a5a5au + sel;
        }

        m.run(0, params);

        for (uint8_t sel = 0; sel < 4; ++sel) {
            const int dst = ref_dst_to_src[align][sel];
            const uint32_t src = field_of(source, align);
            // A dword MOVE skips the destination read (saved = 0xCDCDCDCD, fully masked).
            const uint32_t saved = align == atom::src_dword ? 0xcdcdcdcdu : params.at(sel);
            move_ok += m.ps(sel) == merge(src, saved, dst) ? 1u : 0u;
            const uint32_t old = params.at(4u + sel);
            add_ok += m.ps(4u + sel) == merge(field_of(old, dst) + src, old, dst) ? 1u : 0u;
            const uint32_t old_reg = 0x5a5a5a5au + sel;
            reg_ok += m.bus.registers.at(0x20u + sel) ==
                              merge(src, align == atom::src_dword ? 0xcdcdcdcdu : old_reg, dst) ? 1u : 0u;
        }
    }

    expect(move_ok == 32, "MOVE: 8 source alignments x 4 destination selectors follow atom_dst_to_src");
    expect(add_ok == 32, "ADD: destination field read, combined and merged at the translated alignment");
    expect(reg_ok == 32, "REG destinations merge the same way (read-before-write unless dword)");

    // CLEAR, SHIFT_LEFT/RIGHT, SHL/SHR and MASK alignment forms.
    const uint32_t old = 0x12348078u;
    uint32_t clear_ok = 0;
    uint32_t byte_shift_ok = 0;
    uint32_t full_shift_ok = 0;
    uint32_t mask_ok = 0;

    for (uint8_t align = 0; align < 8; ++align) {
        const Image image = single(0, 0, [&](Code& code) {
            code.clear(ps(0).aligned(align));
            code.shift_left(ps(1).aligned(align), 3);
            code.shift_right(ps(2).aligned(align), 3);

            for (uint8_t sel = 0; sel < 4; ++sel) {
                code.shl(ps(static_cast<uint8_t>(3 + sel)).selected(sel), imm32(5).aligned(align));
                code.mask(ps(static_cast<uint8_t>(7 + sel)).selected(sel), 0x0f0f0f0fu, imm32(0x00a000a0u)
                    .aligned(align));
            }
        });
        Machine m(image);
        m.run(0, std::vector<uint32_t>(11, old));
        // atom_def_dst makes the destination alignment equal to the attribute's.
        clear_ok += m.ps(0) == (old & ~ref_arg_mask[align]) ? 1u : 0u;
        byte_shift_ok += m.ps(1) == merge(field_of(old, align) << 3, old, align) ? 1u : 0u;
        byte_shift_ok += m.ps(2) == merge(field_of(old, align) >> 3, old, align) ? 1u : 0u;

        for (uint8_t sel = 0; sel < 4; ++sel) {
            const int dst = ref_dst_to_src[align][sel];
            // SHL/SHR shift the whole saved destination, then take the field.
            full_shift_ok += m.ps(3u + sel) == merge(field_of(old << 5, dst), old, dst) ? 1u : 0u;
            const uint32_t width_mask = atom::direct_bytes(align) == 4 ? 0x0f0f0f0fu :
                                        atom::direct_bytes(align) == 2 ? 0x0f0fu : 0x0fu;
            const uint32_t width_src = atom::direct_bytes(align) == 4 ? 0x00a000a0u :
                                       atom::direct_bytes(align) == 2 ? 0x00a0u : 0xa0u;
            mask_ok += m.ps(7u + sel) == merge((field_of(old, dst) & width_mask) | width_src, old, dst) ? 1u : 0u;
        }
    }

    expect(clear_ok == 8, "CLEAR clears exactly the attribute's field (atom_def_dst)");
    expect(byte_shift_ok == 16, "SHIFT_LEFT/RIGHT shift the field and drop bits leaving it");
    expect(full_shift_ok == 32, "SHL/SHR use the full saved destination, then mask to the translated field");
    expect(mask_ok == 32, "MASK reads its mask and source at the source width for every selector");

    // CLEAR and the byte shifts ignore attribute bits 7:6 and 2:0 (attr &= 0x38).
    const Image raw = single(0, 0, [](Code& code) {
        code.raw({static_cast<uint8_t>(atom::op::clear + 1), static_cast<uint8_t>((atom::src_byte8 << 3) | 0xc7), 0});
        code.raw({static_cast<uint8_t>(atom::op::shift_left + 1), static_cast<uint8_t>((atom::src_word0 << 3) | 0xc5),
                  1, 4});
    });
    Machine r(raw);
    expect(r.run(0, {0xaabbccddu, 0x12345678u}) == AtomStatus::completed && r.ps(0) == 0xaabb00ddu &&
               r.ps(1) == 0x12346780u,
           "CLEAR and SHIFT_LEFT mask the attribute to bits 5:3 before atom_def_dst");
}

// ---------------------------------------------------------------------------
// The nine special WS indices and persistent context state.
// ---------------------------------------------------------------------------

void workspace_specials() {
    ImageBuilder builder;
    auto& first = builder.command(0, 1, 0).code;
    first.move(ws(atom::ws_quotient), imm32(0x11111111u));
    first.move(ws(atom::ws_remainder), imm32(0x22222222u));
    first.move(ws(atom::ws_dataptr), imm32(0x12345678u));
    first.move(ws(atom::ws_shift), imm32(0x00000123u));
    first.move(ws(atom::ws_or_mask), imm32(0xffffffffu));
    first.move(ws(atom::ws_and_mask), imm32(0));
    first.move(ws(atom::ws_fb_window), imm32(0x87654321u));
    first.move(ws(atom::ws_attributes), imm32(0xabcdef01u));
    first.move(ws(atom::ws_regptr), imm32(0x00ab0010u));

    for (uint8_t index = 0; index < 9; ++index) {
        first.move(ps(index), ws(static_cast<uint8_t>(atom::ws_quotient + index)));
    }

    first.move(reg(0x5), imm32(0x77));
    first.eot();
    // A second table run as a separate execute: what amdgpu_atom_execute_table resets.
    auto& second = builder.command(1, 0, 0).code;

    for (uint8_t index = 0; index < 9; ++index) {
        second.move(ps(index), ws(static_cast<uint8_t>(atom::ws_quotient + index)));
    }

    second.eot();
    auto& third = builder.command(2, 0, 0).code;
    third.move(ws(atom::ws_shift), imm32(31));
    third.move(ps(0), ws(atom::ws_or_mask));
    third.move(ws(atom::ws_shift), imm32(32));
    third.move(ps(1), ws(atom::ws_or_mask));
    third.move(ps(2), ws(atom::ws_and_mask));
    third.eot();
    Machine m(builder.build());
    expect(m.run(0) == AtomStatus::completed, "special WS program runs");
    expect(m.ps(0) == 0x11111111u && m.ps(1) == 0x22222222u, "QUOTIENT and REMAINDER keep 32 bits");
    expect(m.ps(2) == 0x5678u && m.ps(3) == 0x23u, "DATAPTR truncates to u16, SHIFT to u8");
    // shift 0x23 = 35: 1 << 35 on the x86 target is 1 << 3 (atom.c leaves it undefined in C).
    expect(m.ps(4) == 8u && m.ps(5) == ~8u, "OR_MASK / AND_MASK ignore writes and read 1 << shift, ~(1 << shift)");
    expect(m.ps(6) == 0x87654321u && m.ps(7) == 0xef01u && m.ps(8) == 0x10u,
           "FB_WINDOW keeps 32 bits; ATTRIBUTES and REGPTR truncate to u16");
    expect(m.bus.writes_only() == std::vector<Access>{{true, 0x15, 0x77}}, "REGPTR is the REG block offset");
    expect(m.run(1) == AtomStatus::completed && m.ps(0) == 0 && m.ps(1) == 0 && m.ps(2) == 0 && m.ps(3) == 0x23u &&
               m.ps(6) == 0 && m.ps(7) == 0xef01u && m.ps(8) == 0,
           "execute resets quotient, remainder, data block, FB window and REG block; shift and attributes persist");
    expect(m.run(2) == AtomStatus::completed && m.ps(0) == 0x80000000u && m.ps(1) == 1u && m.ps(2) == ~1u,
           "OR_MASK at shift 31 is bit 31; shift 32 wraps to bit 0 (x86 count modulo 32)");
}

// ---------------------------------------------------------------------------
// COMPARE / TEST flags and every jump condition.
// ---------------------------------------------------------------------------

bool condition_holds(Cond cond, uint32_t a, uint32_t b) {
    switch (cond) {
    case Cond::always: return true;
    case Cond::equal: return a == b;
    case Cond::below: return a < b;
    case Cond::above: return a > b;
    case Cond::below_or_equal: return a <= b;
    case Cond::above_or_equal: return a >= b;
    default: return a != b;
    }
}

void compare_and_jump() {
    const std::vector<std::pair<uint32_t, uint32_t>> pairs{
        {5, 3}, {3, 3}, {3, 5}, {0xffffffffu, 1}, {1, 0xffffffffu}, {0, 0}};
    uint32_t correct = 0;

    for (const auto& pair : pairs) {
        const Image image = single(0, 0, [](Code& code) {
            code.compare(ps(0), ps(1));

            for (uint8_t cond = 0; cond < 7; ++cond) {
                const Code::Label taken = code.label();
                const Code::Label next = code.label();
                code.jump(static_cast<Cond>(cond), taken);
                code.move(ps(static_cast<uint8_t>(2 + cond)), imm32(0));
                code.jump(Cond::always, next);
                code.place(taken);
                code.move(ps(static_cast<uint8_t>(2 + cond)), imm32(1));
                code.place(next);
            }
        });
        Machine m(image);
        m.run(0, {pair.first, pair.second, 9, 9, 9, 9, 9, 9, 9});

        for (uint8_t cond = 0; cond < 7; ++cond) {
            correct += m.ps(2u + cond) == (condition_holds(static_cast<Cond>(cond), pair.first, pair.second) ? 1u : 0u)
                           ? 1u : 0u;
        }
    }

    expect(correct == pairs.size() * 7, "COMPARE (unsigned) with all seven jump conditions");

    // TEST sets only cs_equal = (dst & src) == 0; cs_above keeps the COMPARE result.
    const Image test = single(0, 0, [](Code& code) {
        const Code::Label a1 = code.label();
        const Code::Label a2 = code.label();
        const Code::Label a3 = code.label();
        code.compare(ps(0), imm32(3));
        code.test(ps(1), imm32(0x0f));
        code.jump(Cond::equal, a1);
        code.move(ps(4), imm32(0xee));
        code.place(a1);
        code.jump(Cond::above, a2);
        code.move(ps(5), imm32(0xee));
        code.place(a2);
        code.test(ps(1), imm32(0x10));
        code.jump(Cond::not_equal, a3);
        code.move(ps(6), imm32(0xee));
        code.place(a3);
    });
    Machine t(test);
    expect(t.run(0, {9, 0xf0}) == AtomStatus::completed && t.ps(4) == 0 && t.ps(5) == 0 && t.ps(6) == 0,
           "TEST: zero intersection sets equal, cs_above survives, a nonzero intersection clears equal");

    // cs_equal / cs_above live in the shared context across executes.
    ImageBuilder builder;
    auto& set = builder.command(0, 0, 0).code;
    set.compare(ps(0), ps(0));
    set.eot();
    auto& use = builder.command(1, 0, 0).code;
    const Code::Label skip = use.label();
    use.jump(Cond::equal, skip);
    use.move(ps(1), imm32(0xee));
    use.place(skip);
    use.eot();
    Machine p(builder.build());
    expect(p.run(0) == AtomStatus::completed && p.run(1) == AtomStatus::completed && p.ps(1) == 0,
           "comparison flags persist between executes (atom_context cs_equal / cs_above)");
}

// ---------------------------------------------------------------------------
// SWITCH.
// ---------------------------------------------------------------------------

void switch_cases() {
    const auto make = [](Operand src) {
        return single(0, 0, [src](Code& code) {
            const Code::Label one = code.label();
            const Code::Label seven = code.label();
            const Code::Label nine = code.label();
            const Code::Label done = code.label();
            code.switch_(src, {{1, one}, {7, seven}, {9, nine}});
            code.move(ps(1), imm32(0xee));
            code.jump(Cond::always, done);
            code.place(one);
            code.move(ps(1), imm32(0x11));
            code.jump(Cond::always, done);
            code.place(seven);
            code.move(ps(1), imm32(0x77));
            code.jump(Cond::always, done);
            code.place(nine);
            code.move(ps(1), imm32(0x99));
            code.place(done);
        });
    };
    Machine m(make(ps(0)));
    expect(m.run(0, {7}) == AtomStatus::completed && m.ps(1) == 0x77, "SWITCH jumps to the matching case");
    expect(m.run(0, {4}) == AtomStatus::completed && m.ps(1) == 0xee, "SWITCH without a match continues after CASE_END");
    Machine b(make(ps(0).aligned(atom::src_byte8)));
    expect(b.run(0, {0x00000900u}) == AtomStatus::completed && b.ps(1) == 0x99,
           "SWITCH compares the aligned source field with byte-wide case values");
    Machine r(make(reg(0x44).aligned(atom::src_word16)));
    r.bus.registers.at(0x44) = 0x00011234u;
    expect(r.run() == AtomStatus::completed && r.ps(1) == 0x11, "SWITCH on a REG word16 field");

    // A list entry that is neither CASE_MAGIC nor CASE_END: execution continues there.
    const Image bad = single(0, 0, [](Code& code) {
        code.raw({atom::op::switch_, static_cast<uint8_t>((atom::src_byte0 << 3) | 5), 0x02, 0x63, 0x01, 0x06, 0x00,
                  atom::op::nop});
        code.move(ps(1), imm32(0xbad));
    });
    Machine c(bad);
    expect(c.run() == AtomStatus::completed && c.ps(1) == 0xbad && c.report().tolerated_faults == 1 &&
               c.report().first_fault == AtomFault::bad_case,
           "a bad case entry is tolerated and executed as the next opcode");
    Machine s(bad);
    expect(s.run(0, {}, 16, policy_with(1000, 16, 1000, true)) == AtomStatus::tolerated_fault && s.ps(1) == 0 &&
               !s.atom->quarantined(),
           "stop_on_tolerated aborts at a bad case without quarantine");
}

// ---------------------------------------------------------------------------
// CALL_TABLE: parameter shift, per-call workspace, shared context, depth.
// ---------------------------------------------------------------------------

void calls() {
    ImageBuilder builder;
    builder.command_entries(8);
    auto& root = builder.command(0, 1, 8).code;  // ps_shift 2
    root.move(ws(0), imm32(111));
    root.call_table(1);
    root.move(ps(0), ws(0));
    root.call_table(1);
    root.call_table(5);  // master entry 0: skipped
    root.move(ps(1), ws(atom::ws_regptr));
    root.eot();
    auto& one = builder.command(1, 2, 4).code;  // ps_shift 1
    one.move(ps(5), ws(0));
    one.move(ws(0), imm32(222));
    one.move(ps(1), ps(0));
    one.add(ps(6), imm32(1));
    one.call_table(2);
    one.set_reg_block(0x30);
    one.eot();
    auto& two = builder.command(2, 1, 0).code;
    two.add(ps(0), imm32(0x100));
    two.eot();

    struct Depths {
        std::vector<uint32_t> seen;
        static void on(void* context, const AtomInstruction&, uint32_t depth) noexcept {
            static_cast<Depths*>(context)->seen.push_back(depth);
        }
    } depths;

    Machine m(builder.build());
    std::vector<uint32_t> params(16, 0);
    params.at(2) = 0xaa;
    params.at(7) = 0x1234;
    params.at(8) = 5;
    expect(m.run(0, params, 16, atom_default_policy, {&depths, Depths::on}) == AtomStatus::completed,
           "nested call program runs");
    expect(m.ps(0) == 111, "the caller's workspace survives its callees (per-call WS)");
    expect(m.ps(7) == 0, "every call starts with a zeroed workspace (kcalloc)");
    expect(m.ps(3) == 0x1aa && m.ps(8) == 7,
           "callee PS starts ps_shift dwords into the caller's, nested shifts accumulate");
    expect(m.ps(1) == 0x30, "the REG block set by a callee is shared context");
    expect(m.report().calls == 4 && m.report().max_depth == 3 && m.report().tolerated_faults == 0,
           "four calls (absent entry skipped), depth three");
    expect(std::count(depths.seen.begin(), depths.seen.end(), 3u) == 4 &&
               std::count(depths.seen.begin(), depths.seen.end(), 1u) == 7,
           "the observer reports each instruction with its nesting depth");

    // Depth limit: a table calling itself.
    ImageBuilder recursive;
    auto& self = recursive.command(0, 0, 0).code;
    self.add(ps(0), imm32(1));
    self.call_table(0);
    self.eot();
    Machine d(recursive.build());
    expect(d.run(0, {}, 4, policy_with(1000, 5, 1000)) == AtomStatus::call_depth && d.report().max_depth == 5 &&
               d.ps(0) == 5 && d.atom->quarantined(),
           "recursion stops at max_call_depth with call_depth and quarantines");
    Machine full(recursive.build());
    expect(full.run(0, {}, 4) == AtomStatus::call_depth && full.report().max_depth == atom_frame_capacity,
           "the default policy allows exactly the 16 device-owned frames");

    // A callee abort stops the caller at once.
    ImageBuilder aborting;
    auto& caller = aborting.command(0, 0, 0).code;
    caller.call_table(1);
    caller.move(reg(0x10), imm32(1));
    caller.eot();
    auto& callee = aborting.command(1, 0, 0).code;
    callee.move(reg(0x7ff), imm32(1));
    callee.eot();
    Machine a(aborting.build());
    expect(a.run() == AtomStatus::register_range && a.bus.trace.empty() && a.report().failed_table == 1,
           "a callee register outside the aperture aborts the whole execute (no later caller write)");

    // ps_shift beyond the caller's parameter space: an empty window.
    ImageBuilder window;
    auto& wide = window.command(0, 0, 8).code;
    wide.call_table(1);
    wide.eot();
    auto& reader = window.command(1, 0, 0).code;
    reader.move(reg(0x10), ps(0));
    reader.eot();
    Machine w(window.build());
    expect(w.run(0, {0x12345678u}, 1) == AtomStatus::completed && w.bus.registers.at(0x10) == 0xcdcdcdcdu &&
               w.report().tolerated_faults == 2 && w.report().first_fault == AtomFault::ps_window,
           "a caller PS shorter than its ps_shift passes an empty window; reads give 0xCDCDCDCD");

    // Callee table index beyond the command master.
    ImageBuilder beyond;
    beyond.command_entries(2);
    auto& far = beyond.command(0, 0, 0).code;
    far.call_table(9);
    far.eot();
    Machine b(beyond.build());
    expect(b.run() == AtomStatus::malformed_table && b.atom->quarantined(),
           "CALL_TABLE beyond the command master is malformed");

    // An invalid opcode returns from the callee only.
    ImageBuilder invalid;
    auto& outer = invalid.command(0, 0, 0).code;
    outer.call_table(1);
    outer.move(ps(0), imm32(0x600d));
    outer.eot();
    invalid.command(1, 0, 0).code.raw({0x7f});
    Machine i(invalid.build());
    expect(i.run() == AtomStatus::completed && i.ps(0) == 0x600d && i.report().first_fault == AtomFault::invalid_opcode,
           "an invalid opcode leaves only the callee, which returns success");
}

// ---------------------------------------------------------------------------
// SET_ATI_PORT / SET_PCI_PORT / SET_SYS_IO_PORT and IIO programs.
// ---------------------------------------------------------------------------

void ports_and_iio() {
    ImageBuilder builder;
    auto& read = builder.iio_program(1);
    read.move_index(16, 0, 0);
    read.write(0x20);
    read.read(0x21);
    auto& write = builder.iio_program(0x81);
    write.move_index(16, 0, 0);
    write.write(0x20);
    write.move_data(32, 0, 0);
    write.write(0x21);
    auto& fields = builder.iio_program(5);
    fields.set(8, 4);
    fields.clear(4, 8);
    fields.move_attr(8, 0, 24);
    fields.nop();
    fields.write(0x22);
    auto& data = builder.iio_program(0x85);
    data.move_data(8, 8, 0);
    data.write(0x23);
    auto& unknown = builder.iio_program(6);
    unknown.raw(atom::iio::start);
    unknown.raw(0);
    // Width 0 and shift 36: x86 shift counts modulo 32 (atom.c is undefined in C).
    auto& odd = builder.iio_program(8);
    odd.set(0, 0);
    odd.clear(4, 36);
    auto& code = builder.command(0, 0, 0).code;
    code.move(ws(atom::ws_attributes), imm32(0x00c3));
    code.set_port_ati(1);
    code.move(ps(0), reg(0x1234));
    code.move(reg(0x55), imm32(0xcafebabeu));
    code.set_reg_block(0x100);
    code.move(ps(1), reg(0x34));
    code.set_reg_block(0);
    code.set_port_ati(5);
    code.move(ps(2), reg(0x9));
    code.move(reg(0x9), imm32(0x0000ab00u));
    code.set_port_ati(6);
    code.move(ps(3), reg(0x1));
    code.set_port_ati(7);
    code.move(ps(4), reg(0x1));
    code.move(reg(0x1), imm32(5));
    code.set_port_pci(0);
    code.move(ps(5), reg(0x10));
    code.move(reg(0x10), imm32(1));
    code.set_port_sysio(0);
    code.move(ps(6), reg(0x10));
    code.move(reg(0x10), imm32(1));
    code.set_port_ati(8);
    code.move(ps(7), reg(0x10));
    code.set_port_ati(0);
    code.move(reg(0x10), imm32(2));
    code.eot();
    Machine m(builder.build());
    m.bus.registers.at(0x21) = 0x5eed1234u;
    expect(m.atom->iio_programs() == 6 && !m.atom->iio_truncated(), "atom_index_iio indexes six programs");
    std::vector<uint32_t> params(8, 0x77777777u);
    expect(m.run(0, params) == AtomStatus::completed, "port program runs");
    const std::vector<Access> want{
        {true, 0x20, 0xcdcd1234u}, {false, 0x21, 0x5eed1234u},
        {true, 0x20, 0xcdcd0055u}, {true, 0x21, 0xcafebabeu},
        {true, 0x20, 0xcdcd0134u}, {false, 0x21, 0xcafebabeu},
        {true, 0x22, 0xc3cdc0fdu},
        {true, 0x23, 0xcdcdcdabu},
        {true, 0x10, 2},
    };
    expect(m.bus.trace == want, "IIO read (iio[port & 0x7F]) and write (iio[(0x80 | port) & 0xFF]) programs");
    expect(m.ps(0) == 0x5eed1234u && m.ps(1) == 0xcafebabeu && m.ps(2) == 0xc3cdc0fdu,
           "IIO results: READ value, REG block added to the index, SET/CLEAR/MOVE_ATTR fields");
    expect(m.ps(3) == 0 && m.ps(4) == 0 && m.ps(5) == 0 && m.ps(6) == 0,
           "unknown IIO opcode, undefined program, PCI and SYSIO reads return 0");
    expect(m.ps(7) == 0xffffff0fu, "IIO width 0 and shift 36 follow x86 shift counts");
    const auto report = m.report();
    expect(report.tolerated_faults == 7 && report.first_fault == AtomFault::iio_opcode &&
               report.iio_operations == 4 + 5 + 4 + 6 + 3 + 1 + 3,
           "seven tolerated port faults; every IIO operation counted");

    const AtomFault faults[] = {AtomFault::iio_opcode, AtomFault::iio_undefined, AtomFault::pci_port,
                                AtomFault::sysio_port};
    uint32_t stops = 0;

    for (uint32_t index = 0; index < 4; ++index) {
        ImageBuilder stop;
        auto& start = stop.iio_program(6);
        start.raw(atom::iio::start);
        start.raw(0);
        auto& c = stop.command(0, 0, 0).code;

        if (index < 2) {
            c.set_port_ati(index == 0 ? 6 : 7);
        } else if (index == 2) {
            c.set_port_pci(0);
        } else {
            c.set_port_sysio(0);
        }

        c.move(ps(0), reg(0x10));
        c.eot();
        Machine s(stop.build());
        stops += s.run(0, {}, 16, policy_with(1000, 16, 1000, true)) == AtomStatus::tolerated_fault &&
                         s.report().first_fault == faults[index] && !s.atom->quarantined() ? 1u : 0u;
    }

    expect(stops == 4, "stop_on_tolerated turns each port fault into tolerated_fault");

    // IIO REG index beyond the aperture.
    ImageBuilder far;
    far.iio_program(1).read(0x400);
    auto& reader = far.command(0, 0, 0).code;
    reader.set_port_ati(1);
    reader.move(ps(0), reg(0));
    reader.eot();
    Machine f(far.build());
    expect(f.run() == AtomStatus::register_range && f.atom->quarantined() && f.bus.trace.empty(),
           "an IIO READ outside the aperture is register_range");
}

// ---------------------------------------------------------------------------
// SET_REG_BLOCK, SET_FB_BASE and the FB scratch.
// ---------------------------------------------------------------------------

void blocks_and_scratch() {
    ImageBuilder builder;
    auto& first = builder.command(0, 0, 0).code;
    first.set_reg_block(0x10);
    first.move(reg(5), imm32(1));
    first.set_fb_base(imm32(8));
    first.move(fb(0), imm32(0xf00du));
    first.set_fb_base(imm32(0));
    first.move(ps(0), fb(2));
    first.set_fb_base(ps(1));
    first.move(ps(2), fb(0));
    first.set_fb_base(imm32(6));
    first.move(ps(3), fb(1));
    first.set_fb_base(imm32(0));
    first.move(fb(63), imm32(0x63));
    first.move(fb(64), imm32(0x64));
    first.move(ps(4), fb(64));
    first.move(ps(5), fb(63));
    first.move(fb(10), imm32(0x1010));
    first.set_fb_base(imm32(40));
    first.eot();
    auto& second = builder.command(1, 0, 0).code;
    second.move(reg(5), imm32(2));
    second.move(ps(0), fb(10));
    second.move(ps(1), fb(0));
    second.eot();
    Machine m(builder.build());
    expect(m.run(0, {0, 8}) == AtomStatus::completed && m.ps(0) == 0xf00du && m.ps(2) == 0xf00du &&
               m.ps(3) == 0xf00du,
           "SET_FB_BASE moves the FB window by bytes (immediate, PS, unaligned 6 -> dword 1)");
    expect(m.ps(5) == 0x63 && m.ps(4) == 0 && m.report().tolerated_faults == 2 &&
               m.report().first_fault == AtomFault::fb_range,
           "FB index 63 of a 64-dword scratch works; index 64 reads 0 and drops writes (atom.c's '>' would overrun)");
    expect(m.run(1) == AtomStatus::completed && m.ps(0) == 0x1010 && m.ps(1) == 0,
           "the FB scratch persists between executes; the FB window resets");
    expect(m.bus.writes_only() == std::vector<Access>{{true, 0x15, 1}, {true, 0x5, 2}},
           "SET_REG_BLOCK offsets REG indices and resets per execute");

    Machine none(builder.build(), 0);
    expect(none.atom->scratch_bytes() == 0 && none.run(0, {0, 8}) == AtomStatus::completed &&
               none.report().tolerated_faults == 9,
           "without scratch every FB access is a tolerated fb_range fault");
    Machine big(builder.build(), 1u << 20);
    expect(big.atom->scratch_bytes() == atom_scratch_capacity_bytes, "FB scratch requests are clamped to 64 KiB");
}

// ---------------------------------------------------------------------------
// Delays.
// ---------------------------------------------------------------------------

void delays() {
    const Image image = single(0, 0, [](Code& code) {
        code.delay_ms(3);
        code.delay_us(7);
        code.delay_us(0);
        code.delay_ms(0);
    });
    Machine m(image);
    expect(m.run() == AtomStatus::completed && m.bus.delays == std::vector<uint32_t>{3000, 7} &&
               m.report().delay_us == 3007,
           "DELAY_MILLISEC / DELAY_MICROSEC wait through the owner's callback (zero waits skipped)");

    const Image twice = single(0, 0, [](Code& code) {
        code.delay_ms(3);
        code.delay_ms(3);
        code.move(reg(1), imm32(1));
    });
    Machine b(twice);
    const auto offset = b.image.commands.at(0) + atom::command_code_offset + 2;
    expect(b.run(0, {}, 16, policy_with(1000, 16, 5000)) == AtomStatus::delay_budget &&
               b.bus.delays == std::vector<uint32_t>{3000} && b.bus.trace.empty() && !b.atom->quarantined() &&
               b.report().failed_offset == offset && b.report().failed_opcode == atom::op::delay_millisec,
           "a delay beyond the budget is refused before waiting and reported at its offset");
    Machine exact(twice);
    expect(exact.run(0, {}, 16, policy_with(1000, 16, 6000)) == AtomStatus::completed &&
               exact.bus.delays == std::vector<uint32_t>{3000, 3000} && exact.report().delay_us == 6000,
           "delays that reach the budget exactly are still allowed");

    const Image forever = single(0, 0, [](Code& code) {
        const Code::Label top = code.label();
        code.place(top);
        code.delay_ms(255);
        code.jump(Cond::always, top);
    });
    Machine f(forever);
    expect(f.run() == AtomStatus::delay_budget && f.bus.delays.size() == 78 && f.bus.delay_total <= 20000000u &&
               f.report().delay_us == f.bus.delay_total,
           "a delay loop stops inside the 20 s ATOM_CMD_TIMEOUT_SEC budget");
}

// ---------------------------------------------------------------------------
// MUL, DIV, MUL32, DIV32 and the quotient/remainder pair.
// ---------------------------------------------------------------------------

void multiply_divide() {
    const Image image = single(0, 0, [](Code& code) {
        code.div(ps(2), imm32(5));
        code.mul(ps(0), ps(1));
        code.move(ps(3), ws(atom::ws_quotient));
        code.move(ps(4), ws(atom::ws_remainder));
        code.div(ps(2), imm32(0));
        code.move(ps(5), ws(atom::ws_quotient));
        code.move(ps(6), ws(atom::ws_remainder));
        code.mul32(ps(7), ps(7));
        code.move(ps(8), ws(atom::ws_quotient));
        code.move(ps(9), ws(atom::ws_remainder));
        code.move(ws(atom::ws_remainder), imm32(5));
        code.div32(ps(10), imm32(3));
        code.move(ps(11), ws(atom::ws_quotient));
        code.move(ps(12), ws(atom::ws_remainder));
        code.div32(ps(10), imm32(0));
        code.move(ps(13), ws(atom::ws_quotient));
        code.move(ps(14), ws(atom::ws_remainder));
    });
    Machine m(image);
    std::vector<uint32_t> params(16, 0);
    params.at(0) = 0x10000;
    params.at(1) = 0x10001;
    params.at(2) = 17;
    params.at(7) = 0xffffffffu;
    params.at(10) = 6;
    expect(m.run(0, params) == AtomStatus::completed, "multiply/divide program runs");
    expect(m.ps(3) == 0x00010000u && m.ps(4) == 2, "MUL keeps the low 32 bits in QUOTIENT and leaves REMAINDER");
    expect(m.ps(5) == 0 && m.ps(6) == 0, "DIV by zero yields 0 / 0");
    expect(m.ps(8) == 1 && m.ps(9) == 0xfffffffeu, "MUL32: 64-bit product, low in QUOTIENT, high in REMAINDER");
    expect(m.ps(11) == 0xaaaaaaacu && m.ps(12) == 1, "DIV32 divides REMAINDER:dst and splits the 64-bit quotient");
    expect(m.ps(13) == 0 && m.ps(14) == 0, "DIV32 by zero yields 0 / 0");
    expect(m.ps(0) == 0x10000 && m.ps(2) == 17, "MUL/DIV leave their destination unchanged");

    const Image ws_forms = single(2, 0, [](Code& code) {
        code.move(ws(0), imm32(0x80000000u));
        code.mul32(ws(0), imm32(4));
        code.move(ps(0), ws(atom::ws_remainder));
        code.move(ws(1), imm32(100));
        code.div32(ws(1), imm32(7));
        code.move(ps(1), ws(atom::ws_quotient));
        code.move(ps(2), ws(atom::ws_remainder));
        code.div(reg(0x10).selected(1), imm8(3, atom::src_byte8));
        code.move(ps(3), ws(atom::ws_quotient));
    });
    Machine w(ws_forms);
    w.bus.registers.at(0x10) = 0x00001e00u;
    // DIV32 high word 2 (from MUL32): (2 << 32 | 100) / 7.
    expect(w.run() == AtomStatus::completed && w.ps(0) == 2 && w.ps(1) == 0x49249257u && w.ps(2) == 0 &&
               w.ps(3) == 10,
           "WS MUL32/DIV32 forms chain through REMAINDER; DIV reads the aligned destination field");
}

// ---------------------------------------------------------------------------
// Shifts, including counts of 32 and more.
// ---------------------------------------------------------------------------

void shifts() {
    const Image image = single(0, 0, [](Code& code) {
        code.shift_left(ps(0), 4);
        code.shift_right(ps(1), 4);
        code.shift_left(ps(2), 32);
        code.shift_left(ps(3), 33);
        code.shift_right(ps(4), 40);
        code.shl(ps(5), imm32(36));
        code.shl(ps(6), imm32(0x101));
        code.shr(ps(7), imm16(0x220));
        code.shl(ps(8).selected(1), imm8(4));
        code.shift_left(ps(9).aligned(atom::src_byte8), 1);
        code.shr(reg(0x10), ps(10));
    });
    Machine m(image);
    m.bus.registers.at(0x10) = 0x80000000u;
    expect(m.run(0, {0x0f0f0f0fu, 0xf0f0f0f0u, 0x12345678u, 0x12345678u, 0x12345678u, 0x12345678u, 0x12345678u,
                     0x12345678u, 0x12345678u, 0x12348078u, 31}) == AtomStatus::completed,
           "shift program runs");
    expect(m.ps(0) == 0xf0f0f0f0u && m.ps(1) == 0x0f0f0f0fu, "SHIFT_LEFT / SHIFT_RIGHT by a byte count");
    expect(m.ps(2) == 0x12345678u && m.ps(3) == 0x2468acf0u && m.ps(4) == 0x00123456u,
           "byte counts 32, 33 and 40 shift modulo 32 (x86; atom.c is undefined)");
    expect(m.ps(5) == 0x23456780u && m.ps(6) == 0x2468acf0u && m.ps(7) == 0x12345678u,
           "SHL/SHR take the low source byte as the count, then modulo 32");
    expect(m.ps(8) == 0x12346778u && m.ps(9) == 0x12340078u,
           "aligned SHL keeps bits outside the field; SHIFT_LEFT drops the field's carry");
    expect(m.bus.registers.at(0x10) == 1u, "SHR on a REG destination by a PS count");
}

// ---------------------------------------------------------------------------
// Unimplemented, invalid and no-operation opcodes.
// ---------------------------------------------------------------------------

void other_opcodes() {
    const Image unimplemented = single(0, 0, [](Code& code) {
        code.repeat();
        code.save_reg();
        code.restore_reg();
        code.move(ps(0), imm32(0x1234));
    });
    Machine m(unimplemented);
    expect(m.run() == AtomStatus::completed && m.ps(0) == 0x1234 && m.report().tolerated_faults == 3 &&
               m.report().first_fault == AtomFault::unimplemented_opcode && m.report().steps == 5,
           "REPEAT, SAVE_REG and RESTORE_REG continue at the next byte (Linux 'unimplemented!') and are counted");
    Machine s(unimplemented);
    expect(s.run(0, {}, 16, policy_with(1000, 16, 1000, true)) == AtomStatus::tolerated_fault &&
               s.report().failed_opcode == atom::op::repeat && s.ps(0) == 0 && !s.atom->quarantined(),
           "stop_on_tolerated stops at REPEAT");

    uint32_t invalid_ok = 0;

    for (const uint8_t opcode : {uint8_t{0}, uint8_t{127}, uint8_t{200}, uint8_t{255}}) {
        const Image image = single(0, 0, [opcode](Code& code) {
            code.move(ps(0), imm32(1));
            code.raw({opcode});
            code.move(ps(1), imm32(2));
        });
        Machine i(image);
        invalid_ok += i.run() == AtomStatus::completed && i.ps(0) == 1 && i.ps(1) == 0 &&
                              i.report().first_fault == AtomFault::invalid_opcode ? 1u : 0u;
    }

    expect(invalid_ok == 4, "opcodes 0 and 127..255 end the table successfully (atom.c breaks out)");

    std::vector<uint8_t> hidden;
    {
        // The bytes of "MOVE reg(0x30) <- imm32(1)" hidden inside PROCESS_DS data.
        Code inner;
        inner.move(reg(0x30), imm32(1));
        hidden = inner.resolved();
    }

    const Image skipping = single(0, 0, [&](Code& code) {
        code.nop();
        code.beep();
        code.post_card(atom::op::move);
        code.debug(atom::op::move);
        code.process_ds(hidden);
        code.move(reg(0x31), imm32(2));
    });
    Machine k(skipping);
    expect(k.run() == AtomStatus::completed && k.bus.trace == std::vector<Access>{{true, 0x31, 2}} &&
               k.report().steps == 7 && k.report().tolerated_faults == 0,
           "NOP, BEEP, POST_CARD, DEBUG and PROCESS_DS consume their operands without executing them");
}

// ---------------------------------------------------------------------------
// Argument validation, ROM validation, structural failures and I/O errors.
// ---------------------------------------------------------------------------

void validation() {
    const Image simple = single(0, 0, [](Code& code) { code.move(ps(0), imm32(1)); });
    auto unbound = std::make_unique<AtomInterpreter>();
    Bus bus;
    expect(unbound->execute(bus.io(), 0, 0, atom_default_policy) == AtomStatus::invalid_argument &&
               !unbound->bound(),
           "execute before bind is invalid_argument");

    Machine m(simple);
    RegisterIo broken = m.bus.io();
    broken.read32 = nullptr;
    RegisterIo odd = m.bus.io();
    odd.aperture_bytes = 6;
    expect(m.atom->execute(m.bus.io(), 0, 65, atom_default_policy) == AtomStatus::invalid_argument &&
               m.atom->execute(m.bus.io(), 0, 0, policy_with(10, 0, 10)) == AtomStatus::invalid_argument &&
               m.atom->execute(m.bus.io(), 0, 0, policy_with(10, 17, 10)) == AtomStatus::invalid_argument &&
               m.atom->execute(broken, 0, 0, atom_default_policy) == AtomStatus::invalid_argument &&
               m.atom->execute(odd, 0, 0, atom_default_policy) == AtomStatus::invalid_argument,
           "parameter count, call depth and RegisterIo are validated");
    expect(!m.atom->set_parameter(atom_parameter_dwords, 1) && m.atom->parameter(atom_parameter_dwords) == 0 &&
               m.atom->set_parameter(atom_parameter_dwords - 1, 7) &&
               m.atom->parameter(atom_parameter_dwords - 1) == 7,
           "parameter accessors fail closed beyond 64 dwords");
    expect(m.atom->execute(m.bus.io(), 3, 0, atom_default_policy) == AtomStatus::malformed_table,
           "a root table beyond the command master is malformed");
    ImageBuilder gap;
    gap.command_entries(4);
    gap.command(1, 0, 0).code.eot();
    Machine g(gap.build());
    AtomTable table{};
    expect(g.run(0) == AtomStatus::absent_table && !g.atom->quarantined() &&
               g.atom->command_table(0, table) == AtomTableStatus::absent &&
               g.atom->command_table(1, table) == AtomTableStatus::success &&
               table.base == g.image.commands.at(1) &&
               g.atom->command_table(4, table) == AtomTableStatus::invalid_table,
           "absent root table and command_table lookups");

    // ROM validation.
    const auto rejected = [&](const char* what, void (*mutate)(std::vector<uint8_t>&, const Image&)) {
        Image image = simple;
        mutate(image.bytes, image);
        ExactImage rom(image.bytes);
        auto interpreter = std::make_unique<AtomInterpreter>();
        const bool refused = interpreter->bind(rom.view(), 256) == AtomStatus::invalid_rom && !interpreter->bound();
        expect(refused, what);
    };
    rejected("no 0xAA55 BIOS magic", [](std::vector<uint8_t>& b, const Image&) { b.at(0) = 0; });
    rejected("wrong ATI magic", [](std::vector<uint8_t>& b, const Image&) { b.at(0x31) = 'x'; });
    rejected("wrong ATOM magic", [](std::vector<uint8_t>& b, const Image& i) { b.at(i.rom_header + 4) = 'X'; });
    rejected("reversed MOTA signature", [](std::vector<uint8_t>& b, const Image& i) {
        b.at(i.rom_header + 4) = 'M';
        b.at(i.rom_header + 5) = 'O';
        b.at(i.rom_header + 6) = 'T';
        b.at(i.rom_header + 7) = 'A';
    });
    rejected("zero command master pointer", [](std::vector<uint8_t>& b, const Image& i) {
        atom::set_u16(b, i.rom_header + 0x1e, 0);
    });
    rejected("data master outside the image", [](std::vector<uint8_t>& b, const Image& i) {
        atom::set_u16(b, i.rom_header + 0x20, 0xfff0);
    });
    rejected("command master size beyond the image", [](std::vector<uint8_t>& b, const Image& i) {
        atom::set_u16(b, i.command_master, 0xffff);
    });
    rejected("ROM header pointer beyond the image", [](std::vector<uint8_t>& b, const Image&) {
        atom::set_u16(b, 0x48, 0xfffe);
    });
    {
        ExactImage cut(simple.bytes, 0x40);
        auto interpreter = std::make_unique<AtomInterpreter>();
        expect(interpreter->bind(cut.view(), 256) == AtomStatus::invalid_rom, "an image cut before 0x4A");
    }

    // Structural failures quarantine.
    struct Malformed {
        const char* what;
        Image image;
        AtomStatus status;
    };
    std::vector<Malformed> cases;
    cases.push_back({"jump beyond the table", single(0, 0, [](Code& c) { c.jump_raw(Cond::always, 0x400); }),
                     AtomStatus::malformed_table});
    {
        Image image = single(0, 0, [](Code& c) { c.jump_raw(Cond::always, 0); });
        const uint32_t base = image.commands.at(0);
        atom::set_u16(image.bytes, base + atom::command_code_offset + 1, atom::get_u16(image.bytes, base));
        cases.push_back({"jump to the table size", image, AtomStatus::malformed_table});
    }
    cases.push_back({"switch case target beyond the table", single(0, 0, [](Code& c) {
        c.raw({atom::op::switch_, static_cast<uint8_t>((atom::src_byte0 << 3) | 5), 1, 0x63, 1, 0x00, 0x04,
               0x5a, 0x5a});
    }), AtomStatus::malformed_table});
    cases.push_back({"operands truncated by the table end", [] {
        ImageBuilder b;
        b.command(0, 0, 0).code.raw({static_cast<uint8_t>(atom::op::move + 1), 0x05, 0x01});
        return b.build();
    }(), AtomStatus::malformed_table});
    cases.push_back({"code running off the table without EOT", [] {
        ImageBuilder b;
        b.command(0, 0, 0).code.nop();
        return b.build();
    }(), AtomStatus::malformed_table});
    cases.push_back({"ID read beyond the image", single(0, 0, [](Code& c) {
        c.move(ws(atom::ws_dataptr), imm16(0xfff0));
        c.move(ps(0), id(0x20));
    }), AtomStatus::malformed_table});
    cases.push_back({"SET_DATA_BLOCK beyond the data master", single(0, 0, [](Code& c) { c.set_data_block(40); }),
                     AtomStatus::malformed_table});
    cases.push_back({"PROCESS_DS beyond the table", single(0, 0, [](Code& c) {
        c.raw({atom::op::process_ds, 0xff, 0x00});
    }), AtomStatus::malformed_table});
    cases.push_back({"REG outside the aperture", single(0, 0, [](Code& c) { c.move(reg(0x400), imm32(1)); }),
                     AtomStatus::register_range});
    cases.push_back({"REG block pushes the index outside", single(0, 0, [](Code& c) {
        c.set_reg_block(0x3ff);
        c.move(ps(0), reg(1));
    }), AtomStatus::register_range});
    {
        Image image = single(0, 0, [](Code& c) { c.nop(); });
        atom::set_u16(image.bytes, image.commands.at(0), 6);
        cases.push_back({"table header without code", image, AtomStatus::malformed_table});
    }

    for (const auto& item : cases) {
        Machine x(item.image);
        const bool aborted = x.run() == item.status && x.atom->quarantined() && x.bus.trace.empty();
        x.bus.trace.clear();
        expect(aborted && x.run() == AtomStatus::quarantined && x.bus.trace.empty() &&
                   x.report().status == AtomStatus::quarantined,
               item.what);
    }

    {
        // The jump itself is refused (not the fetch after it at base + size).
        Image image = single(0, 0, [](Code& c) { c.jump_raw(Cond::always, 0); });
        const uint32_t base = image.commands.at(0);
        atom::set_u16(image.bytes, base + atom::command_code_offset + 1, atom::get_u16(image.bytes, base));
        Machine j(image);
        expect(j.run() == AtomStatus::malformed_table && j.report().steps == 1 &&
                   j.report().failed_offset == base + atom::command_code_offset &&
                   j.report().failed_opcode == atom::op::jump,
               "a jump to the table size fails at the jump instruction");
    }

    Machine fail(single(0, 0, [](Code& c) {
        c.move(reg(0x10), imm32(1));
        c.move(reg(0x11), imm32(2));
    }));
    fail.bus.fail_at = 1;
    expect(fail.run() == AtomStatus::io_failure && !fail.atom->quarantined() &&
               fail.report().register_writes == 1 && fail.report().failed_offset ==
                   fail.image.commands.at(0) + atom::command_code_offset + 8,
           "a failed register access is io_failure (not structural) at its instruction");
    fail.bus.fail_at = UINT32_MAX;
    expect(fail.run() == AtomStatus::completed, "after an I/O failure the same ROM may run again");

    // stop_on_tolerated for PS, WS and window faults.
    const std::vector<std::pair<AtomFault, Image>> tolerated{
        {AtomFault::ps_index, single(0, 0, [](Code& c) { c.move(ps(0), ps(20)); })},
        {AtomFault::ps_index, single(0, 0, [](Code& c) { c.move(ps(20), imm32(1)); })},
        {AtomFault::ws_index, single(1, 0, [](Code& c) { c.move(ps(0), ws(1)); })},
        {AtomFault::ws_index, single(1, 0, [](Code& c) { c.move(ws(3), imm32(1)); })},
        {AtomFault::fb_range, single(0, 0, [](Code& c) { c.move(fb(200), imm32(1)); })},
    };
    uint32_t stopped = 0;
    uint32_t tolerated_ok = 0;

    for (const auto& item : tolerated) {
        Machine t(item.second);
        tolerated_ok += t.run(0, {}, 16) == AtomStatus::completed && t.report().first_fault == item.first ? 1u : 0u;
        Machine s(item.second);
        stopped += s.run(0, {}, 16, policy_with(100, 16, 100, true)) == AtomStatus::tolerated_fault &&
                           !s.atom->quarantined() ? 1u : 0u;
    }

    expect(tolerated_ok == 5 && stopped == 5, "PS/WS/FB range faults are tolerated, or stop with stop_on_tolerated");
    Machine reads(single(1, 0, [](Code& c) {
        c.move(ps(1), ps(20));
        c.move(ps(2), ws(5));
        c.move(ps(3), ps(20).aligned(atom::src_byte16));
    }));
    expect(reads.run() == AtomStatus::completed && reads.ps(1) == 0xcdcdcdcdu && reads.ps(2) == 0xcdcdcdcdu &&
               reads.ps(3) == 0xcdu,
           "out-of-range PS/WS reads give atom.c's 0xCDCDCDCD, masked by the alignment");
}

// ---------------------------------------------------------------------------
// atom_index_iio bounds.
// ---------------------------------------------------------------------------

void iio_indexing() {
    ImageBuilder builder;
    builder.iio_program(1).read(0x10);
    builder.iio_program(2).write(0x11);
    builder.command(0, 0, 0).code.eot();
    const Image image = builder.build();
    Machine ok(image);
    expect(ok.atom->iio_programs() == 2 && !ok.atom->iio_truncated(), "two IIO programs indexed");

    Image cut = image;
    // The table now ends inside the second program's body.
    atom::set_u16(cut.bytes, cut.iio_table, static_cast<uint16_t>(cut.iio_programs.at(2) - cut.iio_table + 1));
    Machine c(cut);
    expect(c.atom->iio_programs() == 1 && c.atom->iio_truncated(), "a program running past its table stops indexing");

    Image bad = image;
    bad.bytes.at(bad.iio_programs.at(1)) = 0x0a;
    Machine b(bad);
    expect(b.atom->iio_programs() == 0 && b.atom->iio_truncated(), "an opcode above END stops indexing");

    Image outside = image;
    atom::set_u16(outside.bytes, outside.data_master + 4 + 2 * atom::data_indirect_io, 0xfff0);
    Machine o(outside);
    expect(o.bound == AtomStatus::completed && o.atom->iio_programs() == 0 && o.atom->iio_truncated(),
           "an IndirectIOAccess entry outside the image indexes nothing");

    ImageBuilder plain;
    plain.command(0, 0, 0).code.eot();
    Machine p(plain.build());
    expect(p.atom->iio_programs() == 0 && !p.atom->iio_truncated(), "no IndirectIOAccess table: no programs");

    // A runtime IIO program whose body was cut by a later table size change.
    ImageBuilder runtime;
    auto& program = runtime.iio_program(1);

    for (uint32_t index = 0; index < 40; ++index) {
        program.nop();
    }

    auto& code = runtime.command(0, 0, 0).code;
    code.set_port_ati(1);
    code.move(ps(0), reg(1));
    code.eot();
    Machine r(runtime.build());
    expect(r.run(0, {}, 4, policy_with(30, 16, 100)) == AtomStatus::step_budget && r.report().steps == 30 &&
               r.report().iio_operations == 28,
           "IIO operations count against the step budget");
}

// ---------------------------------------------------------------------------
// Budgets.
// ---------------------------------------------------------------------------

void budgets() {
    const Image loop = single(0, 0, [](Code& code) {
        const Code::Label top = code.label();
        code.place(top);
        code.add(ps(0), imm32(1));
        code.jump(Cond::always, top);
    });
    Machine m(loop);
    expect(m.run(0, {}, 4, policy_with(1000, 16, 100)) == AtomStatus::step_budget && m.report().steps == 1000 &&
               m.ps(0) == 500 && !m.atom->quarantined(),
           "an endless loop stops at exactly max_steps without quarantine");
    expect(m.run(0, {}, 4, policy_with(10, 16, 100)) == AtomStatus::step_budget && m.report().steps == 10,
           "the step budget is per execute");
}

// ---------------------------------------------------------------------------
// Rebinding and quarantine persistence.
// ---------------------------------------------------------------------------

void rebinding() {
    const Image broken = single(0, 0, [](Code& code) {
        code.move(fb(1), imm32(0xfeed));
        code.jump_raw(Cond::always, 0x7fff);
    });
    const Image other = single(0, 0, [](Code& code) { code.move(ps(0), fb(1)); });
    ExactImage first(broken.bytes);
    ExactImage copy(broken.bytes);
    ExactImage different(other.bytes);
    auto interpreter = std::make_unique<AtomInterpreter>();
    Bus bus;
    expect(interpreter->bind(first.view(), 256) == AtomStatus::completed &&
               interpreter->execute(bus.io(), 0, 4, atom_default_policy) == AtomStatus::malformed_table &&
               interpreter->quarantined(),
           "a malformed jump quarantines the ROM");
    expect(interpreter->bind(first.view(), 256) == AtomStatus::completed && interpreter->quarantined() &&
               interpreter->execute(bus.io(), 0, 4, atom_default_policy) == AtomStatus::quarantined,
           "rebinding the same view keeps the quarantine");
    expect(interpreter->bind(copy.view(), 256) == AtomStatus::completed && interpreter->quarantined(),
           "rebinding identical content at another address keeps it (size + FNV-1a fingerprint)");
    expect(interpreter->bind(copy.view(), 512) == AtomStatus::completed && !interpreter->quarantined(),
           "the same content with another scratch size starts over");
    interpreter->execute(bus.io(), 0, 4, atom_default_policy);
    expect(interpreter->bind(different.view(), 256) == AtomStatus::completed && !interpreter->quarantined() &&
               interpreter->execute(bus.io(), 0, 4, atom_default_policy) == AtomStatus::completed &&
               interpreter->parameter(0) == 0,
           "different content clears quarantine and starts from a zeroed scratch");
    expect(interpreter->bind(first.view(), 256) == AtomStatus::completed && !interpreter->quarantined(),
           "quarantine is kept only across consecutive binds of the same content");

    // FB scratch persistence across same-content rebinds.
    ImageBuilder builder;
    auto& writer = builder.command(0, 0, 0).code;
    writer.move(fb(3), imm32(0xabc));
    writer.eot();
    auto& reader = builder.command(1, 0, 0).code;
    reader.move(ps(0), fb(3));
    reader.eot();
    const Image keep = builder.build();
    ExactImage a(keep.bytes);
    ExactImage b(keep.bytes);
    auto scratch = std::make_unique<AtomInterpreter>();
    scratch->bind(a.view(), 256);
    scratch->execute(bus.io(), 0, 4, atom_default_policy);
    scratch->bind(b.view(), 256);
    scratch->execute(bus.io(), 1, 4, atom_default_policy);
    const uint32_t kept = scratch->parameter(0);
    scratch->unbind();
    expect(kept == 0xabc && !scratch->bound() && !scratch->quarantined() &&
               scratch->execute(bus.io(), 1, 4, atom_default_policy) == AtomStatus::invalid_argument,
           "same-content rebind keeps the FB scratch; unbind forgets the ROM");
    scratch->bind(a.view(), 256);
    scratch->execute(bus.io(), 1, 4, atom_default_policy);
    expect(scratch->parameter(0) == 0, "binding after unbind starts from a zeroed scratch");

    ExactImage bad(std::vector<uint8_t>(0x100, 0));
    interpreter->bind(first.view(), 256);
    interpreter->execute(bus.io(), 0, 4, atom_default_policy);
    expect(interpreter->bind(bad.view(), 256) == AtomStatus::invalid_rom && !interpreter->bound() &&
               interpreter->bind(first.view(), 256) == AtomStatus::completed && !interpreter->quarantined(),
           "a failed bind unbinds and forgets the quarantine");
    expect(bus.violations == 0, "no rebinding scenario touched a register outside the aperture");
}

// ---------------------------------------------------------------------------
// asic_init at the interpreter level.
// ---------------------------------------------------------------------------

void asic_init_entry() {
    const auto image = [](uint8_t format, uint8_t content) {
        ImageBuilder builder;
        auto& table = builder.command(atom_command_asic_init, 0, 0);
        table.format = format;
        table.content = content;
        table.code.move(reg(0x10), ps(0));
        table.code.move(reg(0x11), ps(1));
        table.code.move(ws(atom::ws_quotient), imm32(0));

        for (uint8_t index = 2; index < 18; ++index) {
            table.code.or_(ws(atom::ws_quotient), ps(index));
        }

        table.code.move(reg(0x12), ws(atom::ws_quotient));
        table.code.move(reg(0x13), ps(18));
        table.code.eot();
        return builder.build();
    };
    Machine m(image(2, 1));

    for (uint32_t index = 0; index < atom_parameter_dwords; ++index) {
        m.atom->set_parameter(index, 0xffffffffu);
    }

    expect(m.atom->asic_init(m.bus.io(), 0x0f123456u, 0x0a654321u, true, atom_default_policy) ==
                   AtomStatus::completed &&
               m.bus.registers.at(0x10) == 0x00123456u && m.bus.registers.at(0x11) == 0x00654321u &&
               m.bus.registers.at(0x12) == 0 && m.bus.registers.at(0x13) == 0xcdcdcdcdu &&
               m.report().tolerated_faults == 1 && m.report().first_fault == AtomFault::ps_index,
           "asic_init passes 24-bit clocks, engineflag 0, 16 zero dwords and exactly 18 PS dwords");
    expect(m.atom->asic_init(m.bus.io(), 100, 200, false, atom_default_policy) == AtomStatus::completed &&
               m.bus.registers.at(0x11) == 0x20000000u + 200 && m.bus.registers.at(0x10) == 100,
           "without fb_reset memflag is b3DRAM_SELF_REFRESH_EXIT (0x20)");
    const AtomAsicInitParameters p = atom_asic_init_parameters(0xffffffffu, 0x01000001u, false);
    expect(p.engine == 0x00ffffffu && p.memory == 0x20000001u, "atom_asic_init_parameters bit layout");
    Machine crev3(image(2, 3));
    Machine frev1(image(1, 1));
    Machine crev0(image(2, 0));
    expect(crev3.atom->asic_init(crev3.bus.io(), 1, 1, true, atom_default_policy) == AtomStatus::completed &&
               frev1.atom->asic_init(frev1.bus.io(), 1, 1, true, atom_default_policy) ==
                   AtomStatus::unsupported_revision &&
               crev0.atom->asic_init(crev0.bus.io(), 1, 1, true, atom_default_policy) ==
                   AtomStatus::unsupported_revision &&
               frev1.bus.trace.empty() && crev0.bus.trace.empty(),
           "ASIC_Init needs format 2 with content >= 1; others run nothing");
    ImageBuilder missing;
    missing.command_entries(2);
    missing.command(1, 0, 0).code.eot();
    Machine a(missing.build());
    auto unbound = std::make_unique<AtomInterpreter>();
    expect(a.atom->asic_init(a.bus.io(), 1, 1, true, atom_default_policy) == AtomStatus::absent_table &&
               unbound->asic_init(a.bus.io(), 1, 1, true, atom_default_policy) == AtomStatus::invalid_argument,
           "a missing ASIC_Init is absent_table; an unbound interpreter is invalid_argument");
}

// ---------------------------------------------------------------------------
// Area 2: randomized programs against the independent reference.
// ---------------------------------------------------------------------------

struct Coverage final {
    std::map<uint8_t, uint32_t> opcodes;
    uint32_t programs = 0;
    uint32_t instructions = 0;
    uint32_t loops = 0;
    uint32_t calls = 0;
    uint32_t switches = 0;
    uint32_t jumps = 0;
    uint32_t iio_operations = 0;
    uint32_t faults = 0;
    uint64_t steps = 0;
    uint32_t mismatches = 0;
};

struct OpcodeCounter final {
    Coverage* coverage;
    std::set<uint32_t>* legal;
    uint32_t illegal = 0;
    static void on(void* context, const AtomInstruction& instruction, uint32_t) noexcept {
        auto& self = *static_cast<OpcodeCounter*>(context);
        ++self.coverage->opcodes[instruction.opcode];

        if (self.legal->count(instruction.offset) == 0) {
            ++self.illegal;
        }
    }
};

void reference_cross_check() {
    constexpr uint32_t programs = 3000;
    atom::Generator generator(0x7900a70a5eedULL);
    Coverage coverage{};
    uint32_t decode_mismatches = 0;
    uint32_t reference_errors = 0;
    uint32_t illegal_offsets = 0;

    for (uint32_t index = 0; index < programs; ++index) {
        const atom::GeneratedProgram program = generator.next();
        ExactImage rom(program.image.bytes);
        auto interpreter = std::make_unique<AtomInterpreter>();
        Bus bus(atom::Generator::register_dwords);
        bus.registers = program.registers;
        std::vector<uint32_t> reference_registers = program.registers;
        atom::Reference reference(program.image.bytes, reference_registers, program.scratch_bytes);
        std::array<uint32_t, atom::Reference::parameter_capacity> params = program.params;
        decode_matches(program.image, decode_mismatches);
        std::set<uint32_t> legal;

        for (const auto& entry : program.image.instructions) {
            for (const auto& emitted : entry.second) {
                legal.insert(program.image.commands.at(entry.first) + atom::command_code_offset + emitted.offset);
            }
        }

        bool same = interpreter->bind(rom.view(), program.scratch_bytes) == AtomStatus::completed;

        for (uint32_t run = 0; run < 2 && same; ++run) {
            for (uint32_t slot = 0; slot < atom_parameter_dwords; ++slot) {
                interpreter->set_parameter(slot, params[slot]);
            }

            const size_t trace_before = bus.trace.size();
            const size_t delays_before = bus.delays.size();
            const size_t reference_trace_before = reference.trace.size();
            const size_t reference_delays_before = reference.delays.size();
            const uint32_t steps_before = reference.steps;
            const uint32_t calls_before = reference.calls;
            const uint32_t faults_before = reference.faults;
            const uint32_t inert_before = reference.inert;
            const uint32_t iio_before = reference.iio_operations;
            const uint64_t delay_before = reference.delay_total;
            reference.max_depth = 0;
            OpcodeCounter counter{&coverage, &legal};
            const AtomStatus status = interpreter->execute(bus.io(), 0, program.ps_dwords, atom_default_policy,
                                                           {&counter, OpcodeCounter::on});
            const int reference_status = reference.execute(0, params, static_cast<int>(program.ps_dwords));
            const AtomExecReport report = interpreter->report();
            illegal_offsets += counter.illegal;

            if (reference.error) {
                ++reference_errors;
            }

            uint32_t reads = 0;
            uint32_t writes = 0;

            for (size_t at = reference_trace_before; at < reference.trace.size(); ++at) {
                (reference.trace.at(at).write ? writes : reads) += 1;
            }

            bool equal = status == AtomStatus::completed && reference_status == 0 && !reference.error &&
                         std::equal(bus.trace.begin() + static_cast<long>(trace_before), bus.trace.end(),
                                    reference.trace.begin() + static_cast<long>(reference_trace_before),
                                    reference.trace.end()) &&
                         std::equal(bus.delays.begin() + static_cast<long>(delays_before), bus.delays.end(),
                                    reference.delays.begin() + static_cast<long>(reference_delays_before),
                                    reference.delays.end()) &&
                         report.steps == reference.steps - steps_before &&
                         report.calls == reference.calls - calls_before &&
                         report.max_depth == reference.max_depth &&
                         report.tolerated_faults == reference.faults - faults_before &&
                         report.inert_pll_mc == reference.inert - inert_before &&
                         report.iio_operations == reference.iio_operations - iio_before &&
                         report.delay_us == reference.delay_total - delay_before &&
                         report.register_reads == reads && report.register_writes == writes &&
                         bus.registers == reference_registers;

            for (uint32_t slot = 0; slot < atom_parameter_dwords; ++slot) {
                equal = equal && interpreter->parameter(slot) == params[slot];
            }

            if (!equal) {
                if (coverage.mismatches < 3) {
                    std::fprintf(stderr, "reference mismatch: program %u run %u status %d/%d steps %u/%u faults %u/%u\n",
                                 index, run, static_cast<int>(status), reference_status, report.steps,
                                 reference.steps - steps_before, report.tolerated_faults,
                                 reference.faults - faults_before);
                }

                ++coverage.mismatches;
                same = false;
            }

            coverage.steps += report.steps;
            coverage.faults += report.tolerated_faults;
            coverage.iio_operations += report.iio_operations;
        }

        ++coverage.programs;
        coverage.instructions += program.instructions;
        coverage.loops += program.loops;
        coverage.calls += program.calls;
        coverage.switches += program.switches;
        coverage.jumps += program.jumps;
    }

    uint32_t executed_opcodes = 0;

    for (const auto& entry : coverage.opcodes) {
        executed_opcodes += entry.second != 0 && entry.first > 0 && entry.first < 127 ? 1u : 0u;
    }

    std::printf("reference cross-check: %u programs x 2 executes, %u generated instructions, %llu steps, "
                "%u distinct valid opcodes executed, %u loops, %u calls, %u switches, %u conditional/forward "
                "jumps, %u IIO operations, %u tolerated faults, %u mismatches\n",
                coverage.programs, coverage.instructions, static_cast<unsigned long long>(coverage.steps),
                executed_opcodes, coverage.loops, coverage.calls, coverage.switches, coverage.jumps,
                coverage.iio_operations, coverage.faults, coverage.mismatches);
    expect(reference_errors == 0, "every generated program stays inside the generator's validity promise");
    expect(decode_mismatches == 0, "the decoder agrees with the encoder on every generated instruction");
    expect(illegal_offsets == 0, "every executed instruction starts at an encoder instruction boundary");
    expect(coverage.mismatches == 0,
           "register traces, delays, PS (WS/special WS/FB via the epilogue), counters match the reference");
    expect(executed_opcodes == 126, "the generated programs executed every one of the 126 valid opcodes");
}

// ---------------------------------------------------------------------------
// Area 3: deterministic mutation fuzzing.
// ---------------------------------------------------------------------------

enum class Mutation : uint32_t {
    random_byte, header, table_size, table_ws_ps, operand, opcode, jump_target, call_index, iio, data_offset,
    command_master, truncate, count
};

const char* mutation_name(Mutation mutation) {
    static const char* names[] = {"random_byte", "header", "table_size", "table_ws_ps", "operand", "opcode",
                                  "jump_target", "call_index", "iio", "data_offset", "command_master", "truncate"};
    return names[static_cast<uint32_t>(mutation)];
}

struct Bounds final {
    size_t size;
    uint32_t illegal = 0;
    uint32_t max_depth;
    static void on(void* context, const AtomInstruction& instruction, uint32_t depth) noexcept {
        auto& self = *static_cast<Bounds*>(context);

        if (instruction.length == 0 || instruction.offset + size_t{instruction.length} > self.size ||
            depth == 0 || depth > self.max_depth) {
            ++self.illegal;
        }
    }
};

void mutate(atom::Rng& rng, Mutation kind, const Image& image, std::vector<uint8_t>& bytes, size_t& size) {
    const auto any_table = [&]() {
        auto it = image.commands.begin();
        std::advance(it, rng.below(static_cast<uint32_t>(image.commands.size())));
        return *it;
    };
    const auto any_instruction = [&](uint8_t& table, Code::Emitted& emitted) {
        const auto chosen = any_table();
        const auto& list = image.instructions.at(chosen.first);
        table = chosen.first;
        emitted = list.at(rng.below(static_cast<uint32_t>(list.size())));
        return chosen.second + atom::command_code_offset;
    };
    const auto poke16 = [&](size_t at, uint16_t value) {
        if (at + 1 < bytes.size()) {
            atom::set_u16(bytes, at, value);
        }
    };
    const auto poke8 = [&](size_t at, uint8_t value) {
        if (at < bytes.size()) {
            bytes.at(at) = value;
        }
    };
    const auto near = [&](uint32_t base) {
        const uint32_t choice = rng.below(4);
        return static_cast<uint16_t>(choice == 0 ? rng.next() : choice == 1 ? base + rng.below(8) - 4u :
                                     choice == 2 ? bytes.size() - rng.below(8) : rng.below(0x100));
    };

    switch (kind) {
    case Mutation::random_byte:
        for (uint32_t count = 1 + rng.below(4); count != 0; --count) {
            bytes.at(rng.below(static_cast<uint32_t>(bytes.size()))) = static_cast<uint8_t>(rng.next());
        }
        break;
    case Mutation::header: {
        const uint32_t choice = rng.below(6);

        if (choice == 0) {
            poke16(0x48, near(image.rom_header));
        } else if (choice == 1) {
            poke16(image.rom_header + 0x1e, near(image.command_master));
        } else if (choice == 2) {
            poke16(image.rom_header + 0x20, near(image.data_master));
        } else if (choice == 3) {
            poke16(image.rom_header, near(0x30));
        } else if (choice == 4) {
            bytes.at(image.rom_header + 4 + rng.below(4)) = static_cast<uint8_t>(rng.next());
        } else {
            bytes.at(rng.percent(50) ? rng.below(2) : 0x30 + rng.below(10)) = static_cast<uint8_t>(rng.next());
        }
        break;
    }
    case Mutation::table_size: {
        const auto table = any_table();
        const uint16_t current = atom::get_u16(bytes, table.second);
        const uint32_t choice = rng.below(4);
        poke16(table.second, static_cast<uint16_t>(choice == 0 ? rng.next() : choice == 1 ? current - 1 - rng.below(8) :
                                                   choice == 2 ? current + 1 + rng.below(8) : rng.below(8)));
        break;
    }
    case Mutation::table_ws_ps: {
        const auto table = any_table();
        bytes.at(table.second + 4 + rng.below(2)) = static_cast<uint8_t>(rng.next());

        if (rng.percent(30)) {
            bytes.at(table.second + 2 + rng.below(2)) = static_cast<uint8_t>(rng.next());
        }
        break;
    }
    case Mutation::operand: {
        uint8_t table = 0;
        Code::Emitted emitted{};
        const uint32_t base = any_instruction(table, emitted);

        if (emitted.length > 1) {
            bytes.at(base + emitted.offset + 1 + rng.below(emitted.length - 1)) = static_cast<uint8_t>(rng.next());
        }
        break;
    }
    case Mutation::opcode: {
        uint8_t table = 0;
        Code::Emitted emitted{};
        const uint32_t base = any_instruction(table, emitted);
        bytes.at(base + emitted.offset) = static_cast<uint8_t>(rng.percent(50) ? rng.below(127) : rng.next());
        break;
    }
    case Mutation::jump_target: {
        for (uint32_t attempt = 0; attempt < 8; ++attempt) {
            uint8_t table = 0;
            Code::Emitted emitted{};
            const uint32_t base = any_instruction(table, emitted);

            if (emitted.target_at != UINT32_MAX) {
                const uint16_t size_field = atom::get_u16(bytes, image.commands.at(table));
                const uint32_t choice = rng.below(3);
                poke16(base + emitted.target_at, static_cast<uint16_t>(choice == 0 ? rng.next() :
                                                 choice == 1 ? size_field - 2 + rng.below(4) : rng.below(size_field)));
                break;
            }
        }
        break;
    }
    case Mutation::call_index: {
        for (uint32_t attempt = 0; attempt < 16; ++attempt) {
            uint8_t table = 0;
            Code::Emitted emitted{};
            const uint32_t base = any_instruction(table, emitted);

            if (emitted.opcode == atom::op::call_table) {
                bytes.at(base + emitted.offset + 1) = static_cast<uint8_t>(rng.percent(50) ? rng.below(12) : rng.next());
                break;
            }
        }
        break;
    }
    case Mutation::iio: {
        if (image.iio_table == 0) {
            bytes.at(rng.below(static_cast<uint32_t>(bytes.size()))) = static_cast<uint8_t>(rng.next());
            break;
        }

        const uint16_t size_field = atom::get_u16(bytes, image.iio_table);
        const uint32_t choice = rng.below(4);

        if (choice == 0) {
            poke16(image.iio_table, static_cast<uint16_t>(size_field - rng.below(12)));
        } else if (choice == 1) {
            poke16(image.data_master + 4 + 2 * atom::data_indirect_io, near(image.iio_table));
        } else {
            poke8(image.iio_table + 4 + rng.below(size_field > 4 ? size_field - 4u : 1u),
                  static_cast<uint8_t>(rng.percent(50) ? rng.below(12) : rng.next()));
        }
        break;
    }
    case Mutation::data_offset: {
        const uint32_t choice = rng.below(3);

        if (choice == 0) {
            poke16(image.data_master + 4 + 2 * rng.below(35), near(image.data_master));
        } else {
            for (uint32_t attempt = 0; attempt < 16; ++attempt) {
                uint8_t table = 0;
                Code::Emitted emitted{};
                const uint32_t base = any_instruction(table, emitted);

                if (emitted.opcode == atom::op::set_data_block) {
                    bytes.at(base + emitted.offset + 1) = static_cast<uint8_t>(rng.next());
                    break;
                }

                // An ID source operand word (attribute arg 4).
                if (emitted.length >= 5 && (bytes.at(base + emitted.offset + 1) & 7u) == 4u) {
                    poke16(base + emitted.offset + emitted.length - 2, static_cast<uint16_t>(rng.next()));
                    break;
                }
            }
        }
        break;
    }
    case Mutation::command_master: {
        const uint16_t entries = static_cast<uint16_t>((atom::get_u16(bytes, image.command_master) - 4) / 2);
        const uint32_t choice = rng.below(3);

        if (choice == 0) {
            poke16(image.command_master, static_cast<uint16_t>(rng.below(24)));
        } else {
            poke16(image.command_master + 4 + 2 * rng.below(entries == 0 ? 1u : entries),
                   near(image.commands.begin()->second));
        }
        break;
    }
    case Mutation::truncate:
        size = rng.below(static_cast<uint32_t>(bytes.size()));
        break;
    default:
        break;
    }
}

void mutation_fuzz() {
    constexpr uint32_t iterations = 24000;
    atom::Generator generator(0xf022a70aULL);
    std::vector<atom::GeneratedProgram> bases;

    for (uint32_t index = 0; index < 48; ++index) {
        bases.push_back(generator.next());
    }

    atom::Rng rng(0x0dd5eedULL);
    std::unique_ptr<ExactImage> resident;
    auto interpreter = std::make_unique<AtomInterpreter>();
    std::map<std::string, uint32_t> statuses;
    std::array<uint32_t, static_cast<uint32_t>(Mutation::count)> per_class{};
    uint32_t bind_failures = 0;
    uint32_t violations = 0;
    uint32_t illegal = 0;
    uint32_t over_budget = 0;
    uint32_t counter_mismatch = 0;
    uint32_t quarantine_errors = 0;
    uint32_t quarantines = 0;
    uint32_t persistence_ok = 0;
    uint32_t asic_init_runs = 0;
    uint64_t total_steps = 0;
    static const char* status_names[] = {"completed", "invalid_argument", "invalid_rom", "absent_table",
                                         "unsupported_revision", "malformed_table", "step_budget", "delay_budget",
                                         "call_depth", "register_range", "io_failure", "tolerated_fault",
                                         "quarantined"};

    for (uint32_t iteration = 0; iteration < iterations; ++iteration) {
        const atom::GeneratedProgram& base = bases.at(rng.below(static_cast<uint32_t>(bases.size())));
        std::vector<uint8_t> bytes = base.image.bytes;
        size_t size = bytes.size();
        const uint32_t count = 1 + rng.below(3);

        for (uint32_t index = 0; index < count; ++index) {
            const auto kind = static_cast<Mutation>(rng.below(static_cast<uint32_t>(Mutation::count)));
            ++per_class.at(static_cast<uint32_t>(kind));
            mutate(rng, kind, base.image, bytes, size);
        }

        // The bound view stays resident until the next bind (the bind contract).
        auto rom = std::make_unique<ExactImage>(bytes, size);
        const AtomStatus bound = interpreter->bind(rom->view(), base.scratch_bytes);
        resident = std::move(rom);

        if (bound != AtomStatus::completed) {
            ++bind_failures;
            ++statuses["bind:invalid_rom"];
            continue;
        }

        Bus bus(atom::Generator::register_dwords);
        bus.registers = base.registers;
        const AtomPolicy policy = policy_with(20000, 1 + rng.below(atom_frame_capacity), 200000 + rng.below(2000000),
                                              rng.percent(20));
        Bounds bounds{size, 0, policy.max_call_depth};

        for (uint32_t slot = 0; slot < atom_parameter_dwords; ++slot) {
            interpreter->set_parameter(slot, base.params[slot]);
        }

        AtomStatus status = AtomStatus::completed;

        if (rng.percent(10)) {
            ++asic_init_runs;
            status = interpreter->asic_init(bus.io(), rng.next(), rng.next(), rng.percent(50), policy);
        } else {
            const uint8_t table = static_cast<uint8_t>(rng.percent(80) ? 0 : rng.below(10));
            status = interpreter->execute(bus.io(), table, rng.percent(80) ? base.ps_dwords : rng.below(65), policy,
                                          {&bounds, Bounds::on});
        }

        const AtomExecReport report = interpreter->report();
        ++statuses[status_names[static_cast<uint32_t>(status)]];
        total_steps += report.steps;
        violations += bus.violations;
        illegal += bounds.illegal;

        if (report.steps > policy.max_steps || bus.delay_total > policy.max_delay_us ||
            report.max_depth > policy.max_call_depth) {
            ++over_budget;
        }

        if (report.delay_us != bus.delay_total || report.register_reads != bus.reads ||
            report.register_writes != bus.writes || report.status != status) {
            ++counter_mismatch;
        }

        const bool structural = status == AtomStatus::malformed_table || status == AtomStatus::register_range ||
                                status == AtomStatus::call_depth;

        if (structural != interpreter->quarantined() && status != AtomStatus::quarantined) {
            ++quarantine_errors;
        }

        if (structural) {
            ++quarantines;
            // The same content stays refused, with no register access, after a rebind.
            auto again = std::make_unique<ExactImage>(bytes, size);
            Bus quiet(atom::Generator::register_dwords);
            const bool rebound = interpreter->bind(again->view(), base.scratch_bytes) == AtomStatus::completed;
            resident = std::move(again);

            if (rebound && interpreter->quarantined() &&
                interpreter->execute(quiet.io(), 0, base.ps_dwords, policy) == AtomStatus::quarantined &&
                quiet.trace.empty() && quiet.delays.empty()) {
                ++persistence_ok;
            }
        }
    }

    std::printf("mutation fuzz: %u iterations (%u ASIC_Init entries), %llu interpreter steps; mutations:",
                iterations, asic_init_runs, static_cast<unsigned long long>(total_steps));

    for (uint32_t index = 0; index < per_class.size(); ++index) {
        std::printf(" %s=%u", mutation_name(static_cast<Mutation>(index)), per_class.at(index));
    }

    std::printf("\nmutation fuzz outcomes:");

    for (const auto& entry : statuses) {
        std::printf(" %s=%u", entry.first.c_str(), entry.second);
    }

    std::printf("\n");
    expect(violations == 0, "fuzz: no register access outside the aperture");
    expect(illegal == 0, "fuzz: every decoded instruction lies inside the image, depth within policy");
    expect(over_budget == 0, "fuzz: step, delay and depth budgets hold for every execution");
    expect(counter_mismatch == 0, "fuzz: report counters match the bus");
    expect(quarantine_errors == 0, "fuzz: quarantine set exactly for malformed_table/register_range/call_depth");
    expect(persistence_ok == quarantines, "fuzz: quarantine persists across rebinding the same content");
    expect(statuses["completed"] > 0 && statuses["malformed_table"] > 0 && statuses["step_budget"] > 0 &&
               statuses["register_range"] > 0 && bind_failures > 0,
           "fuzz reached completion, malformed tables, step budgets, register range and bind refusals");
}

} // namespace

int main() {
    decoder();
    sources();
    destinations();
    alignment();
    workspace_specials();
    compare_and_jump();
    switch_cases();
    calls();
    ports_and_iio();
    blocks_and_scratch();
    delays();
    multiply_divide();
    shifts();
    other_opcodes();
    validation();
    iio_indexing();
    budgets();
    rebinding();
    asic_init_entry();
    reference_cross_check();
    mutation_fuzz();
    return checks.finish("atom_interpreter");
}

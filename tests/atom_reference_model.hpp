/*
Module: rx7900.tests.atom_reference_model
Purpose: Independent atom.c reference semantics and a random valid ATOM program generator.
Created: 2026-10-24
Architecture: Test-only. Reference is a second, deliberately literal C++ reading of
Linux v6.12 atom.c (recursive amdgpu_atom_execute_table_locked, atom_get_src_int,
atom_put_dst, atom_op_*, atom_iio_execute, atom_index_iio) written from the pinned
upstream file, not from src/hardware/atom_interpreter.cpp. Where atom.c is undefined
or reads outside its buffers it follows the port's documented choices (shift counts
modulo 32, FB index fb_base / 4 + idx below the scratch dword count, an empty
callee parameter window when ps_shift exceeds ps_size) so valid programs compare
exactly. Generator emits only valid programs: forward jumps and switch targets,
counted loops on a reserved workspace slot, acyclic CALL_TABLE, in-aperture
registers and in-image data reads; the root table ends with an epilogue that
copies WS, the special WS values and the FB scratch to PS and registers.
*/
#pragma once

#include <array>

#include "atom_test_builder.hpp"

namespace rx7900_test::atom {

// One register access as the device saw it (dword index).
struct Access final {
    bool write;
    uint32_t index;
    uint32_t value;

    bool operator==(const Access& other) const {
        return write == other.write && index == other.index && value == other.value;
    }
};

// atom.c tables, copied from the pinned upstream file.
constexpr uint32_t ref_arg_mask[8] = {0xFFFFFFFFu, 0xFFFFu, 0xFFFF00u, 0xFFFF0000u,
                                      0xFFu, 0xFF00u, 0xFF0000u, 0xFF000000u};
constexpr uint32_t ref_arg_shift[8] = {0, 0, 8, 16, 0, 8, 16, 24};
constexpr int ref_dst_to_src[8][4] = {{0, 0, 0, 0}, {1, 2, 3, 0}, {1, 2, 3, 0}, {1, 2, 3, 0},
                                      {4, 5, 6, 7}, {4, 5, 6, 7}, {4, 5, 6, 7}, {4, 5, 6, 7}};
constexpr int ref_def_dst[8] = {0, 0, 1, 2, 0, 1, 2, 3};
constexpr int ref_iio_len[] = {1, 2, 3, 3, 3, 3, 4, 4, 4, 3};

// x86 shift semantics for the counts atom.c leaves undefined in C.
inline uint32_t ref_shl(uint32_t value, uint32_t count) { return value << (count & 31u); }
inline uint32_t ref_shr(uint32_t value, uint32_t count) { return value >> (count & 31u); }

class Reference final {
public:
    static constexpr uint32_t parameter_capacity = 64;

    Reference(const std::vector<uint8_t>& rom, std::vector<uint32_t>& registers, uint32_t scratch_bytes)
        : rom_(rom), registers_(registers), scratch_(scratch_bytes / 4, 0), scratch_size_bytes_(scratch_bytes) {
        // amdgpu_atom_parse.
        const uint32_t base = u16(0x48);
        cmd_table_ = u16(base + 0x1e);
        data_table_ = u16(base + 0x20);
        index_iio(u16(data_table_ + 0x32) + 4u);
    }

    // amdgpu_atom_execute_table on params (parameter_capacity dwords).
    int execute(int index, std::array<uint32_t, parameter_capacity>& params, int params_size) {
        data_block_ = 0;
        reg_block_ = 0;
        fb_base_ = 0;
        io_mode_ = io_mm;
        divmul_[0] = 0;
        divmul_[1] = 0;
        params_ = &params;
        return execute_locked(index, 0, params_size);
    }

    std::vector<Access> trace;
    std::vector<uint32_t> delays;
    uint64_t delay_total = 0;
    uint32_t steps = 0;
    uint32_t calls = 0;
    uint32_t depth = 0;
    uint32_t max_depth = 0;
    uint32_t faults = 0;
    uint32_t inert = 0;
    uint32_t iio_operations = 0;
    // Set when a program leaves what the generator promises (a test failure).
    bool error = false;

private:
    static constexpr int io_mm = 0;
    static constexpr int io_pci = 1;
    static constexpr int io_sysio = 2;
    static constexpr int io_iio = 0x80;

    struct Exec final {
        uint32_t ps = 0;
        int ps_size = 0;
        std::vector<uint32_t> ws;
        int ws_size = 0;
        int ps_shift = 0;
        uint32_t start = 0;
        bool abort = false;
    };

    uint8_t u8(uint32_t at) {
        if (at >= rom_.size()) {
            error = true;
            return 0;
        }

        return rom_[at];
    }

    uint16_t u16(uint32_t at) { return static_cast<uint16_t>(u8(at) | (u8(at + 1) << 8)); }
    uint32_t u32(uint32_t at) { return u16(at) | (static_cast<uint32_t>(u16(at + 2)) << 16); }

    uint32_t& param(const Exec& ctx, uint32_t index) {
        const uint32_t at = ctx.ps + index;

        if (at >= parameter_capacity) {
            error = true;
            return dummy_;
        }

        return (*params_)[at];
    }

    uint32_t reg_read(uint32_t index) {
        if (index >= registers_.size()) {
            error = true;
            return 0;
        }

        trace.push_back({false, index, registers_[index]});
        return registers_[index];
    }

    void reg_write(uint32_t index, uint32_t value) {
        if (index >= registers_.size()) {
            error = true;
            return;
        }

        trace.push_back({true, index, value});
        registers_[index] = value;
    }

    void index_iio(uint32_t base) {
        // A zero IndirectIOAccess entry indexes nothing in the port; atom.c would
        // walk from offset 4, which the generator never relies on.
        if (u16(data_table_ + 0x32) == 0) {
            return;
        }

        while (u8(base) == 1) {
            iio_[u8(base + 1)] = static_cast<uint16_t>(base + 2);
            base += 2;

            while (u8(base) != 9 && !error) {
                base += static_cast<uint32_t>(ref_iio_len[u8(base)]);
            }

            base += 3;
        }
    }

    uint32_t iio_execute(uint32_t base, uint32_t index, uint32_t data) {
        uint32_t temp = 0xCDCDCDCD;

        for (;;) {
            ++steps;
            ++iio_operations;
            const auto field = [&](uint32_t width_at) { return ref_shr(0xFFFFFFFFu, 32u - u8(width_at)); };

            switch (u8(base)) {
            case 0:
                base += 1;
                break;
            case 2:
                temp = reg_read(u16(base + 1));
                base += 3;
                break;
            case 3:
                reg_write(u16(base + 1), temp);
                base += 3;
                break;
            case 4:
                temp &= ~ref_shl(field(base + 1), u8(base + 2));
                base += 3;
                break;
            case 5:
                temp |= ref_shl(field(base + 1), u8(base + 2));
                base += 3;
                break;
            case 6:
                temp &= ~ref_shl(field(base + 1), u8(base + 3));
                temp |= ref_shl(ref_shr(index, u8(base + 2)) & field(base + 1), u8(base + 3));
                base += 4;
                break;
            case 8:
                temp &= ~ref_shl(field(base + 1), u8(base + 3));
                temp |= ref_shl(ref_shr(data, u8(base + 2)) & field(base + 1), u8(base + 3));
                base += 4;
                break;
            case 7:
                temp &= ~ref_shl(field(base + 1), u8(base + 3));
                temp |= ref_shl(ref_shr(io_attr_, u8(base + 2)) & field(base + 1), u8(base + 3));
                base += 4;
                break;
            case 9:
                return temp;
            default:
                // "Unknown IIO opcode".
                ++faults;
                return 0;
            }

            if (error) {
                return 0;
            }
        }
    }

    uint32_t get_src_int(Exec& ctx, uint8_t attr, uint32_t& ptr, uint32_t* saved) {
        uint32_t idx = 0;
        uint32_t val = 0xCDCDCDCD;
        const uint32_t align = (attr >> 3) & 7u;

        switch (attr & 7u) {
        case 0:
            idx = u16(ptr);
            ptr += 2;
            idx += reg_block_;

            switch (io_mode_) {
            case io_mm:
                val = reg_read(idx);
                break;
            case io_pci:
            case io_sysio:
                ++faults;
                return 0;
            default:
                if (iio_[io_mode_ & 0x7F] == 0) {
                    ++faults;
                    return 0;
                }

                val = iio_execute(iio_[io_mode_ & 0x7F], idx, 0);
            }

            break;
        case 1:
            idx = u8(ptr);
            ptr += 1;

            if (static_cast<int>(idx) < ctx.ps_size) {
                val = param(ctx, idx);
            } else {
                ++faults;
            }

            break;
        case 2:
            idx = u8(ptr);
            ptr += 1;

            switch (idx) {
            case 0x40: val = divmul_[0]; break;
            case 0x41: val = divmul_[1]; break;
            case 0x42: val = data_block_; break;
            case 0x43: val = shift_; break;
            case 0x44: val = ref_shl(1u, shift_); break;
            case 0x45: val = ~ref_shl(1u, shift_); break;
            case 0x46: val = fb_base_; break;
            case 0x47: val = io_attr_; break;
            case 0x48: val = reg_block_; break;
            default:
                if (static_cast<int>(idx) < ctx.ws_size) {
                    val = ctx.ws[idx];
                } else {
                    ++faults;
                }
            }

            break;
        case 4:
            idx = u16(ptr);
            ptr += 2;
            val = u32(idx + data_block_);
            break;
        case 3:
            idx = u8(ptr);
            ptr += 1;

            if (fb_base_ / 4u + idx >= scratch_size_bytes_ / 4u) {
                ++faults;
                val = 0;
            } else {
                val = scratch_[fb_base_ / 4u + idx];
            }

            break;
        case 5:
            if (align == 0) {
                val = u32(ptr);
                ptr += 4;
            } else if (align < 4) {
                val = u16(ptr);
                ptr += 2;
            } else {
                val = u8(ptr);
                ptr += 1;
            }

            return val;
        default:
            // PLL and MC: cail_pll_read / cail_mc_read return 0.
            ptr += 1;
            ++inert;
            val = 0;
            break;
        }

        if (saved != nullptr) {
            *saved = val;
        }

        val &= ref_arg_mask[align];
        val >>= ref_arg_shift[align];
        return val;
    }

    uint32_t get_src(Exec& ctx, uint8_t attr, uint32_t& ptr) { return get_src_int(ctx, attr, ptr, nullptr); }

    uint32_t get_src_direct(uint8_t align, uint32_t& ptr) {
        uint32_t val = 0xCDCDCDCD;

        if (align == 0) {
            val = u32(ptr);
            ptr += 4;
        } else if (align < 4) {
            val = u16(ptr);
            ptr += 2;
        } else {
            val = u8(ptr);
            ptr += 1;
        }

        return val;
    }

    uint32_t get_dst(Exec& ctx, int arg, uint8_t attr, uint32_t& ptr, uint32_t* saved) {
        return get_src_int(ctx, static_cast<uint8_t>(arg | ref_dst_to_src[(attr >> 3) & 7][(attr >> 6) & 3] << 3),
                           ptr, saved);
    }

    void skip_dst(int arg, uint8_t attr, uint32_t& ptr) {
        const int align = ref_dst_to_src[(attr >> 3) & 7][(attr >> 6) & 3];

        if (arg == 0 || arg == 4) {
            ptr += 2;
        } else if (arg == 5) {
            ptr += align == 0 ? 4u : align < 4 ? 2u : 1u;
        } else {
            ptr += 1;
        }
    }

    void put_dst(Exec& ctx, int arg, uint8_t attr, uint32_t& ptr, uint32_t val, uint32_t saved) {
        const int align = ref_dst_to_src[(attr >> 3) & 7][(attr >> 6) & 3];
        uint32_t idx = 0;
        val <<= ref_arg_shift[align];
        val &= ref_arg_mask[align];
        saved &= ~ref_arg_mask[align];
        val |= saved;

        switch (arg) {
        case 0:
            idx = u16(ptr);
            ptr += 2;
            idx += reg_block_;

            switch (io_mode_) {
            case io_mm:
                reg_write(idx, idx == 0 ? val << 2 : val);
                break;
            case io_pci:
            case io_sysio:
                ++faults;
                return;
            default:
                if (iio_[io_mode_ & 0xFF] == 0) {
                    ++faults;
                    return;
                }

                iio_execute(iio_[io_mode_ & 0xFF], idx, val);
            }

            break;
        case 1:
            idx = u8(ptr);
            ptr += 1;

            if (static_cast<int>(idx) >= ctx.ps_size) {
                ++faults;
                return;
            }

            param(ctx, idx) = val;
            break;
        case 2:
            idx = u8(ptr);
            ptr += 1;

            switch (idx) {
            case 0x40: divmul_[0] = val; break;
            case 0x41: divmul_[1] = val; break;
            case 0x42: data_block_ = static_cast<uint16_t>(val); break;
            case 0x43: shift_ = static_cast<uint8_t>(val); break;
            case 0x44:
            case 0x45: break;
            case 0x46: fb_base_ = val; break;
            case 0x47: io_attr_ = static_cast<uint16_t>(val); break;
            case 0x48: reg_block_ = static_cast<uint16_t>(val); break;
            default:
                if (static_cast<int>(idx) >= ctx.ws_size) {
                    ++faults;
                    return;
                }

                ctx.ws[idx] = val;
            }

            break;
        case 3:
            idx = u8(ptr);
            ptr += 1;

            if (fb_base_ / 4u + idx >= scratch_size_bytes_ / 4u) {
                ++faults;
            } else {
                scratch_[fb_base_ / 4u + idx] = val;
            }

            break;
        default:
            // PLL and MC: cail_pll_write / cail_mc_write do nothing.
            ptr += 1;
            ++inert;
            break;
        }
    }

    // The two-operand read-modify-write family (atom_op_add and friends).
    template <typename F>
    void binary(Exec& ctx, uint32_t& ptr, int arg, F apply) {
        const uint8_t attr = u8(ptr++);
        uint32_t saved = 0;
        uint32_t dptr = ptr;
        uint32_t dst = get_dst(ctx, arg, attr, ptr, &saved);
        const uint32_t src = get_src(ctx, attr, ptr);
        dst = apply(dst, src);
        put_dst(ctx, arg, attr, dptr, dst, saved);
    }

    // Destination kind of a six-entry opcode group: REG, PS, WS, FB, PLL, MC.
    static int group_arg(uint8_t opcode, uint8_t first) {
        constexpr int kinds[6] = {0, 1, 2, 3, 6, 7};
        return kinds[opcode - first];
    }

    void op_shift_bytes(Exec& ctx, uint32_t& ptr, int arg, bool left) {
        uint8_t attr = u8(ptr++);
        uint32_t saved = 0;
        uint32_t dptr = ptr;
        attr &= 0x38;
        attr = static_cast<uint8_t>(attr | ref_def_dst[attr >> 3] << 6);
        uint32_t dst = get_dst(ctx, arg, attr, ptr, &saved);
        const uint8_t shift = static_cast<uint8_t>(get_src_direct(4, ptr));
        dst = left ? ref_shl(dst, shift) : ref_shr(dst, shift);
        put_dst(ctx, arg, attr, dptr, dst, saved);
    }

    void op_shift_full(Exec& ctx, uint32_t& ptr, int arg, bool left) {
        const uint8_t attr = u8(ptr++);
        uint32_t saved = 0;
        uint32_t dptr = ptr;
        const int dst_align = ref_dst_to_src[(attr >> 3) & 7][(attr >> 6) & 3];
        uint32_t dst = get_dst(ctx, arg, attr, ptr, &saved);
        dst = saved;
        const uint8_t shift = static_cast<uint8_t>(get_src(ctx, attr, ptr));
        dst = left ? ref_shl(dst, shift) : ref_shr(dst, shift);
        dst &= ref_arg_mask[dst_align];
        dst >>= ref_arg_shift[dst_align];
        put_dst(ctx, arg, attr, dptr, dst, saved);
    }

    void op_switch(Exec& ctx, uint32_t& ptr) {
        const uint8_t attr = u8(ptr++);
        const uint32_t src = get_src(ctx, attr, ptr);

        while (u16(ptr) != 0x5A5A && !error) {
            if (u8(ptr) == 0x63) {
                ptr++;
                const uint32_t val = get_src(ctx, static_cast<uint8_t>((attr & 0x38) | 5), ptr);
                const uint32_t target = u16(ptr);

                if (val == src) {
                    ptr = ctx.start + target;
                    return;
                }

                ptr += 2;
            } else {
                // "Bad case".
                ++faults;
                return;
            }
        }

        ptr += 2;
    }

    void op_calltable(Exec& ctx, uint32_t& ptr) {
        const int idx = u8(ptr++);

        if (u16(cmd_table_ + 4 + 2 * static_cast<uint32_t>(idx)) == 0) {
            return;
        }

        uint32_t offset = ctx.ps + static_cast<uint32_t>(ctx.ps_shift);
        int size = ctx.ps_size - ctx.ps_shift;

        if (size < 0) {
            // ps_shift beyond ps_size: an empty window (see the module comment).
            ++faults;
            offset = ctx.ps + static_cast<uint32_t>(ctx.ps_size);
            size = 0;
        }

        ++calls;

        if (execute_locked(idx, offset, size) != 0) {
            ctx.abort = true;
        }
    }

    void dispatch(Exec& ctx, uint8_t opcode, uint32_t& ptr) {
        const auto in_group = [opcode](uint8_t first) { return opcode >= first && opcode < first + 6; };

        if (in_group(1)) {
            const int arg = group_arg(opcode, 1);
            const uint8_t attr = u8(ptr++);
            uint32_t saved = 0;
            uint32_t dptr = ptr;

            if (((attr >> 3) & 7) != 0) {
                get_dst(ctx, arg, attr, ptr, &saved);
            } else {
                skip_dst(arg, attr, ptr);
                saved = 0xCDCDCDCD;
            }

            const uint32_t src = get_src(ctx, attr, ptr);
            put_dst(ctx, arg, attr, dptr, src, saved);
        } else if (in_group(7)) {
            binary(ctx, ptr, group_arg(opcode, 7), [](uint32_t d, uint32_t s) { return d & s; });
        } else if (in_group(13)) {
            binary(ctx, ptr, group_arg(opcode, 13), [](uint32_t d, uint32_t s) { return d | s; });
        } else if (in_group(19)) {
            op_shift_bytes(ctx, ptr, group_arg(opcode, 19), true);
        } else if (in_group(25)) {
            op_shift_bytes(ctx, ptr, group_arg(opcode, 25), false);
        } else if (in_group(31) || in_group(37) || (opcode >= 123 && opcode <= 126)) {
            const int arg = opcode >= 123 ? (opcode == 123 || opcode == 125 ? 1 : 2) :
                                            group_arg(opcode, in_group(31) ? 31 : 37);
            const uint8_t attr = u8(ptr++);
            const uint32_t dst = get_dst(ctx, arg, attr, ptr, nullptr);
            const uint32_t src = get_src(ctx, attr, ptr);

            if (in_group(31)) {
                divmul_[0] = dst * src;
            } else if (in_group(37)) {
                divmul_[0] = src != 0 ? dst / src : 0;
                divmul_[1] = src != 0 ? dst % src : 0;
            } else if (opcode <= 124) {
                const uint64_t product = static_cast<uint64_t>(dst) * src;
                divmul_[0] = static_cast<uint32_t>(product);
                divmul_[1] = static_cast<uint32_t>(product >> 32);
            } else if (src != 0) {
                uint64_t val64 = dst;
                val64 |= static_cast<uint64_t>(divmul_[1]) << 32;
                val64 /= src;
                divmul_[0] = static_cast<uint32_t>(val64);
                divmul_[1] = static_cast<uint32_t>(val64 >> 32);
            } else {
                divmul_[0] = 0;
                divmul_[1] = 0;
            }
        } else if (in_group(43)) {
            binary(ctx, ptr, group_arg(opcode, 43), [](uint32_t d, uint32_t s) { return d + s; });
        } else if (in_group(49)) {
            binary(ctx, ptr, group_arg(opcode, 49), [](uint32_t d, uint32_t s) { return d - s; });
        } else if (opcode == 55) {
            const uint32_t port = u16(ptr);
            io_mode_ = port == 0 ? io_mm : static_cast<int>(io_iio | port);
            ptr += 2;
        } else if (opcode == 56 || opcode == 57) {
            io_mode_ = opcode == 56 ? io_pci : io_sysio;
            ptr += 1;
        } else if (opcode == 58) {
            reg_block_ = u16(ptr);
            ptr += 2;
        } else if (opcode == 59) {
            const uint8_t attr = u8(ptr++);
            fb_base_ = get_src(ctx, attr, ptr);
        } else if (in_group(60) || in_group(74)) {
            const int arg = group_arg(opcode, in_group(60) ? 60 : 74);
            const uint8_t attr = u8(ptr++);
            const uint32_t dst = get_dst(ctx, arg, attr, ptr, nullptr);
            const uint32_t src = get_src(ctx, attr, ptr);

            if (in_group(60)) {
                cs_equal_ = dst == src;
                cs_above_ = dst > src;
            } else {
                cs_equal_ = (dst & src) == 0;
            }
        } else if (opcode == 66) {
            op_switch(ctx, ptr);
        } else if (opcode >= 67 && opcode <= 73) {
            const uint32_t target = u16(ptr);
            ptr += 2;
            bool execute = false;

            switch (opcode) {
            case 67: execute = true; break;
            case 68: execute = cs_equal_; break;
            case 69: execute = !(cs_above_ || cs_equal_); break;
            case 70: execute = cs_above_; break;
            case 71: execute = !cs_above_; break;
            case 72: execute = cs_above_ || cs_equal_; break;
            default: execute = !cs_equal_; break;
            }

            if (execute) {
                ptr = ctx.start + target;
            }
        } else if (opcode == 80 || opcode == 81) {
            const uint32_t count = u8(ptr++);
            const uint32_t microseconds = opcode == 81 ? count : count * 1000u;
            delay_total += microseconds;

            if (microseconds != 0) {
                delays.push_back(microseconds);
            }
        } else if (opcode == 82) {
            op_calltable(ctx, ptr);
        } else if (opcode == 83 || opcode == 100 || opcode == 101) {
            // REPEAT, SAVE_REG, RESTORE_REG: "unimplemented!".
            ++faults;
        } else if (in_group(84)) {
            const int arg = group_arg(opcode, 84);
            uint8_t attr = u8(ptr++);
            uint32_t saved = 0;
            uint32_t dptr = ptr;
            attr &= 0x38;
            attr = static_cast<uint8_t>(attr | ref_def_dst[attr >> 3] << 6);
            get_dst(ctx, arg, attr, ptr, &saved);
            put_dst(ctx, arg, attr, dptr, 0, saved);
        } else if (in_group(92)) {
            const int arg = group_arg(opcode, 92);
            const uint8_t attr = u8(ptr++);
            uint32_t saved = 0;
            uint32_t dptr = ptr;
            uint32_t dst = get_dst(ctx, arg, attr, ptr, &saved);
            const uint32_t mask = get_src_direct((attr >> 3) & 7, ptr);
            const uint32_t src = get_src(ctx, attr, ptr);
            dst &= mask;
            dst |= src;
            put_dst(ctx, arg, attr, dptr, dst, saved);
        } else if (opcode == 98 || opcode == 121) {
            ptr += 1;
        } else if (opcode == 102) {
            const uint32_t idx = u8(ptr);
            ptr += 1;

            if (idx == 0) {
                data_block_ = 0;
            } else if (idx == 255) {
                data_block_ = static_cast<uint16_t>(ctx.start);
            } else {
                data_block_ = u16(data_table_ + 4 + 2 * idx);
            }
        } else if (in_group(103)) {
            binary(ctx, ptr, group_arg(opcode, 103), [](uint32_t d, uint32_t s) { return d ^ s; });
        } else if (in_group(109)) {
            op_shift_full(ctx, ptr, group_arg(opcode, 109), true);
        } else if (in_group(115)) {
            op_shift_full(ctx, ptr, group_arg(opcode, 115), false);
        } else if (opcode == 122) {
            const uint32_t val = u16(ptr);
            ptr += val + 2;
        }

        // 90 NOP, 91 EOT and 99 BEEP do nothing here.
    }

    int execute_locked(int index, uint32_t ps, int ps_size) {
        const uint32_t base = u16(cmd_table_ + 4 + 2 * static_cast<uint32_t>(index));

        if (base == 0) {
            return -1;
        }

        Exec ctx{};
        ctx.ws_size = u8(base + 4);
        ctx.ws.assign(static_cast<size_t>(ctx.ws_size), 0);
        ctx.ps_shift = (u8(base + 5) & 0x7F) / 4;
        ctx.start = base;
        ctx.ps = ps;
        ctx.ps_size = ps_size;
        uint32_t ptr = base + 6;
        int ret = 0;
        max_depth = ++depth > max_depth ? depth : max_depth;

        while (!error) {
            const uint8_t opcode = u8(ptr++);

            if (ctx.abort) {
                ret = -1;
                break;
            }

            ++steps;

            if (opcode > 0 && opcode < 127) {
                dispatch(ctx, opcode, ptr);
            } else {
                // The port counts the opcode loop's silent break as a tolerated fault.
                ++faults;
                break;
            }

            if (opcode == 91) {
                break;
            }
        }

        --depth;
        return error ? -1 : ret;
    }

    const std::vector<uint8_t>& rom_;
    std::vector<uint32_t>& registers_;
    std::array<uint32_t, parameter_capacity>* params_ = nullptr;
    uint32_t dummy_ = 0;
    uint32_t cmd_table_ = 0;
    uint32_t data_table_ = 0;
    std::array<uint16_t, 256> iio_{};
    uint16_t data_block_ = 0;
    uint32_t fb_base_ = 0;
    uint32_t divmul_[2]{};
    uint16_t io_attr_ = 0;
    uint16_t reg_block_ = 0;
    uint8_t shift_ = 0;
    bool cs_equal_ = false;
    bool cs_above_ = false;
    int io_mode_ = io_mm;
    std::vector<uint32_t> scratch_;
    uint32_t scratch_size_bytes_ = 0;
};

// Deterministic splitmix64 stream.
class Rng final {
public:
    explicit Rng(uint64_t seed) : state_(seed) {}

    uint64_t next64() {
        uint64_t z = (state_ += 0x9e3779b97f4a7c15ULL);
        z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
        z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
        return z ^ (z >> 31);
    }

    uint32_t next() { return static_cast<uint32_t>(next64() >> 32); }
    uint32_t below(uint32_t bound) { return bound == 0 ? 0 : next() % bound; }
    bool percent(uint32_t chance) { return below(100) < chance; }

    template <typename T>
    T pick(const std::vector<T>& items) { return items.at(below(static_cast<uint32_t>(items.size()))); }

private:
    uint64_t state_;
};

struct GeneratedProgram final {
    Image image;
    std::array<uint32_t, Reference::parameter_capacity> params{};
    uint32_t ps_dwords = 0;
    uint32_t scratch_bytes = 0;
    std::vector<uint32_t> registers;
    // Generated feature counts (coverage evidence).
    uint32_t instructions = 0;
    uint32_t loops = 0;
    uint32_t calls = 0;
    uint32_t switches = 0;
    uint32_t jumps = 0;
    uint32_t iio_selects = 0;
};

class Generator final {
public:
    static constexpr uint32_t register_dwords = 1024;
    static constexpr uint32_t scratch_bytes = 256;
    static constexpr uint32_t root_ps_dwords = 48;
    static constexpr uint8_t epilogue_ps = 30;
    static constexpr uint16_t epilogue_registers = 0x200;

    explicit Generator(uint64_t seed) : rng_(seed) {}

    GeneratedProgram next() {
        GeneratedProgram program{};
        ImageBuilder builder;
        builder.command_entries(8);
        builder.tail_padding(0x60);
        program_ = &program;

        // Data tables for SET_DATA_BLOCK / ID reads (payloads are random bytes).
        for (const uint8_t index : {uint8_t{1}, uint8_t{4}, uint8_t{6}}) {
            std::vector<uint8_t> payload(36);

            for (auto& byte : payload) {
                byte = static_cast<uint8_t>(rng_.next());
            }

            builder.data(index, 1, 1, payload);
        }

        // IIO read programs 1/2 and write programs 0x81/0x82 with random bodies.
        for (const uint8_t identifier : {uint8_t{1}, uint8_t{0x81}, uint8_t{2}, uint8_t{0x82}}) {
            random_iio(builder.iio_program(identifier));
        }

        // Root table 0 plus up to two callees among 1..7 (absent ones are skipped calls).
        std::vector<uint8_t> tables{0};

        for (uint8_t index = 1; index < 8; ++index) {
            if (tables.size() < 3 && rng_.percent(25)) {
                tables.push_back(index);
            }
        }

        for (const uint8_t index : tables) {
            const bool root = index == 0;
            const uint8_t ws = static_cast<uint8_t>(root ? 1 + rng_.below(6) : rng_.below(5));
            uint8_t ps_bytes = static_cast<uint8_t>(rng_.below(4) * 4 + (rng_.percent(10) ? 2 : 0));

            if (rng_.percent(10)) {
                ps_bytes = static_cast<uint8_t>(ps_bytes | 0x80);
            }

            auto& table = builder.command(index, ws, ps_bytes);
            Table context{index, ws, &table.code, {}, -1, {}};

            for (uint8_t callee = static_cast<uint8_t>(index + 1); callee < 8; ++callee) {
                context.callees.push_back(callee);
            }

            body(context, root);
        }

        program.image = builder.build();
        program.ps_dwords = root_ps_dwords;
        program.scratch_bytes = scratch_bytes;

        for (uint32_t index = 0; index < Reference::parameter_capacity; ++index) {
            program.params[index] = rng_.percent(20) ? rng_.below(16) : rng_.next();
        }

        program.registers.resize(register_dwords);

        for (auto& value : program.registers) {
            value = rng_.percent(30) ? rng_.below(8) : rng_.next();
        }

        return program;
    }

private:
    struct Table final {
        uint8_t index;
        uint8_t ws;
        Code* code;
        std::vector<Code::Label> pending;
        int loop_slot;
        std::vector<uint8_t> callees;
    };

    void random_iio(IioProgram& program) {
        const uint32_t count = 1 + rng_.below(6);

        for (uint32_t index = 0; index < count; ++index) {
            const uint8_t width = static_cast<uint8_t>(1 + rng_.below(32));
            const uint8_t from = static_cast<uint8_t>(rng_.below(32));
            const uint8_t to = static_cast<uint8_t>(rng_.below(32));

            switch (rng_.below(10)) {
            case 0: program.nop(); break;
            case 1: program.read(static_cast<uint16_t>(rng_.below(0x80))); break;
            case 2: program.write(static_cast<uint16_t>(rng_.below(0x80))); break;
            case 3: program.clear(width, from); break;
            case 4: program.set(width, from); break;
            case 5: program.move_index(width, from, to); break;
            case 6: program.move_attr(width, from, to); break;
            case 7: program.move_data(width, from, to); break;
            case 8:
                if (rng_.percent(15)) {
                    // START inside a body: atom_iio_execute's unknown-opcode path.
                    program.raw(iio::start);
                    program.raw(0);
                } else {
                    program.write(static_cast<uint16_t>(rng_.below(0x80)));
                }
                break;
            default: program.read(static_cast<uint16_t>(rng_.below(0x80))); break;
            }
        }
    }

    Operand with_align(Operand operand) {
        operand.align = static_cast<uint8_t>(rng_.below(8));
        return operand;
    }

    Operand random_immediate() {
        const uint8_t align = static_cast<uint8_t>(rng_.below(8));
        const uint32_t value = rng_.percent(30) ? rng_.below(40) : rng_.next();
        Operand operand = imm32(value);
        operand.align = align;
        operand.value = align == src_dword ? value : align < src_byte0 ? (value & 0xffffu) : (value & 0xffu);
        return operand;
    }

    Operand random_source(const Table& table) {
        switch (rng_.below(9)) {
        case 0:
        case 1: return with_align(reg(static_cast<uint16_t>(rng_.below(0x80))));
        case 2: return with_align(ps(static_cast<uint8_t>(rng_.below(52))));
        case 3: {
            const uint8_t index = rng_.percent(30) ? static_cast<uint8_t>(0x40 + rng_.below(9)) :
                                                     static_cast<uint8_t>(rng_.below(table.ws + 2u));
            return with_align(ws(index));
        }
        case 4: return with_align(fb(static_cast<uint8_t>(rng_.below(72))));
        case 5: return with_align(id(static_cast<uint16_t>(rng_.below(64))));
        case 6: return with_align(rng_.percent(50) ? pll(static_cast<uint8_t>(rng_.next())) :
                                                     mc(static_cast<uint8_t>(rng_.next())));
        default: return random_immediate();
        }
    }

    // Destinations never write the data pointer, the register block or the loop
    // counter (only dedicated safe moves do).
    Operand random_destination(const Table& table, bool ps_ws_only) {
        const uint32_t kind = ps_ws_only ? 1 + rng_.below(2) : rng_.below(6);
        Operand operand{};

        switch (kind) {
        case 0: operand = reg(static_cast<uint16_t>(rng_.below(0x80))); break;
        case 1: operand = ps(static_cast<uint8_t>(rng_.below(52))); break;
        case 2: {
            static const std::vector<uint8_t> specials{0x40, 0x41, 0x43, 0x44, 0x45, 0x46, 0x47};
            uint8_t index = rng_.percent(25) ? rng_.pick(specials) : static_cast<uint8_t>(rng_.below(table.ws + 2u));

            if (static_cast<int>(index) == table.loop_slot) {
                index = 0x40;
            }

            operand = ws(index);
            break;
        }
        case 3: operand = fb(static_cast<uint8_t>(rng_.below(72))); break;
        case 4: operand = pll(static_cast<uint8_t>(rng_.next())); break;
        default: operand = mc(static_cast<uint8_t>(rng_.next())); break;
        }

        operand.align = static_cast<uint8_t>(rng_.below(8));
        operand.sel = static_cast<uint8_t>(rng_.below(4));
        return operand;
    }

    // A forward label placed later in the current segment.
    Code::Label forward(Table& table) {
        const Code::Label label = table.code->label();
        table.pending.push_back(label);
        return label;
    }

    void place_some(Table& table, bool all) {
        std::vector<Code::Label> keep;

        for (const auto label : table.pending) {
            if (all || rng_.percent(30)) {
                table.code->place(label);
            } else {
                keep.push_back(label);
            }
        }

        table.pending = keep;
    }

    void instruction(Table& table) {
        Code& code = *table.code;
        ++program_->instructions;

        switch (rng_.below(30)) {
        case 0: case 1: case 2: case 3:
            code.move(random_destination(table, false), random_source(table));
            break;
        case 4: code.and_(random_destination(table, false), random_source(table)); break;
        case 5: code.or_(random_destination(table, false), random_source(table)); break;
        case 6: code.add(random_destination(table, false), random_source(table)); break;
        case 7: code.sub(random_destination(table, false), random_source(table)); break;
        case 8: code.xor_(random_destination(table, false), random_source(table)); break;
        case 9: code.mask(random_destination(table, false), rng_.next(), random_source(table)); break;
        case 10: code.clear(random_destination(table, false)); break;
        case 11:
            if (rng_.percent(50)) {
                code.shift_left(random_destination(table, false), static_cast<uint8_t>(rng_.below(40)));
            } else {
                code.shift_right(random_destination(table, false), static_cast<uint8_t>(rng_.below(40)));
            }
            break;
        case 12:
            if (rng_.percent(50)) {
                code.shl(random_destination(table, false), random_source(table));
            } else {
                code.shr(random_destination(table, false), random_source(table));
            }
            break;
        case 13: {
            const uint32_t choice = rng_.below(4);

            if (choice == 0) {
                code.mul(random_destination(table, false), random_source(table));
            } else if (choice == 1) {
                code.div(random_destination(table, false), random_source(table));
            } else if (choice == 2) {
                code.mul32(random_destination(table, true), random_source(table));
            } else {
                code.div32(random_destination(table, true), random_source(table));
            }
            break;
        }
        case 14: case 15: {
            if (rng_.percent(50)) {
                code.compare(random_destination(table, false), random_source(table));
            } else {
                code.test(random_destination(table, false), random_source(table));
            }

            code.jump(static_cast<Cond>(rng_.below(7)), forward(table));
            ++program_->jumps;
            break;
        }
        case 16: {
            const Operand src = rng_.percent(50) ? random_immediate() : random_source(table);
            std::vector<Code::Case> cases;
            const uint32_t count = 1 + rng_.below(3);
            const uint32_t limit = src.align == src_dword ? 0xffffffffu : src.align < src_byte0 ? 0xffffu : 0xffu;

            for (uint32_t index = 0; index < count; ++index) {
                // An immediate SWITCH source is compared unmasked (atom_get_src_int IMM).
                const uint32_t value = rng_.percent(40) && src.arg == Arg::imm ? src.value : rng_.next() & limit;
                cases.push_back({value, forward(table)});
            }

            code.switch_(src, cases);
            ++program_->switches;
            break;
        }
        case 17: {
            const uint32_t choice = rng_.below(10);

            if (choice < 4) {
                code.set_port_ati(0);
            } else if (choice < 7) {
                static const std::vector<uint16_t> ports{1, 2, 3, 0x101};
                code.set_port_ati(rng_.pick(ports));
                ++program_->iio_selects;
            } else if (choice < 9) {
                code.set_port_pci(static_cast<uint8_t>(rng_.next()));
            } else {
                code.set_port_sysio(static_cast<uint8_t>(rng_.next()));
            }
            break;
        }
        case 18: {
            static const std::vector<uint16_t> blocks{0, 0x20, 0x100, 0x180};
            code.set_reg_block(rng_.pick(blocks));
            break;
        }
        case 19: {
            static const std::vector<uint8_t> blocks{0, 255, 1, 4, 6, 9, 30};
            code.set_data_block(rng_.pick(blocks));
            break;
        }
        case 20: code.set_fb_base(random_source(table)); break;
        case 21: {
            // The only writes of the data pointer and register block: safe values.
            static const std::vector<uint16_t> data{0, 0x10, 0x40};
            static const std::vector<uint16_t> blocks{0, 0x20, 0x100};

            if (rng_.percent(50)) {
                code.move(ws(ws_dataptr), imm16(rng_.pick(data)));
            } else {
                code.move(ws(ws_regptr), imm16(rng_.pick(blocks)));
            }
            break;
        }
        case 22: case 23:
            if (!table.callees.empty()) {
                code.call_table(rng_.pick(table.callees));
                ++program_->calls;
            } else {
                code.nop();
            }
            break;
        case 24:
            if (rng_.percent(50)) {
                code.delay_us(static_cast<uint8_t>(rng_.below(200)));
            } else {
                code.delay_ms(static_cast<uint8_t>(rng_.below(3)));
            }
            break;
        case 25: {
            const uint32_t choice = rng_.below(5);

            if (choice == 0) {
                code.nop();
            } else if (choice == 1) {
                code.beep();
            } else if (choice == 2) {
                code.post_card(static_cast<uint8_t>(rng_.next()));
            } else if (choice == 3) {
                code.debug(static_cast<uint8_t>(rng_.next()));
            } else {
                std::vector<uint8_t> payload(rng_.below(5));

                for (auto& byte : payload) {
                    byte = static_cast<uint8_t>(rng_.next());
                }

                code.process_ds(payload);
            }
            break;
        }
        case 26:
            if (rng_.percent(50)) {
                const uint32_t choice = rng_.below(3);

                if (choice == 0) {
                    code.repeat();
                } else if (choice == 1) {
                    code.save_reg();
                } else {
                    code.restore_reg();
                }
            } else {
                code.jump(Cond::always, forward(table));
                ++program_->jumps;
            }
            break;
        default:
            code.move(random_destination(table, false), random_source(table));
            break;
        }

        place_some(table, false);
    }

    void segment(Table& table, uint32_t count) {
        for (uint32_t index = 0; index < count; ++index) {
            instruction(table);
        }

        place_some(table, true);
    }

    void body(Table& table, bool root) {
        Code& code = *table.code;
        segment(table, 2 + rng_.below(14));

        // A counted loop on the reserved last workspace slot.
        if (table.ws >= 2 && rng_.percent(50)) {
            table.loop_slot = table.ws - 1;
            const uint8_t slot = static_cast<uint8_t>(table.loop_slot);
            code.move(ws(slot), imm32(1 + rng_.below(4)));
            const Code::Label top = code.label();
            code.place(top);
            segment(table, 1 + rng_.below(8));
            code.sub(ws(slot), imm32(1));
            code.compare(ws(slot), imm32(0));
            code.jump(Cond::not_equal, top);
            table.loop_slot = -1;
            ++program_->loops;
        }

        segment(table, rng_.below(10));

        if (root) {
            epilogue(table);
        }

        if (!root && rng_.percent(10)) {
            // An invalid opcode also ends a table (atom.c breaks out and returns 0).
            code.raw({static_cast<uint8_t>(rng_.percent(50) ? 0 : 127 + rng_.below(129))});
        } else {
            code.eot();
        }
    }

    // Copies the root WS, the nine special WS values and the FB scratch where the
    // test can compare them (PS and registers).
    void epilogue(Table& table) {
        Code& code = *table.code;
        code.set_port_ati(0);
        code.set_reg_block(0);
        uint8_t slot = epilogue_ps;

        for (uint8_t index = 0; index < table.ws; ++index) {
            code.move(ps(slot++), ws(index));
        }

        for (uint8_t index = ws_quotient; index <= ws_regptr; ++index) {
            code.move(ps(slot++), ws(index));
        }

        code.set_fb_base(imm32(0));

        for (uint8_t index = 0; index < scratch_bytes / 4; ++index) {
            code.move(reg(static_cast<uint16_t>(epilogue_registers + index)), fb(index));
        }
    }

    Rng rng_;
    GeneratedProgram* program_ = nullptr;
};

} // namespace rx7900_test::atom

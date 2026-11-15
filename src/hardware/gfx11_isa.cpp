/*
Module: rx7900.hardware.gfx11_isa
Purpose: GFX11 instruction encoder for the raster2d kernel subset.
Created: 2026-10-22
Architecture: See gfx11_isa.hpp. Encodings follow the pinned LLVM 18.1.8 GFX11 MC
tests (reference data only); tests/gfx11_isa_tests.cpp compares them field by field.
*/
#include "gfx11_isa.hpp"

namespace rx7900::hardware::gfx11_isa {
namespace {

// Encoding identifiers (fixed high bits of the first dword).
constexpr uint32_t enc_sop2 = 0x2u << 30;          // [31:30] 10
constexpr uint32_t enc_sopk = 0xbu << 28;          // [31:28] 1011
constexpr uint32_t enc_sop1 = 0x17du << 23;        // [31:23] 101111101
constexpr uint32_t enc_sopc = 0x17eu << 23;        // [31:23] 101111110
constexpr uint32_t enc_sopp = 0x17fu << 23;        // [31:23] 101111111
constexpr uint32_t enc_smem = 0x3du << 26;         // [31:26] 111101
constexpr uint32_t enc_vop1 = 0x3fu << 25;         // [31:25] 0111111
constexpr uint32_t enc_vopc = 0x3eu << 25;         // [31:25] 0111110
constexpr uint32_t enc_vop3 = 0x35u << 26;         // [31:26] 110101
constexpr uint32_t enc_mubuf = 0x38u << 26;        // [31:26] 111000
constexpr uint32_t enc_exp = 0x3eu << 26;          // [31:26] 111110
constexpr uint32_t exp_done = 1u << 11;
constexpr uint32_t exp_row_en = 1u << 13;
constexpr uint32_t mubuf_offen = 1u << 22;
constexpr int32_t smem_offset_min = -(1 << 20);
constexpr int32_t smem_offset_max = (1 << 20) - 1;
constexpr uint32_t mubuf_offset_max = 0xfffu;

bool is_scalar_source(const Operand& op) noexcept {
    return op.kind == OperandKind::sgpr || op.kind == OperandKind::special ||
           op.kind == OperandKind::inline_constant || op.kind == OperandKind::literal;
}

bool is_vector_source(const Operand& op) noexcept {
    return is_scalar_source(op) || op.kind == OperandKind::vgpr;
}

bool is_scalar_destination(const Operand& op) noexcept {
    return op.kind == OperandKind::sgpr || op.kind == OperandKind::special;
}

// Operands that read the scalar constant bus (SGPRs, specials and literals).
bool uses_constant_bus(const Operand& op) noexcept {
    return op.kind == OperandKind::sgpr || op.kind == OperandKind::special ||
           op.kind == OperandKind::literal;
}

uint32_t literal_count(const Operand& a, const Operand& b, const Operand& c) noexcept {
    return (a.kind == OperandKind::literal ? 1u : 0u) + (b.kind == OperandKind::literal ? 1u : 0u) +
           (c.kind == OperandKind::literal ? 1u : 0u);
}

// Distinct constant-bus values among up to three operands.
uint32_t constant_bus_reads(const Operand& a, const Operand& b, const Operand& c) noexcept {
    const Operand* list[3] = {&a, &b, &c};
    uint32_t count = 0;

    for (uint32_t i = 0; i < 3; ++i) {
        if (!uses_constant_bus(*list[i])) {
            continue;
        }

        bool seen = false;

        for (uint32_t j = 0; j < i; ++j) {
            if (uses_constant_bus(*list[j]) && list[j]->code == list[i]->code &&
                list[j]->kind == list[i]->kind &&
                (list[i]->kind != OperandKind::literal || list[j]->literal == list[i]->literal)) {
                seen = true;
            }
        }

        count += seen ? 0u : 1u;
    }

    return count;
}

bool two_source_vop3(Vop3 op) noexcept {
    return op == Vop3::v_mul_lo_u32 || op == Vop3::v_mul_hi_u32 || op == Vop3::v_mbcnt_lo_u32_b32;
}

bool known_vop3(Vop3 op) noexcept {
    switch (op) {
    case Vop3::v_cndmask_b32_e64: case Vop3::v_mad_u32_u24: case Vop3::v_bfe_u32:
    case Vop3::v_bfe_i32: case Vop3::v_bfi_b32: case Vop3::v_lshl_or_b32:
    case Vop3::v_and_or_b32: case Vop3::v_mul_lo_u32: case Vop3::v_mul_hi_u32:
    case Vop3::v_mbcnt_lo_u32_b32:
        return true;
    }

    return false;
}

uint32_t smem_dwords(Smem op) noexcept {
    switch (op) {
    case Smem::s_load_b32: return 1;
    case Smem::s_load_b64: return 2;
    case Smem::s_load_b128: return 4;
    case Smem::s_load_b256: return 8;
    case Smem::s_load_b512: return 16;
    case Smem::s_buffer_load_b32: return 1;
    case Smem::s_buffer_load_b64: return 2;
    case Smem::s_buffer_load_b128: return 4;
    case Smem::s_buffer_load_b256: return 8;
    case Smem::s_buffer_load_b512: return 16;
    }

    return 0;
}

} // namespace

Operand s(uint32_t index) noexcept {
    Operand op{};

    if (index < sgpr_count) {
        op.kind = OperandKind::sgpr;
        op.code = static_cast<uint16_t>(index);
    }

    return op;
}

Operand v(uint32_t index) noexcept {
    Operand op{};

    if (index < vgpr_limit) {
        op.kind = OperandKind::vgpr;
        op.code = static_cast<uint16_t>(code_vgpr_base + index);
    }

    return op;
}

Operand vcc_lo() noexcept { return {OperandKind::special, code_vcc_lo, 0}; }
Operand exec_lo() noexcept { return {OperandKind::special, code_exec_lo, 0}; }
Operand null_sgpr() noexcept { return {OperandKind::special, code_null, 0}; }
Operand m0() noexcept { return {OperandKind::special, code_m0, 0}; }

Operand imm(int32_t value) noexcept {
    if (value >= 0 && value <= 64) {
        return {OperandKind::inline_constant, static_cast<uint16_t>(code_inline_zero + value), 0};
    }

    if (value < 0 && value >= -16) {
        return {OperandKind::inline_constant,
                static_cast<uint16_t>(code_inline_minus_one - 1 - value), 0};
    }

    return literal(static_cast<uint32_t>(value));
}

Operand literal(uint32_t value) noexcept { return {OperandKind::literal, code_literal, value}; }

void Assembler::emit(uint32_t word) noexcept {
    if (!ok_ || finished_) {
        fail();
        return;
    }

    if (count_ >= output_.size() / 4 || !output_.write_u32(static_cast<size_t>(count_) * 4, word)) {
        fail();
        return;
    }

    ++count_;
}

void Assembler::emit_literal(const Operand& a, const Operand& b, const Operand& c) noexcept {
    const Operand* list[3] = {&a, &b, &c};

    for (const Operand* op : list) {
        if (op->kind == OperandKind::literal) {
            emit(op->literal);
            return;
        }
    }
}

void Assembler::sop2(Sop2 op, Operand sdst, Operand src0, Operand src1) noexcept {
    if (!is_scalar_destination(sdst) || !is_scalar_source(src0) || !is_scalar_source(src1) ||
        literal_count(src0, src1, Operand{}) > 1) {
        fail();
        return;
    }

    emit(enc_sop2 | (static_cast<uint32_t>(op) << 23) | (static_cast<uint32_t>(sdst.code) << 16) |
         (static_cast<uint32_t>(src1.code) << 8) | src0.code);
    emit_literal(src0, src1, Operand{});
}

void Assembler::sop1(Sop1 op, Operand sdst, Operand src0) noexcept {
    if (!is_scalar_destination(sdst) || !is_scalar_source(src0)) {
        fail();
        return;
    }

    emit(enc_sop1 | (static_cast<uint32_t>(sdst.code) << 16) | (static_cast<uint32_t>(op) << 8) |
         src0.code);
    emit_literal(src0, Operand{}, Operand{});
}

void Assembler::sopc(Sopc op, Operand src0, Operand src1) noexcept {
    if (!is_scalar_source(src0) || !is_scalar_source(src1) ||
        literal_count(src0, src1, Operand{}) > 1) {
        fail();
        return;
    }

    emit(enc_sopc | (static_cast<uint32_t>(op) << 16) | (static_cast<uint32_t>(src1.code) << 8) |
         src0.code);
    emit_literal(src0, src1, Operand{});
}

void Assembler::sopp(Sopp op, uint16_t simm16) noexcept {
    emit(enc_sopp | (static_cast<uint32_t>(op) << 16) | simm16);
}

void Assembler::sopk(Sopk op, Operand sdst, uint16_t simm16) noexcept {
    if (!is_scalar_destination(sdst)) {
        fail();
        return;
    }

    emit(enc_sopk | (static_cast<uint32_t>(op) << 23) | (static_cast<uint32_t>(sdst.code) << 16) |
         simm16);
}

void Assembler::smem(Smem op, uint32_t sdata, uint32_t sbase, Operand soffset,
                     int32_t offset) noexcept {
    const uint32_t count = smem_dwords(op);
    const uint32_t alignment = count >= 4 ? 4u : count;
    // s_buffer_load reads a V# quad; s_load reads a 64-bit address pair.
    const uint32_t base_size = static_cast<uint32_t>(op) >= 8 ? 4u : 2u;

    if (count == 0 || sdata % alignment != 0 || sdata + count > sgpr_count ||
        sbase % base_size != 0 || sbase + base_size > sgpr_count || offset < smem_offset_min || offset > smem_offset_max ||
        !(soffset.kind == OperandKind::sgpr ||
          (soffset.kind == OperandKind::special && soffset.code != code_exec_hi))) {
        fail();
        return;
    }

    emit(enc_smem | (static_cast<uint32_t>(op) << 18) | (sdata << 6) | (sbase >> 1));
    emit((static_cast<uint32_t>(soffset.code) << 25) | (static_cast<uint32_t>(offset) & 0x1fffffu));
}

void Assembler::vop1(Vop1 op, Operand vdst, Operand src0) noexcept {
    if (vdst.kind != OperandKind::vgpr || !is_vector_source(src0)) {
        fail();
        return;
    }

    emit(enc_vop1 | (static_cast<uint32_t>(vdst.code - code_vgpr_base) << 17) |
         (static_cast<uint32_t>(op) << 9) | src0.code);
    emit_literal(src0, Operand{}, Operand{});
}

void Assembler::readfirstlane(Operand sdst, Operand vsrc0) noexcept {
    constexpr uint32_t op_readfirstlane = 0x02;

    if (!(sdst.kind == OperandKind::sgpr || (sdst.kind == OperandKind::special && sdst.code == code_vcc_lo)) ||
        vsrc0.kind != OperandKind::vgpr) {
        fail();
        return;
    }

    emit(enc_vop1 | (static_cast<uint32_t>(sdst.code) << 17) | (op_readfirstlane << 9) | vsrc0.code);
}

void Assembler::vop2(Vop2 op, Operand vdst, Operand src0, Operand vsrc1) noexcept {
    if (vdst.kind != OperandKind::vgpr || vsrc1.kind != OperandKind::vgpr ||
        !is_vector_source(src0)) {
        fail();
        return;
    }

    emit((static_cast<uint32_t>(op) << 25) | (static_cast<uint32_t>(vdst.code - code_vgpr_base) << 17) |
         (static_cast<uint32_t>(vsrc1.code - code_vgpr_base) << 9) | src0.code);
    emit_literal(src0, Operand{}, Operand{});
}

void Assembler::vopc(Vopc op, Operand src0, Operand vsrc1) noexcept {
    if (vsrc1.kind != OperandKind::vgpr || !is_vector_source(src0)) {
        fail();
        return;
    }

    emit(enc_vopc | (static_cast<uint32_t>(op) << 17) |
         (static_cast<uint32_t>(vsrc1.code - code_vgpr_base) << 9) | src0.code);
    emit_literal(src0, Operand{}, Operand{});
}

void Assembler::vop3(Vop3 op, Operand vdst, Operand src0, Operand src1, Operand src2) noexcept {
    const bool two = two_source_vop3(op);

    if (!known_vop3(op) || vdst.kind != OperandKind::vgpr || !is_vector_source(src0) ||
        !is_vector_source(src1) || (two ? src2.kind != OperandKind::invalid : !is_vector_source(src2)) ||
        literal_count(src0, src1, src2) > 1 || constant_bus_reads(src0, src1, src2) > 2) {
        fail();
        return;
    }

    if (op == Vop3::v_cndmask_b32_e64 && !(src2.kind == OperandKind::sgpr ||
                                           src2.kind == OperandKind::special)) {
        fail();
        return;
    }

    emit(enc_vop3 | (static_cast<uint32_t>(op) << 16) |
         static_cast<uint32_t>(vdst.code - code_vgpr_base));
    emit(src0.code | (static_cast<uint32_t>(src1.code) << 9) |
         (two ? 0u : static_cast<uint32_t>(src2.code) << 18));
    emit_literal(src0, src1, src2);
}

void Assembler::vop3_cmp(Vopc op, Operand sdst, Operand src0, Operand src1) noexcept {
    if (!(sdst.kind == OperandKind::sgpr || (sdst.kind == OperandKind::special &&
                                             sdst.code == code_vcc_lo)) ||
        !is_vector_source(src0) || !is_vector_source(src1) ||
        literal_count(src0, src1, Operand{}) > 1 || constant_bus_reads(src0, src1, Operand{}) > 2) {
        fail();
        return;
    }

    emit(enc_vop3 | (static_cast<uint32_t>(op) << 16) | sdst.code);
    emit(src0.code | (static_cast<uint32_t>(src1.code) << 9));
    emit_literal(src0, src1, Operand{});
}

void Assembler::mubuf(Mubuf op, Operand vdata, Operand vaddr, uint32_t srsrc, Operand soffset,
                      uint32_t offset) noexcept {
    const bool soffset_ok = soffset.kind == OperandKind::sgpr ||
                            (soffset.kind == OperandKind::special && soffset.code == code_null) ||
                            (soffset.kind == OperandKind::inline_constant &&
                             soffset.code == code_inline_zero);

    if (vdata.kind != OperandKind::vgpr || vaddr.kind != OperandKind::vgpr || srsrc % 4 != 0 ||
        srsrc + 4 > sgpr_count || !soffset_ok || offset > mubuf_offset_max) {
        fail();
        return;
    }

    emit(enc_mubuf | (static_cast<uint32_t>(op) << 18) | offset);
    emit(static_cast<uint32_t>(vaddr.code - code_vgpr_base) |
         (static_cast<uint32_t>(vdata.code - code_vgpr_base) << 8) | ((srsrc / 4) << 16) |
         mubuf_offen | (static_cast<uint32_t>(soffset.code) << 24));
}

void Assembler::exp(ExpTarget target, Operand vsrc0, Operand vsrc1, Operand vsrc2, Operand vsrc3,
                    bool done, bool row_en) noexcept {
    const Operand* channels[4] = {&vsrc0, &vsrc1, &vsrc2, &vsrc3};
    uint32_t enable = 0;
    uint32_t data = 0;

    for (uint32_t index = 0; index < 4; ++index) {
        const Operand& channel = *channels[index];

        if (channel.kind == OperandKind::vgpr) {
            enable |= 1u << index;
            data |= static_cast<uint32_t>(channel.code - code_vgpr_base) << (index * 8);
        } else if (channel.kind != OperandKind::invalid) {
            fail();     // only a VGPR or a disabled (off) channel is valid
            return;
        }
    }

    emit(enc_exp | (static_cast<uint32_t>(target) << 4) | enable |
         (done ? exp_done : 0u) | (row_en ? exp_row_en : 0u));
    emit(data);
}

Label Assembler::make_label() noexcept {
    if (label_count_ >= assembler_max_labels) {
        fail();
        return {};
    }

    labels_[label_count_] = -1;
    return {label_count_++};
}

void Assembler::bind(Label label) noexcept {
    if (label.id >= label_count_ || labels_[label.id] >= 0) {
        fail();
        return;
    }

    labels_[label.id] = count_;
}

void Assembler::branch(Sopp op, Label label) noexcept {
    if ((op != Sopp::s_branch && op != Sopp::s_cbranch_scc0 && op != Sopp::s_cbranch_scc1 &&
         op != Sopp::s_cbranch_execz) || label.id >= label_count_ ||
        fixup_count_ >= assembler_max_fixups) {
        fail();
        return;
    }

    fixups_[fixup_count_++] = {count_, label.id, op};
    sopp(op, 0);
}

void Assembler::pad_code_end(uint32_t alignment_bytes) noexcept {
    if (alignment_bytes < 4 || (alignment_bytes & (alignment_bytes - 1)) != 0) {
        fail();
        return;
    }

    while (ok_ && (count_ * 4u) % alignment_bytes != 0) {
        sopp(Sopp::s_code_end, 0);
    }
}

void Assembler::reset(sdma::CommandBuffer output) noexcept {
    output_ = output;
    count_ = 0;
    ok_ = true;
    finished_ = false;
    label_count_ = 0;
    fixup_count_ = 0;
}

bool Assembler::finish() noexcept {
    if (!ok_ || finished_) {
        ok_ = false;
        return false;
    }

    for (uint32_t i = 0; i < fixup_count_; ++i) {
        const Fixup& fixup = fixups_[i];
        const int64_t target = labels_[fixup.label];
        // SOPP branches are relative to the next instruction, in dwords.
        const int64_t distance = target - (static_cast<int64_t>(fixup.position) + 1);

        if (target < 0 || distance < -32768 || distance > 32767) {
            ok_ = false;
            return false;
        }

        const uint32_t word = enc_sopp | (static_cast<uint32_t>(fixup.op) << 16) |
                              (static_cast<uint32_t>(distance) & 0xffffu);

        if (!output_.write_u32(static_cast<size_t>(fixup.position) * 4, word)) {
            ok_ = false;
            return false;
        }
    }

    finished_ = true;
    return true;
}


// ---- Graphics subset (OpenGL fixed-function shaders, 2026-10-23) ----
// Field layouts from the pinned LLVM 18.1.8 gfx11_asm_mubuf.s, gfx11_asm_mimg.s,
// gfx11_asm_ldsdir.s and gfx11_asm_vinterp.s expectations.

namespace {

constexpr uint32_t enc_mimg = 0x3cu << 26;         // [31:26] 111100
constexpr uint32_t enc_ldsdir = 0xceu << 24;       // [31:24] 11001110
constexpr uint32_t enc_vinterp = 0xcdu << 24;      // [31:24] 11001101
constexpr uint32_t mubuf_idxen = 1u << 23;
constexpr uint32_t mubuf_slc = 1u << 12;
constexpr uint32_t mubuf_dlc = 1u << 13;
constexpr uint32_t mubuf_glc = 1u << 14;
constexpr uint32_t ldsdir_op_param = 0;
constexpr uint32_t ldsdir_op_direct = 1;

uint32_t mubuf_dwords(Mubuf op) noexcept {
    return (op == Mubuf::buffer_load_b128 || op == Mubuf::buffer_store_b128) ? 4u : 1u;
}

bool vgpr_tuple(const Operand& op, uint32_t count) noexcept {
    return op.kind == OperandKind::vgpr &&
           static_cast<uint32_t>(op.code - code_vgpr_base) + count <= vgpr_limit;
}

} // namespace

void Assembler::mubuf_ex(Mubuf op, Operand vdata, Operand vaddr, uint32_t srsrc, Operand soffset,
                         uint32_t offset, MubufFlags flags) noexcept {
    const bool soffset_ok = soffset.kind == OperandKind::sgpr ||
                            (soffset.kind == OperandKind::special &&
                             (soffset.code == code_null || soffset.code == code_m0)) ||
                            (soffset.kind == OperandKind::inline_constant &&
                             soffset.code == code_inline_zero);
    const bool uses_vaddr = flags.offen || flags.idxen;

    if (!vgpr_tuple(vdata, mubuf_dwords(op)) || (uses_vaddr && vaddr.kind != OperandKind::vgpr) ||
        (flags.offen && flags.idxen) || srsrc % 4 != 0 || srsrc + 4 > sgpr_count || !soffset_ok ||
        offset > mubuf_offset_max) {
        fail();
        return;
    }

    emit(enc_mubuf | (static_cast<uint32_t>(op) << 18) | offset | (flags.glc ? mubuf_glc : 0u) |
         (flags.slc ? mubuf_slc : 0u) | (flags.dlc ? mubuf_dlc : 0u));
    emit((uses_vaddr ? static_cast<uint32_t>(vaddr.code - code_vgpr_base) : 0u) |
         (static_cast<uint32_t>(vdata.code - code_vgpr_base) << 8) | ((srsrc / 4) << 16) |
         (flags.offen ? mubuf_offen : 0u) | (flags.idxen ? mubuf_idxen : 0u) |
         (static_cast<uint32_t>(soffset.code) << 24));
}

void Assembler::mimg_sample(Mimg op, Operand vdata, Operand vaddr, uint32_t srsrc, uint32_t ssamp,
                            uint32_t dmask, uint32_t dim) noexcept {
    uint32_t channels = 0;

    for (uint32_t bit = 0; bit < 4; ++bit) {
        channels += (dmask >> bit) & 1u;
    }

    // A 2D sample addresses (s, t), plus the LOD for image_sample_l.
    const uint32_t addresses = (dim == mimg_dim_2d ? 2u : 1u) + (op == Mimg::image_sample_l ? 1u : 0u);

    if ((op != Mimg::image_sample && op != Mimg::image_sample_l && op != Mimg::image_sample_lz) ||
        dmask == 0 || dmask > 0xfu || dim > 7u || !vgpr_tuple(vdata, channels) ||
        !vgpr_tuple(vaddr, addresses) || srsrc % 4 != 0 || srsrc + 8 > sgpr_count ||
        ssamp % 4 != 0 || ssamp + 4 > sgpr_count) {
        fail();
        return;
    }

    emit(enc_mimg | (static_cast<uint32_t>(op) << 18) | (dmask << 8) | (dim << 2));
    emit(static_cast<uint32_t>(vaddr.code - code_vgpr_base) |
         (static_cast<uint32_t>(vdata.code - code_vgpr_base) << 8) | ((srsrc / 4) << 16) |
         ((ssamp / 4) << 26));
}

void Assembler::lds_param_load(Operand vdst, uint32_t attribute, uint32_t channel,
                               uint32_t wait_vdst) noexcept {
    if (vdst.kind != OperandKind::vgpr || attribute > 32u || channel > 3u || wait_vdst > 15u) {
        fail();
        return;
    }

    emit(enc_ldsdir | (ldsdir_op_param << 20) | (wait_vdst << 16) | (attribute << 10) |
         (channel << 8) | static_cast<uint32_t>(vdst.code - code_vgpr_base));
}

void Assembler::lds_direct_load(Operand vdst, uint32_t wait_vdst) noexcept {
    if (vdst.kind != OperandKind::vgpr || wait_vdst > 15u) {
        fail();
        return;
    }

    emit(enc_ldsdir | (ldsdir_op_direct << 20) | (wait_vdst << 16) |
         static_cast<uint32_t>(vdst.code - code_vgpr_base));
}

void Assembler::vinterp(Vinterp op, Operand vdst, Operand src0, Operand src1, Operand src2,
                        uint32_t wait_exp) noexcept {
    if ((op != Vinterp::v_interp_p10_f32 && op != Vinterp::v_interp_p2_f32) ||
        vdst.kind != OperandKind::vgpr || src0.kind != OperandKind::vgpr ||
        src1.kind != OperandKind::vgpr || src2.kind != OperandKind::vgpr || wait_exp > 7u) {
        fail();
        return;
    }

    emit(enc_vinterp | (static_cast<uint32_t>(op) << 16) | (wait_exp << 8) |
         static_cast<uint32_t>(vdst.code - code_vgpr_base));
    emit(src0.code | (static_cast<uint32_t>(src1.code) << 9) | (static_cast<uint32_t>(src2.code) << 18));
}

} // namespace rx7900::hardware::gfx11_isa

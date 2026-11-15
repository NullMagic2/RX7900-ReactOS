/*
Module: rx7900.hardware.gfx11_isa
Purpose: Encode the exact GFX11 (RDNA3, GC 11.0.0) shader instruction subset that the
raster2d compute kernels use: SOP1/SOP2/SOPK/SOPC/SOPP, SMEM scalar loads,
VOP1/VOP2/VOPC/VOP3 integer ALU and MUBUF raw buffer loads/stores, plus the graphics
subset the OpenGL fixed-function shaders use (float VALU and compares, EXP pos/prim/mrt,
s_sendmsg, s_buffer_load, MUBUF b128/idxen, MIMG sample, LDSDIR, VINTERP).
Created: 2026-10-22
Architecture: Pure, allocation-free encoder writing little-endian dwords through the
checked sdma::CommandBuffer. raster2d_shaders generates kernels with it at
initialization; tests/gfx11_isa_tests.cpp validates every opcode and operand
field against the pinned LLVM 18.1.8 GFX11 MC test vectors
(work/upstream-gpuinit/llvm-mc-gfx11, SHA256SUMS) and executes the generated code
in an independent decoder/interpreter. No AMDGPU assembler is used or required.
Field layouts and opcodes follow LLVM llvm/test/MC/AMDGPU/gfx11_asm_*.s and
llvm/docs/AMDGPU/AMDGPUAsmGFX11.rst (Apache-2.0 WITH LLVM-exception), reference only.
*/
#pragma once

#include "sdma.hpp"

namespace rx7900::hardware::gfx11_isa {

// Scalar register file visible to a wave32 compute wave (s0..s105).
constexpr uint32_t sgpr_count = 106;
constexpr uint32_t vgpr_limit = 256;

// Source/destination operand codes (8-bit SALU fields, 9-bit VALU src fields).
constexpr uint16_t code_vcc_lo = 106;
constexpr uint16_t code_vcc_hi = 107;
constexpr uint16_t code_null = 124;
constexpr uint16_t code_m0 = 125;
constexpr uint16_t code_exec_lo = 126;
constexpr uint16_t code_exec_hi = 127;
constexpr uint16_t code_inline_zero = 128;   // integers 0..64 are 128..192
constexpr uint16_t code_inline_minus_one = 193; // -1..-16 are 193..208
constexpr uint16_t code_literal = 255;
constexpr uint16_t code_vgpr_base = 256;

enum class OperandKind : uint8_t { invalid, sgpr, special, inline_constant, literal, vgpr };

// One instruction operand. Construct with the helpers below; an invalid operand
// makes the instruction (and the assembler) fail.
struct Operand final {
    OperandKind kind = OperandKind::invalid;
    uint16_t code = 0;
    uint32_t literal = 0;
};

Operand s(uint32_t index) noexcept;        // s0..s105
Operand v(uint32_t index) noexcept;        // v0..v255
Operand vcc_lo() noexcept;
Operand exec_lo() noexcept;
Operand null_sgpr() noexcept;
// m0: GS_ALLOC_REQ payload and lds_param_load primitive mask (graphics subset).
Operand m0() noexcept;
// Integer constant: inline when -16..64, otherwise a 32-bit literal dword.
Operand imm(int32_t value) noexcept;
Operand literal(uint32_t value) noexcept;

enum class Sop2 : uint8_t {
    s_add_u32 = 0x00, s_sub_u32 = 0x01, s_add_i32 = 0x02, s_sub_i32 = 0x03,
    s_addc_u32 = 0x04, s_lshl_b32 = 0x08, s_lshr_b32 = 0x0a, s_and_b32 = 0x16,
    s_or_b32 = 0x18, s_xor_b32 = 0x1a, s_mul_i32 = 0x2c, s_mul_hi_u32 = 0x2d,
    s_cselect_b32 = 0x30
};
// s_bcnt1_i32_b32: population count of src0; SCC = result != 0.
// s_wqm_b32: whole-quad mask of src0 (graphics subset; pixel shaders enter WQM).
enum class Sop1 : uint8_t { s_mov_b32 = 0x00, s_bcnt1_i32_b32 = 0x18, s_wqm_b32 = 0x1c };
enum class Sopc : uint8_t {
    s_cmp_eq_i32 = 0x00, s_cmp_ge_i32 = 0x03, s_cmp_lt_i32 = 0x04, s_cmp_eq_u32 = 0x06,
    s_cmp_lg_u32 = 0x07, s_cmp_ge_u32 = 0x09, s_cmp_lt_u32 = 0x0a
};
enum class Sopp : uint8_t {
    s_nop = 0x00, s_waitcnt = 0x09, s_code_end = 0x1f, s_branch = 0x20,
    s_cbranch_scc0 = 0x21, s_cbranch_scc1 = 0x22, s_cbranch_execz = 0x25, s_endpgm = 0x30,
    s_sendmsg = 0x36
};
// NGG uses s_sendmsg(GS_ALLOC_REQ) to request vertex/primitive output space and
// s_sendmsg(DEALLOC_VGPRS) to retire. The payload travels in simm16.
constexpr uint16_t msg_gs_alloc_req = 9;
constexpr uint16_t msg_dealloc_vgprs = 3;
enum class Sopk : uint8_t { s_waitcnt_vscnt = 0x18 };
// s_load_b32 .. s_load_b512 (1, 2, 4, 8, 16 dwords).
// s_buffer_load_b32 .. b512 (graphics subset) read through a 4-aligned V# quad and
// honour its NUM_RECORDS bound.
enum class Smem : uint8_t { s_load_b32 = 0, s_load_b64 = 1, s_load_b128 = 2, s_load_b256 = 3,
                            s_load_b512 = 4, s_buffer_load_b32 = 8, s_buffer_load_b64 = 9,
                            s_buffer_load_b128 = 10, s_buffer_load_b256 = 11,
                            s_buffer_load_b512 = 12 };
enum class Vop1 : uint16_t {
    v_mov_b32 = 0x01, v_cvt_f32_i32 = 0x05, v_cvt_f32_u32 = 0x06, v_cvt_u32_f32 = 0x07,
    v_cvt_i32_f32 = 0x08, v_rcp_f32 = 0x2a, v_not_b32 = 0x37,
    // Graphics subset: base-2 exponent/logarithm, reciprocal square root, square root.
    v_exp_f32 = 0x25, v_log_f32 = 0x27, v_rsq_f32 = 0x2e, v_sqrt_f32 = 0x33,
    // Graphics subset (2026-10-24): rounding for texture LOD and pixel-rectangle
    // addressing, and the unsigned byte n of a dword to float (RGBA8 fetches).
    v_fract_f32 = 0x20, v_ceil_f32 = 0x22, v_floor_f32 = 0x24,
    v_cvt_f32_ubyte0 = 0x11, v_cvt_f32_ubyte1 = 0x12, v_cvt_f32_ubyte2 = 0x13, v_cvt_f32_ubyte3 = 0x14
};
enum class Vop2 : uint16_t {
    v_cndmask_b32 = 0x01, v_add_f32 = 0x03, v_sub_f32 = 0x04, v_subrev_f32 = 0x05,
    v_mul_legacy_f32 = 0x07, v_mul_f32 = 0x08, v_min_f32 = 0x0f, v_max_f32 = 0x10,
    v_mul_u32_u24 = 0x0b, v_max_i32 = 0x12, v_min_u32 = 0x13,
    v_lshlrev_b32 = 0x18, v_lshrrev_b32 = 0x19, v_ashrrev_i32 = 0x1a, v_and_b32 = 0x1b, v_or_b32 = 0x1c,
    v_xor_b32 = 0x1d, v_add_nc_u32 = 0x25, v_sub_nc_u32 = 0x26, v_subrev_nc_u32 = 0x27
};
enum class Vopc : uint16_t {
    v_cmp_lt_i32 = 0x41, v_cmp_le_i32 = 0x43, v_cmp_gt_i32 = 0x44, v_cmp_ge_i32 = 0x46,
    v_cmp_lt_u32 = 0x49, v_cmp_eq_u32 = 0x4a, v_cmp_le_u32 = 0x4b, v_cmp_gt_u32 = 0x4c,
    v_cmp_ne_u32 = 0x4d, v_cmp_ge_u32 = 0x4e,
    // Graphics subset: ordered float compares, and neq (unordered or not equal).
    v_cmp_lt_f32 = 0x11, v_cmp_eq_f32 = 0x12, v_cmp_le_f32 = 0x13, v_cmp_gt_f32 = 0x14,
    v_cmp_lg_f32 = 0x15, v_cmp_ge_f32 = 0x16, v_cmp_neq_f32 = 0x1d
};
// Native VOP3 opcodes (10-bit). VOP2 promoted to VOP3 is 0x100 + op, VOPC is op.
enum class Vop3 : uint16_t {
    v_cndmask_b32_e64 = 0x101, v_mad_u32_u24 = 0x20b, v_bfe_u32 = 0x210, v_bfe_i32 = 0x211,
    v_bfi_b32 = 0x212, v_lshl_or_b32 = 0x256, v_and_or_b32 = 0x257, v_mul_lo_u32 = 0x32c,
    v_mul_hi_u32 = 0x32d,
    // Wave32 lane prefix count: popcount(src0 & ((1 << lane) - 1)) + src1.
    v_mbcnt_lo_u32_b32 = 0x31f
};
enum class Mubuf : uint8_t { buffer_load_u8 = 0x10, buffer_load_b32 = 0x14, buffer_store_b32 = 0x1a,
                             // Graphics subset: four-dword vertex fetch and attribute-ring store.
                             buffer_load_b128 = 0x17, buffer_store_b128 = 0x1d };

// Addressing and cache-policy bits of a graphics MUBUF access (mubuf_ex). With
// neither offen nor idxen, vaddr is "off".
struct MubufFlags final {
    bool offen = false;
    bool idxen = false;
    bool glc = false;
    bool slc = false;
    bool dlc = false;
};

// MIMG sampling (graphics subset): image_sample (implicit derivatives),
// image_sample_l (explicit LOD as the last address), image_sample_lz (LOD 0).
enum class Mimg : uint8_t { image_sample = 0x1b, image_sample_l = 0x1d, image_sample_lz = 0x1f };
constexpr uint32_t mimg_dim_2d = 1;            // SQ_RSRC_IMG_2D in the MIMG DIM field

// VINTERP parameter interpolation from lds_param_load data (graphics subset).
enum class Vinterp : uint8_t { v_interp_p10_f32 = 0x00, v_interp_p2_f32 = 0x01 };

// EXP (pixel shader color/depth out, vertex shader position/parameter out) targets.
// mrt0..mrt7 are 0..7, mrtz (depth) is 8, null is 9, pos0..pos3 are 12..15. The
// graphics pipeline owner programs how pos/param map to interpolants.
enum class ExpTarget : uint8_t {
    mrt0 = 0, mrt7 = 7, mrtz = 8, exp_null = 9, pos0 = 12, pos1 = 13, pos2 = 14, pos3 = 15,
    prim = 20      // NGG primitive export (vertex indices, edge flags, null bit)
};

// Instruction-format identifiers shared with the test decoder.
enum class Format : uint8_t { sop1, sop2, sopk, sopc, sopp, smem, vop1, vop2, vopc, vop3, mubuf, exp,
                              mimg, ldsdir, vinterp };

// s_waitcnt simm16 on GFX11: expcnt [2:0], lgkmcnt [9:4], vmcnt [15:10]. A counter
// not waited for is encoded at its maximum (7, 63, 63).
constexpr uint32_t waitcnt_max_vm = 63;
constexpr uint32_t waitcnt_max_lgkm = 63;
constexpr uint32_t waitcnt_max_exp = 7;
constexpr uint16_t waitcnt_value(uint32_t vmcnt, uint32_t expcnt, uint32_t lgkmcnt) noexcept {
    return static_cast<uint16_t>(((vmcnt & 0x3fu) << 10) | ((lgkmcnt & 0x3fu) << 4) |
                                 (expcnt & 0x7u));
}

// A forward or backward branch target inside one assembler.
struct Label final {
    uint32_t id = UINT32_MAX;
};

constexpr uint32_t assembler_max_labels = 64;
constexpr uint32_t assembler_max_fixups = 128;

// Sticky-failure assembler. Every emit validates register ranges, operand kinds,
// the constant-bus limit (VOP1/VOP2/VOPC one SGPR or literal in src0; VOP3 two),
// alignment of register tuples and immediate widths before writing; the first
// violation makes ok() false and later emits do nothing. finish() resolves branch
// fixups and fails on unbound labels or out-of-range branch distances. output is
// borrowed: the caller owns it exclusively until finish returns.
class Assembler final {
public:
    explicit Assembler(sdma::CommandBuffer output) noexcept : output_(output) {}
    Assembler(const Assembler&) = delete;
    Assembler& operator=(const Assembler&) = delete;

    void sop2(Sop2 op, Operand sdst, Operand src0, Operand src1) noexcept;
    void sop1(Sop1 op, Operand sdst, Operand src0) noexcept;
    void sopc(Sopc op, Operand src0, Operand src1) noexcept;
    void sopp(Sopp op, uint16_t simm16) noexcept;
    void sopk(Sopk op, Operand sdst, uint16_t simm16) noexcept;
    // sdata: first SGPR of the destination tuple (aligned to 4 for 4+ dwords, 2 for
    // b64); sbase: even first SGPR of the 64-bit address pair; soffset: an SGPR or
    // null_sgpr(); offset: signed 21-bit byte offset.
    void smem(Smem op, uint32_t sdata, uint32_t sbase, Operand soffset, int32_t offset) noexcept;
    void vop1(Vop1 op, Operand vdst, Operand src0) noexcept;
    // v_readfirstlane_b32: the first active lane of a VGPR into an SGPR (VOP1 0x02
    // with the SGPR in the VDST field).
    void readfirstlane(Operand sdst, Operand vsrc0) noexcept;
    void vop2(Vop2 op, Operand vdst, Operand src0, Operand vsrc1) noexcept;
    // Writes vcc_lo (wave32).
    void vopc(Vopc op, Operand src0, Operand vsrc1) noexcept;
    void vop3(Vop3 op, Operand vdst, Operand src0, Operand src1, Operand src2) noexcept;
    // VOPC promoted to VOP3: writes the wave32 lane mask to sdst (an SGPR or vcc_lo).
    void vop3_cmp(Vopc op, Operand sdst, Operand src0, Operand src1) noexcept;
    // vaddr: VGPR offset (offen); srsrc: first SGPR of a 4-aligned V# quad;
    // soffset: SGPR, null_sgpr() or imm(0); offset: unsigned 12-bit byte offset.
    void mubuf(Mubuf op, Operand vdata, Operand vaddr, uint32_t srsrc, Operand soffset,
               uint32_t offset) noexcept;
    // Export up to four VGPR channels (vsrc0..vsrc3); a disabled channel passes an
    // invalid operand (default Operand{}) and clears its enable bit. done marks the
    // final export of its kind; row_en is the GFX11 row export. The enable mask is
    // derived from which channels are VGPRs.
    void exp(ExpTarget target, Operand vsrc0, Operand vsrc1, Operand vsrc2, Operand vsrc3,
             bool done, bool row_en = false) noexcept;
    // Graphics MUBUF: vdata is the first VGPR of a 1- or 4-dword tuple (by op);
    // vaddr is a VGPR when flags.offen or flags.idxen is set, otherwise ignored.
    void mubuf_ex(Mubuf op, Operand vdata, Operand vaddr, uint32_t srsrc, Operand soffset,
                  uint32_t offset, MubufFlags flags) noexcept;
    // MIMG sample: vdata first of popcount(dmask) VGPRs; vaddr first of the
    // consecutive address VGPRs (non-NSA); srsrc 4-aligned T# octet; ssamp
    // 4-aligned S# quad; dmask 1..15; dim the MIMG DIM code.
    void mimg_sample(Mimg op, Operand vdata, Operand vaddr, uint32_t srsrc, uint32_t ssamp,
                     uint32_t dmask, uint32_t dim) noexcept;
    // LDSDIR: lds_param_load vdst, attr<attribute>.<channel> wait_vdst:<wait_vdst>
    // (m0 holds the primitive mask) and lds_direct_load vdst wait_vdst.
    void lds_param_load(Operand vdst, uint32_t attribute, uint32_t channel,
                        uint32_t wait_vdst) noexcept;
    void lds_direct_load(Operand vdst, uint32_t wait_vdst) noexcept;
    // VINTERP with VGPR sources: p10 D = P10 * src1 + P0, p2 D = P20 * src1 + src2,
    // where P0/P10/P20 are src0 read from quad lanes 0/1/2. wait_exp 0..7.
    void vinterp(Vinterp op, Operand vdst, Operand src0, Operand src1, Operand src2,
                 uint32_t wait_exp) noexcept;
    // s_sendmsg with an NGG message payload (msg_gs_alloc_req, msg_dealloc_vgprs).
    void sendmsg(uint16_t message) noexcept { sopp(Sopp::s_sendmsg, message); }

    Label make_label() noexcept;
    void bind(Label label) noexcept;
    // s_branch, s_cbranch_scc0/scc1/execz to label.
    void branch(Sopp op, Label label) noexcept;

    void waitcnt(uint32_t vmcnt, uint32_t expcnt, uint32_t lgkmcnt) noexcept {
        sopp(Sopp::s_waitcnt, waitcnt_value(vmcnt, expcnt, lgkmcnt));
    }
    void endpgm() noexcept { sopp(Sopp::s_endpgm, 0); }

    // Pads with s_code_end to a multiple of alignment_bytes (a power of two
    // multiple of 4).
    void pad_code_end(uint32_t alignment_bytes) noexcept;

    bool finish() noexcept;
    // Restarts the assembler on a new output with no labels or fixups, so a
    // caller can keep one Assembler in owned storage (for example a kernel
    // workspace) instead of constructing it on the stack for every program.
    void reset(sdma::CommandBuffer output) noexcept;
    bool ok() const noexcept { return ok_; }
    uint32_t dwords() const noexcept { return count_; }

private:
    struct Fixup final {
        uint32_t position;
        uint32_t label;
        Sopp op;
    };

    void emit(uint32_t word) noexcept;
    void emit_literal(const Operand& a, const Operand& b, const Operand& c) noexcept;
    void fail() noexcept { ok_ = false; }

    sdma::CommandBuffer output_;
    uint32_t count_ = 0;
    bool ok_ = true;
    bool finished_ = false;
    int64_t labels_[assembler_max_labels]{};
    uint32_t label_count_ = 0;
    Fixup fixups_[assembler_max_fixups]{};
    uint32_t fixup_count_ = 0;
};

} // namespace rx7900::hardware::gfx11_isa

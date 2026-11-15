/*
Module: rx7900.tests.gfx11_isa
Purpose: Validate every GFX11 instruction encoding the raster2d kernels use against
the pinned LLVM 18.1.8 GFX11 MC expected bytes
(work/upstream-gpuinit/llvm-mc-gfx11, SHA256SUMS), and exercise the shared ISA
interpreter (tests/gfx11_interp.hpp) on small wave32 programs: SALU arithmetic and
SCC branches, VALU under an EXEC mask, SMEM scalar loads and MUBUF raw
loads/stores with out-of-bounds behaviour. Host-only; no GPU.
*/
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "gfx11_isa.hpp"
#include "raster2d_shaders.hpp"
#include "gfx11_interp.hpp"

namespace {

using namespace rx7900::hardware::gfx11_isa;
namespace sdma = rx7900::sdma;

unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* contract) {
    ++checks;

    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", contract);
    }
}

// Assemble one instruction and compare its bytes with an LLVM MC expectation.
template <typename Emit>
void vector(Emit emit, std::initializer_list<uint8_t> expected, const char* text) {
    uint8_t buffer[32] = {};
    Assembler a(sdma::CommandBuffer(buffer, sizeof(buffer)));
    emit(a);
    const bool finished = a.finish();
    const bool size_ok = a.dwords() * 4u == expected.size();
    bool bytes_ok = finished && size_ok;
    size_t index = 0;

    for (uint8_t value : expected) {
        if (index >= sizeof(buffer) || buffer[index] != value) {
            bytes_ok = false;
        }

        ++index;
    }

    if (!bytes_ok) {
        std::fprintf(stderr, "FAIL enc %s: got", text);

        for (uint32_t i = 0; i < a.dwords() * 4u; ++i) {
            std::fprintf(stderr, " %02x", buffer[i]);
        }

        std::fprintf(stderr, "\n");
    }

    expect(bytes_ok, text);
}

void test_encodings() {
    // SOP1 / SOP2 / SOPC / SOPP / SOPK (gfx11_asm_sop1/sop2/sopc/sopp/sopk.s).
    vector([](Assembler& a) { a.sop1(Sop1::s_mov_b32, s(0), s(1)); },
           {0x01, 0x00, 0x80, 0xbe}, "s_mov_b32 s0, s1");
    vector([](Assembler& a) { a.sop1(Sop1::s_mov_b32, s(0), imm(0)); },
           {0x80, 0x00, 0x80, 0xbe}, "s_mov_b32 s0, 0");
    vector([](Assembler& a) { a.sop1(Sop1::s_mov_b32, s(0), literal(0xaf123456)); },
           {0xff, 0x00, 0x80, 0xbe, 0x56, 0x34, 0x12, 0xaf}, "s_mov_b32 s0, 0xaf123456");
    vector([](Assembler& a) { a.sop2(Sop2::s_add_u32, s(0), s(1), s(2)); },
           {0x01, 0x02, 0x00, 0x80}, "s_add_u32 s0, s1, s2");
    vector([](Assembler& a) { a.sop2(Sop2::s_add_i32, s(0), s(1), s(2)); },
           {0x01, 0x02, 0x00, 0x81}, "s_add_i32 s0, s1, s2");
    vector([](Assembler& a) { a.sop2(Sop2::s_mul_i32, s(0), s(1), s(2)); },
           {0x01, 0x02, 0x00, 0x96}, "s_mul_i32 s0, s1, s2");
    vector([](Assembler& a) { a.sop2(Sop2::s_mul_hi_u32, s(0), s(1), s(2)); },
           {0x01, 0x02, 0x80, 0x96}, "s_mul_hi_u32 s0, s1, s2");
    vector([](Assembler& a) { a.sop2(Sop2::s_and_b32, s(0), s(1), s(2)); },
           {0x01, 0x02, 0x00, 0x8b}, "s_and_b32 s0, s1, s2");
    vector([](Assembler& a) { a.sop2(Sop2::s_and_b32, exec_lo(), s(1), s(2)); },
           {0x01, 0x02, 0x7e, 0x8b}, "s_and_b32 exec_lo, s1, s2");
    vector([](Assembler& a) { a.sop2(Sop2::s_cselect_b32, s(0), s(1), s(2)); },
           {0x01, 0x02, 0x00, 0x98}, "s_cselect_b32 s0, s1, s2");
    vector([](Assembler& a) { a.sop2(Sop2::s_lshl_b32, s(0), s(1), imm(0)); },
           {0x01, 0x80, 0x00, 0x84}, "s_lshl_b32 s0, s1, 0");
    vector([](Assembler& a) { a.sop2(Sop2::s_cselect_b32, s(0), imm(-1), s(2)); },
           {0xc1, 0x02, 0x00, 0x98}, "s_cselect_b32 s0, -1, s2");
    vector([](Assembler& a) { a.sopc(Sopc::s_cmp_eq_u32, s(0), s(1)); },
           {0x00, 0x01, 0x06, 0xbf}, "s_cmp_eq_u32 s0, s1");
    vector([](Assembler& a) { a.sopc(Sopc::s_cmp_lt_u32, s(0), imm(0)); },
           {0x00, 0x80, 0x0a, 0xbf}, "s_cmp_lt_u32 s0, 0");
    vector([](Assembler& a) { a.sopc(Sopc::s_cmp_ge_i32, s(0), literal(0xaf123456)); },
           {0x00, 0xff, 0x03, 0xbf, 0x56, 0x34, 0x12, 0xaf}, "s_cmp_ge_i32 s0, 0xaf123456");
    vector([](Assembler& a) { a.endpgm(); }, {0x00, 0x00, 0xb0, 0xbf}, "s_endpgm");
    vector([](Assembler& a) { a.sopp(Sopp::s_code_end, 0); },
           {0x00, 0x00, 0x9f, 0xbf}, "s_code_end");
    vector([](Assembler& a) { a.waitcnt(0, 0, 0); }, {0x00, 0x00, 0x89, 0xbf}, "s_waitcnt 0");
    vector([](Assembler& a) { a.waitcnt(0, 0, waitcnt_max_lgkm); },
           {0xf0, 0x03, 0x89, 0xbf}, "s_waitcnt vmcnt(0) expcnt(0)");
    vector([](Assembler& a) { a.sopk(Sopk::s_waitcnt_vscnt, null_sgpr(), 0x1234); },
           {0x34, 0x12, 0x7c, 0xbc}, "s_waitcnt_vscnt null, 0x1234");

    // SMEM (gfx11_asm_smem.s).
    vector([](Assembler& a) { a.smem(Smem::s_load_b32, 5, 2, s(0), 0); },
           {0x41, 0x01, 0x00, 0xf4, 0x00, 0x00, 0x00, 0x00}, "s_load_b32 s5, s[2:3], s0");
    vector([](Assembler& a) { a.smem(Smem::s_load_b512, 20, 2, s(0), 0); },
           {0x01, 0x05, 0x10, 0xf4, 0x00, 0x00, 0x00, 0x00}, "s_load_b512 s[20:35], s[2:3], s0");
    vector([](Assembler& a) { a.smem(Smem::s_load_b512, 20, 2, null_sgpr(), 0); },
           {0x01, 0x05, 0x10, 0xf4, 0x00, 0x00, 0x00, 0xf8}, "s_load_b512 s[20:35], s[2:3], null");

    // VOP1 / VOP2 (gfx11_asm_vop1/vop2.s).
    vector([](Assembler& a) { a.vop1(Vop1::v_mov_b32, v(5), s(1)); },
           {0x01, 0x02, 0x0a, 0x7e}, "v_mov_b32 v5, s1");
    vector([](Assembler& a) { a.vop1(Vop1::v_not_b32, v(5), v(1)); },
           {0x01, 0x6f, 0x0a, 0x7e}, "v_not_b32 v5, v1");
    vector([](Assembler& a) { a.vop2(Vop2::v_add_nc_u32, v(5), s(1), v(2)); },
           {0x01, 0x04, 0x0a, 0x4a}, "v_add_nc_u32 v5, s1, v2");
    vector([](Assembler& a) { a.vop2(Vop2::v_and_b32, v(5), v(1), v(2)); },
           {0x01, 0x05, 0x0a, 0x36}, "v_and_b32 v5, v1, v2");
    vector([](Assembler& a) { a.vop2(Vop2::v_lshlrev_b32, v(5), s(1), v(2)); },
           {0x01, 0x04, 0x0a, 0x30}, "v_lshlrev_b32 v5, s1, v2");
    vector([](Assembler& a) { a.vop2(Vop2::v_ashrrev_i32, v(5), v(1), v(2)); },
           {0x01, 0x05, 0x0a, 0x34}, "v_ashrrev_i32 v5, v1, v2");
    vector([](Assembler& a) { a.vop2(Vop2::v_mul_u32_u24, v(255), literal(0xaf123456), v(255)); },
           {0xff, 0xfe, 0xff, 0x17, 0x56, 0x34, 0x12, 0xaf}, "v_mul_u32_u24 v255, lit, v255");
    vector([](Assembler& a) { a.vop2(Vop2::v_min_u32, v(5), s(1), v(2)); },
           {0x01, 0x04, 0x0a, 0x26}, "v_min_u32 v5, s1, v2");

    // VOP2 and VOP1 float graphics subset (gfx11_asm_vop2/vop1.s). These drive
    // the generated OpenGL vertex/pixel shaders.
    vector([](Assembler& a) { a.vop2(Vop2::v_add_f32, v(5), v(1), v(2)); },
           {0x01, 0x05, 0x0a, 0x06}, "v_add_f32 v5, v1, v2");
    vector([](Assembler& a) { a.vop2(Vop2::v_mul_f32, v(5), v(1), v(2)); },
           {0x01, 0x05, 0x0a, 0x10}, "v_mul_f32 v5, v1, v2");
    vector([](Assembler& a) { a.vop2(Vop2::v_sub_f32, v(5), v(1), v(2)); },
           {0x01, 0x05, 0x0a, 0x08}, "v_sub_f32 v5, v1, v2");
    vector([](Assembler& a) { a.vop2(Vop2::v_subrev_f32, v(5), v(1), v(2)); },
           {0x01, 0x05, 0x0a, 0x0a}, "v_subrev_f32 v5, v1, v2");
    vector([](Assembler& a) { a.vop2(Vop2::v_min_f32, v(5), v(1), v(2)); },
           {0x01, 0x05, 0x0a, 0x1e}, "v_min_f32 v5, v1, v2");
    vector([](Assembler& a) { a.vop2(Vop2::v_max_f32, v(5), v(1), v(2)); },
           {0x01, 0x05, 0x0a, 0x20}, "v_max_f32 v5, v1, v2");
    vector([](Assembler& a) { a.vop2(Vop2::v_mul_legacy_f32, v(5), v(1), v(2)); },
           {0x01, 0x05, 0x0a, 0x0e}, "v_mul_legacy_f32 v5, v1, v2");
    vector([](Assembler& a) { a.vop1(Vop1::v_cvt_f32_i32, v(5), v(1)); },
           {0x01, 0x0b, 0x0a, 0x7e}, "v_cvt_f32_i32 v5, v1");
    vector([](Assembler& a) { a.vop1(Vop1::v_cvt_i32_f32, v(5), v(1)); },
           {0x01, 0x11, 0x0a, 0x7e}, "v_cvt_i32_f32 v5, v1");
    vector([](Assembler& a) { a.vop1(Vop1::v_cvt_f32_u32, v(5), v(1)); },
           {0x01, 0x0d, 0x0a, 0x7e}, "v_cvt_f32_u32 v5, v1");
    vector([](Assembler& a) { a.vop1(Vop1::v_cvt_u32_f32, v(5), v(1)); },
           {0x01, 0x0f, 0x0a, 0x7e}, "v_cvt_u32_f32 v5, v1");
    vector([](Assembler& a) { a.vop1(Vop1::v_rcp_f32, v(5), v(1)); },
           {0x01, 0x55, 0x0a, 0x7e}, "v_rcp_f32 v5, v1");

    // EXP position/color outputs (gfx11_asm_exp.s, exp.s, exp-gfx10.s).
    vector([](Assembler& a) { a.exp(ExpTarget::mrt0, v(4), v(3), v(2), v(1), false); },
           {0x0f, 0x00, 0x00, 0xf8, 0x04, 0x03, 0x02, 0x01}, "exp mrt0 v4, v3, v2, v1");
    vector([](Assembler& a) { a.exp(ExpTarget::mrt0, v(4), v(3), v(2), v(1), true); },
           {0x0f, 0x08, 0x00, 0xf8, 0x04, 0x03, 0x02, 0x01}, "exp mrt0 v4, v3, v2, v1 done");
    vector([](Assembler& a) { a.exp(ExpTarget::mrt0, Operand{}, Operand{}, Operand{}, v(1), true); },
           {0x08, 0x08, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x01}, "exp mrt0 off, off, off, v1 done");
    vector([](Assembler& a) { a.exp(ExpTarget::mrtz, v(4), v(3), Operand{}, Operand{}, true, true); },
           {0x83, 0x28, 0x00, 0xf8, 0x04, 0x03, 0x00, 0x00}, "exp mrtz v4, v3, off, off done row_en");
    vector([](Assembler& a) { a.exp(ExpTarget::pos0, v(4), v(3), v(2), v(1), false); },
           {0xcf, 0x00, 0x00, 0xf8, 0x04, 0x03, 0x02, 0x01}, "exp pos0 v4, v3, v2, v1");

    // s_sendmsg for NGG allocation/retirement (gfx11_asm_sopp.s).
    vector([](Assembler& a) { a.sendmsg(2); },
           {0x02, 0x00, 0xb6, 0xbf}, "s_sendmsg sendmsg(MSG_HS_TESSFACTOR)");
    vector([](Assembler& a) { a.sendmsg(msg_dealloc_vgprs); },
           {0x03, 0x00, 0xb6, 0xbf}, "s_sendmsg sendmsg(MSG_DEALLOC_VGPRS)");

    // VOP3 native (gfx11_asm_vop3.s).
    vector([](Assembler& a) { a.vop3(Vop3::v_bfi_b32, v(5), v(1), v(2), s(3)); },
           {0x05, 0x00, 0x12, 0xd6, 0x01, 0x05, 0x0e, 0x00}, "v_bfi_b32 v5, v1, v2, s3");
    vector([](Assembler& a) { a.vop3(Vop3::v_bfe_u32, v(5), v(255), s(2), s(105)); },
           {0x05, 0x00, 0x10, 0xd6, 0xff, 0x05, 0xa4, 0x01}, "v_bfe_u32 v5, v255, s2, s105");
    vector([](Assembler& a) { a.vop3(Vop3::v_bfe_i32, v(5), v(1), v(2), s(3)); },
           {0x05, 0x00, 0x11, 0xd6, 0x01, 0x05, 0x0e, 0x00}, "v_bfe_i32 v5, v1, v2, s3");
    vector([](Assembler& a) { a.vop3(Vop3::v_lshl_or_b32, v(5), v(1), v(2), s(3)); },
           {0x05, 0x00, 0x56, 0xd6, 0x01, 0x05, 0x0e, 0x00}, "v_lshl_or_b32 v5, v1, v2, s3");
    vector([](Assembler& a) { a.vop3(Vop3::v_and_or_b32, v(5), v(1), v(2), s(3)); },
           {0x05, 0x00, 0x57, 0xd6, 0x01, 0x05, 0x0e, 0x00}, "v_and_or_b32 v5, v1, v2, s3");
    vector([](Assembler& a) { a.vop3(Vop3::v_mad_u32_u24, v(5), v(1), v(2), s(3)); },
           {0x05, 0x00, 0x0b, 0xd6, 0x01, 0x05, 0x0e, 0x00}, "v_mad_u32_u24 v5, v1, v2, s3");
    vector([](Assembler& a) { a.vop3(Vop3::v_mul_lo_u32, v(5), v(1), v(2), Operand{}); },
           {0x05, 0x00, 0x2c, 0xd7, 0x01, 0x05, 0x02, 0x00}, "v_mul_lo_u32 v5, v1, v2");
    vector([](Assembler& a) { a.vop3(Vop3::v_mul_hi_u32, v(5), v(1), v(2), Operand{}); },
           {0x05, 0x00, 0x2d, 0xd7, 0x01, 0x05, 0x02, 0x00}, "v_mul_hi_u32 v5, v1, v2");
    vector([](Assembler& a) { a.vop3(Vop3::v_cndmask_b32_e64, v(5), v(1), v(2), s(3)); },
           {0x05, 0x00, 0x01, 0xd5, 0x01, 0x05, 0x0e, 0x00}, "v_cndmask_b32_e64 v5, v1, v2, s3");

    // VOPC promoted to VOP3 writing an SGPR mask (gfx11_asm_vop3_from_vopc.s).
    vector([](Assembler& a) { a.vop3_cmp(Vopc::v_cmp_gt_u32, s(5), v(1), v(2)); },
           {0x05, 0x00, 0x4c, 0xd4, 0x01, 0x05, 0x02, 0x00}, "v_cmp_gt_u32_e64 s5, v1, v2");
    vector([](Assembler& a) { a.vop3_cmp(Vopc::v_cmp_ne_u32, s(5), v(1), v(2)); },
           {0x05, 0x00, 0x4d, 0xd4, 0x01, 0x05, 0x02, 0x00}, "v_cmp_ne_u32_e64 s5, v1, v2");
    vector([](Assembler& a) { a.vop3_cmp(Vopc::v_cmp_le_i32, s(5), v(1), v(2)); },
           {0x05, 0x00, 0x43, 0xd4, 0x01, 0x05, 0x02, 0x00}, "v_cmp_le_i32_e64 s5, v1, v2");
    vector([](Assembler& a) { a.vop3_cmp(Vopc::v_cmp_gt_i32, s(5), v(1), v(2)); },
           {0x05, 0x00, 0x44, 0xd4, 0x01, 0x05, 0x02, 0x00}, "v_cmp_gt_i32_e64 s5, v1, v2");

    // Running pattern phase (raster2d stretch mask with a tile): wave prefix counts.
    // gfx11_asm_vop1.s, gfx11_asm_sop1.s, gfx11_asm_vop3.s.
    vector([](Assembler& a) { a.readfirstlane(s(5), v(1)); }, {0x01, 0x05, 0x0a, 0x7e},
           "v_readfirstlane_b32 s5, v1");
    vector([](Assembler& a) { a.readfirstlane(s(105), v(1)); }, {0x01, 0x05, 0xd2, 0x7e},
           "v_readfirstlane_b32 s105, v1");
    vector([](Assembler& a) { a.sop1(Sop1::s_bcnt1_i32_b32, s(0), s(1)); }, {0x01, 0x18, 0x80, 0xbe},
           "s_bcnt1_i32_b32 s0, s1");
    vector([](Assembler& a) { a.sop1(Sop1::s_bcnt1_i32_b32, s(105), s(104)); }, {0x68, 0x18, 0xe9, 0xbe},
           "s_bcnt1_i32_b32 s105, s104");
    vector([](Assembler& a) { a.sop1(Sop1::s_bcnt1_i32_b32, exec_lo(), s(1)); }, {0x01, 0x18, 0xfe, 0xbe},
           "s_bcnt1_i32_b32 exec_lo, s1");
    vector([](Assembler& a) { a.vop3(Vop3::v_mbcnt_lo_u32_b32, v(5), v(1), v(2), Operand{}); },
           {0x05, 0x00, 0x1f, 0xd7, 0x01, 0x05, 0x02, 0x00}, "v_mbcnt_lo_u32_b32 v5, v1, v2");
    vector([](Assembler& a) { a.vop3(Vop3::v_mbcnt_lo_u32_b32, v(5), s(1), s(2), Operand{}); },
           {0x05, 0x00, 0x1f, 0xd7, 0x01, 0x04, 0x00, 0x00}, "v_mbcnt_lo_u32_b32 v5, s1, s2");
    vector([](Assembler& a) { a.vop3(Vop3::v_mbcnt_lo_u32_b32, v(5), s(105), s(105), Operand{}); },
           {0x05, 0x00, 0x1f, 0xd7, 0x69, 0xd2, 0x00, 0x00}, "v_mbcnt_lo_u32_b32 v5, s105, s105");
    vector([](Assembler& a) { a.vop3(Vop3::v_mbcnt_lo_u32_b32, v(5), v(255), v(255), Operand{}); },
           {0x05, 0x00, 0x1f, 0xd7, 0xff, 0xff, 0x03, 0x00}, "v_mbcnt_lo_u32_b32 v5, v255, v255");

    // MUBUF raw offen (gfx11_asm_mubuf.s).
    vector([](Assembler& a) { a.mubuf(Mubuf::buffer_load_b32, v(5), v(0), 8, s(3), 4095); },
           {0xff, 0x0f, 0x50, 0xe0, 0x00, 0x05, 0x42, 0x03},
           "buffer_load_b32 v5, v0, s[8:11], s3 offen offset:4095");
    vector([](Assembler& a) { a.mubuf(Mubuf::buffer_load_u8, v(5), v(0), 8, s(3), 4095); },
           {0xff, 0x0f, 0x40, 0xe0, 0x00, 0x05, 0x42, 0x03},
           "buffer_load_u8 v5, v0, s[8:11], s3 offen offset:4095");
    vector([](Assembler& a) { a.mubuf(Mubuf::buffer_store_b32, v(1), v(0), 12, s(4), 4095); },
           {0xff, 0x0f, 0x68, 0xe0, 0x00, 0x01, 0x43, 0x04},
           "buffer_store_b32 v1, v0, s[12:15], s4 offen offset:4095");
    // soffset 0 (inline constant) encodes as the 0x80 source code.
    vector([](Assembler& a) { a.mubuf(Mubuf::buffer_load_b32, v(5), v(0), 8, imm(0), 4095); },
           {0xff, 0x0f, 0x50, 0xe0, 0x00, 0x05, 0x42, 0x80},
           "buffer_load_b32 v5, v0, s[8:11], 0 offen offset:4095");
}

void test_rejections() {
    uint8_t buffer[32] = {};
    {
        Assembler a(sdma::CommandBuffer(buffer, sizeof(buffer)));
        a.sop2(Sop2::s_add_u32, s(0), literal(1), literal(2)); // two literals
        expect(!a.ok(), "two scalar literals rejected");
    }
    {
        Assembler a(sdma::CommandBuffer(buffer, sizeof(buffer)));
        a.vop3(Vop3::v_bfi_b32, v(5), literal(1), literal(2), s(3)); // two literals in VOP3
        expect(!a.ok(), "two VOP3 literals rejected");
    }
    {
        Assembler a(sdma::CommandBuffer(buffer, sizeof(buffer)));
        a.smem(Smem::s_load_b128, 21, 2, null_sgpr(), 0); // misaligned sdata
        expect(!a.ok(), "misaligned s_load_b128 destination rejected");
    }
    {
        Assembler a(sdma::CommandBuffer(buffer, sizeof(buffer)));
        a.mubuf(Mubuf::buffer_load_b32, v(5), v(0), 9, s(3), 0); // misaligned V#
        expect(!a.ok(), "misaligned buffer descriptor rejected");
    }
    {
        Assembler a(sdma::CommandBuffer(buffer, sizeof(buffer)));
        a.vop3_cmp(Vopc::v_cmp_gt_u32, v(5), v(1), v(2)); // VGPR mask destination
        expect(!a.ok(), "VGPR compare destination rejected");
    }
    {
        uint8_t small[4] = {};
        Assembler a(sdma::CommandBuffer(small, sizeof(small)));
        a.sop1(Sop1::s_mov_b32, s(0), literal(1)); // 8 bytes into 4
        a.finish();
        expect(!a.ok(), "output overflow rejected");
    }
}

void test_interpreter() {
    gfx11_interp::Wave wave;
    uint8_t code[256] = {};
    Assembler a(sdma::CommandBuffer(code, sizeof(code)));

    // SALU: s4 = (s2 + s3) * 3; SCC branch skips an add when s4 >= 100.
    a.sop2(Sop2::s_add_u32, s(4), s(2), s(3));
    a.sop2(Sop2::s_mul_i32, s(4), s(4), imm(3));
    a.sopc(Sopc::s_cmp_ge_u32, s(4), literal(100));
    const Label skip = a.make_label();
    a.branch(Sopp::s_cbranch_scc1, skip);
    a.sop2(Sop2::s_add_u32, s(4), s(4), imm(1));
    a.bind(skip);
    a.endpgm();
    expect(a.finish(), "interp program assembles");

    wave = {};
    wave.sgpr[2] = 10;
    wave.sgpr[3] = 20;
    gfx11_interp::run(code, a.dwords(), wave);
    expect(wave.sgpr[4] == 91, "SALU + taken branch (30*3=90, +1=91)");

    wave = {};
    wave.sgpr[2] = 40;
    wave.sgpr[3] = 40;
    gfx11_interp::run(code, a.dwords(), wave);
    expect(wave.sgpr[4] == 240, "SALU + skipped branch (80*3=240)");

    // VALU under EXEC + MUBUF raw store with OOB drop.
    uint8_t code2[256] = {};
    Assembler b(sdma::CommandBuffer(code2, sizeof(code2)));
    b.vop2(Vop2::v_add_nc_u32, v(1), imm(1), v(0));          // v1 = lane + 1
    b.vop2(Vop2::v_lshlrev_b32, v(2), imm(2), v(0));         // byte offset = lane * 4
    b.mubuf(Mubuf::buffer_store_b32, v(1), v(2), 0, imm(0), 0);
    b.sopk(Sopk::s_waitcnt_vscnt, null_sgpr(), 0);
    b.endpgm();
    expect(b.finish(), "interp store program assembles");

    gfx11_interp::Wave w2;
    w2.exec = 0x0000ffffu;
    for (uint32_t lane = 0; lane < gfx11_interp::lanes; ++lane) { w2.vgpr[0][lane] = lane; }                                    // 16 lanes enabled
    gfx11_interp::Buffer surface;
    surface.base = 0x1000;
    surface.bytes.assign(40, 0xaa);                           // only 10 dwords: lane 10..15 OOB
    w2.sgpr[0] = static_cast<uint32_t>(surface.base);
    w2.sgpr[1] = static_cast<uint32_t>(surface.base >> 32);
    // descriptor in s[0:3]; raw buffer over 40 bytes.
    uint32_t desc[4];
    rx7900::hardware::raster2d_buffer_descriptor(surface.base, 40, desc);
    w2.sgpr[0] = desc[0];
    w2.sgpr[1] = desc[1];
    w2.sgpr[2] = desc[2];
    w2.sgpr[3] = desc[3];
    gfx11_interp::Memory memory;
    memory.buffers.push_back(&surface);
    gfx11_interp::run(code2, b.dwords(), w2, &memory);

    bool store_ok = true;

    for (uint32_t lane = 0; lane < 10; ++lane) {
        uint32_t value = 0;
        std::memcpy(&value, surface.bytes.data() + lane * 4, 4);

        if (value != lane + 1) {
            store_ok = false;
        }
    }

    expect(store_ok, "EXEC-masked raw store writes enabled in-bounds lanes");

    // Prefix counting: s6 = first active lane of v0, s7 = popcount(s5), v3 =
    // popcount(s5 below the lane) + s7 (the running pattern-phase building blocks).
    uint8_t code3[256] = {};
    Assembler c(sdma::CommandBuffer(code3, sizeof(code3)));
    c.readfirstlane(s(6), v(0));
    c.sop1(Sop1::s_bcnt1_i32_b32, s(7), s(5));
    c.vop3(Vop3::v_mbcnt_lo_u32_b32, v(3), s(5), s(7), Operand{});
    c.endpgm();
    expect(c.finish(), "prefix-count program assembles");
    gfx11_interp::Wave w3;
    w3.exec = 0xfffffff0u;
    w3.sgpr[5] = 0x8000a5a5u;
    for (uint32_t lane = 0; lane < gfx11_interp::lanes; ++lane) { w3.vgpr[0][lane] = 100 + lane; }
    gfx11_interp::run(code3, c.dwords(), w3);
    bool prefix_ok = w3.sgpr[6] == 104 && w3.sgpr[7] == 9 && w3.scc;
    for (uint32_t lane = 4; lane < gfx11_interp::lanes; ++lane) {
        uint32_t below = 0;
        for (uint32_t bit = 0; bit < lane; ++bit) { below += (0x8000a5a5u >> bit) & 1u; }
        prefix_ok = prefix_ok && w3.vgpr[3][lane] == below + 9;
    }
    expect(prefix_ok, "readfirstlane/s_bcnt1/v_mbcnt_lo prefix semantics (first active lane, SCC, lane mask)");
}

// Graphics subset used by the OpenGL fixed-function shaders (gl_shader_gfx11), each
// against the pinned LLVM 18.1.8 GFX11 MC expectation named in the comment.
void test_graphics_encodings() {
    // gfx11_asm_sop1.s
    vector([](Assembler& a) { a.sop1(Sop1::s_wqm_b32, s(0), s(1)); },
           {0x01, 0x1c, 0x80, 0xbe}, "s_wqm_b32 s0, s1");
    vector([](Assembler& a) { a.sop1(Sop1::s_wqm_b32, exec_lo(), s(1)); },
           {0x01, 0x1c, 0xfe, 0xbe}, "s_wqm_b32 exec_lo, s1");
    vector([](Assembler& a) { a.sop1(Sop1::s_mov_b32, m0(), s(1)); },
           {0x01, 0x00, 0xfd, 0xbe}, "s_mov_b32 m0, s1");
    vector([](Assembler& a) { a.sop1(Sop1::s_mov_b32, exec_lo(), s(1)); },
           {0x01, 0x00, 0xfe, 0xbe}, "s_mov_b32 exec_lo, s1");
    // gfx11_asm_sop2.s
    vector([](Assembler& a) { a.sop2(Sop2::s_and_b32, exec_lo(), s(1), s(2)); },
           {0x01, 0x02, 0x7e, 0x8b}, "s_and_b32 exec_lo, s1, s2");
    vector([](Assembler& a) { a.sop2(Sop2::s_and_b32, s(0), exec_lo(), s(2)); },
           {0x7e, 0x02, 0x00, 0x8b}, "s_and_b32 s0, exec_lo, s2");
    // gfx11_asm_sopk.s / gfx11_asm_sopp.s
    vector([](Assembler& a) { a.sopk(Sopk::s_waitcnt_vscnt, null_sgpr(), 0x1234); },
           {0x34, 0x12, 0x7c, 0xbc}, "s_waitcnt_vscnt null, 0x1234");
    vector([](Assembler& a) { a.sopp(Sopp::s_cbranch_execz, 0x1234); },
           {0x34, 0x12, 0xa5, 0xbf}, "s_cbranch_execz 0x1234");
    vector([](Assembler& a) { a.sendmsg(0xc1d1); },
           {0xd1, 0xc1, 0xb6, 0xbf}, "s_sendmsg 0xc1d1 (simm16 field for GS_ALLOC_REQ = 9)");
    // gfx11_asm_smem.s
    vector([](Assembler& a) { a.smem(Smem::s_buffer_load_b32, 5, 4, s(0), 0); },
           {0x42, 0x01, 0x20, 0xf4, 0x00, 0x00, 0x00, 0x00}, "s_buffer_load_b32 s5, s[4:7], s0");
    vector([](Assembler& a) { a.smem(Smem::s_buffer_load_b32, 5, 4, null_sgpr(), 0); },
           {0x42, 0x01, 0x20, 0xf4, 0x00, 0x00, 0x00, 0xf8}, "s_buffer_load_b32 s5, s[4:7], 0x0");
    vector([](Assembler& a) { a.smem(Smem::s_buffer_load_b128, 20, 4, m0(), 0); },
           {0x02, 0x05, 0x28, 0xf4, 0x00, 0x00, 0x00, 0xfa}, "s_buffer_load_b128 s[20:23], s[4:7], m0");
    vector([](Assembler& a) { a.smem(Smem::s_buffer_load_b256, 20, 4, null_sgpr(), 0); },
           {0x02, 0x05, 0x2c, 0xf4, 0x00, 0x00, 0x00, 0xf8}, "s_buffer_load_b256 s[20:27], s[4:7], 0x0");
    vector([](Assembler& a) { a.smem(Smem::s_buffer_load_b512, 20, 4, s(0), 0); },
           {0x02, 0x05, 0x30, 0xf4, 0x00, 0x00, 0x00, 0x00}, "s_buffer_load_b512 s[20:35], s[4:7], s0");
    vector([](Assembler& a) { a.smem(Smem::s_buffer_load_b512, 20, 4, null_sgpr(), 0); },
           {0x02, 0x05, 0x30, 0xf4, 0x00, 0x00, 0x00, 0xf8}, "s_buffer_load_b512 s[20:35], s[4:7], 0x0");
    // gfx11_asm_vop1.s
    vector([](Assembler& a) { a.vop1(Vop1::v_exp_f32, v(5), v(1)); },
           {0x01, 0x4b, 0x0a, 0x7e}, "v_exp_f32 v5, v1");
    vector([](Assembler& a) { a.vop1(Vop1::v_log_f32, v(5), v(1)); },
           {0x01, 0x4f, 0x0a, 0x7e}, "v_log_f32 v5, v1");
    vector([](Assembler& a) { a.vop1(Vop1::v_rsq_f32, v(5), v(1)); },
           {0x01, 0x5d, 0x0a, 0x7e}, "v_rsq_f32 v5, v1");
    vector([](Assembler& a) { a.vop1(Vop1::v_sqrt_f32, v(5), v(1)); },
           {0x01, 0x67, 0x0a, 0x7e}, "v_sqrt_f32 v5, v1");
    vector([](Assembler& a) { a.vop1(Vop1::v_mov_b32, v(5), s(1)); },
           {0x01, 0x02, 0x0a, 0x7e}, "v_mov_b32 v5, s1");
    // gfx11_asm_vop1.s (2026-10-24): LOD rounding and RGBA8 byte conversion.
    vector([](Assembler& a) { a.vop1(Vop1::v_floor_f32, v(5), v(1)); },
           {0x01, 0x49, 0x0a, 0x7e}, "v_floor_f32 v5, v1");
    vector([](Assembler& a) { a.vop1(Vop1::v_floor_f32, v(5), s(1)); },
           {0x01, 0x48, 0x0a, 0x7e}, "v_floor_f32 v5, s1");
    vector([](Assembler& a) { a.vop1(Vop1::v_ceil_f32, v(5), v(1)); },
           {0x01, 0x45, 0x0a, 0x7e}, "v_ceil_f32 v5, v1");
    vector([](Assembler& a) { a.vop1(Vop1::v_fract_f32, v(5), v(1)); },
           {0x01, 0x41, 0x0a, 0x7e}, "v_fract_f32 v5, v1");
    vector([](Assembler& a) { a.vop1(Vop1::v_fract_f32, v(255), literal(0xaf123456)); },
           {0xff, 0x40, 0xfe, 0x7f, 0x56, 0x34, 0x12, 0xaf}, "v_fract_f32 v255, 0xaf123456");
    vector([](Assembler& a) { a.vop1(Vop1::v_cvt_f32_ubyte0, v(5), v(1)); },
           {0x01, 0x23, 0x0a, 0x7e}, "v_cvt_f32_ubyte0 v5, v1");
    vector([](Assembler& a) { a.vop1(Vop1::v_cvt_f32_ubyte1, v(5), v(1)); },
           {0x01, 0x25, 0x0a, 0x7e}, "v_cvt_f32_ubyte1 v5, v1");
    vector([](Assembler& a) { a.vop1(Vop1::v_cvt_f32_ubyte2, v(5), v(1)); },
           {0x01, 0x27, 0x0a, 0x7e}, "v_cvt_f32_ubyte2 v5, v1");
    vector([](Assembler& a) { a.vop1(Vop1::v_cvt_f32_ubyte3, v(5), v(1)); },
           {0x01, 0x29, 0x0a, 0x7e}, "v_cvt_f32_ubyte3 v5, v1");
    // gfx11_asm_vop2.s
    vector([](Assembler& a) { a.vop2(Vop2::v_mul_f32, v(5), s(1), v(2)); },
           {0x01, 0x04, 0x0a, 0x10}, "v_mul_f32 v5, s1, v2");
    vector([](Assembler& a) { a.vop2(Vop2::v_sub_f32, v(5), s(1), v(2)); },
           {0x01, 0x04, 0x0a, 0x08}, "v_sub_f32 v5, s1, v2");
    vector([](Assembler& a) { a.vop2(Vop2::v_add_f32, v(5), s(1), v(2)); },
           {0x01, 0x04, 0x0a, 0x06}, "v_add_f32 v5, s1, v2");
    vector([](Assembler& a) { a.vop2(Vop2::v_mul_f32, v(255), literal(0xaf123456), v(255)); },
           {0xff, 0xfe, 0xff, 0x11, 0x56, 0x34, 0x12, 0xaf}, "v_mul_f32 v255, 0xaf123456, v255");
    vector([](Assembler& a) { a.vop2(Vop2::v_cndmask_b32, v(5), v(1), v(2)); },
           {0x01, 0x05, 0x0a, 0x02}, "v_cndmask_b32 v5, v1, v2, vcc_lo");
    // gfx11_asm_vop3_from_vopc.s / gfx11_asm_vopc.s: float compares.
    vector([](Assembler& a) { a.vop3_cmp(Vopc::v_cmp_lt_f32, s(5), v(1), v(2)); },
           {0x05, 0x00, 0x11, 0xd4, 0x01, 0x05, 0x02, 0x00}, "v_cmp_lt_f32_e64 s5, v1, v2");
    vector([](Assembler& a) { a.vop3_cmp(Vopc::v_cmp_lt_f32, s(5), s(1), s(2)); },
           {0x05, 0x00, 0x11, 0xd4, 0x01, 0x04, 0x00, 0x00}, "v_cmp_lt_f32_e64 s5, s1, s2");
    vector([](Assembler& a) { a.vop3_cmp(Vopc::v_cmp_eq_f32, s(5), v(1), v(2)); },
           {0x05, 0x00, 0x12, 0xd4, 0x01, 0x05, 0x02, 0x00}, "v_cmp_eq_f32_e64 s5, v1, v2");
    vector([](Assembler& a) { a.vop3_cmp(Vopc::v_cmp_le_f32, s(5), v(1), v(2)); },
           {0x05, 0x00, 0x13, 0xd4, 0x01, 0x05, 0x02, 0x00}, "v_cmp_le_f32_e64 s5, v1, v2");
    vector([](Assembler& a) { a.vop3_cmp(Vopc::v_cmp_gt_f32, s(5), v(1), v(2)); },
           {0x05, 0x00, 0x14, 0xd4, 0x01, 0x05, 0x02, 0x00}, "v_cmp_gt_f32_e64 s5, v1, v2");
    vector([](Assembler& a) { a.vop3_cmp(Vopc::v_cmp_lg_f32, s(5), v(1), v(2)); },
           {0x05, 0x00, 0x15, 0xd4, 0x01, 0x05, 0x02, 0x00}, "v_cmp_lg_f32_e64 s5, v1, v2");
    vector([](Assembler& a) { a.vop3_cmp(Vopc::v_cmp_ge_f32, s(5), v(1), v(2)); },
           {0x05, 0x00, 0x16, 0xd4, 0x01, 0x05, 0x02, 0x00}, "v_cmp_ge_f32_e64 s5, v1, v2");
    vector([](Assembler& a) { a.vop3_cmp(Vopc::v_cmp_neq_f32, s(5), v(1), v(2)); },
           {0x05, 0x00, 0x1d, 0xd4, 0x01, 0x05, 0x02, 0x00}, "v_cmp_neq_f32_e64 s5, v1, v2");
    vector([](Assembler& a) { a.vop3_cmp(Vopc::v_cmp_ne_u32, s(5), v(1), v(2)); },
           {0x05, 0x00, 0x4d, 0xd4, 0x01, 0x05, 0x02, 0x00}, "v_cmp_ne_u32_e64 s5, v1, v2");
    vector([](Assembler& a) { a.vop3_cmp(Vopc::v_cmp_gt_u32, s(5), v(1), v(2)); },
           {0x05, 0x00, 0x4c, 0xd4, 0x01, 0x05, 0x02, 0x00}, "v_cmp_gt_u32_e64 s5, v1, v2");
    vector([](Assembler& a) { a.vopc(Vopc::v_cmp_lt_f32, v(1), v(2)); },
           {0x01, 0x05, 0x22, 0x7c}, "v_cmp_lt_f32 vcc_lo, v1, v2");
    // gfx11_asm_mubuf.s
    MubufFlags offen{};
    offen.offen = true;
    MubufFlags idxen{};
    idxen.idxen = true;
    MubufFlags glc{};
    glc.glc = true;
    MubufFlags slc{};
    slc.slc = true;
    MubufFlags dlc{};
    dlc.dlc = true;
    MubufFlags all_cache{};
    all_cache.glc = all_cache.slc = all_cache.dlc = true;
    vector([&](Assembler& a) { a.mubuf_ex(Mubuf::buffer_load_b128, v(5), v(0), 8, s(3), 4095, offen); },
           {0xff, 0x0f, 0x5c, 0xe0, 0x00, 0x05, 0x42, 0x03},
           "buffer_load_b128 v[5:8], v0, s[8:11], s3 offen offset:4095");
    vector([&](Assembler& a) { a.mubuf_ex(Mubuf::buffer_load_b128, v(5), v(0), 8, s(3), 4095, idxen); },
           {0xff, 0x0f, 0x5c, 0xe0, 0x00, 0x05, 0x82, 0x03},
           "buffer_load_b128 v[5:8], v0, s[8:11], s3 idxen offset:4095");
    vector([&](Assembler& a) { a.mubuf_ex(Mubuf::buffer_load_b128, v(5), Operand{}, 8, s(3), 7, {}); },
           {0x07, 0x00, 0x5c, 0xe0, 0x00, 0x05, 0x02, 0x03},
           "buffer_load_b128 v[5:8], off, s[8:11], s3 offset:7");
    vector([&](Assembler& a) { a.mubuf_ex(Mubuf::buffer_load_b128, v(5), Operand{}, 8, imm(0), 4095, {}); },
           {0xff, 0x0f, 0x5c, 0xe0, 0x00, 0x05, 0x02, 0x80},
           "buffer_load_b128 v[5:8], off, s[8:11], 0 offset:4095");
    vector([&](Assembler& a) { a.mubuf_ex(Mubuf::buffer_load_b128, v(5), Operand{}, 8, s(3), 4095, all_cache); },
           {0xff, 0x7f, 0x5c, 0xe0, 0x00, 0x05, 0x02, 0x03},
           "buffer_load_b128 v[5:8], off, s[8:11], s3 offset:4095 glc slc dlc");
    vector([&](Assembler& a) { a.mubuf_ex(Mubuf::buffer_store_b128, v(1), v(0), 12, s(4), 4095, idxen); },
           {0xff, 0x0f, 0x74, 0xe0, 0x00, 0x01, 0x83, 0x04},
           "buffer_store_b128 v[1:4], v0, s[12:15], s4 idxen offset:4095");
    vector([&](Assembler& a) { a.mubuf_ex(Mubuf::buffer_store_b128, v(1), v(0), 12, s(4), 4095, offen); },
           {0xff, 0x0f, 0x74, 0xe0, 0x00, 0x01, 0x43, 0x04},
           "buffer_store_b128 v[1:4], v0, s[12:15], s4 offen offset:4095");
    vector([&](Assembler& a) { a.mubuf_ex(Mubuf::buffer_store_b128, v(1), Operand{}, 12, s(4), 4095, glc); },
           {0xff, 0x4f, 0x74, 0xe0, 0x00, 0x01, 0x03, 0x04},
           "buffer_store_b128 v[1:4], off, s[12:15], s4 offset:4095 glc");
    vector([&](Assembler& a) { a.mubuf_ex(Mubuf::buffer_store_b128, v(1), Operand{}, 12, s(4), 4095, slc); },
           {0xff, 0x1f, 0x74, 0xe0, 0x00, 0x01, 0x03, 0x04},
           "buffer_store_b128 v[1:4], off, s[12:15], s4 offset:4095 slc");
    vector([&](Assembler& a) { a.mubuf_ex(Mubuf::buffer_store_b128, v(1), Operand{}, 12, s(4), 4095, dlc); },
           {0xff, 0x2f, 0x74, 0xe0, 0x00, 0x01, 0x03, 0x04},
           "buffer_store_b128 v[1:4], off, s[12:15], s4 offset:4095 dlc");
    vector([&](Assembler& a) { a.mubuf_ex(Mubuf::buffer_store_b128, v(1), Operand{}, 12, s(4), 0, {}); },
           {0x00, 0x00, 0x74, 0xe0, 0x00, 0x01, 0x03, 0x04},
           "buffer_store_b128 v[1:4], off, s[12:15], s4 offset:0");
    // gfx11_asm_mimg.s
    vector([](Assembler& a) { a.mimg_sample(Mimg::image_sample, v(5), v(1), 8, 12, 0x3, mimg_dim_2d); },
           {0x04, 0x03, 0x6c, 0xf0, 0x01, 0x05, 0x02, 0x0c},
           "image_sample v[5:6], v[1:2], s[8:15], s[12:15] dmask:0x3 dim:SQ_RSRC_IMG_2D");
    vector([](Assembler& a) { a.mimg_sample(Mimg::image_sample, v(254), v(254), 8, 12, 0x3, mimg_dim_2d); },
           {0x04, 0x03, 0x6c, 0xf0, 0xfe, 0xfe, 0x02, 0x0c},
           "image_sample v[254:255], v[254:255], s[8:15], s[12:15] dmask:0x3 dim:SQ_RSRC_IMG_2D");
    vector([](Assembler& a) { a.mimg_sample(Mimg::image_sample_lz, v(5), v(1), 8, 12, 0x3, mimg_dim_2d); },
           {0x04, 0x03, 0x7c, 0xf0, 0x01, 0x05, 0x02, 0x0c},
           "image_sample_lz v[5:6], v[1:2], s[8:15], s[12:15] dmask:0x3 dim:SQ_RSRC_IMG_2D");
    vector([](Assembler& a) { a.mimg_sample(Mimg::image_sample_lz, v(254), v(254), 8, 12, 0x3, mimg_dim_2d); },
           {0x04, 0x03, 0x7c, 0xf0, 0xfe, 0xfe, 0x02, 0x0c},
           "image_sample_lz v[254:255], v[254:255], s[8:15], s[12:15] dmask:0x3 dim:SQ_RSRC_IMG_2D");
    vector([](Assembler& a) { a.mimg_sample(Mimg::image_sample_l, v(5), v(1), 8, 12, 0x3, mimg_dim_2d); },
           {0x04, 0x03, 0x74, 0xf0, 0x01, 0x05, 0x02, 0x0c},
           "image_sample_l v[5:6], v[1:3], s[8:15], s[12:15] dmask:0x3 dim:SQ_RSRC_IMG_2D");
    // gfx11_asm_ldsdir.s
    vector([](Assembler& a) { a.lds_param_load(v(1), 0, 0, 15); },
           {0x01, 0x00, 0x0f, 0xce}, "lds_param_load v1, attr0.x wait_vdst:15");
    vector([](Assembler& a) { a.lds_param_load(v(2), 0, 1, 14); },
           {0x02, 0x01, 0x0e, 0xce}, "lds_param_load v2, attr0.y wait_vdst:14");
    vector([](Assembler& a) { a.lds_param_load(v(4), 0, 3, 12); },
           {0x04, 0x03, 0x0c, 0xce}, "lds_param_load v4, attr0.w wait_vdst:12");
    vector([](Assembler& a) { a.lds_param_load(v(7), 2, 1, 9); },
           {0x07, 0x09, 0x09, 0xce}, "lds_param_load v7, attr2.y wait_vdst:9");
    vector([](Assembler& a) { a.lds_param_load(v(10), 11, 0, 6); },
           {0x0a, 0x2c, 0x06, 0xce}, "lds_param_load v10, attr11.x wait_vdst:6");
    vector([](Assembler& a) { a.lds_param_load(v(12), 32, 2, 4); },
           {0x0c, 0x82, 0x04, 0xce}, "lds_param_load v12, attr32.z wait_vdst:4");
    vector([](Assembler& a) { a.lds_direct_load(v(1), 15); },
           {0x01, 0x00, 0x1f, 0xce}, "lds_direct_load v1 wait_vdst:15");
    vector([](Assembler& a) { a.lds_direct_load(v(16), 0); },
           {0x10, 0x00, 0x10, 0xce}, "lds_direct_load v16 wait_vdst:0");
    // gfx11_asm_vinterp.s
    vector([](Assembler& a) { a.vinterp(Vinterp::v_interp_p10_f32, v(0), v(1), v(2), v(3), 0); },
           {0x00, 0x00, 0x00, 0xcd, 0x01, 0x05, 0x0e, 0x04}, "v_interp_p10_f32 v0, v1, v2, v3 wait_exp:0");
    vector([](Assembler& a) { a.vinterp(Vinterp::v_interp_p10_f32, v(3), v(12), v(22), v(32), 0); },
           {0x03, 0x00, 0x00, 0xcd, 0x0c, 0x2d, 0x82, 0x04}, "v_interp_p10_f32 v3, v12, v22, v32 wait_exp:0");
    vector([](Assembler& a) { a.vinterp(Vinterp::v_interp_p10_f32, v(0), v(1), v(2), v(3), 7); },
           {0x00, 0x07, 0x00, 0xcd, 0x01, 0x05, 0x0e, 0x04}, "v_interp_p10_f32 v0, v1, v2, v3 wait_exp:7");
    vector([](Assembler& a) { a.vinterp(Vinterp::v_interp_p2_f32, v(0), v(1), v(2), v(3), 0); },
           {0x00, 0x00, 0x01, 0xcd, 0x01, 0x05, 0x0e, 0x04}, "v_interp_p2_f32 v0, v1, v2, v3 wait_exp:0");
    vector([](Assembler& a) { a.vinterp(Vinterp::v_interp_p2_f32, v(1), v(10), v(20), v(30), 0); },
           {0x01, 0x00, 0x01, 0xcd, 0x0a, 0x29, 0x7a, 0x04}, "v_interp_p2_f32 v1, v10, v20, v30 wait_exp:0");
    vector([](Assembler& a) { a.vinterp(Vinterp::v_interp_p2_f32, v(0), v(1), v(2), v(3), 1); },
           {0x00, 0x01, 0x01, 0xcd, 0x01, 0x05, 0x0e, 0x04}, "v_interp_p2_f32 v0, v1, v2, v3 wait_exp:1");
    // exp-gfx10.s / exp.s: NGG primitive and later position targets.
    vector([](Assembler& a) { a.exp(ExpTarget::prim, v(1), Operand{}, Operand{}, Operand{}, false); },
           {0x41, 0x01, 0x00, 0xf8, 0x01, 0x00, 0x00, 0x00}, "exp prim v1, off, off, off");
    vector([](Assembler& a) { a.exp(ExpTarget::prim, v(2), v(3), Operand{}, Operand{}, false); },
           {0x43, 0x01, 0x00, 0xf8, 0x02, 0x03, 0x00, 0x00}, "exp prim v2, v3, off, off");
    vector([](Assembler& a) { a.exp(ExpTarget::pos0, v(4), v(3), v(2), v(1), true); },
           {0xcf, 0x08, 0x00, 0xf8, 0x04, 0x03, 0x02, 0x01}, "exp pos0 v4, v3, v2, v1 done");
    vector([](Assembler& a) { a.exp(ExpTarget::mrt0, Operand{}, Operand{}, Operand{}, Operand{}, true); },
           {0x00, 0x08, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00}, "exp mrt0 off, off, off, off done");

    // Rejections of malformed graphics operands.
    uint8_t buffer[32] = {};
    {
        Assembler a(sdma::CommandBuffer(buffer, sizeof(buffer)));
        a.mubuf_ex(Mubuf::buffer_load_b128, v(253), v(0), 8, s(3), 0, offen);
        expect(!a.ok(), "b128 VGPR tuple past v255 rejected");
    }
    {
        Assembler a(sdma::CommandBuffer(buffer, sizeof(buffer)));
        a.mimg_sample(Mimg::image_sample_lz, v(5), v(1), 6, 12, 0xf, mimg_dim_2d);
        expect(!a.ok(), "misaligned T# rejected");
    }
    {
        Assembler a(sdma::CommandBuffer(buffer, sizeof(buffer)));
        a.smem(Smem::s_buffer_load_b512, 20, 6, null_sgpr(), 0);
        expect(!a.ok(), "s_buffer_load with a non-quad V# base rejected");
    }
    {
        Assembler a(sdma::CommandBuffer(buffer, sizeof(buffer)));
        a.lds_param_load(v(1), 33, 0, 0);
        expect(!a.ok(), "lds_param_load attribute beyond attr32 rejected");
    }
    {
        Assembler a(sdma::CommandBuffer(buffer, sizeof(buffer)));
        a.vinterp(Vinterp::v_interp_p10_f32, v(0), s(1), v(2), v(3), 0);
        expect(!a.ok(), "v_interp with an SGPR source rejected");
    }
}

} // namespace

int main() {
    test_encodings();
    test_graphics_encodings();
    test_rejections();
    test_interpreter();
    std::printf("gfx11_isa: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

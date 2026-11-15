/*
Module: rx7900.tests.dcn32_dsc
Purpose: Check the DCN 3.2 DSC policy, rate control, PPS and DSC engine programming against Linux v6.12 and VESA DSC 1.2a.
Created: 2026-10-24
Architecture: Host test over src/display/dcn32_dsc. The golden vectors were
produced by a scratch oracle that compiles the unmodified pinned Linux v6.12
sources (dc/dsc/dc_dsc.c, dc/dsc/dcn20/dcn20_dsc.c, dc/dml/dsc/rc_calc_fpu.c with
qp_tables.h, dc/dsc/rc_calc.c, rc_calc_dpi.c, dc/basics/fixpt31_32.c and the
drm_dsc_pps_payload_pack / drm_dsc_compute_rc_parameters functions of
drm/display/drm_dsc_helper.c) for x86-64 with the DCN 3.2 DSC register and mask
lists of dcn32_resource.c over a sparse register file; each array records the
oracle command line (prep: engine, picture width/height, slices h/v, bpp x16,
line buffer depth, DSC minor version, block prediction, ODM; policy: the 16
DPCD 0x060.. bytes, the 3 branch bytes, pixel clock in 100 Hz, h/v active,
h/v total, link kbps). Independently of the packer, every PPS is decoded with
the VESA DSC 1.2a picture parameter set layout and compared with the
configuration. Synthetic sinks; this is not hardware evidence.
*/
#include <cstdio>
#include <cstring>
#include <vector>

#include "asic_test_device.hpp"
#include "dcn32_dsc.hpp"
#include "dcn32_stream_model.hpp"

namespace {

using namespace rx7900::display;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

// oracle prep 1 3840 2160 2 20 256 13 2 1 0
constexpr uint32_t prep_4k144_hbr3_values[]{
    13u, 8u, 1u, 1920u, 108u, 3840u, 2160u, 256u, 11u, 11u, 256u, 984u, 1u, 15u, 2048u, 8192u, 3u, 12u, 10u, 320u,
    2653u, 288u, 190u, 4336u, 3840u, 2u, 1u, 0u, 0u, 0u, 6u, 3u, 14u, 28u, 42u, 56u, 70u, 84u, 98u, 105u, 112u,
    119u, 121u, 123u, 125u, 126u, 0u, 0u, 10u, 0u, 1u, 8u, 0u, 1u, 6u, 0u, 2u, 4u, 1u, 2u, 2u, 1u, 3u, 0u, 1u, 3u,
    62u, 2u, 4u, 60u, 2u, 5u, 58u, 3u, 5u, 56u, 4u, 5u, 54u, 4u, 6u, 54u, 5u, 6u, 52u, 6u, 7u, 52u, 7u, 8u, 52u,
    0u, 2u, 20u, 512u, 15u, 1u, 19840u, 536870912u, 1920u, 1u,};
constexpr uint8_t prep_4k144_hbr3_pps[dsc_pps_bytes]{
    18, 0, 0, 141, 49, 0, 8, 112, 15, 0, 0, 108, 7, 128, 15, 0, 1, 0, 3, 216, 0, 10, 10, 93, 1, 64, 0, 15, 1, 32,
    0, 190, 8, 0, 16, 240, 3, 12, 32, 0, 6, 11, 11, 51, 14, 28, 42, 56, 70, 84, 98, 105, 112, 119, 121, 123, 125,
    126, 0, 10, 0, 72, 0, 70, 0, 132, 8, 130, 8, 192, 8, 254, 17, 60, 17, 122, 25, 120, 33, 118, 33, 182, 41, 180,
    49, 244, 58, 52, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,};
constexpr uint32_t prep_4k144_hbr3_trace[]{
    0x10651d, 0x00000000, 0x106521, 0x00080000, 0x106522, 0x08700f00, 0x106526, 0x0013011f, 0x106527, 0x00004d80,
    0x106529, 0x00000000, 0x10652a, 0x8d000002, 0x10652b, 0x0f003100, 0x10652c, 0x08700f00, 0x10652d, 0x006c0780,
    0x10652e, 0x00000100, 0x10652f, 0x0a5d000a, 0x106530, 0x000f0140, 0x106531, 0x00be0120, 0x106532, 0x00000000,
    0x106533, 0x10f00800, 0x106534, 0x20000c03, 0x106535, 0x330b0b06, 0x106536, 0x382a1c0e, 0x106537, 0x69625446,
    0x106538, 0x7b797770, 0x106539, 0x28007e7d, 0x10653a, 0x18202020, 0x10653b, 0x08411040, 0x10653c, 0xf8610061,
    0x10653d, 0xe8a2f082, 0x10653e, 0xd8a4e0a3, 0x10653f, 0xd0c5d8c4, 0x106540, 0xd107d0e6, 0x00651c, 0x00000000,
    0x004f25, 0x00000000, 0x00651c, 0x00000000, 0x10651c, 0x00000001, 0x004f25, 0x00000000, 0x104f25, 0x00000011,
    0x004f25, 0x00000011, 0x104f25, 0x00000010, 0x00651c, 0x00000001, 0x004f25, 0x00000010, 0x104f25, 0x00000010,
    0x00651c, 0x00000001, 0x10651c, 0x00000000,};
// oracle prep 0 3840 2160 2 20 205 13 2 1 0
constexpr uint32_t prep_4k144_hbr2_values[]{
    13u, 8u, 1u, 1920u, 108u, 3840u, 2160u, 205u, 11u, 11u, 320u, 1229u, 1u, 15u, 2048u, 8192u, 3u, 12u, 10u, 320u,
    2651u, 288u, 190u, 4332u, 3075u, 2u, 1u, 0u, 0u, 0u, 6u, 3u, 14u, 28u, 42u, 56u, 70u, 84u, 98u, 105u, 112u,
    119u, 121u, 123u, 125u, 126u, 0u, 2u, 4u, 0u, 3u, 2u, 1u, 4u, 2u, 1u, 5u, 0u, 3u, 6u, 62u, 3u, 6u, 60u, 3u, 6u,
    58u, 3u, 7u, 57u, 3u, 7u, 57u, 3u, 8u, 55u, 4u, 8u, 54u, 5u, 9u, 54u, 5u, 9u, 52u, 7u, 10u, 52u, 10u, 11u, 52u,
    0u, 2u, 20u, 410u, 15u, 1u, 19846u, 429916160u, 1920u, 1u,};
constexpr uint8_t prep_4k144_hbr2_pps[dsc_pps_bytes]{
    18, 0, 0, 141, 48, 205, 8, 112, 15, 0, 0, 108, 7, 128, 12, 3, 1, 64, 4, 205, 0, 10, 10, 91, 1, 64, 0, 15, 1,
    32, 0, 190, 8, 0, 16, 236, 3, 12, 32, 0, 6, 11, 11, 51, 14, 28, 42, 56, 70, 84, 98, 105, 112, 119, 121, 123,
    125, 126, 0, 132, 0, 194, 9, 2, 9, 64, 25, 190, 25, 188, 25, 186, 25, 249, 25, 249, 26, 55, 34, 54, 42, 118,
    42, 116, 58, 180, 82, 244, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};
constexpr uint32_t prep_4k144_hbr2_trace[]{
    0x1064c1, 0x00000000, 0x1064c5, 0x00080000, 0x1064c6, 0x08700f00, 0x1064ca, 0x0013011f, 0x1064cb, 0x00004d86,
    0x1064cd, 0x00000000, 0x1064ce, 0x8d000002, 0x1064cf, 0x0c0330cd, 0x1064d0, 0x08700f00, 0x1064d1, 0x006c0780,
    0x1064d2, 0x00000140, 0x1064d3, 0x0a5b000a, 0x1064d4, 0x000f0140, 0x1064d5, 0x00be0120, 0x1064d6, 0x00000000,
    0x1064d7, 0x10ec0800, 0x1064d8, 0x20000c03, 0x1064d9, 0x330b0b06, 0x1064da, 0x382a1c0e, 0x1064db, 0x69625446,
    0x1064dc, 0x7b797770, 0x1064dd, 0x10407e7d, 0x1064de, 0x08810860, 0x1064df, 0xf8c300a1, 0x1064e0, 0xe8c3f0c3,
    0x1064e1, 0xe4e3e4e3, 0x1064e2, 0xd904dd03, 0x1064e3, 0xd125d925, 0x1064e4, 0xd16ad147, 0x0064c0, 0x00000000,
    0x004f24, 0x00000000, 0x0064c0, 0x00000000, 0x1064c0, 0x00000001, 0x004f24, 0x00000000, 0x104f24, 0x00000001,
    0x004f24, 0x00000001, 0x104f24, 0x00000000, 0x0064c0, 0x00000001, 0x004f24, 0x00000000, 0x104f24, 0x00000000,
    0x0064c0, 0x00000001, 0x1064c0, 0x00000000,};
// oracle prep 2 3840 4320 2 40 195 13 2 1 1
constexpr uint32_t prep_8k60_odm_values[]{
    13u, 8u, 1u, 1920u, 108u, 3840u, 4320u, 195u, 11u, 11u, 336u, 1292u, 1u, 15u, 2048u, 8192u, 3u, 12u, 10u, 320u,
    2322u, 288u, 190u, 4337u, 2925u, 2u, 1u, 0u, 0u, 0u, 6u, 3u, 14u, 28u, 42u, 56u, 70u, 84u, 98u, 105u, 112u,
    119u, 121u, 123u, 125u, 126u, 0u, 2u, 3u, 0u, 3u, 1u, 1u, 4u, 0u, 1u, 5u, 62u, 3u, 6u, 60u, 3u, 6u, 58u, 3u,
    7u, 56u, 3u, 8u, 56u, 3u, 8u, 56u, 3u, 9u, 54u, 5u, 9u, 54u, 5u, 9u, 54u, 5u, 9u, 52u, 7u, 10u, 52u, 10u, 11u,
    52u, 0u, 2u, 40u, 390u, 15u, 1u, 19841u, 408944640u, 1920u, 1u,};
constexpr uint8_t prep_8k60_odm_pps[dsc_pps_bytes]{
    18, 0, 0, 141, 48, 195, 16, 224, 15, 0, 0, 108, 7, 128, 11, 109, 1, 80, 5, 12, 0, 10, 9, 18, 1, 64, 0, 15, 1,
    32, 0, 190, 8, 0, 16, 241, 3, 12, 32, 0, 6, 11, 11, 51, 14, 28, 42, 56, 70, 84, 98, 105, 112, 119, 121, 123,
    125, 126, 0, 131, 0, 193, 9, 0, 9, 126, 25, 188, 25, 186, 25, 248, 26, 56, 26, 56, 26, 118, 42, 118, 42, 118,
    42, 116, 58, 180, 82, 244, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};
constexpr uint32_t prep_8k60_odm_trace[]{
    0x106579, 0x00000000, 0x10657d, 0x00080000, 0x10657e, 0x10e00f00, 0x106582, 0x0027011f, 0x106583, 0x00004d81,
    0x106585, 0x00000000, 0x106586, 0x8d000002, 0x106587, 0x0b6d30c3, 0x106588, 0x10e00f00, 0x106589, 0x006c0780,
    0x10658a, 0x00000150, 0x10658b, 0x0912000a, 0x10658c, 0x000f0140, 0x10658d, 0x00be0120, 0x10658e, 0x00000000,
    0x10658f, 0x10f10800, 0x106590, 0x20000c03, 0x106591, 0x330b0b06, 0x106592, 0x382a1c0e, 0x106593, 0x69625446,
    0x106594, 0x7b797770, 0x106595, 0x0c407e7d, 0x106596, 0x00810460, 0x106597, 0xf0c3f8a1, 0x106598, 0xe0e3e8c3,
    0x106599, 0xe103e103, 0x10659a, 0xd925d923, 0x10659b, 0xd125d925, 0x10659c, 0xd16ad147, 0x006578, 0x00000000,
    0x004f26, 0x00000000, 0x006578, 0x00000000, 0x106578, 0x00000001, 0x004f26, 0x00000000, 0x104f26, 0x00000021,
    0x004f26, 0x00000021, 0x104f26, 0x00000020, 0x006578, 0x00000001, 0x004f26, 0x00000020, 0x104f26, 0x00000020,
    0x006578, 0x00000001, 0x106578, 0x00000000,};
// oracle prep 3 1920 1080 1 10 128 8 1 0 0
constexpr uint32_t prep_1080_dsc11_values[]{
    8u, 8u, 1u, 1920u, 108u, 1920u, 1080u, 128u, 11u, 11u, 512u, 1456u, 0u, 15u, 6144u, 8192u, 3u, 12u, 32u, 26u,
    3563u, 288u, 68u, 4336u, 1920u, 1u, 1u, 0u, 0u, 0u, 6u, 3u, 14u, 28u, 42u, 56u, 70u, 84u, 98u, 105u, 112u,
    119u, 121u, 123u, 125u, 126u, 0u, 4u, 2u, 0u, 4u, 0u, 1u, 5u, 0u, 1u, 6u, 62u, 3u, 7u, 60u, 3u, 7u, 58u, 3u,
    7u, 56u, 3u, 8u, 56u, 3u, 9u, 56u, 4u, 10u, 54u, 5u, 10u, 54u, 5u, 11u, 54u, 5u, 11u, 52u, 8u, 12u, 52u, 12u,
    13u, 52u, 0u, 1u, 10u, 256u, 0u, 0u, 15744u, 268435456u, 1920u, 1u,};
constexpr uint8_t prep_1080_dsc11_pps[dsc_pps_bytes]{
    17, 0, 0, 136, 16, 128, 4, 56, 7, 128, 0, 108, 7, 128, 7, 128, 2, 0, 5, 176, 0, 32, 13, 235, 0, 26, 0, 15, 1,
    32, 0, 68, 24, 0, 16, 240, 3, 12, 32, 0, 6, 11, 11, 51, 14, 28, 42, 56, 70, 84, 98, 105, 112, 119, 121, 123,
    125, 126, 1, 2, 1, 0, 9, 64, 9, 190, 25, 252, 25, 250, 25, 248, 26, 56, 26, 120, 34, 182, 42, 182, 42, 246, 42,
    244, 67, 52, 99, 116, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};
constexpr uint32_t prep_1080_dsc11_trace[]{
    0x1065d5, 0x00000000, 0x1065d9, 0x00080000, 0x1065da, 0x04380780, 0x1065de, 0x00090000, 0x1065df, 0x00003d80,
    0x1065e1, 0x00000000, 0x1065e2, 0x88000001, 0x1065e3, 0x07801080, 0x1065e4, 0x04380780, 0x1065e5, 0x006c0780,
    0x1065e6, 0x00000200, 0x1065e7, 0x0deb0020, 0x1065e8, 0x000f001a, 0x1065e9, 0x00440120, 0x1065ea, 0x00000000,
    0x1065eb, 0x10f01800, 0x1065ec, 0x20000c03, 0x1065ed, 0x330b0b06, 0x1065ee, 0x382a1c0e, 0x1065ef, 0x69625446,
    0x1065f0, 0x7b797770, 0x1065f1, 0x08807e7d, 0x1065f2, 0x00a10080, 0x1065f3, 0xf0e3f8c1, 0x1065f4, 0xe0e3e8e3,
    0x1065f5, 0xe123e103, 0x1065f6, 0xd945d944, 0x1065f7, 0xd165d965, 0x1065f8, 0xd1acd188, 0x0065d4, 0x00000000,
    0x004f27, 0x00000000, 0x0065d4, 0x00000000, 0x1065d4, 0x00000001, 0x004f27, 0x00000000, 0x104f27, 0x00000031,
    0x004f27, 0x00000031, 0x104f27, 0x00000030, 0x0065d4, 0x00000001, 0x004f27, 0x00000030, 0x104f27, 0x00000030,
    0x0065d4, 0x00000001, 0x1065d4, 0x00000000,};
// oracle policy 1 21 3 0 2b 4 1 0 0 3 6 bb d 0 0 0 0 0 0 12909504 3840 2160 3920 2287 25142400
constexpr uint32_t policy_4k144_hbr3[]{
    1u, 1u, 33u, 65536u, 43u, 0u, 13u, 1u, 0u, 3u, 6u, 850u, 850u, 4160u, 16u, 0u, 0u, 0u, 1u, 1u, 2u, 20u, 256u,
    1u, 13u, 2u, 1u, 20823821u,};
// oracle policy 1 21 3 0 2b 4 1 0 0 3 6 bb d 0 0 0 0 0 0 12909504 3840 2160 3920 2287 16761600
constexpr uint32_t policy_4k144_hbr2[]{
    1u, 1u, 33u, 65536u, 43u, 0u, 13u, 1u, 0u, 3u, 6u, 850u, 850u, 4160u, 16u, 0u, 0u, 0u, 1u, 1u, 2u, 20u, 205u,
    1u, 13u, 2u, 1u, 16708916u,};
// oracle policy 1 21 3 0 2b 4 1 0 0 3 6 bb d 0 0 0 0 0 0 20397456 7680 4320 7760 4381 25142400
constexpr uint32_t policy_8k60_hbr3[]{
    1u, 1u, 33u, 65536u, 43u, 0u, 13u, 1u, 0u, 3u, 6u, 850u, 850u, 4160u, 16u, 0u, 0u, 0u, 1u, 1u, 4u, 40u, 195u,
    1u, 13u, 2u, 1u, 25128563u,};
// oracle policy 1 21 3 0 2b 4 1 0 0 3 6 bb d 0 0 0 0 0 0 5940000 3840 2160 4400 2250 17280000
constexpr uint32_t policy_4k60_fits[]{
    1u, 1u, 33u, 65536u, 43u, 0u, 13u, 1u, 0u, 3u, 6u, 850u, 850u, 4160u, 16u, 0u, 0u, 0u, 1u, 0u, 0u, 0u, 0u, 0u,
    0u, 0u, 0u,};
// oracle policy 0 21 3 0 2b 4 1 0 0 3 6 bb d 0 0 0 0 0 0 12909504 3840 2160 3920 2287 25142400
constexpr uint32_t policy_no_dsc[]{
    0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u,};
// oracle policy 1 11 3 0 03 4 1 0 0 1 2 77 8 0 0 0 3 2 10 12909504 3840 2160 3920 2287 16761600
constexpr uint32_t policy_dsc11_branch[]{
    1u, 1u, 17u, 65536u, 3u, 0u, 13u, 1u, 0u, 1u, 2u, 650u, 650u, 2560u, 16u, 750u, 700u, 5120u, 1u, 0u, 0u, 0u,
    0u, 0u, 0u, 0u, 0u,};
// oracle rc <bpp_x16> 1920 108 2 for bpp 12.0625 .. 15.0 (the 8/3, 4/3 and 2/3 products): ofs[0..14]
constexpr int8_t rc_ofs_12_to_15[][15]{
    {2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -10, -12, -12, -12},
    {2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -10, -12, -12, -12},
    {3, 1, 0, -2, -4, -6, -8, -8, -8, -10, -10, -10, -12, -12, -12},
    {3, 1, 1, -1, -3, -5, -7, -8, -8, -10, -10, -10, -12, -12, -12},
    {3, 1, 1, -1, -3, -5, -7, -8, -8, -10, -10, -10, -12, -12, -12},
    {3, 1, 1, -1, -3, -5, -7, -7, -8, -10, -10, -10, -12, -12, -12},
    {3, 1, 1, -1, -3, -5, -7, -7, -8, -10, -10, -10, -12, -12, -12},
    {3, 1, 1, -1, -3, -5, -7, -7, -8, -10, -10, -10, -12, -12, -12},
    {4, 2, 1, -1, -3, -5, -7, -7, -8, -10, -10, -10, -12, -12, -12},
    {4, 2, 1, -1, -3, -5, -7, -7, -8, -10, -10, -10, -12, -12, -12},
    {4, 2, 1, -1, -3, -5, -7, -7, -8, -10, -10, -10, -12, -12, -12},
    {4, 2, 2, 0, -2, -4, -6, -7, -7, -9, -10, -10, -12, -12, -12},
    {4, 2, 2, 0, -2, -4, -6, -7, -7, -9, -10, -10, -12, -12, -12},
    {4, 2, 2, 0, -2, -4, -6, -7, -7, -9, -10, -10, -12, -12, -12},
    {5, 3, 2, 0, -2, -4, -6, -7, -7, -9, -10, -10, -12, -12, -12},
    {5, 3, 2, 0, -2, -4, -6, -7, -7, -9, -10, -10, -12, -12, -12},
    {5, 3, 2, 0, -2, -4, -6, -7, -7, -9, -10, -10, -12, -12, -12},
    {5, 3, 2, 0, -2, -4, -6, -6, -7, -9, -10, -10, -12, -12, -12},
    {5, 3, 2, 0, -2, -4, -6, -6, -7, -9, -10, -10, -12, -12, -12},
    {5, 3, 3, 1, -1, -3, -5, -6, -7, -9, -10, -10, -12, -12, -12},
    {6, 4, 3, 1, -1, -3, -5, -6, -7, -9, -10, -10, -12, -12, -12},
    {6, 4, 3, 1, -1, -3, -5, -6, -7, -9, -10, -10, -12, -12, -12},
    {6, 4, 3, 1, -1, -3, -5, -6, -7, -9, -10, -10, -12, -12, -12},
    {6, 4, 3, 1, -1, -3, -5, -6, -7, -9, -10, -10, -12, -12, -12},
    {6, 4, 3, 1, -1, -3, -5, -6, -7, -9, -10, -10, -12, -12, -12},
    {6, 4, 3, 1, -1, -3, -5, -6, -7, -9, -10, -10, -12, -12, -12},
    {7, 5, 3, 1, -1, -3, -5, -6, -7, -9, -10, -10, -12, -12, -12},
    {7, 5, 4, 2, 0, -2, -4, -6, -7, -9, -10, -10, -12, -12, -12},
    {7, 5, 4, 2, 0, -2, -4, -6, -7, -9, -10, -10, -12, -12, -12},
    {7, 5, 4, 2, 0, -2, -4, -5, -7, -9, -10, -10, -12, -12, -12},
    {7, 5, 4, 2, 0, -2, -4, -5, -7, -9, -10, -10, -12, -12, -12},
    {7, 5, 4, 2, 0, -2, -4, -5, -7, -9, -10, -10, -12, -12, -12},
    {8, 6, 4, 2, 0, -2, -4, -5, -7, -9, -10, -10, -12, -12, -12},
    {8, 6, 4, 2, 0, -2, -4, -5, -7, -9, -10, -10, -12, -12, -12},
    {8, 6, 4, 2, 0, -2, -4, -5, -7, -9, -10, -10, -12, -12, -12},
    {8, 6, 5, 3, 1, -1, -3, -5, -6, -8, -10, -10, -12, -12, -12},
    {8, 6, 5, 3, 1, -1, -3, -5, -6, -8, -10, -10, -12, -12, -12},
    {8, 6, 5, 3, 1, -1, -3, -5, -6, -8, -10, -10, -12, -12, -12},
    {9, 7, 5, 3, 1, -1, -3, -5, -6, -8, -10, -10, -12, -12, -12},
    {9, 7, 5, 3, 1, -1, -3, -5, -6, -8, -10, -10, -12, -12, -12},
    {9, 7, 5, 3, 1, -1, -3, -5, -6, -8, -10, -10, -12, -12, -12},
    {9, 7, 5, 3, 1, -1, -3, -4, -6, -8, -10, -10, -12, -12, -12},
    {9, 7, 5, 3, 1, -1, -3, -4, -6, -8, -10, -10, -12, -12, -12},
    {9, 7, 6, 4, 2, 0, -2, -4, -6, -8, -10, -10, -12, -12, -12},
    {10, 8, 6, 4, 2, 0, -2, -4, -6, -8, -10, -10, -12, -12, -12},
    {10, 8, 6, 4, 2, 0, -2, -4, -6, -8, -10, -10, -12, -12, -12},
    {10, 8, 6, 4, 2, 0, -2, -4, -6, -8, -10, -10, -12, -12, -12},
    {10, 8, 6, 4, 2, 0, -2, -4, -6, -8, -10, -10, -12, -12, -12},
};
// oracle rc <bpp_x16> 1366 768 2 for bpp 6.0 .. 24.0 in steps of 1/16: initial_fullness_offset, initial_xmit_delay, first_line_bpg_offset
constexpr uint16_t rc_scalars_6_to_24[][3]{
    {6144, 683, 15},
    {6144, 676, 15},
    {6144, 669, 15},
    {6144, 662, 15},
    {6144, 655, 15},
    {6144, 649, 15},
    {6144, 643, 15},
    {6144, 636, 15},
    {6144, 630, 15},
    {6144, 624, 15},
    {6144, 618, 15},
    {6144, 612, 15},
    {6144, 607, 15},
    {6144, 601, 15},
    {6144, 596, 15},
    {6144, 590, 15},
    {6144, 585, 15},
    {6144, 580, 15},
    {6144, 575, 15},
    {6144, 570, 15},
    {6144, 565, 15},
    {6144, 560, 15},
    {6144, 555, 15},
    {6144, 551, 15},
    {6144, 546, 15},
    {6144, 542, 15},
    {6144, 537, 15},
    {6144, 533, 15},
    {6144, 529, 15},
    {6144, 524, 15},
    {6144, 520, 15},
    {6144, 516, 15},
    {6144, 512, 15},
    {6128, 508, 15},
    {6112, 504, 15},
    {6096, 500, 15},
    {6080, 496, 15},
    {6064, 493, 15},
    {6048, 489, 15},
    {6032, 485, 15},
    {6016, 482, 15},
    {6000, 478, 15},
    {5984, 475, 15},
    {5968, 471, 15},
    {5952, 468, 15},
    {5936, 465, 15},
    {5920, 462, 15},
    {5904, 458, 15},
    {5888, 455, 15},
    {5872, 452, 15},
    {5856, 449, 15},
    {5840, 446, 15},
    {5824, 443, 15},
    {5808, 440, 15},
    {5792, 437, 15},
    {5776, 434, 15},
    {5760, 431, 15},
    {5744, 428, 15},
    {5728, 426, 15},
    {5712, 423, 15},
    {5696, 420, 15},
    {5680, 417, 15},
    {5664, 415, 15},
    {5648, 412, 15},
    {5632, 410, 15},
    {5520, 407, 15},
    {5408, 405, 15},
    {5296, 402, 15},
    {5184, 400, 15},
    {5072, 397, 15},
    {4960, 395, 15},
    {4848, 392, 15},
    {4736, 390, 15},
    {4624, 388, 15},
    {4512, 386, 15},
    {4400, 383, 15},
    {4288, 381, 15},
    {4176, 379, 15},
    {4064, 377, 15},
    {3952, 374, 15},
    {3840, 372, 15},
    {3728, 370, 15},
    {3616, 368, 15},
    {3504, 366, 15},
    {3392, 364, 15},
    {3280, 362, 15},
    {3168, 360, 15},
    {3056, 358, 15},
    {2944, 356, 15},
    {2832, 354, 15},
    {2720, 352, 15},
    {2608, 350, 15},
    {2496, 349, 15},
    {2384, 347, 15},
    {2272, 345, 15},
    {2160, 343, 15},
    {2048, 341, 15},
    {2048, 340, 15},
    {2048, 338, 15},
    {2048, 336, 15},
    {2048, 334, 15},
    {2048, 333, 15},
    {2048, 331, 15},
    {2048, 329, 15},
    {2048, 328, 15},
    {2048, 326, 15},
    {2048, 324, 15},
    {2048, 323, 15},
    {2048, 321, 15},
    {2048, 320, 15},
    {2048, 318, 15},
    {2048, 317, 15},
    {2048, 315, 15},
    {2048, 314, 15},
    {2048, 312, 15},
    {2048, 311, 15},
    {2048, 309, 15},
    {2048, 308, 15},
    {2048, 306, 15},
    {2048, 305, 15},
    {2048, 303, 15},
    {2048, 302, 15},
    {2048, 301, 15},
    {2048, 299, 15},
    {2048, 298, 15},
    {2048, 297, 15},
    {2048, 295, 15},
    {2048, 294, 15},
    {2048, 293, 15},
    {2048, 291, 15},
    {2048, 290, 15},
    {2048, 289, 15},
    {2048, 287, 15},
    {2048, 286, 15},
    {2048, 285, 15},
    {2048, 284, 15},
    {2048, 282, 15},
    {2048, 281, 15},
    {2048, 280, 15},
    {2048, 279, 15},
    {2048, 278, 15},
    {2048, 277, 15},
    {2048, 275, 15},
    {2048, 274, 15},
    {2048, 273, 15},
    {2048, 272, 15},
    {2048, 271, 15},
    {2048, 270, 15},
    {2048, 269, 15},
    {2048, 267, 15},
    {2048, 266, 15},
    {2048, 265, 15},
    {2048, 264, 15},
    {2048, 263, 15},
    {2048, 262, 15},
    {2048, 261, 15},
    {2048, 260, 15},
    {2048, 259, 15},
    {2048, 258, 15},
    {2048, 257, 15},
    {2048, 256, 15},
    {2048, 255, 15},
    {2048, 254, 15},
    {2048, 253, 15},
    {2048, 252, 15},
    {2048, 251, 15},
    {2048, 250, 15},
    {2048, 249, 15},
    {2048, 248, 15},
    {2048, 247, 15},
    {2048, 246, 15},
    {2048, 245, 15},
    {2048, 245, 15},
    {2048, 244, 15},
    {2048, 243, 15},
    {2048, 242, 15},
    {2048, 241, 15},
    {2048, 240, 15},
    {2048, 239, 15},
    {2048, 238, 15},
    {2048, 237, 15},
    {2048, 237, 15},
    {2048, 236, 15},
    {2048, 235, 15},
    {2048, 234, 15},
    {2048, 233, 15},
    {2048, 232, 15},
    {2048, 232, 15},
    {2048, 231, 15},
    {2048, 230, 15},
    {2048, 229, 15},
    {2048, 228, 15},
    {2048, 228, 15},
    {2048, 227, 15},
    {2048, 226, 15},
    {2048, 225, 15},
    {2048, 224, 15},
    {2048, 224, 15},
    {2048, 223, 15},
    {2048, 222, 15},
    {2048, 221, 15},
    {2048, 221, 15},
    {2048, 220, 15},
    {2048, 219, 15},
    {2048, 218, 15},
    {2048, 218, 15},
    {2048, 217, 15},
    {2048, 216, 15},
    {2048, 216, 15},
    {2048, 215, 15},
    {2048, 214, 15},
    {2048, 213, 15},
    {2048, 213, 15},
    {2048, 212, 15},
    {2048, 211, 15},
    {2048, 211, 15},
    {2048, 210, 15},
    {2048, 209, 15},
    {2048, 209, 15},
    {2048, 208, 15},
    {2048, 207, 15},
    {2048, 207, 15},
    {2048, 206, 15},
    {2048, 205, 15},
    {2048, 205, 15},
    {2048, 204, 15},
    {2048, 204, 15},
    {2048, 203, 15},
    {2048, 202, 15},
    {2048, 202, 15},
    {2048, 201, 15},
    {2048, 200, 15},
    {2048, 200, 15},
    {2048, 199, 15},
    {2048, 199, 15},
    {2048, 198, 15},
    {2048, 197, 15},
    {2048, 197, 15},
    {2048, 196, 15},
    {2048, 196, 15},
    {2048, 195, 15},
    {2048, 194, 14},
    {2048, 194, 14},
    {2048, 193, 14},
    {2048, 193, 14},
    {2048, 192, 14},
    {2048, 192, 13},
    {2048, 191, 13},
    {2048, 191, 13},
    {2048, 190, 13},
    {2048, 189, 13},
    {2048, 189, 12},
    {2048, 188, 12},
    {2048, 188, 12},
    {2048, 187, 12},
    {2048, 187, 12},
    {2048, 186, 12},
    {2048, 186, 11},
    {2048, 185, 11},
    {2048, 185, 11},
    {2048, 184, 11},
    {2048, 184, 11},
    {2048, 183, 10},
    {2048, 183, 10},
    {2048, 182, 10},
    {2048, 182, 10},
    {2048, 181, 10},
    {2048, 181, 9},
    {2048, 180, 9},
    {2048, 180, 9},
    {2048, 179, 9},
    {2048, 179, 9},
    {2048, 178, 9},
    {2048, 178, 8},
    {2048, 177, 8},
    {2048, 177, 8},
    {2048, 176, 8},
    {2048, 176, 8},
    {2048, 175, 7},
    {2048, 175, 7},
    {2048, 174, 7},
    {2048, 174, 7},
    {2048, 173, 7},
    {2048, 173, 6},
    {2048, 172, 6},
    {2048, 172, 6},
    {2048, 172, 6},
    {2048, 171, 6},
    {2048, 171, 6},
};

// Every field the oracle prints for a dsc_prepare_config result, in its order.
std::vector<uint32_t> flatten(const DscRegisterValues& v, const DscOptcConfig& o) {
    const DscPps& p = v.pps;
    std::vector<uint32_t> out{p.line_buf_depth, p.bits_per_component, p.convert_rgb, p.slice_width, p.slice_height,
                              p.pic_width, p.pic_height, p.bits_per_pixel, p.rc_quant_incr_limit0,
                              p.rc_quant_incr_limit1, p.initial_xmit_delay, p.initial_dec_delay,
                              p.block_pred_enable, p.first_line_bpg_offset, p.initial_offset, p.rc_model_size,
                              p.flatness_min_qp, p.flatness_max_qp, p.initial_scale_value,
                              p.scale_decrement_interval, p.scale_increment_interval, p.nfl_bpg_offset,
                              p.slice_bpg_offset, p.final_offset, p.slice_chunk_size, p.dsc_version_minor,
                              p.dsc_version_major, p.second_line_bpg_offset, p.nsl_bpg_offset,
                              p.second_line_offset_adj, p.rc_edge_factor, p.rc_tgt_offset_high};
    for (uint16_t threshold : p.rc_buf_thresh) {
        out.push_back(threshold);
    }
    for (const DscRcRange& range : p.rc_range_params) {
        out.push_back(range.range_min_qp);
        out.push_back(range.range_max_qp);
        out.push_back(range.range_bpg_offset);
    }
    for (uint32_t value : {v.pixel_format, v.num_slices_h, v.num_slices_v, v.bpp_x32, v.ich_reset_at_eol,
                           v.alternate_ich_encoding_en, v.rc_buffer_model_size, o.bytes_per_pixel, o.slice_width,
                           o.is_pixel_format_444 ? 1u : 0u}) {
        out.push_back(value);
    }
    return out;
}

uint16_t be16(const uint8_t* bytes) { return static_cast<uint16_t>((bytes[0] << 8) | bytes[1]); }

// VESA DSC 1.2a picture parameter set (Table 4-1/4-2 byte layout), decoded
// field by field without the packer.
bool vesa_decode_matches(const uint8_t (&pps)[dsc_pps_bytes], const DscPps& p) {
    bool ok = (pps[0] >> 4) == p.dsc_version_major && (pps[0] & 0x0f) == p.dsc_version_minor && pps[1] == 0 &&
              pps[2] == 0 && (pps[3] >> 4) == p.bits_per_component && (pps[3] & 0x0f) == p.line_buf_depth &&
              ((pps[4] >> 5) & 1) == (p.block_pred_enable ? 1 : 0) && ((pps[4] >> 4) & 1) == (p.convert_rgb ? 1 : 0) &&
              ((pps[4] >> 3) & 1) == (p.simple_422 ? 1 : 0) && ((pps[4] >> 2) & 1) == (p.vbr_enable ? 1 : 0) &&
              (((pps[4] & 3) << 8) | pps[5]) == p.bits_per_pixel && be16(&pps[6]) == p.pic_height &&
              be16(&pps[8]) == p.pic_width && be16(&pps[10]) == p.slice_height && be16(&pps[12]) == p.slice_width &&
              be16(&pps[14]) == p.slice_chunk_size && (((pps[16] & 3) << 8) | pps[17]) == p.initial_xmit_delay &&
              be16(&pps[18]) == p.initial_dec_delay && pps[20] == 0 && pps[21] == p.initial_scale_value &&
              be16(&pps[22]) == p.scale_increment_interval &&
              (((pps[24] & 0x0f) << 8) | pps[25]) == p.scale_decrement_interval && pps[26] == 0 &&
              (pps[27] & 0x1f) == p.first_line_bpg_offset && be16(&pps[28]) == p.nfl_bpg_offset &&
              be16(&pps[30]) == p.slice_bpg_offset && be16(&pps[32]) == p.initial_offset &&
              be16(&pps[34]) == p.final_offset && (pps[36] & 0x1f) == p.flatness_min_qp &&
              (pps[37] & 0x1f) == p.flatness_max_qp && be16(&pps[38]) == p.rc_model_size &&
              (pps[40] & 0x0f) == p.rc_edge_factor && (pps[41] & 0x1f) == p.rc_quant_incr_limit0 &&
              (pps[42] & 0x1f) == p.rc_quant_incr_limit1 && (pps[43] >> 4) == p.rc_tgt_offset_high &&
              (pps[43] & 0x0f) == p.rc_tgt_offset_low && ((pps[88] >> 1) & 1) == (p.native_420 ? 1 : 0) &&
              (pps[88] & 1) == (p.native_422 ? 1 : 0) && (pps[89] & 0x1f) == p.second_line_bpg_offset &&
              be16(&pps[90]) == p.nsl_bpg_offset && be16(&pps[92]) == p.second_line_offset_adj;
    for (uint32_t i = 0; ok && i < dsc_buf_ranges - 1; ++i) {
        ok = pps[44 + i] == p.rc_buf_thresh[i];
    }
    for (uint32_t i = 0; ok && i < dsc_buf_ranges; ++i) {
        const uint16_t word = be16(&pps[58 + 2 * i]);
        ok = (word >> 11) == p.rc_range_params[i].range_min_qp &&
             ((word >> 6) & 0x1f) == p.rc_range_params[i].range_max_qp &&
             (word & 0x3f) == p.rc_range_params[i].range_bpg_offset;
    }
    for (uint32_t i = 94; ok && i < dsc_pps_bytes; ++i) {
        ok = pps[i] == 0;
    }
    return ok;
}

struct PrepCase final {
    const char* name;
    uint32_t engine;
    uint32_t pic_width;
    uint32_t pic_height;
    DscConfig config;
    bool is_odm;
    const uint32_t* values;
    size_t value_count;
    const uint8_t* pps;
    const uint32_t* trace;
    size_t trace_count;
};

void prepare_case(const PrepCase& c) {
    DscRegisterValues values{};
    DscOptcConfig optc{};
    expect(dsc_prepare_config(c.pic_width, c.pic_height, c.is_odm, c.config, values, optc), c.name);
    const std::vector<uint32_t> fields = flatten(values, optc);
    bool same = fields.size() == c.value_count;
    for (size_t i = 0; same && i < fields.size(); ++i) {
        if (fields[i] != c.values[i]) {
            std::fprintf(stderr, "%s: field %zu is %u, Linux %u\n", c.name, i, fields[i], c.values[i]);
            same = false;
        }
    }
    expect(same, "dsc_prepare_config register values equal Linux");
    uint8_t pps[dsc_pps_bytes]{};
    dsc_pack_pps(values.pps, pps);
    expect(std::memcmp(pps, c.pps, dsc_pps_bytes) == 0, "the 128-byte PPS equals Linux drm_dsc_pps_payload_pack");
    expect(vesa_decode_matches(pps, values.pps), "the PPS decodes to the configuration (VESA DSC 1.2a layout)");

    // dsc2_set_config, dsc2_enable(opp = engine), dsc2_disconnect, dsc2_disable.
    rx7900_test::stream::Model model;
    Dcn32StreamIo io;
    expect(io.bind(model.io(), rx7900_test::stream::segment[1], rx7900_test::stream::segment[2],
                   rx7900_test::stream::segment[3]), "stream IO binds");
    expect(dcn32_dsc::write_registers(io, c.engine, values) && dcn32_dsc::enable(io, c.engine, c.engine) &&
               dcn32_dsc::disconnect(io, c.engine) && dcn32_dsc::disable(io, c.engine),
           "DSC engine programming completes");
    std::vector<uint32_t> trace;
    for (const rx7900_test::stream::Op& op : model.trace) {
        trace.push_back((op.kind == rx7900_test::stream::Kind::write ? 0x100000u : 0u) | (op.offset / 4u));
        trace.push_back(op.value);
    }
    bool trace_same = trace.size() == c.trace_count;
    for (size_t i = 0; trace_same && i < trace.size(); ++i) {
        trace_same = trace[i] == c.trace[i];
        if (!trace_same) {
            std::fprintf(stderr, "%s: register access %zu differs (%08x, Linux %08x)\n", c.name, i / 2, trace[i],
                         c.trace[i]);
        }
    }
    expect(trace_same, "DSC engine register accesses equal Linux dcn20_dsc.c with the DCN 3.2 masks");
}

DscConfig config(uint32_t slices_h, uint32_t slices_v, uint32_t bpp, uint32_t linebuf, uint32_t minor, bool block) {
    DscConfig c{};
    c.num_slices_h = slices_h;
    c.num_slices_v = slices_v;
    c.bits_per_pixel = bpp;
    c.linebuf_depth = linebuf;
    c.version_minor = minor;
    c.block_pred_enable = block;
    c.is_dp = true;
    return c;
}

#define PREP(name, engine, w, h, cfg, odm)                                                                    \
    PrepCase { #name, engine, w, h, cfg, odm, name##_values, sizeof(name##_values) / sizeof(name##_values[0]), \
               name##_pps, name##_trace, sizeof(name##_trace) / sizeof(name##_trace[0]) }

void preparation() {
    prepare_case(PREP(prep_4k144_hbr3, 1, 3840, 2160, config(2, 20, 256, 13, 2, true), false));
    prepare_case(PREP(prep_4k144_hbr2, 0, 3840, 2160, config(2, 20, 205, 13, 2, true), false));
    prepare_case(PREP(prep_8k60_odm, 2, 3840, 4320, config(2, 40, 195, 13, 2, true), true));
    prepare_case(PREP(prep_1080_dsc11, 3, 1920, 1080, config(1, 10, 128, 8, 1, false), false));

    // The binary64 products (8/3.0, 4/3.0, 2/3.0) of the offsets for 12 < bpp < 15,
    // including the exact ties (bpp 12 + 3/16, 12 + 6/16, 12 + 12/16).
    for (uint32_t bpp = 193; bpp <= 240; ++bpp) {
        DscRegisterValues values{};
        DscOptcConfig optc{};
        bool same = dsc_prepare_config(1920, 108, false, config(1, 1, bpp, 13, 2, true), values, optc);
        for (uint32_t i = 0; same && i < dsc_buf_ranges; ++i) {
            same = values.pps.rc_range_params[i].range_bpg_offset ==
                   (static_cast<uint8_t>(rc_ofs_12_to_15[bpp - 193][i]) & 0x3fu);
        }
        if (!same) {
            std::fprintf(stderr, "bpp x16 %u: range offsets differ from Linux\n", bpp);
        }
        expect(same, "rate-control offsets equal Linux for 12 < bpp <= 15");
    }
    // initial_fullness_offset, initial_xmit_delay and first_line_bpg_offset for
    // every bpp from 6 to 24 in 1/16 steps.
    unsigned scalar_mismatches = 0;
    for (uint32_t bpp = 96; bpp <= 384; ++bpp) {
        DscRegisterValues values{};
        DscOptcConfig optc{};
        const bool prepared = dsc_prepare_config(1366, 768, false, config(1, 1, bpp, 13, 2, true), values, optc);
        const uint16_t* expected = rc_scalars_6_to_24[bpp - 96];
        if (!prepared || values.pps.initial_offset != expected[0] || values.pps.initial_xmit_delay != expected[1] ||
            values.pps.first_line_bpg_offset != expected[2]) {
            ++scalar_mismatches;
        }
    }
    expect(scalar_mismatches == 0, "rate-control scalars equal Linux for every bpp 6.0 .. 24.0");

    DscRegisterValues values{};
    DscOptcConfig optc{};
    values.num_slices_h = 77;
    expect(!dsc_prepare_config(3840, 2160, false, config(2, 7, 256, 13, 2, true), values, optc) &&
               values.num_slices_h == 77, "a picture height not divisible by the slice count is refused untouched");
    expect(!dsc_prepare_config(3840, 2160, false, config(2, 20, 95, 13, 2, true), values, optc),
           "below 6 bpp is refused");
    expect(!dsc_prepare_config(3840, 2160, false, config(2, 20, 256, 14, 1, true), values, optc),
           "a DSC 1.1 line buffer deeper than 13 bits is refused");
    expect(!dsc_prepare_config(3840, 2160, false, config(2, 20, 256, 13, 3, true), values, optc),
           "DSC 1.3 is refused");
    expect(!dsc_prepare_config(3840, 2160, false, config(2, 20, 400, 13, 2, true), values, optc),
           "a bpp beyond the 24 bpp qp table is refused");
    expect(!dsc_prepare_config(0, 2160, false, config(2, 20, 256, 13, 2, true), values, optc), "zero width");
}

// ---- policy (dc_dsc_parse_dsc_dpcd + dc_dsc_compute_config) ----

using Dpcd = uint8_t[dsc_dpcd_caps_bytes];
using Branch = uint8_t[dsc_branch_caps_bytes];

template <size_t N>
void policy_case(const char* name, const Dpcd& dpcd, const Branch& branch, const DscTiming& timing,
                 uint32_t target_kbps, const uint32_t (&expected)[N]) {
    DscSinkCaps caps{};
    const bool parsed = parse_dsc_dpcd(dpcd, branch, caps);
    DscConfig cfg{};
    const bool ok = dsc_compute_config(caps, target_kbps, timing, cfg);
    std::vector<uint32_t> out{parsed ? 1u : 0u, caps.is_dsc_supported ? 1u : 0u, caps.dsc_version,
                              static_cast<uint32_t>(caps.rc_buffer_size), caps.slice_caps1, caps.slice_caps2,
                              static_cast<uint32_t>(caps.lb_bit_depth), caps.is_block_pred_supported ? 1u : 0u,
                              static_cast<uint32_t>(caps.edp_max_bits_per_pixel), caps.color_formats,
                              caps.color_depth, static_cast<uint32_t>(caps.throughput_mode_0_mps),
                              static_cast<uint32_t>(caps.throughput_mode_1_mps),
                              static_cast<uint32_t>(caps.max_slice_width), caps.bpp_increment_div,
                              caps.branch_overall_throughput_0_mps, caps.branch_overall_throughput_1_mps,
                              caps.branch_max_line_width, caps.is_dp ? 1u : 0u, ok ? 1u : 0u, cfg.num_slices_h,
                              cfg.num_slices_v, cfg.bits_per_pixel, cfg.block_pred_enable ? 1u : 0u,
                              cfg.linebuf_depth, cfg.version_minor, cfg.is_dp ? 1u : 0u};
    if (ok) {
        out.push_back(dsc_stream_bandwidth_kbps(timing, cfg.bits_per_pixel, cfg.num_slices_h, cfg.is_dp));
    }
    bool same = out.size() == N;
    for (size_t i = 0; same && i < out.size(); ++i) {
        if (out[i] != expected[i]) {
            std::fprintf(stderr, "%s: value %zu is %u, Linux %u\n", name, i, out[i], expected[i]);
            same = false;
        }
    }
    expect(same, name);
}

void policy() {
    // A DP 1.4 DSC 1.2 sink: 1/2/4/8 slices, 13-bit line buffer, block
    // prediction, RGB/444, 8/10 bpc, 850 Mpixel/s per slice, 4160-pixel slices.
    const Dpcd sink{0x01, 0x21, 0x03, 0x00, 0x2b, 0x04, 0x01, 0x00, 0x00, 0x03, 0x06, 0xbb, 0x0d, 0x00, 0x00, 0x00};
    const Dpcd no_dsc{0x00, 0x21, 0x03, 0x00, 0x2b, 0x04, 0x01, 0x00, 0x00, 0x03, 0x06, 0xbb, 0x0d, 0x00, 0x00, 0x00};
    const Dpcd dsc11{0x01, 0x11, 0x03, 0x00, 0x03, 0x04, 0x01, 0x00, 0x00, 0x01, 0x02, 0x77, 0x08, 0x00, 0x00, 0x00};
    const Branch no_branch{0, 0, 0};
    const Branch branch{0x03, 0x02, 0x10};
    const DscTiming t4k144{12909504, 3840, 2160, 3920, 2287};
    const DscTiming t8k60{20397456, 7680, 4320, 7760, 4381};
    const DscTiming t4k60{5940000, 3840, 2160, 4400, 2250};
    policy_case("4K144 over HBR3 x4 + FEC: 16 bpp", sink, no_branch, t4k144, 25142400, policy_4k144_hbr3);
    policy_case("4K144 over HBR2 x4 + FEC: bpp from the bandwidth", sink, no_branch, t4k144, 16761600,
                policy_4k144_hbr2);
    policy_case("8K60 over HBR3 x4 + FEC: 4 slices", sink, no_branch, t8k60, 25142400, policy_8k60_hbr3);
    policy_case("4K60 fits HBR2 x4 uncompressed: no DSC", sink, no_branch, t4k60, 17280000, policy_4k60_fits);
    policy_case("a sink without DSC decompression", no_dsc, no_branch, t4k144, 25142400, policy_no_dsc);
    policy_case("a DSC 1.1 branch sink", dsc11, branch, t4k144, 16761600, policy_dsc11_branch);

    // DPCD fields DC rejects: an unknown line-buffer depth code, bpp increment code 5.
    const uint8_t bad_depth[dsc_dpcd_caps_bytes]{0x01, 0x21, 0x03, 0x00, 0x2b, 0x09, 0x01, 0x00,
                                                 0x00, 0x03, 0x06, 0xbb, 0x0d, 0x00, 0x00, 0x00};
    const uint8_t bad_increment[dsc_dpcd_caps_bytes]{0x01, 0x21, 0x03, 0x00, 0x2b, 0x04, 0x01, 0x00,
                                                     0x00, 0x03, 0x06, 0xbb, 0x0d, 0x00, 0x00, 0x05};
    DscSinkCaps caps{};
    expect(!parse_dsc_dpcd(bad_depth, nullptr, caps), "an undefined line buffer depth is refused");
    expect(!parse_dsc_dpcd(bad_increment, nullptr, caps), "an undefined bpp increment is refused");
    const uint8_t good[dsc_dpcd_caps_bytes]{0x01, 0x21, 0x03, 0x00, 0x2b, 0x04, 0x01, 0x00,
                                            0x00, 0x03, 0x06, 0xbb, 0x0d, 0x00, 0x00, 0x00};
    expect(parse_dsc_dpcd(good, nullptr, caps) && !caps.is_dp, "without branch bytes is_dp stays false (Linux)");

    // dp_link_bandwidth_kbps (link_validation.c) and the uncompressed rate.
    expect(dp_link_bandwidth_kbps(0x1e, 4, false) == 25920000 && dp_link_bandwidth_kbps(0x1e, 4, true) == 25142400 &&
               dp_link_bandwidth_kbps(0x14, 4, true) == 16761600 && dp_link_bandwidth_kbps(0x0a, 2, false) == 4320000 &&
               dp_link_bandwidth_kbps(0x06, 1, true) == 1257120,
           "8b/10b link bandwidth with and without the 97 percent FEC efficiency");
    expect(uncompressed_bandwidth_kbps(t4k144) == 30982800, "uncompressed RGB 8 bpc bandwidth");
}

} // namespace

int main() {
    preparation();
    policy();
    return checks.finish("dcn32_dsc");
}

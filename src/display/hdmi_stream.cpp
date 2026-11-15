/*
Module: rx7900.display.hdmi_stream
Purpose: CTA HDMI sink parsing, CEA/HDMI VIC matching, TMDS limits and AVI/VSIF/VSC packets.
Created: 2026-10-24
Architecture: Allocation-free; the only inputs are a checked EDID view and the
timing the mode controller will program. Adapted (logic and the CEA mode data,
transcribed by a scratch generator from the pinned drm_edid.c) from Linux v6.12
drivers/gpu/drm/drm_edid.c (MIT-style notice: Copyright (c) 2006 Luc Verhaegen,
2007-2008 Intel Corporation, 2010 Red Hat, Inc.) and AMD DC
core/dc_resource.c, modules/info_packet/info_packet.c (Copyright 2016-2018
Advanced Micro Devices, Inc., MIT notice). Notices: licenses/AMDGPU-OUTPUT.txt
(drm_edid.c) and licenses/AMDGPU-DISPLAY.txt (AMD DC).
*/
#include "hdmi_stream.hpp"

namespace rx7900::display {
namespace {

constexpr uint32_t cea_phsync = 1u << 0;    // DRM_MODE_FLAG_PHSYNC
constexpr uint32_t cea_nhsync = 1u << 1;    // DRM_MODE_FLAG_NHSYNC
constexpr uint32_t cea_pvsync = 1u << 2;    // DRM_MODE_FLAG_PVSYNC
constexpr uint32_t cea_nvsync = 1u << 3;    // DRM_MODE_FLAG_NVSYNC
constexpr uint32_t cea_interlace = 1u << 4; // DRM_MODE_FLAG_INTERLACE
constexpr uint32_t cea_dblclk = 1u << 12;   // DRM_MODE_FLAG_DBLCLK

struct CeaMode final {
    uint8_t vic;
    uint32_t clock_khz;
    uint16_t hdisplay;
    uint16_t hsync_start;
    uint16_t hsync_end;
    uint16_t htotal;
    uint16_t vdisplay;
    uint16_t vsync_start;
    uint16_t vsync_end;
    uint16_t vtotal;
    uint32_t flags;
    uint8_t aspect; // HDMI_PICTURE_ASPECT_*: 1 4:3, 2 16:9, 3 64:27, 4 256:135
};

// Generated from work/upstream-dcn-stream/drm_edid.c edid_cea_modes_1 (VIC
// 1..127), edid_cea_modes_193 (VIC 193..219) and edid_4k_modes (HDMI VIC 1..4).
constexpr CeaMode cea_modes_1[] = {
    {1, 25175, 640, 656, 752, 800, 480, 490, 492, 525, cea_nhsync | cea_nvsync, 1},
    {2, 27000, 720, 736, 798, 858, 480, 489, 495, 525, cea_nhsync | cea_nvsync, 1},
    {3, 27000, 720, 736, 798, 858, 480, 489, 495, 525, cea_nhsync | cea_nvsync, 2},
    {4, 74250, 1280, 1390, 1430, 1650, 720, 725, 730, 750, cea_phsync | cea_pvsync, 2},
    {5, 74250, 1920, 2008, 2052, 2200, 1080, 1084, 1094, 1125, cea_phsync | cea_pvsync | cea_interlace, 2},
    {6, 13500, 720, 739, 801, 858, 480, 488, 494, 525, cea_nhsync | cea_nvsync | cea_interlace | cea_dblclk, 1},
    {7, 13500, 720, 739, 801, 858, 480, 488, 494, 525, cea_nhsync | cea_nvsync | cea_interlace | cea_dblclk, 2},
    {8, 13500, 720, 739, 801, 858, 240, 244, 247, 262, cea_nhsync | cea_nvsync | cea_dblclk, 1},
    {9, 13500, 720, 739, 801, 858, 240, 244, 247, 262, cea_nhsync | cea_nvsync | cea_dblclk, 2},
    {10, 54000, 2880, 2956, 3204, 3432, 480, 488, 494, 525, cea_nhsync | cea_nvsync | cea_interlace, 1},
    {11, 54000, 2880, 2956, 3204, 3432, 480, 488, 494, 525, cea_nhsync | cea_nvsync | cea_interlace, 2},
    {12, 54000, 2880, 2956, 3204, 3432, 240, 244, 247, 262, cea_nhsync | cea_nvsync, 1},
    {13, 54000, 2880, 2956, 3204, 3432, 240, 244, 247, 262, cea_nhsync | cea_nvsync, 2},
    {14, 54000, 1440, 1472, 1596, 1716, 480, 489, 495, 525, cea_nhsync | cea_nvsync, 1},
    {15, 54000, 1440, 1472, 1596, 1716, 480, 489, 495, 525, cea_nhsync | cea_nvsync, 2},
    {16, 148500, 1920, 2008, 2052, 2200, 1080, 1084, 1089, 1125, cea_phsync | cea_pvsync, 2},
    {17, 27000, 720, 732, 796, 864, 576, 581, 586, 625, cea_nhsync | cea_nvsync, 1},
    {18, 27000, 720, 732, 796, 864, 576, 581, 586, 625, cea_nhsync | cea_nvsync, 2},
    {19, 74250, 1280, 1720, 1760, 1980, 720, 725, 730, 750, cea_phsync | cea_pvsync, 2},
    {20, 74250, 1920, 2448, 2492, 2640, 1080, 1084, 1094, 1125, cea_phsync | cea_pvsync | cea_interlace, 2},
    {21, 13500, 720, 732, 795, 864, 576, 580, 586, 625, cea_nhsync | cea_nvsync | cea_interlace | cea_dblclk, 1},
    {22, 13500, 720, 732, 795, 864, 576, 580, 586, 625, cea_nhsync | cea_nvsync | cea_interlace | cea_dblclk, 2},
    {23, 13500, 720, 732, 795, 864, 288, 290, 293, 312, cea_nhsync | cea_nvsync | cea_dblclk, 1},
    {24, 13500, 720, 732, 795, 864, 288, 290, 293, 312, cea_nhsync | cea_nvsync | cea_dblclk, 2},
    {25, 54000, 2880, 2928, 3180, 3456, 576, 580, 586, 625, cea_nhsync | cea_nvsync | cea_interlace, 1},
    {26, 54000, 2880, 2928, 3180, 3456, 576, 580, 586, 625, cea_nhsync | cea_nvsync | cea_interlace, 2},
    {27, 54000, 2880, 2928, 3180, 3456, 288, 290, 293, 312, cea_nhsync | cea_nvsync, 1},
    {28, 54000, 2880, 2928, 3180, 3456, 288, 290, 293, 312, cea_nhsync | cea_nvsync, 2},
    {29, 54000, 1440, 1464, 1592, 1728, 576, 581, 586, 625, cea_nhsync | cea_nvsync, 1},
    {30, 54000, 1440, 1464, 1592, 1728, 576, 581, 586, 625, cea_nhsync | cea_nvsync, 2},
    {31, 148500, 1920, 2448, 2492, 2640, 1080, 1084, 1089, 1125, cea_phsync | cea_pvsync, 2},
    {32, 74250, 1920, 2558, 2602, 2750, 1080, 1084, 1089, 1125, cea_phsync | cea_pvsync, 2},
    {33, 74250, 1920, 2448, 2492, 2640, 1080, 1084, 1089, 1125, cea_phsync | cea_pvsync, 2},
    {34, 74250, 1920, 2008, 2052, 2200, 1080, 1084, 1089, 1125, cea_phsync | cea_pvsync, 2},
    {35, 108000, 2880, 2944, 3192, 3432, 480, 489, 495, 525, cea_nhsync | cea_nvsync, 1},
    {36, 108000, 2880, 2944, 3192, 3432, 480, 489, 495, 525, cea_nhsync | cea_nvsync, 2},
    {37, 108000, 2880, 2928, 3184, 3456, 576, 581, 586, 625, cea_nhsync | cea_nvsync, 1},
    {38, 108000, 2880, 2928, 3184, 3456, 576, 581, 586, 625, cea_nhsync | cea_nvsync, 2},
    {39, 72000, 1920, 1952, 2120, 2304, 1080, 1126, 1136, 1250, cea_phsync | cea_nvsync | cea_interlace, 2},
    {40, 148500, 1920, 2448, 2492, 2640, 1080, 1084, 1094, 1125, cea_phsync | cea_pvsync | cea_interlace, 2},
    {41, 148500, 1280, 1720, 1760, 1980, 720, 725, 730, 750, cea_phsync | cea_pvsync, 2},
    {42, 54000, 720, 732, 796, 864, 576, 581, 586, 625, cea_nhsync | cea_nvsync, 1},
    {43, 54000, 720, 732, 796, 864, 576, 581, 586, 625, cea_nhsync | cea_nvsync, 2},
    {44, 27000, 720, 732, 795, 864, 576, 580, 586, 625, cea_nhsync | cea_nvsync | cea_interlace | cea_dblclk, 1},
    {45, 27000, 720, 732, 795, 864, 576, 580, 586, 625, cea_nhsync | cea_nvsync | cea_interlace | cea_dblclk, 2},
    {46, 148500, 1920, 2008, 2052, 2200, 1080, 1084, 1094, 1125, cea_phsync | cea_pvsync | cea_interlace, 2},
    {47, 148500, 1280, 1390, 1430, 1650, 720, 725, 730, 750, cea_phsync | cea_pvsync, 2},
    {48, 54000, 720, 736, 798, 858, 480, 489, 495, 525, cea_nhsync | cea_nvsync, 1},
    {49, 54000, 720, 736, 798, 858, 480, 489, 495, 525, cea_nhsync | cea_nvsync, 2},
    {50, 27000, 720, 739, 801, 858, 480, 488, 494, 525, cea_nhsync | cea_nvsync | cea_interlace | cea_dblclk, 1},
    {51, 27000, 720, 739, 801, 858, 480, 488, 494, 525, cea_nhsync | cea_nvsync | cea_interlace | cea_dblclk, 2},
    {52, 108000, 720, 732, 796, 864, 576, 581, 586, 625, cea_nhsync | cea_nvsync, 1},
    {53, 108000, 720, 732, 796, 864, 576, 581, 586, 625, cea_nhsync | cea_nvsync, 2},
    {54, 54000, 720, 732, 795, 864, 576, 580, 586, 625, cea_nhsync | cea_nvsync | cea_interlace | cea_dblclk, 1},
    {55, 54000, 720, 732, 795, 864, 576, 580, 586, 625, cea_nhsync | cea_nvsync | cea_interlace | cea_dblclk, 2},
    {56, 108000, 720, 736, 798, 858, 480, 489, 495, 525, cea_nhsync | cea_nvsync, 1},
    {57, 108000, 720, 736, 798, 858, 480, 489, 495, 525, cea_nhsync | cea_nvsync, 2},
    {58, 54000, 720, 739, 801, 858, 480, 488, 494, 525, cea_nhsync | cea_nvsync | cea_interlace | cea_dblclk, 1},
    {59, 54000, 720, 739, 801, 858, 480, 488, 494, 525, cea_nhsync | cea_nvsync | cea_interlace | cea_dblclk, 2},
    {60, 59400, 1280, 3040, 3080, 3300, 720, 725, 730, 750, cea_phsync | cea_pvsync, 2},
    {61, 74250, 1280, 3700, 3740, 3960, 720, 725, 730, 750, cea_phsync | cea_pvsync, 2},
    {62, 74250, 1280, 3040, 3080, 3300, 720, 725, 730, 750, cea_phsync | cea_pvsync, 2},
    {63, 297000, 1920, 2008, 2052, 2200, 1080, 1084, 1089, 1125, cea_phsync | cea_pvsync, 2},
    {64, 297000, 1920, 2448, 2492, 2640, 1080, 1084, 1089, 1125, cea_phsync | cea_pvsync, 2},
    {65, 59400, 1280, 3040, 3080, 3300, 720, 725, 730, 750, cea_phsync | cea_pvsync, 3},
    {66, 74250, 1280, 3700, 3740, 3960, 720, 725, 730, 750, cea_phsync | cea_pvsync, 3},
    {67, 74250, 1280, 3040, 3080, 3300, 720, 725, 730, 750, cea_phsync | cea_pvsync, 3},
    {68, 74250, 1280, 1720, 1760, 1980, 720, 725, 730, 750, cea_phsync | cea_pvsync, 3},
    {69, 74250, 1280, 1390, 1430, 1650, 720, 725, 730, 750, cea_phsync | cea_pvsync, 3},
    {70, 148500, 1280, 1720, 1760, 1980, 720, 725, 730, 750, cea_phsync | cea_pvsync, 3},
    {71, 148500, 1280, 1390, 1430, 1650, 720, 725, 730, 750, cea_phsync | cea_pvsync, 3},
    {72, 74250, 1920, 2558, 2602, 2750, 1080, 1084, 1089, 1125, cea_phsync | cea_pvsync, 3},
    {73, 74250, 1920, 2448, 2492, 2640, 1080, 1084, 1089, 1125, cea_phsync | cea_pvsync, 3},
    {74, 74250, 1920, 2008, 2052, 2200, 1080, 1084, 1089, 1125, cea_phsync | cea_pvsync, 3},
    {75, 148500, 1920, 2448, 2492, 2640, 1080, 1084, 1089, 1125, cea_phsync | cea_pvsync, 3},
    {76, 148500, 1920, 2008, 2052, 2200, 1080, 1084, 1089, 1125, cea_phsync | cea_pvsync, 3},
    {77, 297000, 1920, 2448, 2492, 2640, 1080, 1084, 1089, 1125, cea_phsync | cea_pvsync, 3},
    {78, 297000, 1920, 2008, 2052, 2200, 1080, 1084, 1089, 1125, cea_phsync | cea_pvsync, 3},
    {79, 59400, 1680, 3040, 3080, 3300, 720, 725, 730, 750, cea_phsync | cea_pvsync, 3},
    {80, 59400, 1680, 2908, 2948, 3168, 720, 725, 730, 750, cea_phsync | cea_pvsync, 3},
    {81, 59400, 1680, 2380, 2420, 2640, 720, 725, 730, 750, cea_phsync | cea_pvsync, 3},
    {82, 82500, 1680, 1940, 1980, 2200, 720, 725, 730, 750, cea_phsync | cea_pvsync, 3},
    {83, 99000, 1680, 1940, 1980, 2200, 720, 725, 730, 750, cea_phsync | cea_pvsync, 3},
    {84, 165000, 1680, 1740, 1780, 2000, 720, 725, 730, 825, cea_phsync | cea_pvsync, 3},
    {85, 198000, 1680, 1740, 1780, 2000, 720, 725, 730, 825, cea_phsync | cea_pvsync, 3},
    {86, 99000, 2560, 3558, 3602, 3750, 1080, 1084, 1089, 1100, cea_phsync | cea_pvsync, 3},
    {87, 90000, 2560, 3008, 3052, 3200, 1080, 1084, 1089, 1125, cea_phsync | cea_pvsync, 3},
    {88, 118800, 2560, 3328, 3372, 3520, 1080, 1084, 1089, 1125, cea_phsync | cea_pvsync, 3},
    {89, 185625, 2560, 3108, 3152, 3300, 1080, 1084, 1089, 1125, cea_phsync | cea_pvsync, 3},
    {90, 198000, 2560, 2808, 2852, 3000, 1080, 1084, 1089, 1100, cea_phsync | cea_pvsync, 3},
    {91, 371250, 2560, 2778, 2822, 2970, 1080, 1084, 1089, 1250, cea_phsync | cea_pvsync, 3},
    {92, 495000, 2560, 3108, 3152, 3300, 1080, 1084, 1089, 1250, cea_phsync | cea_pvsync, 3},
    {93, 297000, 3840, 5116, 5204, 5500, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 2},
    {94, 297000, 3840, 4896, 4984, 5280, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 2},
    {95, 297000, 3840, 4016, 4104, 4400, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 2},
    {96, 594000, 3840, 4896, 4984, 5280, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 2},
    {97, 594000, 3840, 4016, 4104, 4400, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 2},
    {98, 297000, 4096, 5116, 5204, 5500, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 4},
    {99, 297000, 4096, 5064, 5152, 5280, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 4},
    {100, 297000, 4096, 4184, 4272, 4400, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 4},
    {101, 594000, 4096, 5064, 5152, 5280, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 4},
    {102, 594000, 4096, 4184, 4272, 4400, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 4},
    {103, 297000, 3840, 5116, 5204, 5500, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 3},
    {104, 297000, 3840, 4896, 4984, 5280, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 3},
    {105, 297000, 3840, 4016, 4104, 4400, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 3},
    {106, 594000, 3840, 4896, 4984, 5280, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 3},
    {107, 594000, 3840, 4016, 4104, 4400, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 3},
    {108, 90000, 1280, 2240, 2280, 2500, 720, 725, 730, 750, cea_phsync | cea_pvsync, 2},
    {109, 90000, 1280, 2240, 2280, 2500, 720, 725, 730, 750, cea_phsync | cea_pvsync, 3},
    {110, 99000, 1680, 2490, 2530, 2750, 720, 725, 730, 750, cea_phsync | cea_pvsync, 3},
    {111, 148500, 1920, 2558, 2602, 2750, 1080, 1084, 1089, 1125, cea_phsync | cea_pvsync, 2},
    {112, 148500, 1920, 2558, 2602, 2750, 1080, 1084, 1089, 1125, cea_phsync | cea_pvsync, 3},
    {113, 198000, 2560, 3558, 3602, 3750, 1080, 1084, 1089, 1100, cea_phsync | cea_pvsync, 3},
    {114, 594000, 3840, 5116, 5204, 5500, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 2},
    {115, 594000, 4096, 5116, 5204, 5500, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 4},
    {116, 594000, 3840, 5116, 5204, 5500, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 3},
    {117, 1188000, 3840, 4896, 4984, 5280, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 2},
    {118, 1188000, 3840, 4016, 4104, 4400, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 2},
    {119, 1188000, 3840, 4896, 4984, 5280, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 3},
    {120, 1188000, 3840, 4016, 4104, 4400, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 3},
    {121, 396000, 5120, 7116, 7204, 7500, 2160, 2168, 2178, 2200, cea_phsync | cea_pvsync, 3},
    {122, 396000, 5120, 6816, 6904, 7200, 2160, 2168, 2178, 2200, cea_phsync | cea_pvsync, 3},
    {123, 396000, 5120, 5784, 5872, 6000, 2160, 2168, 2178, 2200, cea_phsync | cea_pvsync, 3},
    {124, 742500, 5120, 5866, 5954, 6250, 2160, 2168, 2178, 2475, cea_phsync | cea_pvsync, 3},
    {125, 742500, 5120, 6216, 6304, 6600, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 3},
    {126, 742500, 5120, 5284, 5372, 5500, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 3},
    {127, 1485000, 5120, 6216, 6304, 6600, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 3},
};
constexpr CeaMode cea_modes_193[] = {
    {193, 1485000, 5120, 5284, 5372, 5500, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 3},
    {194, 1188000, 7680, 10232, 10408, 11000, 4320, 4336, 4356, 4500, cea_phsync | cea_pvsync, 2},
    {195, 1188000, 7680, 10032, 10208, 10800, 4320, 4336, 4356, 4400, cea_phsync | cea_pvsync, 2},
    {196, 1188000, 7680, 8232, 8408, 9000, 4320, 4336, 4356, 4400, cea_phsync | cea_pvsync, 2},
    {197, 2376000, 7680, 10232, 10408, 11000, 4320, 4336, 4356, 4500, cea_phsync | cea_pvsync, 2},
    {198, 2376000, 7680, 10032, 10208, 10800, 4320, 4336, 4356, 4400, cea_phsync | cea_pvsync, 2},
    {199, 2376000, 7680, 8232, 8408, 9000, 4320, 4336, 4356, 4400, cea_phsync | cea_pvsync, 2},
    {200, 4752000, 7680, 9792, 9968, 10560, 4320, 4336, 4356, 4500, cea_phsync | cea_pvsync, 2},
    {201, 4752000, 7680, 8032, 8208, 8800, 4320, 4336, 4356, 4500, cea_phsync | cea_pvsync, 2},
    {202, 1188000, 7680, 10232, 10408, 11000, 4320, 4336, 4356, 4500, cea_phsync | cea_pvsync, 3},
    {203, 1188000, 7680, 10032, 10208, 10800, 4320, 4336, 4356, 4400, cea_phsync | cea_pvsync, 3},
    {204, 1188000, 7680, 8232, 8408, 9000, 4320, 4336, 4356, 4400, cea_phsync | cea_pvsync, 3},
    {205, 2376000, 7680, 10232, 10408, 11000, 4320, 4336, 4356, 4500, cea_phsync | cea_pvsync, 3},
    {206, 2376000, 7680, 10032, 10208, 10800, 4320, 4336, 4356, 4400, cea_phsync | cea_pvsync, 3},
    {207, 2376000, 7680, 8232, 8408, 9000, 4320, 4336, 4356, 4400, cea_phsync | cea_pvsync, 3},
    {208, 4752000, 7680, 9792, 9968, 10560, 4320, 4336, 4356, 4500, cea_phsync | cea_pvsync, 3},
    {209, 4752000, 7680, 8032, 8208, 8800, 4320, 4336, 4356, 4500, cea_phsync | cea_pvsync, 3},
    {210, 1485000, 10240, 11732, 11908, 12500, 4320, 4336, 4356, 4950, cea_phsync | cea_pvsync, 3},
    {211, 1485000, 10240, 12732, 12908, 13500, 4320, 4336, 4356, 4400, cea_phsync | cea_pvsync, 3},
    {212, 1485000, 10240, 10528, 10704, 11000, 4320, 4336, 4356, 4500, cea_phsync | cea_pvsync, 3},
    {213, 2970000, 10240, 11732, 11908, 12500, 4320, 4336, 4356, 4950, cea_phsync | cea_pvsync, 3},
    {214, 2970000, 10240, 12732, 12908, 13500, 4320, 4336, 4356, 4400, cea_phsync | cea_pvsync, 3},
    {215, 2970000, 10240, 10528, 10704, 11000, 4320, 4336, 4356, 4500, cea_phsync | cea_pvsync, 3},
    {216, 5940000, 10240, 12432, 12608, 13200, 4320, 4336, 4356, 4500, cea_phsync | cea_pvsync, 3},
    {217, 5940000, 10240, 10528, 10704, 11000, 4320, 4336, 4356, 4500, cea_phsync | cea_pvsync, 3},
    {218, 1188000, 4096, 4896, 4984, 5280, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 4},
    {219, 1188000, 4096, 4184, 4272, 4400, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 4},
};
constexpr CeaMode hdmi_4k_modes[] = {
    {1, 297000, 3840, 4016, 4104, 4400, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 2},
    {2, 297000, 3840, 4896, 4984, 5280, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 2},
    {3, 297000, 3840, 5116, 5204, 5500, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 2},
    {4, 297000, 4096, 5116, 5204, 5500, 2160, 2168, 2178, 2250, cea_phsync | cea_pvsync, 4},
};

constexpr uint32_t hdmi_ieee_oui = 0x000c03;
constexpr uint32_t hdmi_forum_ieee_oui = 0xc45dd8;
constexpr uint8_t cta_extension_tag = 0x02;  // CEA_EXT
constexpr uint8_t cta_db_vendor = 3;
constexpr uint8_t cta_db_extended_tag = 7;
constexpr uint8_t cta_ext_db_hf_scdb = 0x79;
constexpr uint8_t cta_ext_db_420_video_data = 14;
constexpr uint8_t cta_ext_db_420_video_cap_map = 15;
constexpr uint8_t cta_db_video = 2;
constexpr uint32_t edid_block = 128;

// dcn32 link_enc_feature.max_hdmi_pixel_clock and dce TMDS_MAX_PIXEL_CLOCK.
constexpr uint32_t dcn32_max_hdmi_pixel_clock_khz = 600000;
constexpr uint32_t tmds_max_single_link_khz = 165000;
constexpr uint32_t hdmi_clock_channel_rate_more_340m_khz = 340000;

// HDMI_INFOFRAME_TYPE_*, HDMI_AVI_INFOFRAME_SIZE (include/linux/hdmi.h).
constexpr uint8_t infoframe_type_vendor = 0x81;
constexpr uint8_t infoframe_type_avi = 0x82;
constexpr uint8_t avi_infoframe_size = 13;

uint32_t div_round_closest(uint64_t numerator, uint64_t denominator) noexcept {
    return static_cast<uint32_t>((numerator + denominator / 2) / denominator);
}

uint32_t mode_vrefresh(const CeaMode& mode) noexcept {
    const uint32_t num = (mode.flags & cea_interlace) != 0 ? 2u : 1u;
    const uint64_t den = static_cast<uint64_t>(mode.htotal) * mode.vtotal;
    return den == 0 ? 0 : div_round_closest(static_cast<uint64_t>(mode.clock_khz) * num * 1000u, den);
}

// cea_mode_alternate_clock (also hdmi_mode_alternate_clock).
uint32_t alternate_clock(const CeaMode& mode) noexcept {
    if (mode_vrefresh(mode) % 6 != 0) {
        return mode.clock_khz;
    }

    if (mode.vdisplay == 240 || mode.vdisplay == 480) {
        return div_round_closest(static_cast<uint64_t>(mode.clock_khz) * 1001u, 1000u);
    }

    return div_round_closest(static_cast<uint64_t>(mode.clock_khz) * 1000u, 1001u);
}

// KHZ2PICOS(a) = 1000000000UL / a.
uint32_t khz_to_picos(uint32_t khz) noexcept {
    return khz == 0 ? 0 : 1000000000u / khz;
}

struct DrmMode final {
    uint32_t clock_khz;
    uint32_t hdisplay;
    uint32_t hsync_start;
    uint32_t hsync_end;
    uint32_t htotal;
    uint32_t vdisplay;
    uint32_t vsync_start;
    uint32_t vsync_end;
    uint32_t vtotal;
    uint32_t flags;
};

DrmMode drm_mode_of(const Timing& timing) noexcept {
    DrmMode mode{};
    mode.clock_khz = timing.pixel_clock_khz;
    mode.hdisplay = timing.width;
    mode.hsync_start = timing.width + timing.h_front_porch;
    mode.hsync_end = mode.hsync_start + timing.h_sync_width;
    mode.htotal = timing.h_total;
    mode.vdisplay = timing.height;
    mode.vsync_start = timing.height + timing.v_front_porch;
    mode.vsync_end = mode.vsync_start + timing.v_sync_width;
    mode.vtotal = timing.v_total;
    mode.flags = ((timing.flags & positive_hsync) != 0 ? cea_phsync : cea_nhsync) |
                 ((timing.flags & positive_vsync) != 0 ? cea_pvsync : cea_nvsync);
    return mode;
}

// drm_mode_match(DRM_MODE_MATCH_TIMINGS | DRM_MODE_MATCH_FLAGS); hskew and
// vscan are zero on both sides.
bool same_timings(const DrmMode& mode, const CeaMode& cea, uint32_t vextra) noexcept {
    return mode.hdisplay == cea.hdisplay && mode.hsync_start == cea.hsync_start &&
           mode.hsync_end == cea.hsync_end && mode.htotal == cea.htotal &&
           mode.vdisplay == cea.vdisplay && mode.vsync_start == cea.vsync_start + vextra &&
           mode.vsync_end == cea.vsync_end + vextra && mode.vtotal == cea.vtotal + vextra &&
           mode.flags == cea.flags;
}

// cea_mode_alternate_timings: VICs 8, 9, 12, 13 (vtotal 262) and 23, 24, 27,
// 28 (vtotal 312) also match with the vertical front porch one line longer.
bool has_alternate_timings(uint8_t vic) noexcept {
    return vic == 8 || vic == 9 || vic == 12 || vic == 13 || vic == 23 || vic == 24 || vic == 27 ||
           vic == 28;
}

bool cea_matches(const DrmMode& mode, const CeaMode& cea) noexcept {
    if (same_timings(mode, cea, 0)) {
        return true;
    }

    return has_alternate_timings(cea.vic) && same_timings(mode, cea, 1);
}

int32_t clock_distance(uint32_t a, uint32_t b) noexcept {
    return a > b ? static_cast<int32_t>(a - b) : static_cast<int32_t>(b - a);
}

template<size_t Count>
uint8_t match_table(const CeaMode (&table)[Count], const DrmMode& mode, bool exact,
                    uint32_t tolerance_khz) noexcept {
    for (const CeaMode& cea : table) {
        const uint32_t clock1 = cea.clock_khz;
        const uint32_t clock2 = alternate_clock(cea);

        if (exact) {
            const uint32_t picos = khz_to_picos(mode.clock_khz);

            if (picos != khz_to_picos(clock1) && picos != khz_to_picos(clock2)) {
                continue;
            }
        } else if (clock_distance(mode.clock_khz, clock1) > static_cast<int32_t>(tolerance_khz) &&
                   clock_distance(mode.clock_khz, clock2) > static_cast<int32_t>(tolerance_khz)) {
            continue;
        }

        if (cea_matches(mode, cea)) {
            return cea.vic;
        }
    }

    return 0;
}

uint8_t match_cea(const DrmMode& mode, bool exact, uint32_t tolerance_khz) noexcept {
    if (mode.clock_khz == 0) {
        return 0;
    }

    const uint8_t vic = match_table(cea_modes_1, mode, exact, tolerance_khz);
    return vic != 0 ? vic : match_table(cea_modes_193, mode, exact, tolerance_khz);
}

const CeaMode* cea_mode_for_vic(uint8_t vic) noexcept {
    if (vic >= 1 && vic <= 127) {
        return &cea_modes_1[vic - 1];
    }

    if (vic >= 193 && vic <= 219) {
        return &cea_modes_193[vic - 193];
    }

    return nullptr;
}

uint8_t checksum_complement(uint32_t sum) noexcept {
    return static_cast<uint8_t>(0x100u - (sum & 0xffu));
}

// One CTA data block: tag/length at offset; payload bytes follow.
struct DataBlock final {
    ByteView bytes{nullptr, 0};
    uint8_t tag = 0;
    uint8_t length = 0;
};

uint8_t at(const ByteView& view, size_t offset) noexcept {
    uint8_t value = 0;
    view.read_u8(offset, value);
    return value;
}

uint32_t vendor_oui(const DataBlock& db) noexcept {
    return (static_cast<uint32_t>(at(db.bytes, 3)) << 16) | (static_cast<uint32_t>(at(db.bytes, 2)) << 8) |
           at(db.bytes, 1);
}

// drm_parse_hdmi_vsdb_video (db[] counts the tag byte as db[0]).
void parse_hdmi_vsdb(const DataBlock& db, HdmiSinkInfo& info) noexcept {
    info.is_hdmi = true;

    if (db.length >= 6) {
        info.dvi_dual = (at(db.bytes, 6) & 1u) != 0;
    }

    if (db.length >= 7) {
        info.max_tmds_clock_khz = static_cast<uint32_t>(at(db.bytes, 7)) * 5000u;
    }

    if (db.length >= 8 && (at(db.bytes, 8) & 0x20u) != 0) {
        info.has_hdmi_infoframe = true;
    }
}

// drm_parse_hdmi_forum_scds (hf_scds[] counts the tag byte as hf_scds[0]).
void parse_hdmi_forum_scds(const DataBlock& db, HdmiSinkInfo& info) noexcept {
    info.has_hdmi_infoframe = true;
    const uint8_t flags = at(db.bytes, 6);

    if ((flags & 0x80u) != 0) {
        info.scdc_supported = true;
    }

    const uint8_t rate = at(db.bytes, 5);

    if (rate != 0) {
        const uint32_t max_tmds_clock = static_cast<uint32_t>(rate) * 5000u;

        if (max_tmds_clock > 340000u) {
            info.max_tmds_clock_khz = max_tmds_clock;
        }

        if (info.scdc_supported) {
            info.scrambling_supported = true;

            if ((flags & 0x08u) != 0) {
                info.scrambling_low_rates = true;
            }
        }
    }
}

// svd_to_vic: bit 7 is the native-mode flag for SVDs 129..192.
uint8_t svd_to_vic(uint8_t svd) noexcept {
    if ((svd >= 1 && svd <= 64) || (svd >= 129 && svd <= 192)) {
        return static_cast<uint8_t>(svd & 127u);
    }

    return svd;
}

bool valid_cea_vic(uint8_t vic) noexcept {
    return (vic >= 1 && vic <= 127) || (vic >= 193 && vic <= 219);
}

void mark_vic(HdmiSinkInfo& info, uint8_t vic) noexcept {
    info.vdb_vics[vic / 32u] |= 1u << (vic % 32u);
}

bool listed_vic(const HdmiSinkInfo& info, uint8_t vic) noexcept {
    return vic != 0 && (info.vdb_vics[vic / 32u] & (1u << (vic % 32u))) != 0;
}

// The drm_parse_cea_ext else-if chain for the blocks this module consumes.
void parse_data_block(const DataBlock& db, HdmiSinkInfo& info) noexcept {
    const bool extended = db.tag == cta_db_extended_tag && db.length >= 1;
    const uint8_t extended_tag = extended ? at(db.bytes, 1) : 0;

    if (db.tag == cta_db_vendor && db.length >= 5 && vendor_oui(db) == hdmi_ieee_oui) {
        parse_hdmi_vsdb(db, info);
    } else if ((db.tag == cta_db_vendor && db.length >= 7 && vendor_oui(db) == hdmi_forum_ieee_oui) ||
               (extended && extended_tag == cta_ext_db_hf_scdb && db.length >= 7)) {
        parse_hdmi_forum_scds(db, info);
    } else if (extended && extended_tag == cta_ext_db_420_video_cap_map) {
        // parse_cta_y420cmdb: an empty map means every mode; otherwise any set bit.
        bool any = db.length == 1;

        for (uint32_t index = 0; index + 1 < db.length && index < 8; ++index) {
            any = any || at(db.bytes, 2u + index) != 0;
        }

        info.ycbcr420 = info.ycbcr420 || any;
    } else if (extended && extended_tag == cta_ext_db_420_video_data) {
        for (uint32_t index = 0; index + 1 < db.length; ++index) {
            if (valid_cea_vic(svd_to_vic(at(db.bytes, 2u + index)))) {
                info.ycbcr420 = true;
            }
        }
    } else if (db.tag == cta_db_video) {
        for (uint32_t index = 0; index < db.length; ++index) {
            const uint8_t vic = svd_to_vic(at(db.bytes, 1u + index));

            if (valid_cea_vic(vic)) {
                mark_vic(info, vic);
            }
        }
    }
}

} // namespace

bool hdmi2_sink(const HdmiSinkInfo& sink) noexcept {
    return sink.scdc_supported || sink.ycbcr420;
}

namespace {

// The drm_parse_cea_ext walk shared by every CTA consumer: on_extension sees
// each CTA extension block, on_block each data block of a revision 3+
// extension in order. The caller has checked the whole-block blob size.
template <typename Extension, typename Block>
void walk_cta_extensions(ByteView edid, Extension on_extension, Block on_block) noexcept {
    const size_t blocks = edid.size() / edid_block;

    for (size_t block = 1; block < blocks; ++block) {
        ByteView ext{nullptr, 0};

        if (!edid.subview(block * edid_block, edid_block, ext) || at(ext, 0) != cta_extension_tag) {
            continue;
        }

        on_extension(ext);

        // __cea_db_iter_edid_next: revision 3+ and a data block collection.
        const uint8_t dtd_start = at(ext, 2);

        if (at(ext, 1) < 3 || dtd_start < 4 || dtd_start > 127) {
            continue;
        }

        size_t index = 4;
        const size_t end = dtd_start;

        while (index + 1 <= end) {
            DataBlock db{};
            const uint8_t tag_length = at(ext, index);
            db.tag = static_cast<uint8_t>(tag_length >> 5);
            db.length = static_cast<uint8_t>(tag_length & 0x1fu);

            if (index + 1 + db.length > end) {
                break; // __cea_db_iter_current_block refuses an overlong block.
            }

            ext.subview(index, 1u + db.length, db.bytes);

            on_block(db);

            index += 1u + db.length;
        }
    }
}

// HDMI 1.4b section 8.3.2 HDMI VSDB layout after Max_TMDS_Clock (byte 7):
// byte 8 flags (bit 7 latency fields, bit 6 interlaced latency fields, bit 5
// HDMI_Video_present), the optional 2 + 2 latency bytes, then with
// HDMI_Video_present one 3D/image-size byte and one byte holding
// HDMI_VIC_LEN (bits 7..5) and HDMI_3D_LEN, followed by HDMI_VIC_LEN VICs.
// Indices count the tag byte as 0; a list running past the block is cut.
void collect_hdmi_vics(const DataBlock& db, CtaVideoList& list) noexcept {
    if (db.length < 8) {
        return;
    }

    const uint8_t flags = at(db.bytes, 8);
    uint32_t index = 9;

    if ((flags & 0x80u) != 0) {
        index += 2;
    }

    if ((flags & 0x40u) != 0) {
        index += 2;
    }

    if ((flags & 0x20u) == 0 || index + 1 > db.length) {
        return;
    }

    const uint32_t count = at(db.bytes, index + 1u) >> 5;
    index += 2;

    for (uint32_t item = 0; item < count; ++item) {
        if (index + item > db.length) {
            ++list.dropped;
            continue;
        }

        const uint8_t vic = at(db.bytes, index + item);

        if (vic < 1 || vic > 4 || list.hdmi_vic_count >= cta_hdmi_vic_capacity) {
            ++list.dropped;
            continue;
        }

        list.hdmi_vics[list.hdmi_vic_count++] = vic;
    }
}

bool timing_of(const CeaMode& mode, Timing& output) noexcept {
    if ((mode.flags & (cea_interlace | cea_dblclk)) != 0) {
        return false; // interlaced or pixel-repeated: not an RGB progressive scanout
    }

    Timing timing{};
    timing.pixel_clock_khz = mode.clock_khz;
    timing.width = mode.hdisplay;
    timing.h_total = mode.htotal;
    timing.h_front_porch = static_cast<uint32_t>(mode.hsync_start - mode.hdisplay);
    timing.h_sync_width = static_cast<uint32_t>(mode.hsync_end - mode.hsync_start);
    timing.height = mode.vdisplay;
    timing.v_total = mode.vtotal;
    timing.v_front_porch = static_cast<uint32_t>(mode.vsync_start - mode.vdisplay);
    timing.v_sync_width = static_cast<uint32_t>(mode.vsync_end - mode.vsync_start);
    timing.flags = ((mode.flags & cea_phsync) != 0 ? positive_hsync : 0u) |
                   ((mode.flags & cea_pvsync) != 0 ? positive_vsync : 0u);
    output = timing;
    return true;
}

} // namespace

HdmiStatus parse_hdmi_sink(ByteView edid, HdmiSinkInfo& output) noexcept {
    if (edid.size() < edid_block || edid.size() % edid_block != 0) {
        return HdmiStatus::invalid_argument;
    }

    HdmiSinkInfo info{};
    walk_cta_extensions(
        edid,
        [&info](const ByteView& ext) noexcept {
            if (info.cea_revision == 0) {
                info.cea_revision = at(ext, 1);
            }
        },
        [&info](const DataBlock& db) noexcept { parse_data_block(db, info); });
    output = info;
    return HdmiStatus::success;
}

HdmiStatus parse_cta_video(ByteView edid, CtaVideoList& output) noexcept {
    if (edid.size() < edid_block || edid.size() % edid_block != 0) {
        return HdmiStatus::invalid_argument;
    }

    CtaVideoList list{};
    walk_cta_extensions(
        edid, [](const ByteView&) noexcept {},
        [&list](const DataBlock& db) noexcept {
            if (db.tag == cta_db_video) {
                for (uint32_t index = 0; index < db.length; ++index) {
                    const uint8_t vic = svd_to_vic(at(db.bytes, 1u + index));

                    if (!valid_cea_vic(vic) || list.vic_count >= cta_video_capacity) {
                        ++list.dropped;
                        continue;
                    }

                    list.vics[list.vic_count++] = vic;
                }
            } else if (db.tag == cta_db_vendor && db.length >= 5 && vendor_oui(db) == hdmi_ieee_oui) {
                collect_hdmi_vics(db, list);
            }
        });
    output = list;
    return HdmiStatus::success;
}

bool cea_vic_timing(uint8_t vic, Timing& output) noexcept {
    const CeaMode* mode = cea_mode_for_vic(vic);
    return mode != nullptr && timing_of(*mode, output);
}

bool hdmi_vic_timing(uint8_t hdmi_vic, Timing& output) noexcept {
    return hdmi_vic >= 1 && hdmi_vic <= 4 && timing_of(hdmi_4k_modes[hdmi_vic - 1u], output);
}

uint32_t drm_vrefresh_hz(const Timing& timing) noexcept {
    const uint64_t den = static_cast<uint64_t>(timing.h_total) * timing.v_total;
    return den == 0 ? 0 : div_round_closest(static_cast<uint64_t>(timing.pixel_clock_khz) * 1000u, den);
}

uint8_t match_cea_vic(const Timing& timing) noexcept {
    return match_cea(drm_mode_of(timing), true, 0);
}

uint8_t match_hdmi_vic(const Timing& timing) noexcept {
    const DrmMode mode = drm_mode_of(timing);

    if (mode.clock_khz == 0) {
        return 0;
    }

    for (const CeaMode& hdmi : hdmi_4k_modes) {
        const uint32_t picos = khz_to_picos(mode.clock_khz);

        if ((picos == khz_to_picos(hdmi.clock_khz) || picos == khz_to_picos(alternate_clock(hdmi))) &&
            same_timings(mode, hdmi, 0)) {
            return hdmi.vic;
        }
    }

    return 0;
}

uint32_t fixup_detailed_cea_clock(const Timing& timing) noexcept {
    const DrmMode mode = drm_mode_of(timing);
    const CeaMode* match = nullptr;
    const uint8_t vic = match_cea(mode, false, 5);

    if (vic != 0) {
        match = cea_mode_for_vic(vic);
    } else if (mode.clock_khz != 0) {
        for (const CeaMode& hdmi : hdmi_4k_modes) {
            if ((clock_distance(mode.clock_khz, hdmi.clock_khz) <= 5 ||
                 clock_distance(mode.clock_khz, alternate_clock(hdmi)) <= 5) &&
                same_timings(mode, hdmi, 0)) {
                match = &hdmi;
                break;
            }
        }
    }

    if (match == nullptr) {
        return timing.pixel_clock_khz;
    }

    const uint32_t clock1 = match->clock_khz;
    const uint32_t clock2 = alternate_clock(*match);
    return clock_distance(mode.clock_khz, clock1) < clock_distance(mode.clock_khz, clock2) ? clock1 : clock2;
}

void build_avi_infoframe(const Timing& timing, uint8_t vic, uint8_t hdmi_vic, InfoPacket& output) noexcept {
    InfoPacket packet{};
    const uint8_t version = (hdmi_vic == 0 && vic >= 128) ? 3 : 2;
    const uint8_t code = hdmi_vic != 0 ? 0 : vic;
    // union hdmi_info_packet bit layout (little-endian bitfields, LSB first).
    // sb[1]: S0_S1 = SCANNING_TYPE_UNDERSCAN (2), B0_B1 = BAR_INFO_BOTH_VALID
    // (3), A0 = ACTIVE_FORMAT_VALID (1), Y = 0 (RGB).
    packet.sb[1] = static_cast<uint8_t>(2u | (3u << 2) | (1u << 4));
    // sb[2]: R0_R3 = ACTIVE_FORMAT_ASPECT_RATIO_SAME_AS_PICTURE (8), M0_M1 =
    // ASPECT_RATIO_NO_DATA (0), C0_C1 = COLORIMETRY_NO_DATA (0) for SRGB.
    packet.sb[2] = 8;
    // sb[3]: SC0_SC1 = PICTURE_SCALING_UNIFORM (0), Q0_Q1 = DEFAULT (0), EC = 0,
    // ITC = 1 (DISPLAY_CONTENT_TYPE_NO_DATA).
    packet.sb[3] = 0x80;
    packet.sb[4] = code;
    // sb[5]: PR = 0, CN = 0, YQ = YYC_QUANTIZATION_LIMITED_RANGE (0).
    packet.sb[5] = 0;
    // bar_top = v_border_top (0); bar_bottom = v_total - border + 1; bar_left = 0;
    // bar_right = h_total - border + 1 (uint16 little endian, truncated as C does).
    const uint16_t bar_bottom = static_cast<uint16_t>(timing.v_total + 1u);
    const uint16_t bar_right = static_cast<uint16_t>(timing.h_total + 1u);
    packet.sb[8] = static_cast<uint8_t>(bar_bottom);
    packet.sb[9] = static_cast<uint8_t>(bar_bottom >> 8);
    packet.sb[12] = static_cast<uint8_t>(bar_right);
    packet.sb[13] = static_cast<uint8_t>(bar_right >> 8);
    uint32_t sum = infoframe_type_avi + avi_infoframe_size + version;

    for (uint32_t index = 1; index <= avi_infoframe_size; ++index) {
        sum += packet.sb[index];
    }

    packet.sb[0] = checksum_complement(sum);
    packet.hb0 = infoframe_type_avi;
    packet.hb1 = version;
    packet.hb2 = avi_infoframe_size;
    packet.hb3 = 0;
    packet.valid = true;
    output = packet;
}

void build_hdmi_vsif(const Timing& timing, uint8_t hdmi_vic, InfoPacket& output) noexcept {
    InfoPacket packet{};

    if (hdmi_vic == 0 || timing.h_total < 3840 || timing.v_total < 2160) {
        output = packet; // 2D without an HDMI VIC: no VSIF.
        return;
    }

    const uint8_t length = 5;
    packet.sb[1] = 0x03;
    packet.sb[2] = 0x0c;
    packet.sb[3] = 0x00;
    packet.sb[4] = 1u << 5;
    packet.sb[5] = hdmi_vic;
    packet.hb0 = infoframe_type_vendor;
    packet.hb1 = 0x01;
    packet.hb2 = length;
    uint32_t sum = packet.hb0 + packet.hb1 + packet.hb2;

    for (uint32_t index = 1; index <= length; ++index) {
        sum += packet.sb[index];
    }

    packet.sb[0] = checksum_complement(sum);
    packet.valid = true;
    output = packet;
}

void build_dp_vsc(bool use_vsc_sdp_for_colorimetry, InfoPacket& output) noexcept {
    InfoPacket packet{};

    if (use_vsc_sdp_for_colorimetry) {
        // vsc_packet_rev5: RGB (0), ColorimetryRGB_DP_sRGB (0), 8 bpc (1).
        packet.hb0 = 0x00;
        packet.hb1 = 0x07;
        packet.hb2 = 0x05;
        packet.hb3 = 0x13;
        packet.sb[16] = 0;
        packet.sb[17] = 1;
        packet.sb[18] = 0;
        packet.valid = true;
    }

    output = packet;
}

HdmiStatus plan_tmds_stream(const Timing& timing, const HdmiSinkInfo& sink, TmdsStreamPlan& output) noexcept {
    OptcTiming fields{};

    if (prepare_optc_timing(timing, fields) != Status::success) {
        return HdmiStatus::unsupported_timing;
    }

    TmdsStreamPlan plan{};
    plan.signal = sink.is_hdmi ? TmdsSignal::hdmi : TmdsSignal::dvi_single_link;
    // get_tmds_output_pixel_clock_100hz at COLOR_DEPTH_888 RGB: no scaling.
    plan.tmds_khz = timing.pixel_clock_khz;

    if (plan.signal == TmdsSignal::dvi_single_link) {
        // dcn10_link_encoder_validate_dvi_output on a single-link HDMI connector.
        if (plan.tmds_khz > tmds_max_single_link_khz) {
            return HdmiStatus::exceeds_tmds_limit;
        }

        output = plan;
        return HdmiStatus::success;
    }

    if (plan.tmds_khz > dcn32_max_hdmi_pixel_clock_khz ||
        (sink.max_tmds_clock_khz != 0 && plan.tmds_khz > sink.max_tmds_clock_khz)) {
        return HdmiStatus::exceeds_tmds_limit;
    }

    if (plan.tmds_khz >= hdmi_clock_channel_rate_more_340m_khz) {
        // enc32_stream_encoder_hdmi_set_stream_attribute enables scrambling
        // with the 1/4 clock channel from 340 MHz; refuse sinks without it.
        if (plan.tmds_khz > hdmi_clock_channel_rate_more_340m_khz &&
            (!sink.scdc_supported || !sink.scrambling_supported)) {
            return HdmiStatus::needs_scrambling;
        }

        plan.scramble_above_340 = true;
    }

    // drm_hdmi_avi_infoframe_from_display_mode / drm_mode_cea_vic and
    // drm_hdmi_vendor_infoframe_from_display_mode / drm_mode_hdmi_vic.
    // drm_mode_hdmi_vic, drm_mode_cea_vic and vic_for_avi_infoframe: an HDMI
    // 1.4 sink only receives VICs above 64 that its VDB lists.
    plan.hdmi_vic = sink.has_hdmi_infoframe ? match_hdmi_vic(timing) : 0;
    plan.vic = plan.hdmi_vic != 0 ? 0 : match_cea_vic(timing);

    if (!hdmi2_sink(sink) && plan.vic > 64 && !listed_vic(sink, plan.vic)) {
        plan.vic = 0;
    }

    build_avi_infoframe(timing, plan.vic, plan.hdmi_vic, plan.avi);
    build_hdmi_vsif(timing, plan.hdmi_vic, plan.vsif);
    output = plan;
    return HdmiStatus::success;
}

} // namespace rx7900::display

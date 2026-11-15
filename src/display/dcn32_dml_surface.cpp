/*
 * Copyright 2022 Advanced Micro Devices, Inc. All rights reserved.
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
 * Authors: AMD
 */
/*
Module: rx7900.display.dcn32_dml_surface
Purpose: DML32 surface, swath, DET, ODM, clock and output-link utilities.
Created: 2026-10-24 (adaptation; the upstream notice above is preserved)
Architecture: Part of the bounded C++17 port of the Linux v6.12 DML32 model behind
src/display/dcn32_bandwidth (one stream, up to four pipes). Identifiers, statement order and
arithmetic (including the float-typed dml_min/max/floor/ceil helpers and every
int/unsigned conversion) deliberately follow the pinned upstream text so each
function can be compared line by line with work/upstream-dcn-stream; this is a
documented exception to the snake_case rule, as in src/opengl/gfx11_registers.hpp.
Per-plane and per-pipe arrays hold the four DCN 3.2 pipes (DML32_NUM_PLANES; Linux
sizes them DC__NUM_DPP__MAX = 8) and per-state arrays DML32_NUM_STATES entries; debug printing is removed.
Upstream: dml/dcn32/display_mode_vba_util_32.c (first part).
*/
#include "dcn32_dml_util.hpp"

namespace rx7900::display::dml32 {
#define DCN32_MAX_FMT_420_BUFFER_WIDTH 4096

unsigned int dml32_dscceComputeDelay(
        unsigned int bpc,
        double BPP,
        unsigned int sliceWidth,
        unsigned int numSlices,
        enum output_format_class pixelFormat,
        enum output_encoder_class /* Output */)
{
    // valid bpc         = source bits per component in the set of {8, 10, 12}
    // valid bpp         = increments of 1/16 of a bit
    //                    min = 6/7/8 in N420/N422/444, respectively
    //                    max = such that compression is 1:1
    //valid sliceWidth  = number of pixels per slice line,
    //  must be less than or equal to 5184/numSlices (or 4096/numSlices in 420 mode)
    //valid numSlices   = number of slices in the horiziontal direction per DSC engine in the set of {1, 2, 3, 4}
    //valid pixelFormat = pixel/color format in the set of {:N444_RGB, :S422, :N422, :N420}

    // fixed value
    unsigned int rcModelSize = 8192;

    // N422/N420 operate at 2 pixels per clock
    unsigned int pixelsPerClock, lstall, D, initalXmitDelay, w, s, ix, wx, p, l0, a, ax, L,
    Delay, pixels;

    if (pixelFormat == dm_420)
        pixelsPerClock = 2;
    else if (pixelFormat == dm_n422)
        pixelsPerClock = 2;
    // #all other modes operate at 1 pixel per clock
    else
        pixelsPerClock = 1;

    //initial transmit delay as per PPS
    initalXmitDelay = dml_round(rcModelSize / 2.0 / BPP / pixelsPerClock);

    //compute ssm delay
    if (bpc == 8)
        D = 81;
    else if (bpc == 10)
        D = 89;
    else
        D = 113;

    //divide by pixel per cycle to compute slice width as seen by DSC
    w = sliceWidth / pixelsPerClock;

    //422 mode has an additional cycle of delay
    if (pixelFormat == dm_420 || pixelFormat == dm_444 || pixelFormat == dm_n422)
        s = 0;
    else
        s = 1;

    //main calculation for the dscce
    ix = initalXmitDelay + 45;
    wx = (w + 2) / 3;
    p = 3 * wx - w;
    l0 = ix / w;
    a = ix + p * l0;
    ax = (a + 2) / 3 + D + 6 + 1;
    L = (ax + wx - 1) / wx;
    if ((ix % w) == 0 && p != 0)
        lstall = 1;
    else
        lstall = 0;
    Delay = L * wx * (numSlices - 1) + ax + s + lstall + 22;

    //dsc processes 3 pixel containers per cycle and a container can contain 1 or 2 pixels
    pixels = Delay * 3 * pixelsPerClock;

    return pixels;
}

unsigned int dml32_dscComputeDelay(enum output_format_class pixelFormat, enum output_encoder_class /* Output */)
{
    unsigned int Delay = 0;

    if (pixelFormat == dm_420) {
        //   sfr
        Delay = Delay + 2;
        //   dsccif
        Delay = Delay + 0;
        //   dscc - input deserializer
        Delay = Delay + 3;
        //   dscc gets pixels every other cycle
        Delay = Delay + 2;
        //   dscc - input cdc fifo
        Delay = Delay + 12;
        //   dscc gets pixels every other cycle
        Delay = Delay + 13;
        //   dscc - cdc uncertainty
        Delay = Delay + 2;
        //   dscc - output cdc fifo
        Delay = Delay + 7;
        //   dscc gets pixels every other cycle
        Delay = Delay + 3;
        //   dscc - cdc uncertainty
        Delay = Delay + 2;
        //   dscc - output serializer
        Delay = Delay + 1;
        //   sft
        Delay = Delay + 1;
    } else if (pixelFormat == dm_n422 || (pixelFormat != dm_444)) {
        //   sfr
        Delay = Delay + 2;
        //   dsccif
        Delay = Delay + 1;
        //   dscc - input deserializer
        Delay = Delay + 5;
        //  dscc - input cdc fifo
        Delay = Delay + 25;
        //   dscc - cdc uncertainty
        Delay = Delay + 2;
        //   dscc - output cdc fifo
        Delay = Delay + 10;
        //   dscc - cdc uncertainty
        Delay = Delay + 2;
        //   dscc - output serializer
        Delay = Delay + 1;
        //   sft
        Delay = Delay + 1;
    } else {
        //   sfr
        Delay = Delay + 2;
        //   dsccif
        Delay = Delay + 0;
        //   dscc - input deserializer
        Delay = Delay + 3;
        //   dscc - input cdc fifo
        Delay = Delay + 12;
        //   dscc - cdc uncertainty
        Delay = Delay + 2;
        //   dscc - output cdc fifo
        Delay = Delay + 7;
        //   dscc - output serializer
        Delay = Delay + 1;
        //   dscc - cdc uncertainty
        Delay = Delay + 2;
        //   sft
        Delay = Delay + 1;
    }

    return Delay;
}

bool IsVertical(enum dm_rotation_angle Scan)
{
    bool is_vert = false;

    if (Scan == dm_rotation_90 || Scan == dm_rotation_90m || Scan == dm_rotation_270 || Scan == dm_rotation_270m)
        is_vert = true;
    else
        is_vert = false;
    return is_vert;
}

void dml32_CalculateSinglePipeDPPCLKAndSCLThroughput(
        double HRatio,
        double HRatioChroma,
        double VRatio,
        double VRatioChroma,
        double MaxDCHUBToPSCLThroughput,
        double MaxPSCLToLBThroughput,
        double PixelClock,
        enum source_format_class SourcePixelFormat,
        unsigned int HTaps,
        unsigned int HTapsChroma,
        unsigned int VTaps,
        unsigned int VTapsChroma,

        /* output */
        double *PSCL_THROUGHPUT,
        double *PSCL_THROUGHPUT_CHROMA,
        double *DPPCLKUsingSingleDPP)
{
    double DPPCLKUsingSingleDPPLuma;
    double DPPCLKUsingSingleDPPChroma;

    if (HRatio > 1) {
        *PSCL_THROUGHPUT = dml_min(MaxDCHUBToPSCLThroughput, MaxPSCLToLBThroughput * HRatio /
                dml_ceil(static_cast<double>(HTaps) / 6.0, 1.0));
    } else {
        *PSCL_THROUGHPUT = dml_min(MaxDCHUBToPSCLThroughput, MaxPSCLToLBThroughput);
    }

    DPPCLKUsingSingleDPPLuma = PixelClock * dml_max3(VTaps / 6 * dml_min(1, HRatio), HRatio * VRatio /
            *PSCL_THROUGHPUT, 1);

    if ((HTaps > 6 || VTaps > 6) && DPPCLKUsingSingleDPPLuma < 2 * PixelClock)
        DPPCLKUsingSingleDPPLuma = 2 * PixelClock;

    if ((SourcePixelFormat != dm_420_8 && SourcePixelFormat != dm_420_10 && SourcePixelFormat != dm_420_12 &&
            SourcePixelFormat != dm_rgbe_alpha)) {
        *PSCL_THROUGHPUT_CHROMA = 0;
        *DPPCLKUsingSingleDPP = DPPCLKUsingSingleDPPLuma;
    } else {
        if (HRatioChroma > 1) {
            *PSCL_THROUGHPUT_CHROMA = dml_min(MaxDCHUBToPSCLThroughput, MaxPSCLToLBThroughput *
                    HRatioChroma / dml_ceil(static_cast<double>(HTapsChroma) / 6.0, 1.0));
        } else {
            *PSCL_THROUGHPUT_CHROMA = dml_min(MaxDCHUBToPSCLThroughput, MaxPSCLToLBThroughput);
        }
        DPPCLKUsingSingleDPPChroma = PixelClock * dml_max3(VTapsChroma / 6 * dml_min(1, HRatioChroma),
                HRatioChroma * VRatioChroma / *PSCL_THROUGHPUT_CHROMA, 1);
        if ((HTapsChroma > 6 || VTapsChroma > 6) && DPPCLKUsingSingleDPPChroma < 2 * PixelClock)
            DPPCLKUsingSingleDPPChroma = 2 * PixelClock;
        *DPPCLKUsingSingleDPP = dml_max(DPPCLKUsingSingleDPPLuma, DPPCLKUsingSingleDPPChroma);
    }
}

void dml32_CalculateBytePerPixelAndBlockSizes(
        enum source_format_class SourcePixelFormat,
        enum dm_swizzle_mode SurfaceTiling,

        /* Output */
        unsigned int *BytePerPixelY,
        unsigned int *BytePerPixelC,
        double  *BytePerPixelDETY,
        double  *BytePerPixelDETC,
        unsigned int *BlockHeight256BytesY,
        unsigned int *BlockHeight256BytesC,
        unsigned int *BlockWidth256BytesY,
        unsigned int *BlockWidth256BytesC,
        unsigned int *MacroTileHeightY,
        unsigned int *MacroTileHeightC,
        unsigned int *MacroTileWidthY,
        unsigned int *MacroTileWidthC)
{
    if (SourcePixelFormat == dm_444_64) {
        *BytePerPixelDETY = 8;
        *BytePerPixelDETC = 0;
        *BytePerPixelY = 8;
        *BytePerPixelC = 0;
    } else if (SourcePixelFormat == dm_444_32 || SourcePixelFormat == dm_rgbe) {
        *BytePerPixelDETY = 4;
        *BytePerPixelDETC = 0;
        *BytePerPixelY = 4;
        *BytePerPixelC = 0;
    } else if (SourcePixelFormat == dm_444_16) {
        *BytePerPixelDETY = 2;
        *BytePerPixelDETC = 0;
        *BytePerPixelY = 2;
        *BytePerPixelC = 0;
    } else if (SourcePixelFormat == dm_444_8) {
        *BytePerPixelDETY = 1;
        *BytePerPixelDETC = 0;
        *BytePerPixelY = 1;
        *BytePerPixelC = 0;
    } else if (SourcePixelFormat == dm_rgbe_alpha) {
        *BytePerPixelDETY = 4;
        *BytePerPixelDETC = 1;
        *BytePerPixelY = 4;
        *BytePerPixelC = 1;
    } else if (SourcePixelFormat == dm_420_8) {
        *BytePerPixelDETY = 1;
        *BytePerPixelDETC = 2;
        *BytePerPixelY = 1;
        *BytePerPixelC = 2;
    } else if (SourcePixelFormat == dm_420_12) {
        *BytePerPixelDETY = 2;
        *BytePerPixelDETC = 4;
        *BytePerPixelY = 2;
        *BytePerPixelC = 4;
    } else {
        *BytePerPixelDETY = 4.0 / 3;
        *BytePerPixelDETC = 8.0 / 3;
        *BytePerPixelY = 2;
        *BytePerPixelC = 4;
    }
    if ((SourcePixelFormat == dm_444_64 || SourcePixelFormat == dm_444_32
            || SourcePixelFormat == dm_444_16
            || SourcePixelFormat == dm_444_8
            || SourcePixelFormat == dm_mono_16
            || SourcePixelFormat == dm_mono_8
            || SourcePixelFormat == dm_rgbe)) {
        if (SurfaceTiling == dm_sw_linear)
            *BlockHeight256BytesY = 1;
        else if (SourcePixelFormat == dm_444_64)
            *BlockHeight256BytesY = 4;
        else if (SourcePixelFormat == dm_444_8)
            *BlockHeight256BytesY = 16;
        else
            *BlockHeight256BytesY = 8;

        *BlockWidth256BytesY = 256U / *BytePerPixelY / *BlockHeight256BytesY;
        *BlockHeight256BytesC = 0;
        *BlockWidth256BytesC = 0;
    } else {
        if (SurfaceTiling == dm_sw_linear) {
            *BlockHeight256BytesY = 1;
            *BlockHeight256BytesC = 1;
        } else if (SourcePixelFormat == dm_rgbe_alpha) {
            *BlockHeight256BytesY = 8;
            *BlockHeight256BytesC = 16;
        } else if (SourcePixelFormat == dm_420_8) {
            *BlockHeight256BytesY = 16;
            *BlockHeight256BytesC = 8;
        } else {
            *BlockHeight256BytesY = 8;
            *BlockHeight256BytesC = 8;
        }
        *BlockWidth256BytesY = 256U / *BytePerPixelY / *BlockHeight256BytesY;
        *BlockWidth256BytesC = 256U / *BytePerPixelC / *BlockHeight256BytesC;
    }

    if (SurfaceTiling == dm_sw_linear) {
        *MacroTileHeightY = *BlockHeight256BytesY;
        *MacroTileWidthY = 256 / *BytePerPixelY / *MacroTileHeightY;
        *MacroTileHeightC = *BlockHeight256BytesC;
        if (*MacroTileHeightC == 0)
            *MacroTileWidthC = 0;
        else
            *MacroTileWidthC = 256 / *BytePerPixelC / *MacroTileHeightC;
    } else if (SurfaceTiling == dm_sw_64kb_d || SurfaceTiling == dm_sw_64kb_d_t ||
            SurfaceTiling == dm_sw_64kb_d_x || SurfaceTiling == dm_sw_64kb_r_x) {
        *MacroTileHeightY = 16 * *BlockHeight256BytesY;
        *MacroTileWidthY = 65536 / *BytePerPixelY / *MacroTileHeightY;
        *MacroTileHeightC = 16 * *BlockHeight256BytesC;
        if (*MacroTileHeightC == 0)
            *MacroTileWidthC = 0;
        else
            *MacroTileWidthC = 65536 / *BytePerPixelC / *MacroTileHeightC;
    } else {
        *MacroTileHeightY = 32 * *BlockHeight256BytesY;
        *MacroTileWidthY = 65536 * 4 / *BytePerPixelY / *MacroTileHeightY;
        *MacroTileHeightC = 32 * *BlockHeight256BytesC;
        if (*MacroTileHeightC == 0)
            *MacroTileWidthC = 0;
        else
            *MacroTileWidthC = 65536 * 4 / *BytePerPixelC / *MacroTileHeightC;
    }

} // CalculateBytePerPixelAndBlockSizes

void dml32_CalculateSwathAndDETConfiguration(
        unsigned int DETSizeOverride[],
        enum dm_use_mall_for_pstate_change_mode UseMALLForPStateChange[],
        unsigned int ConfigReturnBufferSizeInKByte,
        unsigned int MaxTotalDETInKByte,
        unsigned int MinCompressedBufferSizeInKByte,
        double ForceSingleDPP,
        unsigned int NumberOfActiveSurfaces,
        unsigned int nomDETInKByte,
        enum unbounded_requesting_policy UseUnboundedRequestingFinal,
        bool DisableUnboundRequestIfCompBufReservedSpaceNeedAdjustment,
        unsigned int PixelChunkSizeKBytes,
        unsigned int ROBSizeKBytes,
        unsigned int CompressedBufferSegmentSizeInkByteFinal,
        enum output_encoder_class Output[],
        double ReadBandwidthLuma[],
        double ReadBandwidthChroma[],
        double MaximumSwathWidthLuma[],
        double MaximumSwathWidthChroma[],
        enum dm_rotation_angle SourceRotation[],
        bool ViewportStationary[],
        enum source_format_class SourcePixelFormat[],
        enum dm_swizzle_mode SurfaceTiling[],
        unsigned int ViewportWidth[],
        unsigned int ViewportHeight[],
        unsigned int ViewportXStart[],
        unsigned int ViewportYStart[],
        unsigned int ViewportXStartC[],
        unsigned int ViewportYStartC[],
        unsigned int SurfaceWidthY[],
        unsigned int SurfaceWidthC[],
        unsigned int SurfaceHeightY[],
        unsigned int SurfaceHeightC[],
        unsigned int Read256BytesBlockHeightY[],
        unsigned int Read256BytesBlockHeightC[],
        unsigned int Read256BytesBlockWidthY[],
        unsigned int Read256BytesBlockWidthC[],
        enum odm_combine_mode ODMMode[],
        unsigned int BlendingAndTiming[],
        unsigned int BytePerPixY[],
        unsigned int BytePerPixC[],
        double BytePerPixDETY[],
        double BytePerPixDETC[],
        unsigned int HActive[],
        double HRatio[],
        double /* HRatioChroma */[],
        unsigned int DPPPerSurface[],

        /* Output */
        unsigned int swath_width_luma_ub[],
        unsigned int swath_width_chroma_ub[],
        double SwathWidth[],
        double SwathWidthChroma[],
        unsigned int SwathHeightY[],
        unsigned int SwathHeightC[],
        unsigned int DETBufferSizeInKByte[],
        unsigned int DETBufferSizeY[],
        unsigned int DETBufferSizeC[],
        bool *UnboundedRequestEnabled,
        unsigned int *CompressedBufferSizeInkByte,
        unsigned int *CompBufReservedSpaceKBytes,
        bool *CompBufReservedSpaceNeedAdjustment,
        bool ViewportSizeSupportPerSurface[],
        bool *ViewportSizeSupport)
{
    unsigned int MaximumSwathHeightY[DML32_NUM_PLANES];
    unsigned int MaximumSwathHeightC[DML32_NUM_PLANES];
    // Zero-initialised: Linux reads [0] below even with no active surface
    // (never the case here); GCC 13 -O2 flags that read.
    unsigned int RoundedUpMaxSwathSizeBytesY[DML32_NUM_PLANES]{};
    unsigned int RoundedUpMaxSwathSizeBytesC[DML32_NUM_PLANES]{};
    unsigned int RoundedUpSwathSizeBytesY;
    unsigned int RoundedUpSwathSizeBytesC;
    double SwathWidthdoubleDPP[DML32_NUM_PLANES];
    double SwathWidthdoubleDPPChroma[DML32_NUM_PLANES];
    unsigned int k;
    unsigned int TotalActiveDPP = 0;
    bool NoChromaSurfaces = true;
    unsigned int DETBufferSizeInKByteForSwathCalculation;

    dml32_CalculateSwathWidth(ForceSingleDPP,
            NumberOfActiveSurfaces,
            SourcePixelFormat,
            SourceRotation,
            ViewportStationary,
            ViewportWidth,
            ViewportHeight,
            ViewportXStart,
            ViewportYStart,
            ViewportXStartC,
            ViewportYStartC,
            SurfaceWidthY,
            SurfaceWidthC,
            SurfaceHeightY,
            SurfaceHeightC,
            ODMMode,
            BytePerPixY,
            BytePerPixC,
            Read256BytesBlockHeightY,
            Read256BytesBlockHeightC,
            Read256BytesBlockWidthY,
            Read256BytesBlockWidthC,
            BlendingAndTiming,
            HActive,
            HRatio,
            DPPPerSurface,

            /* Output */
            SwathWidthdoubleDPP,
            SwathWidthdoubleDPPChroma,
            SwathWidth,
            SwathWidthChroma,
            MaximumSwathHeightY,
            MaximumSwathHeightC,
            swath_width_luma_ub,
            swath_width_chroma_ub);

    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        RoundedUpMaxSwathSizeBytesY[k] = swath_width_luma_ub[k] * BytePerPixDETY[k] * MaximumSwathHeightY[k];
        RoundedUpMaxSwathSizeBytesC[k] = swath_width_chroma_ub[k] * BytePerPixDETC[k] * MaximumSwathHeightC[k];

        if (SourcePixelFormat[k] == dm_420_10) {
            RoundedUpMaxSwathSizeBytesY[k] = dml_ceil(static_cast<unsigned int>(RoundedUpMaxSwathSizeBytesY[k]), 256);
            RoundedUpMaxSwathSizeBytesC[k] = dml_ceil(static_cast<unsigned int>(RoundedUpMaxSwathSizeBytesC[k]), 256);
        }
    }

    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        TotalActiveDPP = TotalActiveDPP + (ForceSingleDPP ? 1 : DPPPerSurface[k]);
        if (SourcePixelFormat[k] == dm_420_8 || SourcePixelFormat[k] == dm_420_10 ||
                SourcePixelFormat[k] == dm_420_12 || SourcePixelFormat[k] == dm_rgbe_alpha) {
            NoChromaSurfaces = false;
        }
    }

    // By default, just set the reserved space to 2 pixel chunks size
    *CompBufReservedSpaceKBytes = PixelChunkSizeKBytes * 2;

    // if unbounded req is enabled, program reserved space such that the ROB will not hold more than 8 swaths worth of data
    // - assume worst-case compression rate of 4. [ROB size - 8 * swath_size / max_compression ratio]
    // - assume for "narrow" vp case in which the ROB can fit 8 swaths, the DET should be big enough to do full size req
    *CompBufReservedSpaceNeedAdjustment = (static_cast<int>(ROBSizeKBytes) - static_cast<int>(*CompBufReservedSpaceKBytes)) > static_cast<int>(RoundedUpMaxSwathSizeBytesY[0]/512);

    if (*CompBufReservedSpaceNeedAdjustment == 1) {
        *CompBufReservedSpaceKBytes = ROBSizeKBytes - RoundedUpMaxSwathSizeBytesY[0]/512;
    }

    *UnboundedRequestEnabled = dml32_UnboundedRequest(UseUnboundedRequestingFinal, TotalActiveDPP, NoChromaSurfaces, Output[0], SurfaceTiling[0], *CompBufReservedSpaceNeedAdjustment, DisableUnboundRequestIfCompBufReservedSpaceNeedAdjustment);

    dml32_CalculateDETBufferSize(DETSizeOverride,
            UseMALLForPStateChange,
            ForceSingleDPP,
            NumberOfActiveSurfaces,
            *UnboundedRequestEnabled,
            nomDETInKByte,
            MaxTotalDETInKByte,
            ConfigReturnBufferSizeInKByte,
            MinCompressedBufferSizeInKByte,
            CompressedBufferSegmentSizeInkByteFinal,
            SourcePixelFormat,
            ReadBandwidthLuma,
            ReadBandwidthChroma,
            RoundedUpMaxSwathSizeBytesY,
            RoundedUpMaxSwathSizeBytesC,
            DPPPerSurface,

            /* Output */
            DETBufferSizeInKByte,    // per hubp pipe
            CompressedBufferSizeInkByte);

    *ViewportSizeSupport = true;
    for (k = 0; k < NumberOfActiveSurfaces; ++k) {

        DETBufferSizeInKByteForSwathCalculation = (UseMALLForPStateChange[k] ==
                dm_use_mall_pstate_change_phantom_pipe ? 1024 : DETBufferSizeInKByte[k]);

        if (RoundedUpMaxSwathSizeBytesY[k] + RoundedUpMaxSwathSizeBytesC[k] <=
                DETBufferSizeInKByteForSwathCalculation * 1024 / 2) {
            SwathHeightY[k] = MaximumSwathHeightY[k];
            SwathHeightC[k] = MaximumSwathHeightC[k];
            RoundedUpSwathSizeBytesY = RoundedUpMaxSwathSizeBytesY[k];
            RoundedUpSwathSizeBytesC = RoundedUpMaxSwathSizeBytesC[k];
        } else if (RoundedUpMaxSwathSizeBytesY[k] >= 1.5 * RoundedUpMaxSwathSizeBytesC[k] &&
                RoundedUpMaxSwathSizeBytesY[k] / 2 + RoundedUpMaxSwathSizeBytesC[k] <=
                DETBufferSizeInKByteForSwathCalculation * 1024 / 2) {
            SwathHeightY[k] = MaximumSwathHeightY[k] / 2;
            SwathHeightC[k] = MaximumSwathHeightC[k];
            RoundedUpSwathSizeBytesY = RoundedUpMaxSwathSizeBytesY[k] / 2;
            RoundedUpSwathSizeBytesC = RoundedUpMaxSwathSizeBytesC[k];
        } else if (RoundedUpMaxSwathSizeBytesY[k] < 1.5 * RoundedUpMaxSwathSizeBytesC[k] &&
                RoundedUpMaxSwathSizeBytesY[k] + RoundedUpMaxSwathSizeBytesC[k] / 2 <=
                DETBufferSizeInKByteForSwathCalculation * 1024 / 2) {
            SwathHeightY[k] = MaximumSwathHeightY[k];
            SwathHeightC[k] = MaximumSwathHeightC[k] / 2;
            RoundedUpSwathSizeBytesY = RoundedUpMaxSwathSizeBytesY[k];
            RoundedUpSwathSizeBytesC = RoundedUpMaxSwathSizeBytesC[k] / 2;
        } else {
            SwathHeightY[k] = MaximumSwathHeightY[k] / 2;
            SwathHeightC[k] = MaximumSwathHeightC[k] / 2;
            RoundedUpSwathSizeBytesY = RoundedUpMaxSwathSizeBytesY[k] / 2;
            RoundedUpSwathSizeBytesC = RoundedUpMaxSwathSizeBytesC[k] / 2;
        }

        if ((RoundedUpMaxSwathSizeBytesY[k] / 2 + RoundedUpMaxSwathSizeBytesC[k] / 2 >
                DETBufferSizeInKByteForSwathCalculation * 1024 / 2)
                || SwathWidth[k] > MaximumSwathWidthLuma[k] || (SwathHeightC[k] > 0 &&
                        SwathWidthChroma[k] > MaximumSwathWidthChroma[k])) {
            *ViewportSizeSupport = false;
            ViewportSizeSupportPerSurface[k] = false;
        } else {
            ViewportSizeSupportPerSurface[k] = true;
        }

        if (SwathHeightC[k] == 0) {
            DETBufferSizeY[k] = DETBufferSizeInKByte[k] * 1024;
            DETBufferSizeC[k] = 0;
        } else if (RoundedUpSwathSizeBytesY <= 1.5 * RoundedUpSwathSizeBytesC) {
            DETBufferSizeY[k] = DETBufferSizeInKByte[k] * 1024 / 2;
            DETBufferSizeC[k] = DETBufferSizeInKByte[k] * 1024 / 2;
        } else {
            DETBufferSizeY[k] = dml_floor(DETBufferSizeInKByte[k] * 1024 * 2 / 3, 1024);
            DETBufferSizeC[k] = DETBufferSizeInKByte[k] * 1024 - DETBufferSizeY[k];
        }

    }
} // CalculateSwathAndDETConfiguration

void dml32_CalculateSwathWidth(
        bool                ForceSingleDPP,
        unsigned int            NumberOfActiveSurfaces,
        enum source_format_class    SourcePixelFormat[],
        enum dm_rotation_angle      SourceRotation[],
        bool                ViewportStationary[],
        unsigned int            ViewportWidth[],
        unsigned int            ViewportHeight[],
        unsigned int            ViewportXStart[],
        unsigned int            ViewportYStart[],
        unsigned int            ViewportXStartC[],
        unsigned int            ViewportYStartC[],
        unsigned int            SurfaceWidthY[],
        unsigned int            SurfaceWidthC[],
        unsigned int            SurfaceHeightY[],
        unsigned int            SurfaceHeightC[],
        enum odm_combine_mode       ODMMode[],
        unsigned int            /* BytePerPixY */[],
        unsigned int            BytePerPixC[],
        unsigned int            Read256BytesBlockHeightY[],
        unsigned int            Read256BytesBlockHeightC[],
        unsigned int            Read256BytesBlockWidthY[],
        unsigned int            Read256BytesBlockWidthC[],
        unsigned int            BlendingAndTiming[],
        unsigned int            HActive[],
        double              HRatio[],
        unsigned int            DPPPerSurface[],

        /* Output */
        double              SwathWidthdoubleDPPY[],
        double              SwathWidthdoubleDPPC[],
        double              SwathWidthY[], // per-pipe
        double              SwathWidthC[], // per-pipe
        unsigned int            MaximumSwathHeightY[],
        unsigned int            MaximumSwathHeightC[],
        unsigned int            swath_width_luma_ub[], // per-pipe
        unsigned int            swath_width_chroma_ub[]) // per-pipe
{
    unsigned int k, j;
    enum odm_combine_mode MainSurfaceODMMode;

    unsigned int surface_width_ub_l;
    unsigned int surface_height_ub_l;
    unsigned int surface_width_ub_c = 0;
    unsigned int surface_height_ub_c = 0;

    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        if (!IsVertical(SourceRotation[k]))
            SwathWidthdoubleDPPY[k] = ViewportWidth[k];
        else
            SwathWidthdoubleDPPY[k] = ViewportHeight[k];

        MainSurfaceODMMode = ODMMode[k];
        for (j = 0; j < NumberOfActiveSurfaces; ++j) {
            if (BlendingAndTiming[k] == j)
                MainSurfaceODMMode = ODMMode[j];
        }

        if (ForceSingleDPP) {
            SwathWidthY[k] = SwathWidthdoubleDPPY[k];
        } else {
            if (MainSurfaceODMMode == dm_odm_combine_mode_4to1) {
                SwathWidthY[k] = dml_min(SwathWidthdoubleDPPY[k],
                        dml_round(HActive[k] / 4.0 * HRatio[k]));
            } else if (MainSurfaceODMMode == dm_odm_combine_mode_2to1) {
                SwathWidthY[k] = dml_min(SwathWidthdoubleDPPY[k],
                        dml_round(HActive[k] / 2.0 * HRatio[k]));
            } else if (DPPPerSurface[k] == 2) {
                SwathWidthY[k] = SwathWidthdoubleDPPY[k] / 2;
            } else {
                SwathWidthY[k] = SwathWidthdoubleDPPY[k];
            }
        }

        if (SourcePixelFormat[k] == dm_420_8 || SourcePixelFormat[k] == dm_420_10 ||
                SourcePixelFormat[k] == dm_420_12) {
            SwathWidthC[k] = SwathWidthY[k] / 2;
            SwathWidthdoubleDPPC[k] = SwathWidthdoubleDPPY[k] / 2;
        } else {
            SwathWidthC[k] = SwathWidthY[k];
            SwathWidthdoubleDPPC[k] = SwathWidthdoubleDPPY[k];
        }

        if (ForceSingleDPP == true) {
            SwathWidthY[k] = SwathWidthdoubleDPPY[k];
            SwathWidthC[k] = SwathWidthdoubleDPPC[k];
        }

        surface_width_ub_l  = dml_ceil(SurfaceWidthY[k], Read256BytesBlockWidthY[k]);
        surface_height_ub_l = dml_ceil(SurfaceHeightY[k], Read256BytesBlockHeightY[k]);

        if (!IsVertical(SourceRotation[k])) {
            MaximumSwathHeightY[k] = Read256BytesBlockHeightY[k];
            MaximumSwathHeightC[k] = Read256BytesBlockHeightC[k];
            if (ViewportStationary[k] && DPPPerSurface[k] == 1) {
                swath_width_luma_ub[k] = dml_min(surface_width_ub_l,
                        dml_floor(ViewportXStart[k] +
                                SwathWidthY[k] +
                                Read256BytesBlockWidthY[k] - 1,
                                Read256BytesBlockWidthY[k]) -
                                dml_floor(ViewportXStart[k],
                                Read256BytesBlockWidthY[k]));
            } else {
                swath_width_luma_ub[k] = dml_min(surface_width_ub_l,
                        dml_ceil(SwathWidthY[k] - 1,
                                Read256BytesBlockWidthY[k]) +
                                Read256BytesBlockWidthY[k]);
            }
            if (BytePerPixC[k] > 0) {
                surface_width_ub_c  = dml_ceil(SurfaceWidthC[k], Read256BytesBlockWidthC[k]);
                if (ViewportStationary[k] && DPPPerSurface[k] == 1) {
                    swath_width_chroma_ub[k] = dml_min(surface_width_ub_c,
                            dml_floor(ViewportXStartC[k] + SwathWidthC[k] +
                                    Read256BytesBlockWidthC[k] - 1,
                                    Read256BytesBlockWidthC[k]) -
                                    dml_floor(ViewportXStartC[k],
                                    Read256BytesBlockWidthC[k]));
                } else {
                    swath_width_chroma_ub[k] = dml_min(surface_width_ub_c,
                            dml_ceil(SwathWidthC[k] - 1,
                                Read256BytesBlockWidthC[k]) +
                                Read256BytesBlockWidthC[k]);
                }
            } else {
                swath_width_chroma_ub[k] = 0;
            }
        } else {
            MaximumSwathHeightY[k] = Read256BytesBlockWidthY[k];
            MaximumSwathHeightC[k] = Read256BytesBlockWidthC[k];

            if (ViewportStationary[k] && DPPPerSurface[k] == 1) {
                swath_width_luma_ub[k] = dml_min(surface_height_ub_l, dml_floor(ViewportYStart[k] +
                        SwathWidthY[k] + Read256BytesBlockHeightY[k] - 1,
                        Read256BytesBlockHeightY[k]) -
                        dml_floor(ViewportYStart[k], Read256BytesBlockHeightY[k]));
            } else {
                swath_width_luma_ub[k] = dml_min(surface_height_ub_l, dml_ceil(SwathWidthY[k] - 1,
                        Read256BytesBlockHeightY[k]) + Read256BytesBlockHeightY[k]);
            }
            if (BytePerPixC[k] > 0) {
                surface_height_ub_c = dml_ceil(SurfaceHeightC[k], Read256BytesBlockHeightC[k]);
                if (ViewportStationary[k] && DPPPerSurface[k] == 1) {
                    swath_width_chroma_ub[k] = dml_min(surface_height_ub_c,
                            dml_floor(ViewportYStartC[k] + SwathWidthC[k] +
                                    Read256BytesBlockHeightC[k] - 1,
                                    Read256BytesBlockHeightC[k]) -
                                    dml_floor(ViewportYStartC[k],
                                            Read256BytesBlockHeightC[k]));
                } else {
                    swath_width_chroma_ub[k] = dml_min(surface_height_ub_c,
                            dml_ceil(SwathWidthC[k] - 1, Read256BytesBlockHeightC[k]) +
                            Read256BytesBlockHeightC[k]);
                }
            } else {
                swath_width_chroma_ub[k] = 0;
            }
        }

    }
} // CalculateSwathWidth

bool dml32_UnboundedRequest(enum unbounded_requesting_policy UseUnboundedRequestingFinal,
            unsigned int TotalNumberOfActiveDPP,
            bool NoChroma,
            enum output_encoder_class Output,
            enum dm_swizzle_mode SurfaceTiling,
            bool CompBufReservedSpaceNeedAdjustment,
            bool DisableUnboundRequestIfCompBufReservedSpaceNeedAdjustment)
{
    bool ret_val = false;

    ret_val = (UseUnboundedRequestingFinal != dm_unbounded_requesting_disable &&
            TotalNumberOfActiveDPP == 1 && NoChroma);
    if (UseUnboundedRequestingFinal == dm_unbounded_requesting_edp_only && Output != dm_edp)
        ret_val = false;

    if (SurfaceTiling == dm_sw_linear)
        ret_val = false;

    if (CompBufReservedSpaceNeedAdjustment == 1 && DisableUnboundRequestIfCompBufReservedSpaceNeedAdjustment)
        ret_val = false;

    return (ret_val);
}

void dml32_CalculateDETBufferSize(
        unsigned int DETSizeOverride[],
        enum dm_use_mall_for_pstate_change_mode UseMALLForPStateChange[],
        bool ForceSingleDPP,
        unsigned int NumberOfActiveSurfaces,
        bool UnboundedRequestEnabled,
        unsigned int nomDETInKByte,
        unsigned int MaxTotalDETInKByte,
        unsigned int ConfigReturnBufferSizeInKByte,
        unsigned int MinCompressedBufferSizeInKByte,
        unsigned int CompressedBufferSegmentSizeInkByteFinal,
        enum source_format_class SourcePixelFormat[],
        double ReadBandwidthLuma[],
        double ReadBandwidthChroma[],
        unsigned int RoundedUpMaxSwathSizeBytesY[],
        unsigned int RoundedUpMaxSwathSizeBytesC[],
        unsigned int DPPPerSurface[],
        /* Output */
        unsigned int DETBufferSizeInKByte[],
        unsigned int *CompressedBufferSizeInkByte)
{
    unsigned int DETBufferSizePoolInKByte;
    unsigned int NextDETBufferPieceInKByte;
    bool DETPieceAssignedToThisSurfaceAlready[DML32_NUM_PLANES];
    bool NextPotentialSurfaceToAssignDETPieceFound;
    unsigned int NextSurfaceToAssignDETPiece;
    double TotalBandwidth;
    double BandwidthOfSurfacesNotAssignedDETPiece;
    unsigned int max_minDET;
    unsigned int minDET;
    unsigned int minDET_pipe;
    unsigned int j, k;

    // Note: Will use default det size if that fits 2 swaths
    if (UnboundedRequestEnabled) {
        if (DETSizeOverride[0] > 0) {
            DETBufferSizeInKByte[0] = DETSizeOverride[0];
        } else {
            DETBufferSizeInKByte[0] = dml_max(nomDETInKByte, dml_ceil(2.0 *
                    (static_cast<double>(RoundedUpMaxSwathSizeBytesY[0]) +
                            static_cast<double>(RoundedUpMaxSwathSizeBytesC[0])) / 1024.0, 64.0));
        }
        *CompressedBufferSizeInkByte = ConfigReturnBufferSizeInKByte - DETBufferSizeInKByte[0];
    } else {
        DETBufferSizePoolInKByte = MaxTotalDETInKByte;
        for (k = 0; k < NumberOfActiveSurfaces; ++k) {
            DETBufferSizeInKByte[k] = nomDETInKByte;
            if (SourcePixelFormat[k] == dm_420_8 || SourcePixelFormat[k] == dm_420_10 ||
                    SourcePixelFormat[k] == dm_420_12) {
                max_minDET = nomDETInKByte - 64;
            } else {
                max_minDET = nomDETInKByte;
            }
            minDET = 128;
            minDET_pipe = 0;

            // add DET resource until can hold 2 full swaths
            while (minDET <= max_minDET && minDET_pipe == 0) {
                if (2.0 * (static_cast<double>(RoundedUpMaxSwathSizeBytesY[k]) +
                        static_cast<double>(RoundedUpMaxSwathSizeBytesC[k])) / 1024.0 <= minDET)
                    minDET_pipe = minDET;
                minDET = minDET + 64;
            }

            if (minDET_pipe == 0) {
                minDET_pipe = dml_max(128, dml_ceil((static_cast<double>(RoundedUpMaxSwathSizeBytesY[k]) +
                        static_cast<double>(RoundedUpMaxSwathSizeBytesC[k])) / 1024.0, 64));
            }

            if (UseMALLForPStateChange[k] == dm_use_mall_pstate_change_phantom_pipe) {
                DETBufferSizeInKByte[k] = 0;
            } else if (DETSizeOverride[k] > 0) {
                DETBufferSizeInKByte[k] = DETSizeOverride[k];
                DETBufferSizePoolInKByte = DETBufferSizePoolInKByte -
                        (ForceSingleDPP ? 1 : DPPPerSurface[k]) * DETSizeOverride[k];
            } else if ((ForceSingleDPP ? 1 : DPPPerSurface[k]) * minDET_pipe <= DETBufferSizePoolInKByte) {
                DETBufferSizeInKByte[k] = minDET_pipe;
                DETBufferSizePoolInKByte = DETBufferSizePoolInKByte -
                        (ForceSingleDPP ? 1 : DPPPerSurface[k]) * minDET_pipe;
            }

        }

        TotalBandwidth = 0;
        for (k = 0; k < NumberOfActiveSurfaces; ++k) {
            if (UseMALLForPStateChange[k] != dm_use_mall_pstate_change_phantom_pipe)
                TotalBandwidth = TotalBandwidth + ReadBandwidthLuma[k] + ReadBandwidthChroma[k];
        }
        BandwidthOfSurfacesNotAssignedDETPiece = TotalBandwidth;
        for (k = 0; k < NumberOfActiveSurfaces; ++k) {

            if (UseMALLForPStateChange[k] == dm_use_mall_pstate_change_phantom_pipe) {
                DETPieceAssignedToThisSurfaceAlready[k] = true;
            } else if (DETSizeOverride[k] > 0 || ((static_cast<double>(ForceSingleDPP ? 1 : DPPPerSurface[k]) *
                    static_cast<double>(DETBufferSizeInKByte[k]) / static_cast<double>(MaxTotalDETInKByte)) >=
                    ((ReadBandwidthLuma[k] + ReadBandwidthChroma[k]) / TotalBandwidth))) {
                DETPieceAssignedToThisSurfaceAlready[k] = true;
                BandwidthOfSurfacesNotAssignedDETPiece = BandwidthOfSurfacesNotAssignedDETPiece -
                        ReadBandwidthLuma[k] - ReadBandwidthChroma[k];
            } else {
                DETPieceAssignedToThisSurfaceAlready[k] = false;
            }
        }

        for (j = 0; j < NumberOfActiveSurfaces; ++j) {
            NextPotentialSurfaceToAssignDETPieceFound = false;
            NextSurfaceToAssignDETPiece = 0;

            for (k = 0; k < NumberOfActiveSurfaces; ++k) {
                if (!DETPieceAssignedToThisSurfaceAlready[k] &&
                        (!NextPotentialSurfaceToAssignDETPieceFound ||
                        ReadBandwidthLuma[k] + ReadBandwidthChroma[k] <
                        ReadBandwidthLuma[NextSurfaceToAssignDETPiece] +
                        ReadBandwidthChroma[NextSurfaceToAssignDETPiece])) {
                    NextSurfaceToAssignDETPiece = k;
                    NextPotentialSurfaceToAssignDETPieceFound = true;
                }
            }

            if (NextPotentialSurfaceToAssignDETPieceFound) {
                // Note: To show the banker's rounding behavior in VBA and also the fact
                // that the DET buffer size varies due to precision issue
                //
                //double tmp1 =  (static_cast<double>(DETBufferSizePoolInKByte) *
                // (ReadBandwidthLuma[NextSurfaceToAssignDETPiece] +
                // ReadBandwidthChroma[NextSurfaceToAssignDETPiece]) /
                // BandwidthOfSurfacesNotAssignedDETPiece /
                // ((ForceSingleDPP ? 1 : DPPPerSurface[NextSurfaceToAssignDETPiece]) * 64.0));
                //double tmp2 =  dml_round(static_cast<double>(DETBufferSizePoolInKByte) *
                // (ReadBandwidthLuma[NextSurfaceToAssignDETPiece] +
                // ReadBandwidthChroma[NextSurfaceToAssignDETPiece]) /
                 //BandwidthOfSurfacesNotAssignedDETPiece /
                // ((ForceSingleDPP ? 1 : DPPPerSurface[NextSurfaceToAssignDETPiece]) * 64.0));
                //
                //;
                //;

                NextDETBufferPieceInKByte = dml_min(
                    dml_round(static_cast<double>(DETBufferSizePoolInKByte) *
                        (ReadBandwidthLuma[NextSurfaceToAssignDETPiece] +
                        ReadBandwidthChroma[NextSurfaceToAssignDETPiece]) /
                        BandwidthOfSurfacesNotAssignedDETPiece /
                        ((ForceSingleDPP ? 1 :
                                DPPPerSurface[NextSurfaceToAssignDETPiece]) * 64.0)) *
                        (ForceSingleDPP ? 1 :
                                DPPPerSurface[NextSurfaceToAssignDETPiece]) * 64.0,
                        dml_floor(static_cast<double>(DETBufferSizePoolInKByte),
                        (ForceSingleDPP ? 1 :
                                DPPPerSurface[NextSurfaceToAssignDETPiece]) * 64.0));

                // Above calculation can assign the entire DET buffer allocation to a single pipe.
                // We should limit the per-pipe DET size to the nominal / max per pipe.
                if (NextDETBufferPieceInKByte > nomDETInKByte * (ForceSingleDPP ? 1 : DPPPerSurface[k])) {
                    if (DETBufferSizeInKByte[NextSurfaceToAssignDETPiece] <
                            nomDETInKByte * (ForceSingleDPP ? 1 : DPPPerSurface[k])) {
                        NextDETBufferPieceInKByte = nomDETInKByte * (ForceSingleDPP ? 1 : DPPPerSurface[k]) -
                                DETBufferSizeInKByte[NextSurfaceToAssignDETPiece];
                    } else {
                        // Case where DETBufferSizeInKByte[NextSurfaceToAssignDETPiece]
                        // already has the max per-pipe value
                        NextDETBufferPieceInKByte = 0;
                    }
                }

                DETBufferSizeInKByte[NextSurfaceToAssignDETPiece] =
                        DETBufferSizeInKByte[NextSurfaceToAssignDETPiece]
                        + NextDETBufferPieceInKByte
                        / (ForceSingleDPP ? 1 : DPPPerSurface[NextSurfaceToAssignDETPiece]);

                DETBufferSizePoolInKByte = DETBufferSizePoolInKByte - NextDETBufferPieceInKByte;
                DETPieceAssignedToThisSurfaceAlready[NextSurfaceToAssignDETPiece] = true;
                BandwidthOfSurfacesNotAssignedDETPiece = BandwidthOfSurfacesNotAssignedDETPiece -
                        (ReadBandwidthLuma[NextSurfaceToAssignDETPiece] +
                                ReadBandwidthChroma[NextSurfaceToAssignDETPiece]);
            }
        }
        *CompressedBufferSizeInkByte = MinCompressedBufferSizeInKByte;
    }
    *CompressedBufferSizeInkByte = *CompressedBufferSizeInkByte * CompressedBufferSegmentSizeInkByteFinal / 64;

} // CalculateDETBufferSize

void dml32_CalculateODMMode(
        unsigned int MaximumPixelsPerLinePerDSCUnit,
        unsigned int HActive,
        enum output_format_class OutFormat,
        enum output_encoder_class Output,
        enum odm_combine_policy ODMUse,
        double StateDispclk,
        double MaxDispclk,
        bool DSCEnable,
        unsigned int TotalNumberOfActiveDPP,
        unsigned int MaxNumDPP,
        double PixelClock,
        double DISPCLKDPPCLKDSCCLKDownSpreading,
        double DISPCLKRampingMargin,
        double DISPCLKDPPCLKVCOSpeed,
        unsigned int NumberOfDSCSlices,

        /* Output */
        bool *TotalAvailablePipesSupport,
        unsigned int *NumberOfDPP,
        enum odm_combine_mode *ODMMode,
        double *RequiredDISPCLKPerSurface)
{

    double SurfaceRequiredDISPCLKWithoutODMCombine;
    double SurfaceRequiredDISPCLKWithODMCombineTwoToOne;
    double SurfaceRequiredDISPCLKWithODMCombineFourToOne;

    SurfaceRequiredDISPCLKWithoutODMCombine = dml32_CalculateRequiredDispclk(dm_odm_combine_mode_disabled,
            PixelClock, DISPCLKDPPCLKDSCCLKDownSpreading, DISPCLKRampingMargin, DISPCLKDPPCLKVCOSpeed,
            MaxDispclk);
    SurfaceRequiredDISPCLKWithODMCombineTwoToOne = dml32_CalculateRequiredDispclk(dm_odm_combine_mode_2to1,
            PixelClock, DISPCLKDPPCLKDSCCLKDownSpreading, DISPCLKRampingMargin, DISPCLKDPPCLKVCOSpeed,
            MaxDispclk);
    SurfaceRequiredDISPCLKWithODMCombineFourToOne = dml32_CalculateRequiredDispclk(dm_odm_combine_mode_4to1,
            PixelClock, DISPCLKDPPCLKDSCCLKDownSpreading, DISPCLKRampingMargin, DISPCLKDPPCLKVCOSpeed,
            MaxDispclk);
    *TotalAvailablePipesSupport = true;
    *ODMMode = dm_odm_combine_mode_disabled; // initialize as disable

    if (ODMUse == dm_odm_combine_policy_none)
        *ODMMode = dm_odm_combine_mode_disabled;

    *RequiredDISPCLKPerSurface = SurfaceRequiredDISPCLKWithoutODMCombine;
    *NumberOfDPP = 0;

    // FIXME check ODMUse == "" condition does it mean bypass or Gabriel means something like don't care??
    // (ODMUse == "" || ODMUse == "CombineAsNeeded")

    if (!(Output == dm_hdmi || Output == dm_dp || Output == dm_edp) && (ODMUse == dm_odm_combine_policy_4to1 ||
            ((SurfaceRequiredDISPCLKWithODMCombineTwoToOne > StateDispclk ||
                    (DSCEnable && (HActive > 2 * MaximumPixelsPerLinePerDSCUnit))
                    || NumberOfDSCSlices > 8)))) {
        if (TotalNumberOfActiveDPP + 4 <= MaxNumDPP) {
            *ODMMode = dm_odm_combine_mode_4to1;
            *RequiredDISPCLKPerSurface = SurfaceRequiredDISPCLKWithODMCombineFourToOne;
            *NumberOfDPP = 4;
        } else {
            *TotalAvailablePipesSupport = false;
        }
    } else if (Output != dm_hdmi && (ODMUse == dm_odm_combine_policy_2to1 ||
            (((SurfaceRequiredDISPCLKWithoutODMCombine > StateDispclk &&
                    SurfaceRequiredDISPCLKWithODMCombineTwoToOne <= StateDispclk) ||
                    (DSCEnable && (HActive > MaximumPixelsPerLinePerDSCUnit))
                    || (NumberOfDSCSlices <= 8 && NumberOfDSCSlices > 4))))) {
        if (TotalNumberOfActiveDPP + 2 <= MaxNumDPP) {
            *ODMMode = dm_odm_combine_mode_2to1;
            *RequiredDISPCLKPerSurface = SurfaceRequiredDISPCLKWithODMCombineTwoToOne;
            *NumberOfDPP = 2;
        } else {
            *TotalAvailablePipesSupport = false;
        }
    } else {
        if (TotalNumberOfActiveDPP + 1 <= MaxNumDPP)
            *NumberOfDPP = 1;
        else
            *TotalAvailablePipesSupport = false;
    }
    if (OutFormat == dm_420 && HActive > DCN32_MAX_FMT_420_BUFFER_WIDTH &&
            ODMUse != dm_odm_combine_policy_4to1) {
        if (HActive > DCN32_MAX_FMT_420_BUFFER_WIDTH * 4) {
            *ODMMode = dm_odm_combine_mode_disabled;
            *NumberOfDPP = 0;
            *TotalAvailablePipesSupport = false;
        } else if (HActive > DCN32_MAX_FMT_420_BUFFER_WIDTH * 2 ||
                *ODMMode == dm_odm_combine_mode_4to1) {
            *ODMMode = dm_odm_combine_mode_4to1;
            *RequiredDISPCLKPerSurface = SurfaceRequiredDISPCLKWithODMCombineFourToOne;
            *NumberOfDPP = 4;
        } else {
            *ODMMode = dm_odm_combine_mode_2to1;
            *RequiredDISPCLKPerSurface = SurfaceRequiredDISPCLKWithODMCombineTwoToOne;
            *NumberOfDPP = 2;
        }
    }
    if (Output == dm_hdmi && OutFormat == dm_420 &&
            HActive > DCN32_MAX_FMT_420_BUFFER_WIDTH) {
        *ODMMode = dm_odm_combine_mode_disabled;
        *NumberOfDPP = 0;
        *TotalAvailablePipesSupport = false;
    }
}

double dml32_CalculateRequiredDispclk(
        enum odm_combine_mode ODMMode,
        double PixelClock,
        double DISPCLKDPPCLKDSCCLKDownSpreading,
        double DISPCLKRampingMargin,
        double DISPCLKDPPCLKVCOSpeed,
        double MaxDispclk)
{
    double RequiredDispclk = 0.;
    double PixelClockAfterODM;
    double DISPCLKWithRampingRoundedToDFSGranularity;
    double DISPCLKWithoutRampingRoundedToDFSGranularity;
    double MaxDispclkRoundedDownToDFSGranularity;

    if (ODMMode == dm_odm_combine_mode_4to1)
        PixelClockAfterODM = PixelClock / 4;
    else if (ODMMode == dm_odm_combine_mode_2to1)
        PixelClockAfterODM = PixelClock / 2;
    else
        PixelClockAfterODM = PixelClock;

    DISPCLKWithRampingRoundedToDFSGranularity = dml32_RoundToDFSGranularity(
            PixelClockAfterODM * (1 + DISPCLKDPPCLKDSCCLKDownSpreading / 100)
                    * (1 + DISPCLKRampingMargin / 100), 1, DISPCLKDPPCLKVCOSpeed);

    DISPCLKWithoutRampingRoundedToDFSGranularity = dml32_RoundToDFSGranularity(
            PixelClockAfterODM * (1 + DISPCLKDPPCLKDSCCLKDownSpreading / 100), 1, DISPCLKDPPCLKVCOSpeed);

    MaxDispclkRoundedDownToDFSGranularity = dml32_RoundToDFSGranularity(MaxDispclk, 0, DISPCLKDPPCLKVCOSpeed);

    if (DISPCLKWithoutRampingRoundedToDFSGranularity > MaxDispclkRoundedDownToDFSGranularity)
        RequiredDispclk = DISPCLKWithoutRampingRoundedToDFSGranularity;
    else if (DISPCLKWithRampingRoundedToDFSGranularity > MaxDispclkRoundedDownToDFSGranularity)
        RequiredDispclk = MaxDispclkRoundedDownToDFSGranularity;
    else
        RequiredDispclk = DISPCLKWithRampingRoundedToDFSGranularity;

    return RequiredDispclk;
}

double dml32_RoundToDFSGranularity(double Clock, bool round_up, double VCOSpeed)
{
    if (Clock <= 0.0)
        return 0.0;

    if (round_up)
        return VCOSpeed * 4.0 / dml_floor(VCOSpeed * 4.0 / Clock, 1.0);
    else
        return VCOSpeed * 4.0 / dml_ceil(VCOSpeed * 4.0 / Clock, 1.0);
}

void dml32_CalculateOutputLink(
        double PHYCLKPerState,
        double /* PHYCLKD18PerState */,
        double PHYCLKD32PerState,
        double Downspreading,
        bool IsMainSurfaceUsingTheIndicatedTiming,
        enum output_encoder_class Output,
        enum output_format_class OutputFormat,
        unsigned int HTotal,
        unsigned int HActive,
        double PixelClockBackEnd,
        double ForcedOutputLinkBPP,
        unsigned int DSCInputBitPerComponent,
        unsigned int NumberOfDSCSlices,
        double AudioSampleRate,
        unsigned int AudioSampleLayout,
        enum odm_combine_mode ODMModeNoDSC,
        enum odm_combine_mode ODMModeDSC,
        bool DSCEnable,
        unsigned int OutputLinkDPLanes,
        enum dm_output_link_dp_rate OutputLinkDPRate,

        /* Output */
        bool *RequiresDSC,
        double *RequiresFEC,
        double  *OutBpp,
        enum dm_output_type *OutputType,
        enum dm_output_rate *OutputRate,
        unsigned int *RequiredSlots)
{
    bool LinkDSCEnable;
    unsigned int dummy;
    *RequiresDSC = false;
    *RequiresFEC = false;
    *OutBpp = 0;
    *OutputType = dm_output_type_unknown;
    *OutputRate = dm_output_rate_unknown;

    if (IsMainSurfaceUsingTheIndicatedTiming) {
        if (Output == dm_hdmi) {
            *RequiresDSC = false;
            *RequiresFEC = false;
            *OutBpp = dml32_TruncToValidBPP(dml_min(600, PHYCLKPerState) * 10, 3, HTotal, HActive,
                    PixelClockBackEnd, ForcedOutputLinkBPP, false, Output, OutputFormat,
                    DSCInputBitPerComponent, NumberOfDSCSlices, AudioSampleRate, AudioSampleLayout,
                    ODMModeNoDSC, ODMModeDSC, &dummy);
            //OutputTypeAndRate = "HDMI";
            *OutputType = dm_output_type_hdmi;

        } else if (Output == dm_dp || Output == dm_dp2p0 || Output == dm_edp) {
            if (DSCEnable == true) {
                *RequiresDSC = true;
                LinkDSCEnable = true;
                if (Output == dm_dp || Output == dm_dp2p0)
                    *RequiresFEC = true;
                else
                    *RequiresFEC = false;
            } else {
                *RequiresDSC = false;
                LinkDSCEnable = false;
                if (Output == dm_dp2p0)
                    *RequiresFEC = true;
                else
                    *RequiresFEC = false;
            }
            if (Output == dm_dp2p0) {
                *OutBpp = 0;
                if ((OutputLinkDPRate == dm_dp_rate_na || OutputLinkDPRate == dm_dp_rate_uhbr10) &&
                        PHYCLKD32PerState >= 10000.0 / 32) {
                    *OutBpp = dml32_TruncToValidBPP((1 - Downspreading / 100) * 10000,
                            OutputLinkDPLanes, HTotal, HActive, PixelClockBackEnd,
                            ForcedOutputLinkBPP, LinkDSCEnable, Output, OutputFormat,
                            DSCInputBitPerComponent, NumberOfDSCSlices, AudioSampleRate,
                            AudioSampleLayout, ODMModeNoDSC, ODMModeDSC, RequiredSlots);
                    if (*OutBpp == 0 && PHYCLKD32PerState < 13500.0 / 32 && DSCEnable == true &&
                            ForcedOutputLinkBPP == 0) {
                        *RequiresDSC = true;
                        LinkDSCEnable = true;
                        *OutBpp = dml32_TruncToValidBPP((1 - Downspreading / 100) * 10000,
                                OutputLinkDPLanes, HTotal, HActive, PixelClockBackEnd,
                                ForcedOutputLinkBPP, LinkDSCEnable, Output,
                                OutputFormat, DSCInputBitPerComponent,
                                NumberOfDSCSlices, AudioSampleRate, AudioSampleLayout,
                                ODMModeNoDSC, ODMModeDSC, RequiredSlots);
                    }
                    //OutputTypeAndRate = Output & " UHBR10";
                    *OutputType = dm_output_type_dp2p0;
                    *OutputRate = dm_output_rate_dp_rate_uhbr10;
                }
                if ((OutputLinkDPRate == dm_dp_rate_na || OutputLinkDPRate == dm_dp_rate_uhbr13p5) &&
                        *OutBpp == 0 && PHYCLKD32PerState >= 13500.0 / 32) {
                    *OutBpp = dml32_TruncToValidBPP((1 - Downspreading / 100) * 13500,
                            OutputLinkDPLanes, HTotal, HActive, PixelClockBackEnd,
                            ForcedOutputLinkBPP, LinkDSCEnable, Output, OutputFormat,
                            DSCInputBitPerComponent, NumberOfDSCSlices, AudioSampleRate,
                            AudioSampleLayout, ODMModeNoDSC, ODMModeDSC, RequiredSlots);

                    if (*OutBpp == 0 && PHYCLKD32PerState < 20000 / 32 && DSCEnable == true &&
                            ForcedOutputLinkBPP == 0) {
                        *RequiresDSC = true;
                        LinkDSCEnable = true;
                        *OutBpp = dml32_TruncToValidBPP((1 - Downspreading / 100) * 13500,
                                OutputLinkDPLanes, HTotal, HActive, PixelClockBackEnd,
                                ForcedOutputLinkBPP, LinkDSCEnable, Output,
                                OutputFormat, DSCInputBitPerComponent,
                                NumberOfDSCSlices, AudioSampleRate, AudioSampleLayout,
                                ODMModeNoDSC, ODMModeDSC, RequiredSlots);
                    }
                    //OutputTypeAndRate = Output & " UHBR13p5";
                    *OutputType = dm_output_type_dp2p0;
                    *OutputRate = dm_output_rate_dp_rate_uhbr13p5;
                }
                if ((OutputLinkDPRate == dm_dp_rate_na || OutputLinkDPRate == dm_dp_rate_uhbr20) &&
                        *OutBpp == 0 && PHYCLKD32PerState >= 20000 / 32) {
                    *OutBpp = dml32_TruncToValidBPP((1 - Downspreading / 100) * 20000,
                            OutputLinkDPLanes, HTotal, HActive, PixelClockBackEnd,
                            ForcedOutputLinkBPP, LinkDSCEnable, Output, OutputFormat,
                            DSCInputBitPerComponent, NumberOfDSCSlices, AudioSampleRate,
                            AudioSampleLayout, ODMModeNoDSC, ODMModeDSC, RequiredSlots);
                    if (*OutBpp == 0 && DSCEnable == true && ForcedOutputLinkBPP == 0) {
                        *RequiresDSC = true;
                        LinkDSCEnable = true;
                        *OutBpp = dml32_TruncToValidBPP((1 - Downspreading / 100) * 20000,
                                OutputLinkDPLanes, HTotal, HActive, PixelClockBackEnd,
                                ForcedOutputLinkBPP, LinkDSCEnable, Output,
                                OutputFormat, DSCInputBitPerComponent,
                                NumberOfDSCSlices, AudioSampleRate, AudioSampleLayout,
                                ODMModeNoDSC, ODMModeDSC, RequiredSlots);
                    }
                    //OutputTypeAndRate = Output & " UHBR20";
                    *OutputType = dm_output_type_dp2p0;
                    *OutputRate = dm_output_rate_dp_rate_uhbr20;
                }
            } else {
                *OutBpp = 0;
                if ((OutputLinkDPRate == dm_dp_rate_na || OutputLinkDPRate == dm_dp_rate_hbr) &&
                        PHYCLKPerState >= 270) {
                    *OutBpp = dml32_TruncToValidBPP((1 - Downspreading / 100) * 2700,
                            OutputLinkDPLanes, HTotal, HActive, PixelClockBackEnd,
                            ForcedOutputLinkBPP, LinkDSCEnable, Output, OutputFormat,
                            DSCInputBitPerComponent, NumberOfDSCSlices, AudioSampleRate,
                            AudioSampleLayout, ODMModeNoDSC, ODMModeDSC, RequiredSlots);
                    if (*OutBpp == 0 && PHYCLKPerState < 540 && DSCEnable == true &&
                            ForcedOutputLinkBPP == 0) {
                        *RequiresDSC = true;
                        LinkDSCEnable = true;
                        if (Output == dm_dp)
                            *RequiresFEC = true;
                        *OutBpp = dml32_TruncToValidBPP((1 - Downspreading / 100) * 2700,
                                OutputLinkDPLanes, HTotal, HActive, PixelClockBackEnd,
                                ForcedOutputLinkBPP, LinkDSCEnable, Output,
                                OutputFormat, DSCInputBitPerComponent,
                                NumberOfDSCSlices, AudioSampleRate, AudioSampleLayout,
                                ODMModeNoDSC, ODMModeDSC, RequiredSlots);
                    }
                    //OutputTypeAndRate = Output & " HBR";
                    *OutputType = (Output == dm_dp) ? dm_output_type_dp : dm_output_type_edp;
                    *OutputRate = dm_output_rate_dp_rate_hbr;
                }
                if ((OutputLinkDPRate == dm_dp_rate_na || OutputLinkDPRate == dm_dp_rate_hbr2) &&
                        *OutBpp == 0 && PHYCLKPerState >= 540) {
                    *OutBpp = dml32_TruncToValidBPP((1 - Downspreading / 100) * 5400,
                            OutputLinkDPLanes, HTotal, HActive, PixelClockBackEnd,
                            ForcedOutputLinkBPP, LinkDSCEnable, Output, OutputFormat,
                            DSCInputBitPerComponent, NumberOfDSCSlices, AudioSampleRate,
                            AudioSampleLayout, ODMModeNoDSC, ODMModeDSC, RequiredSlots);

                    if (*OutBpp == 0 && PHYCLKPerState < 810 && DSCEnable == true &&
                            ForcedOutputLinkBPP == 0) {
                        *RequiresDSC = true;
                        LinkDSCEnable = true;
                        if (Output == dm_dp)
                            *RequiresFEC = true;

                        *OutBpp = dml32_TruncToValidBPP((1 - Downspreading / 100) * 5400,
                                OutputLinkDPLanes, HTotal, HActive, PixelClockBackEnd,
                                ForcedOutputLinkBPP, LinkDSCEnable, Output,
                                OutputFormat, DSCInputBitPerComponent,
                                NumberOfDSCSlices, AudioSampleRate, AudioSampleLayout,
                                ODMModeNoDSC, ODMModeDSC, RequiredSlots);
                    }
                    //OutputTypeAndRate = Output & " HBR2";
                    *OutputType = (Output == dm_dp) ? dm_output_type_dp : dm_output_type_edp;
                    *OutputRate = dm_output_rate_dp_rate_hbr2;
                }
                if ((OutputLinkDPRate == dm_dp_rate_na || OutputLinkDPRate == dm_dp_rate_hbr3) && *OutBpp == 0 && PHYCLKPerState >= 810) {
                    *OutBpp = dml32_TruncToValidBPP((1 - Downspreading / 100) * 8100,
                            OutputLinkDPLanes, HTotal, HActive, PixelClockBackEnd,
                            ForcedOutputLinkBPP, LinkDSCEnable, Output,
                            OutputFormat, DSCInputBitPerComponent, NumberOfDSCSlices,
                            AudioSampleRate, AudioSampleLayout, ODMModeNoDSC, ODMModeDSC,
                            RequiredSlots);

                    if (*OutBpp == 0 && DSCEnable == true && ForcedOutputLinkBPP == 0) {
                        *RequiresDSC = true;
                        LinkDSCEnable = true;
                        if (Output == dm_dp)
                            *RequiresFEC = true;

                        *OutBpp = dml32_TruncToValidBPP((1 - Downspreading / 100) * 8100,
                                OutputLinkDPLanes, HTotal, HActive, PixelClockBackEnd,
                                ForcedOutputLinkBPP, LinkDSCEnable, Output,
                                OutputFormat, DSCInputBitPerComponent,
                                NumberOfDSCSlices, AudioSampleRate, AudioSampleLayout,
                                ODMModeNoDSC, ODMModeDSC, RequiredSlots);
                    }
                    //OutputTypeAndRate = Output & " HBR3";
                    *OutputType = (Output == dm_dp) ? dm_output_type_dp : dm_output_type_edp;
                    *OutputRate = dm_output_rate_dp_rate_hbr3;
                }
            }
        }
    }
}

void dml32_CalculateDPPCLK(
        unsigned int NumberOfActiveSurfaces,
        double DISPCLKDPPCLKDSCCLKDownSpreading,
        double DISPCLKDPPCLKVCOSpeed,
        double DPPCLKUsingSingleDPP[],
        unsigned int DPPPerSurface[],

        /* output */
        double *GlobalDPPCLK,
        double Dppclk[])
{
    unsigned int k;
    *GlobalDPPCLK = 0;
    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        Dppclk[k] = DPPCLKUsingSingleDPP[k] / DPPPerSurface[k] * (1 + DISPCLKDPPCLKDSCCLKDownSpreading / 100);
        *GlobalDPPCLK = dml_max(*GlobalDPPCLK, Dppclk[k]);
    }
    *GlobalDPPCLK = dml32_RoundToDFSGranularity(*GlobalDPPCLK, 1, DISPCLKDPPCLKVCOSpeed);
    for (k = 0; k < NumberOfActiveSurfaces; ++k)
        Dppclk[k] = *GlobalDPPCLK / 255 * dml_ceil(Dppclk[k] * 255.0 / *GlobalDPPCLK, 1.0);
}

double dml32_TruncToValidBPP(
        double LinkBitRate,
        unsigned int Lanes,
        unsigned int /* HTotal */,
        unsigned int /* HActive */,
        double PixelClock,
        double DesiredBPP,
        bool DSCEnable,
        enum output_encoder_class Output,
        enum output_format_class Format,
        unsigned int DSCInputBitPerComponent,
        unsigned int /* DSCSlices */,
        unsigned int /* AudioRate */,
        unsigned int /* AudioLayout */,
        enum odm_combine_mode ODMModeNoDSC,
        enum odm_combine_mode ODMModeDSC,
        /* Output */
        unsigned int *RequiredSlots)
{
    double    MaxLinkBPP;
    unsigned int   MinDSCBPP;
    double    MaxDSCBPP;
    unsigned int   NonDSCBPP0;
    unsigned int   NonDSCBPP1;
    unsigned int   NonDSCBPP2;

    if (Format == dm_420) {
        NonDSCBPP0 = 12;
        NonDSCBPP1 = 15;
        NonDSCBPP2 = 18;
        MinDSCBPP = 6;
        MaxDSCBPP = 1.5 * DSCInputBitPerComponent - 1.0 / 16;
    } else if (Format == dm_444) {
        NonDSCBPP0 = 24;
        NonDSCBPP1 = 30;
        NonDSCBPP2 = 36;
        MinDSCBPP = 8;
        MaxDSCBPP = 3 * DSCInputBitPerComponent - 1.0 / 16;
    } else {
        if (Output == dm_hdmi) {
            NonDSCBPP0 = 24;
            NonDSCBPP1 = 24;
            NonDSCBPP2 = 24;
        } else {
            NonDSCBPP0 = 16;
            NonDSCBPP1 = 20;
            NonDSCBPP2 = 24;
        }
        if (Format == dm_n422) {
            MinDSCBPP = 7;
            MaxDSCBPP = 2 * DSCInputBitPerComponent - 1.0 / 16.0;
        } else {
            MinDSCBPP = 8;
            MaxDSCBPP = 3 * DSCInputBitPerComponent - 1.0 / 16.0;
        }
    }
    if (Output == dm_dp2p0) {
        MaxLinkBPP = LinkBitRate * Lanes / PixelClock * 128 / 132 * 383 / 384 * 65536 / 65540;
    } else if (DSCEnable && Output == dm_dp) {
        MaxLinkBPP = LinkBitRate / 10 * 8 * Lanes / PixelClock * (1 - 2.4 / 100);
    } else {
        MaxLinkBPP = LinkBitRate / 10 * 8 * Lanes / PixelClock;
    }

    if (DSCEnable) {
        if (ODMModeDSC == dm_odm_combine_mode_4to1)
            MaxLinkBPP = dml_min(MaxLinkBPP, 16);
        else if (ODMModeDSC == dm_odm_combine_mode_2to1)
            MaxLinkBPP = dml_min(MaxLinkBPP, 32);
        else if (ODMModeDSC == dm_odm_split_mode_1to2)
            MaxLinkBPP = 2 * MaxLinkBPP;
    } else {
        if (ODMModeNoDSC == dm_odm_combine_mode_4to1)
            MaxLinkBPP = dml_min(MaxLinkBPP, 16);
        else if (ODMModeNoDSC == dm_odm_combine_mode_2to1)
            MaxLinkBPP = dml_min(MaxLinkBPP, 32);
        else if (ODMModeNoDSC == dm_odm_split_mode_1to2)
            MaxLinkBPP = 2 * MaxLinkBPP;
    }

    *RequiredSlots = dml_ceil(DesiredBPP / MaxLinkBPP * 64, 1);

    if (DesiredBPP == 0) {
        if (DSCEnable) {
            if (MaxLinkBPP < MinDSCBPP)
                return BPP_INVALID;
            else if (MaxLinkBPP >= MaxDSCBPP)
                return MaxDSCBPP;
            else
                return dml_floor(16.0 * MaxLinkBPP, 1.0) / 16.0;
        } else {
            if (MaxLinkBPP >= NonDSCBPP2)
                return NonDSCBPP2;
            else if (MaxLinkBPP >= NonDSCBPP1)
                return NonDSCBPP1;
            else if (MaxLinkBPP >= NonDSCBPP0)
                return 16.0;
            else
                return BPP_INVALID;
        }
    } else {
        if (!((DSCEnable == false && (DesiredBPP == NonDSCBPP2 || DesiredBPP == NonDSCBPP1 ||
                DesiredBPP <= NonDSCBPP0)) ||
                (DSCEnable && DesiredBPP >= MinDSCBPP && DesiredBPP <= MaxDSCBPP)))
            return BPP_INVALID;
        else
            return DesiredBPP;
    }
} // TruncToValidBPP

double dml32_RequiredDTBCLK(
        bool              DSCEnable,
        double               PixelClock,
        enum output_format_class  OutputFormat,
        double               OutputBpp,
        unsigned int              DSCSlices,
        unsigned int                 HTotal,
        unsigned int                 HActive,
        unsigned int              AudioRate,
        unsigned int              AudioLayout)
{
    double PixelWordRate;
    double HCActive;
    double HCBlank;
    double AverageTribyteRate;
    double HActiveTribyteRate;

    if (DSCEnable != true)
        return dml_max(PixelClock / 4.0 * OutputBpp / 24.0, 25.0);

    PixelWordRate = PixelClock /  (OutputFormat == dm_444 ? 1 : 2);
    HCActive = dml_ceil(DSCSlices * dml_ceil(OutputBpp *
            dml_ceil(HActive / DSCSlices, 1) / 8.0, 1) / 3.0, 1);
    HCBlank = 64 + 32 *
            dml_ceil(AudioRate *  (AudioLayout == 1 ? 1 : 0.25) * HTotal / (PixelClock * 1000), 1);
    AverageTribyteRate = PixelWordRate * (HCActive + HCBlank) / HTotal;
    HActiveTribyteRate = PixelWordRate * HCActive / HActive;
    return dml_max4(PixelWordRate / 4.0, AverageTribyteRate / 4.0, HActiveTribyteRate / 4.0, 25.0) * 1.002;
}

unsigned int dml32_DSCDelayRequirement(bool DSCEnabled,
        enum odm_combine_mode ODMMode,
        unsigned int DSCInputBitPerComponent,
        double OutputBpp,
        unsigned int HActive,
        unsigned int HTotal,
        unsigned int NumberOfDSCSlices,
        enum output_format_class  OutputFormat,
        enum output_encoder_class Output,
        double PixelClock,
        double PixelClockBackEnd,
        double dsc_delay_factor_wa)
{
    unsigned int DSCDelayRequirement_val;

    if (DSCEnabled == true && OutputBpp != 0) {
        if (ODMMode == dm_odm_combine_mode_4to1) {
            DSCDelayRequirement_val = 4 * (dml32_dscceComputeDelay(DSCInputBitPerComponent, OutputBpp,
                    dml_ceil(HActive / NumberOfDSCSlices, 1), NumberOfDSCSlices / 4,
                    OutputFormat, Output) + dml32_dscComputeDelay(OutputFormat, Output));
        } else if (ODMMode == dm_odm_combine_mode_2to1) {
            DSCDelayRequirement_val = 2 * (dml32_dscceComputeDelay(DSCInputBitPerComponent, OutputBpp,
                    dml_ceil(HActive / NumberOfDSCSlices, 1), NumberOfDSCSlices / 2,
                    OutputFormat, Output) + dml32_dscComputeDelay(OutputFormat, Output));
        } else {
            DSCDelayRequirement_val = dml32_dscceComputeDelay(DSCInputBitPerComponent, OutputBpp,
                    dml_ceil(HActive / NumberOfDSCSlices, 1), NumberOfDSCSlices,
                    OutputFormat, Output) + dml32_dscComputeDelay(OutputFormat, Output);
        }

        DSCDelayRequirement_val = DSCDelayRequirement_val + (HTotal - HActive) *
                dml_ceil(static_cast<double>(DSCDelayRequirement_val) / HActive, 1);

        DSCDelayRequirement_val = DSCDelayRequirement_val * PixelClock / PixelClockBackEnd;

    } else {
        DSCDelayRequirement_val = 0;
    }

    return dml_ceil(DSCDelayRequirement_val * dsc_delay_factor_wa, 1);
}

void dml32_CalculateSurfaceSizeInMall(
        unsigned int NumberOfActiveSurfaces,
        unsigned int MALLAllocatedForDCN,
        enum dm_use_mall_for_static_screen_mode UseMALLForStaticScreen[],
        enum dm_use_mall_for_pstate_change_mode UsesMALLForPStateChange[],
        bool DCCEnable[],
        bool ViewportStationary[],
        unsigned int ViewportXStartY[],
        unsigned int ViewportYStartY[],
        unsigned int ViewportXStartC[],
        unsigned int ViewportYStartC[],
        unsigned int ViewportWidthY[],
        unsigned int ViewportHeightY[],
        unsigned int BytesPerPixelY[],
        unsigned int ViewportWidthC[],
        unsigned int ViewportHeightC[],
        unsigned int BytesPerPixelC[],
        unsigned int SurfaceWidthY[],
        unsigned int SurfaceWidthC[],
        unsigned int SurfaceHeightY[],
        unsigned int SurfaceHeightC[],
        unsigned int Read256BytesBlockWidthY[],
        unsigned int Read256BytesBlockWidthC[],
        unsigned int Read256BytesBlockHeightY[],
        unsigned int Read256BytesBlockHeightC[],
        unsigned int ReadBlockWidthY[],
        unsigned int ReadBlockWidthC[],
        unsigned int ReadBlockHeightY[],
        unsigned int ReadBlockHeightC[],
        unsigned int DCCMetaPitchY[],
        unsigned int DCCMetaPitchC[],

        /* Output */
        unsigned int    SurfaceSizeInMALL[],
        bool *ExceededMALLSize)
{
    unsigned int k;
    unsigned int TotalSurfaceSizeInMALLForSS = 0;
    unsigned int TotalSurfaceSizeInMALLForSubVP = 0;
    unsigned int MALLAllocatedForDCNInBytes = MALLAllocatedForDCN * 1024 * 1024;

    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        if (ViewportStationary[k]) {
            SurfaceSizeInMALL[k] = dml_min(dml_ceil(SurfaceWidthY[k], ReadBlockWidthY[k]),
                    dml_floor(ViewportXStartY[k] + ViewportWidthY[k] + ReadBlockWidthY[k] - 1,
                        ReadBlockWidthY[k]) - dml_floor(ViewportXStartY[k],
                        ReadBlockWidthY[k])) * dml_min(dml_ceil(SurfaceHeightY[k],
                        ReadBlockHeightY[k]), dml_floor(ViewportYStartY[k] +
                        ViewportHeightY[k] + ReadBlockHeightY[k] - 1, ReadBlockHeightY[k]) -
                        dml_floor(ViewportYStartY[k], ReadBlockHeightY[k])) * BytesPerPixelY[k];

            if (ReadBlockWidthC[k] > 0) {
                SurfaceSizeInMALL[k] = SurfaceSizeInMALL[k] +
                        dml_min(dml_ceil(SurfaceWidthC[k], ReadBlockWidthC[k]),
                            dml_floor(ViewportXStartC[k] + ViewportWidthC[k] +
                            ReadBlockWidthC[k] - 1, ReadBlockWidthC[k]) -
                            dml_floor(ViewportXStartC[k], ReadBlockWidthC[k])) *
                            dml_min(dml_ceil(SurfaceHeightC[k], ReadBlockHeightC[k]),
                            dml_floor(ViewportYStartC[k] + ViewportHeightC[k] +
                            ReadBlockHeightC[k] - 1, ReadBlockHeightC[k]) -
                            dml_floor(ViewportYStartC[k], ReadBlockHeightC[k])) *
                            BytesPerPixelC[k];
            }
            if (DCCEnable[k] == true) {
                SurfaceSizeInMALL[k] = SurfaceSizeInMALL[k] +
                        (dml_min(dml_ceil(DCCMetaPitchY[k], 8 * Read256BytesBlockWidthY[k]),
                            dml_floor(ViewportXStartY[k] + ViewportWidthY[k] + 8 *
                            Read256BytesBlockWidthY[k] - 1, 8 * Read256BytesBlockWidthY[k])
                            - dml_floor(ViewportXStartY[k], 8 * Read256BytesBlockWidthY[k]))
                            * dml_min(dml_ceil(SurfaceHeightY[k], 8 *
                            Read256BytesBlockHeightY[k]), dml_floor(ViewportYStartY[k] +
                            ViewportHeightY[k] + 8 * Read256BytesBlockHeightY[k] - 1, 8 *
                            Read256BytesBlockHeightY[k]) - dml_floor(ViewportYStartY[k], 8 *
                            Read256BytesBlockHeightY[k])) * BytesPerPixelY[k] / 256) + (64 * 1024);
                if (Read256BytesBlockWidthC[k] > 0) {
                    SurfaceSizeInMALL[k] = SurfaceSizeInMALL[k] +
                            dml_min(dml_ceil(DCCMetaPitchC[k], 8 *
                                Read256BytesBlockWidthC[k]),
                                dml_floor(ViewportXStartC[k] + ViewportWidthC[k] + 8
                                * Read256BytesBlockWidthC[k] - 1, 8 *
                                Read256BytesBlockWidthC[k]) -
                                dml_floor(ViewportXStartC[k], 8 *
                                Read256BytesBlockWidthC[k])) *
                                dml_min(dml_ceil(SurfaceHeightC[k], 8 *
                                Read256BytesBlockHeightC[k]),
                                dml_floor(ViewportYStartC[k] + ViewportHeightC[k] +
                                8 * Read256BytesBlockHeightC[k] - 1, 8 *
                                Read256BytesBlockHeightC[k]) -
                                dml_floor(ViewportYStartC[k], 8 *
                                Read256BytesBlockHeightC[k])) *
                                BytesPerPixelC[k] / 256;
                }
            }
        } else {
            SurfaceSizeInMALL[k] = dml_ceil(dml_min(SurfaceWidthY[k], ViewportWidthY[k] +
                    ReadBlockWidthY[k] - 1), ReadBlockWidthY[k]) *
                    dml_ceil(dml_min(SurfaceHeightY[k], ViewportHeightY[k] +
                            ReadBlockHeightY[k] - 1), ReadBlockHeightY[k]) *
                            BytesPerPixelY[k];
            if (ReadBlockWidthC[k] > 0) {
                SurfaceSizeInMALL[k] = SurfaceSizeInMALL[k] +
                        dml_ceil(dml_min(SurfaceWidthC[k], ViewportWidthC[k] +
                                ReadBlockWidthC[k] - 1), ReadBlockWidthC[k]) *
                        dml_ceil(dml_min(SurfaceHeightC[k], ViewportHeightC[k] +
                                ReadBlockHeightC[k] - 1), ReadBlockHeightC[k]) *
                                BytesPerPixelC[k];
            }
            if (DCCEnable[k] == true) {
                SurfaceSizeInMALL[k] = SurfaceSizeInMALL[k] +
                        (dml_ceil(dml_min(DCCMetaPitchY[k], ViewportWidthY[k] + 8 *
                                Read256BytesBlockWidthY[k] - 1), 8 *
                                Read256BytesBlockWidthY[k]) *
                        dml_ceil(dml_min(SurfaceHeightY[k], ViewportHeightY[k] + 8 *
                                Read256BytesBlockHeightY[k] - 1), 8 *
                                Read256BytesBlockHeightY[k]) * BytesPerPixelY[k] / 256) + (64 * 1024);

                if (Read256BytesBlockWidthC[k] > 0) {
                    SurfaceSizeInMALL[k] = SurfaceSizeInMALL[k] +
                            dml_ceil(dml_min(DCCMetaPitchC[k], ViewportWidthC[k] + 8 *
                                    Read256BytesBlockWidthC[k] - 1), 8 *
                                    Read256BytesBlockWidthC[k]) *
                            dml_ceil(dml_min(SurfaceHeightC[k], ViewportHeightC[k] + 8 *
                                    Read256BytesBlockHeightC[k] - 1), 8 *
                                    Read256BytesBlockHeightC[k]) *
                                    BytesPerPixelC[k] / 256;
                }
            }
        }
    }

    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        /* SS and Subvp counted separate as they are never used at the same time */
        if (UsesMALLForPStateChange[k] == dm_use_mall_pstate_change_phantom_pipe)
            TotalSurfaceSizeInMALLForSubVP = TotalSurfaceSizeInMALLForSubVP + SurfaceSizeInMALL[k];
        else if (UseMALLForStaticScreen[k] == dm_use_mall_static_screen_enable)
            TotalSurfaceSizeInMALLForSS = TotalSurfaceSizeInMALLForSS + SurfaceSizeInMALL[k];
    }
    *ExceededMALLSize =  (TotalSurfaceSizeInMALLForSS > MALLAllocatedForDCNInBytes) ||
                            (TotalSurfaceSizeInMALLForSubVP > MALLAllocatedForDCNInBytes);
} // CalculateSurfaceSizeInMall

} // namespace rx7900::display::dml32

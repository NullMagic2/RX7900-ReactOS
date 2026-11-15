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
Module: rx7900.display.dcn32_dml_program
Purpose: DML32 mode programming: clocks, prefetch schedule, watermarks and stutter.
Created: 2026-10-24 (adaptation; the upstream notice above is preserved)
Architecture: Part of the bounded C++17 port of the Linux v6.12 DML32 model behind
src/display/dcn32_bandwidth (one stream, up to four pipes). Identifiers, statement order and
arithmetic (including the float-typed dml_min/max/floor/ceil helpers and every
int/unsigned conversion) deliberately follow the pinned upstream text so each
function can be compared line by line with work/upstream-dcn-stream; this is a
documented exception to the snake_case rule, as in src/opengl/gfx11_registers.hpp.
Per-plane and per-pipe arrays hold the four DCN 3.2 pipes (DML32_NUM_PLANES; Linux
sizes them DC__NUM_DPP__MAX = 8) and per-state arrays DML32_NUM_STATES entries; debug printing is removed.
Upstream: dml/dcn32/display_mode_vba_32.c (dml32_recalculate and its programming pass).
*/
#include "dcn32_dml_util.hpp"

namespace rx7900::display::dml32 {
void dml32_recalculate(struct display_mode_lib *mode_lib);
static void DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation(
        struct display_mode_lib *mode_lib);
void dml32_ModeSupportAndSystemConfigurationFull(struct display_mode_lib *mode_lib);

void dml32_recalculate(struct display_mode_lib *mode_lib)
{
    ModeSupportAndSystemConfiguration(mode_lib);

    dml32_CalculateMaxDETAndMinCompressedBufferSize(mode_lib->vba.ConfigReturnBufferSizeInKByte,
            mode_lib->vba.ROBBufferSizeInKByte,
            DC__NUM_DPP,
            false, //mode_lib->vba.override_setting.nomDETInKByteOverrideEnable,
            0, //mode_lib->vba.override_setting.nomDETInKByteOverrideValue,

            /* Output */
            &mode_lib->vba.MaxTotalDETInKByte, &mode_lib->vba.nomDETInKByte,
            &mode_lib->vba.MinCompressedBufferSizeInKByte);

    PixelClockAdjustmentForProgressiveToInterlaceUnit(mode_lib);
    DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation(mode_lib);
}


// DISPCLK/DPPCLK, swath/DET, deep sleep, DSC delay, MALL size and PTE rows.
static DML32_NOINLINE void dml32_program_clocks_swath_and_rows(struct display_mode_lib *mode_lib)
{
    struct vba_vars_st *v = &mode_lib->vba;
    unsigned int j, k;

    v->WritebackDISPCLK = 0.0;
    v->GlobalDPPCLK = 0.0;

    // DISPCLK and DPPCLK Calculation (writeback is never enabled: WritebackDISPCLK stays 0)
    v->DISPCLK_calculated = v->WritebackDISPCLK;

    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        if (mode_lib->vba.BlendingAndTiming[k] == k) {
            v->DISPCLK_calculated = dml_max(v->DISPCLK_calculated,
                    dml32_CalculateRequiredDispclk(
                            mode_lib->vba.ODMCombineEnabled[k],
                            mode_lib->vba.PixelClock[k],
                            mode_lib->vba.DISPCLKDPPCLKDSCCLKDownSpreading,
                            mode_lib->vba.DISPCLKRampingMargin,
                            mode_lib->vba.DISPCLKDPPCLKVCOSpeed,
                            mode_lib->vba.MaxDppclk[v->soc.num_states - 1]));
        }
    }

    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        dml32_CalculateSinglePipeDPPCLKAndSCLThroughput(mode_lib->vba.HRatio[k],
                mode_lib->vba.HRatioChroma[k],
                mode_lib->vba.VRatio[k],
                mode_lib->vba.VRatioChroma[k],
                mode_lib->vba.MaxDCHUBToPSCLThroughput,
                mode_lib->vba.MaxPSCLToLBThroughput,
                mode_lib->vba.PixelClock[k],
                mode_lib->vba.SourcePixelFormat[k],
                mode_lib->vba.htaps[k],
                mode_lib->vba.HTAPsChroma[k],
                mode_lib->vba.vtaps[k],
                mode_lib->vba.VTAPsChroma[k],

                /* Output */
                &v->PSCL_THROUGHPUT_LUMA[k], &v->PSCL_THROUGHPUT_CHROMA[k],
                &v->DPPCLKUsingSingleDPP[k]);
    }

    dml32_CalculateDPPCLK(mode_lib->vba.NumberOfActiveSurfaces, mode_lib->vba.DISPCLKDPPCLKDSCCLKDownSpreading,
            mode_lib->vba.DISPCLKDPPCLKVCOSpeed, v->DPPCLKUsingSingleDPP, dml_uint(mode_lib->vba.DPPPerPlane),
            /* Output */
            &v->GlobalDPPCLK, v->DPPCLK);

    for (k = 0; k < v->NumberOfActiveSurfaces; ++k) {
        v->DPPCLK_calculated[k] = v->DPPCLK[k];
    }

    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        dml32_CalculateBytePerPixelAndBlockSizes(
                mode_lib->vba.SourcePixelFormat[k],
                mode_lib->vba.SurfaceTiling[k],

                /* Output */
                &v->BytePerPixelY[k],
                &v->BytePerPixelC[k],
                &v->BytePerPixelDETY[k],
                &v->BytePerPixelDETC[k],
                &v->BlockHeight256BytesY[k],
                &v->BlockHeight256BytesC[k],
                &v->BlockWidth256BytesY[k],
                &v->BlockWidth256BytesC[k],
                &v->BlockHeightY[k],
                &v->BlockHeightC[k],
                &v->BlockWidthY[k],
                &v->BlockWidthC[k]);
    }

    dml32_CalculateSwathWidth(
            false,  // ForceSingleDPP
            mode_lib->vba.NumberOfActiveSurfaces,
            mode_lib->vba.SourcePixelFormat,
            mode_lib->vba.SourceRotation,
            mode_lib->vba.ViewportStationary,
            mode_lib->vba.ViewportWidth,
            mode_lib->vba.ViewportHeight,
            mode_lib->vba.ViewportXStartY,
            mode_lib->vba.ViewportYStartY,
            mode_lib->vba.ViewportXStartC,
            mode_lib->vba.ViewportYStartC,
            mode_lib->vba.SurfaceWidthY,
            mode_lib->vba.SurfaceWidthC,
            mode_lib->vba.SurfaceHeightY,
            mode_lib->vba.SurfaceHeightC,
            mode_lib->vba.ODMCombineEnabled,
            v->BytePerPixelY,
            v->BytePerPixelC,
            v->BlockHeight256BytesY,
            v->BlockHeight256BytesC,
            v->BlockWidth256BytesY,
            v->BlockWidth256BytesC,
            mode_lib->vba.BlendingAndTiming,
            mode_lib->vba.HActive,
            mode_lib->vba.HRatio,
            dml_uint(mode_lib->vba.DPPPerPlane),

            /* Output */
            v->SwathWidthSingleDPPY, v->SwathWidthSingleDPPC, v->SwathWidthY, v->SwathWidthC,
            v->dummy_vars
                .DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation
                .dummy_integer_array[0], // Integer             MaximumSwathHeightY[]
            v->dummy_vars
                .DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation
                .dummy_integer_array[1], // Integer             MaximumSwathHeightC[]
            v->swath_width_luma_ub, v->swath_width_chroma_ub);

    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        v->ReadBandwidthSurfaceLuma[k] = v->SwathWidthSingleDPPY[k] * v->BytePerPixelY[k]
                / (mode_lib->vba.HTotal[k] / mode_lib->vba.PixelClock[k]) * mode_lib->vba.VRatio[k];
        v->ReadBandwidthSurfaceChroma[k] = v->SwathWidthSingleDPPC[k] * v->BytePerPixelC[k]
                / (mode_lib->vba.HTotal[k] / mode_lib->vba.PixelClock[k])
                * mode_lib->vba.VRatioChroma[k];
    }

    {
        // VBA_DELTA
        // Calculate DET size, swath height
        dml32_CalculateSwathAndDETConfiguration(
                mode_lib->vba.DETSizeOverride,
                mode_lib->vba.UsesMALLForPStateChange,
                mode_lib->vba.ConfigReturnBufferSizeInKByte,
                mode_lib->vba.MaxTotalDETInKByte,
                mode_lib->vba.MinCompressedBufferSizeInKByte,
                false, /* ForceSingleDPP */
                mode_lib->vba.NumberOfActiveSurfaces,
                mode_lib->vba.nomDETInKByte,
                mode_lib->vba.UseUnboundedRequesting,
                mode_lib->vba.DisableUnboundRequestIfCompBufReservedSpaceNeedAdjustment,
                mode_lib->vba.ip.pixel_chunk_size_kbytes,
                mode_lib->vba.ip.rob_buffer_size_kbytes,
                mode_lib->vba.CompressedBufferSegmentSizeInkByteFinal,
                v->dummy_vars
                    .DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation
                    .dummy_output_encoder_array, /* output_encoder_class Output[] */
                v->ReadBandwidthSurfaceLuma,
                v->ReadBandwidthSurfaceChroma,
                v->dummy_vars
                    .DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation
                    .dummy_single_array[0], /* Single MaximumSwathWidthLuma[] */
                v->dummy_vars
                    .DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation
                    .dummy_single_array[1], /* Single MaximumSwathWidthChroma[] */
                mode_lib->vba.SourceRotation,
                mode_lib->vba.ViewportStationary,
                mode_lib->vba.SourcePixelFormat,
                mode_lib->vba.SurfaceTiling,
                mode_lib->vba.ViewportWidth,
                mode_lib->vba.ViewportHeight,
                mode_lib->vba.ViewportXStartY,
                mode_lib->vba.ViewportYStartY,
                mode_lib->vba.ViewportXStartC,
                mode_lib->vba.ViewportYStartC,
                mode_lib->vba.SurfaceWidthY,
                mode_lib->vba.SurfaceWidthC,
                mode_lib->vba.SurfaceHeightY,
                mode_lib->vba.SurfaceHeightC,
                v->BlockHeight256BytesY,
                v->BlockHeight256BytesC,
                v->BlockWidth256BytesY,
                v->BlockWidth256BytesC,
                mode_lib->vba.ODMCombineEnabled,
                mode_lib->vba.BlendingAndTiming,
                v->BytePerPixelY,
                v->BytePerPixelC,
                v->BytePerPixelDETY,
                v->BytePerPixelDETC,
                mode_lib->vba.HActive,
                mode_lib->vba.HRatio,
                mode_lib->vba.HRatioChroma,
                dml_uint(mode_lib->vba.DPPPerPlane),

                /* Output */
                v->dummy_vars
                    .DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation
                    .dummy_long_array[0], /* Long swath_width_luma_ub[] */
                v->dummy_vars
                    .DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation
                    .dummy_long_array[1], /* Long swath_width_chroma_ub[] */
                v->dummy_vars
                    .DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation
                    .dummy_double_array[0], /* Long SwathWidth[] */
                v->dummy_vars
                    .DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation
                    .dummy_double_array[1], /* Long SwathWidthChroma[] */
                mode_lib->vba.SwathHeightY,
                mode_lib->vba.SwathHeightC,
                mode_lib->vba.DETBufferSizeInKByte,
                mode_lib->vba.DETBufferSizeY,
                mode_lib->vba.DETBufferSizeC,
                &v->UnboundedRequestEnabled,
                dml_uint(&v->CompressedBufferSizeInkByte),
                &v->CompBufReservedSpaceKBytes,
                &v->dummy_vars
                    .DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation
                    .dummy_boolean,       /* bool *CompBufReservedSpaceNeedAjustment */
                v->dummy_vars
                    .DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation
                    .dummy_boolean_array, /* bool ViewportSizeSupportPerSurface[] */
                &v->dummy_vars
                     .DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation
                     .dummy_boolean); /* bool *ViewportSizeSupport */
    }

    v->CompBufReservedSpaceZs     = v->CompBufReservedSpaceKBytes * 1024.0 / 256.0;
    v->CompBufReservedSpace64B    = v->CompBufReservedSpaceKBytes * 1024.0 / 64.0;

    // DCFCLK Deep Sleep
    dml32_CalculateDCFCLKDeepSleep(
            mode_lib->vba.NumberOfActiveSurfaces,
            v->BytePerPixelY,
            v->BytePerPixelC,
            mode_lib->vba.VRatio,
            mode_lib->vba.VRatioChroma,
            v->SwathWidthY,
            v->SwathWidthC,
            dml_uint(mode_lib->vba.DPPPerPlane),
            mode_lib->vba.HRatio,
            mode_lib->vba.HRatioChroma,
            mode_lib->vba.PixelClock,
            v->PSCL_THROUGHPUT_LUMA,
            v->PSCL_THROUGHPUT_CHROMA,
            mode_lib->vba.DPPCLK,
            v->ReadBandwidthSurfaceLuma,
            v->ReadBandwidthSurfaceChroma,
            mode_lib->vba.ReturnBusWidth,

            /* Output */
            &v->DCFCLKDeepSleep);

    // DSCCLK
    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        if ((mode_lib->vba.BlendingAndTiming[k] != k) || !mode_lib->vba.DSCEnabled[k]) {
            v->DSCCLK_calculated[k] = 0.0;
        } else {
            if (mode_lib->vba.OutputFormat[k] == dm_420)
                mode_lib->vba.DSCFormatFactor = 2;
            else if (mode_lib->vba.OutputFormat[k] == dm_444)
                mode_lib->vba.DSCFormatFactor = 1;
            else if (mode_lib->vba.OutputFormat[k] == dm_n422)
                mode_lib->vba.DSCFormatFactor = 2;
            else
                mode_lib->vba.DSCFormatFactor = 1;
            if (mode_lib->vba.ODMCombineEnabled[k] == dm_odm_combine_mode_4to1)
                v->DSCCLK_calculated[k] = mode_lib->vba.PixelClockBackEnd[k] / 12
                        / mode_lib->vba.DSCFormatFactor
                        / (1 - mode_lib->vba.DISPCLKDPPCLKDSCCLKDownSpreading / 100);
            else if (mode_lib->vba.ODMCombineEnabled[k] == dm_odm_combine_mode_2to1)
                v->DSCCLK_calculated[k] = mode_lib->vba.PixelClockBackEnd[k] / 6
                        / mode_lib->vba.DSCFormatFactor
                        / (1 - mode_lib->vba.DISPCLKDPPCLKDSCCLKDownSpreading / 100);
            else
                v->DSCCLK_calculated[k] = mode_lib->vba.PixelClockBackEnd[k] / 3
                        / mode_lib->vba.DSCFormatFactor
                        / (1 - mode_lib->vba.DISPCLKDPPCLKDSCCLKDownSpreading / 100);
        }
    }

    // DSC Delay
    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        v->DSCDelay[k] = dml32_DSCDelayRequirement(mode_lib->vba.DSCEnabled[k],
                mode_lib->vba.ODMCombineEnabled[k], mode_lib->vba.DSCInputBitPerComponent[k],
                mode_lib->vba.OutputBppPerState[mode_lib->vba.VoltageLevel][k],
                mode_lib->vba.HActive[k], mode_lib->vba.HTotal[k],
                mode_lib->vba.NumberOfDSCSlices[k], mode_lib->vba.OutputFormat[k],
                mode_lib->vba.Output[k], mode_lib->vba.PixelClock[k],
                mode_lib->vba.PixelClockBackEnd[k], mode_lib->vba.ip.dsc_delay_factor_wa);
    }

    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k)
        for (j = 0; j < mode_lib->vba.NumberOfActiveSurfaces; ++j) // NumberOfSurfaces
            if (j != k && mode_lib->vba.BlendingAndTiming[k] == j && mode_lib->vba.DSCEnabled[j])
                v->DSCDelay[k] = v->DSCDelay[j];

    //Immediate Flip
    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        v->ImmediateFlipSupportedSurface[k] = mode_lib->vba.ImmediateFlipSupport
                && (mode_lib->vba.ImmediateFlipRequirement[k] != dm_immediate_flip_not_required);
    }

    // Prefetch
    dml32_CalculateSurfaceSizeInMall(
                mode_lib->vba.NumberOfActiveSurfaces,
                mode_lib->vba.MALLAllocatedForDCNFinal,
                mode_lib->vba.UseMALLForStaticScreen,
                mode_lib->vba.UsesMALLForPStateChange,
                mode_lib->vba.DCCEnable,
                mode_lib->vba.ViewportStationary,
                mode_lib->vba.ViewportXStartY,
                mode_lib->vba.ViewportYStartY,
                mode_lib->vba.ViewportXStartC,
                mode_lib->vba.ViewportYStartC,
                mode_lib->vba.ViewportWidth,
                mode_lib->vba.ViewportHeight,
                v->BytePerPixelY,
                mode_lib->vba.ViewportWidthChroma,
                mode_lib->vba.ViewportHeightChroma,
                v->BytePerPixelC,
                mode_lib->vba.SurfaceWidthY,
                mode_lib->vba.SurfaceWidthC,
                mode_lib->vba.SurfaceHeightY,
                mode_lib->vba.SurfaceHeightC,
                v->BlockWidth256BytesY,
                v->BlockWidth256BytesC,
                v->BlockHeight256BytesY,
                v->BlockHeight256BytesC,
                v->BlockWidthY,
                v->BlockWidthC,
                v->BlockHeightY,
                v->BlockHeightC,
                mode_lib->vba.DCCMetaPitchY,
                mode_lib->vba.DCCMetaPitchC,

                /* Output */
                v->SurfaceSizeInMALL,
                &v->dummy_vars.
                DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation
                .dummy_boolean2); /* Boolean *ExceededMALLSize */

    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].PixelClock = mode_lib->vba.PixelClock[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].DPPPerSurface = mode_lib->vba.DPPPerPlane[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].SourceRotation = mode_lib->vba.SourceRotation[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].ViewportHeight = mode_lib->vba.ViewportHeight[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].ViewportHeightChroma = mode_lib->vba.ViewportHeightChroma[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].BlockWidth256BytesY = v->BlockWidth256BytesY[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].BlockHeight256BytesY = v->BlockHeight256BytesY[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].BlockWidth256BytesC = v->BlockWidth256BytesC[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].BlockHeight256BytesC = v->BlockHeight256BytesC[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].BlockWidthY = v->BlockWidthY[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].BlockHeightY = v->BlockHeightY[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].BlockWidthC = v->BlockWidthC[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].BlockHeightC = v->BlockHeightC[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].InterlaceEnable = mode_lib->vba.Interlace[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].HTotal = mode_lib->vba.HTotal[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].DCCEnable = mode_lib->vba.DCCEnable[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].SourcePixelFormat = mode_lib->vba.SourcePixelFormat[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].SurfaceTiling = mode_lib->vba.SurfaceTiling[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].BytePerPixelY = v->BytePerPixelY[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].BytePerPixelC = v->BytePerPixelC[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].ProgressiveToInterlaceUnitInOPP = mode_lib->vba.ProgressiveToInterlaceUnitInOPP;
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].VRatio = mode_lib->vba.VRatio[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].VRatioChroma = mode_lib->vba.VRatioChroma[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].VTaps = mode_lib->vba.vtaps[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].VTapsChroma = mode_lib->vba.VTAPsChroma[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].PitchY = mode_lib->vba.PitchY[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].DCCMetaPitchY = mode_lib->vba.DCCMetaPitchY[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].PitchC = mode_lib->vba.PitchC[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].DCCMetaPitchC = mode_lib->vba.DCCMetaPitchC[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].ViewportStationary = mode_lib->vba.ViewportStationary[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].ViewportXStart = mode_lib->vba.ViewportXStartY[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].ViewportYStart = mode_lib->vba.ViewportYStartY[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].ViewportXStartC = mode_lib->vba.ViewportXStartC[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].ViewportYStartC = mode_lib->vba.ViewportYStartC[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].FORCE_ONE_ROW_FOR_FRAME = mode_lib->vba.ForceOneRowForFrame[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].SwathHeightY = mode_lib->vba.SwathHeightY[k];
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters[k].SwathHeightC = mode_lib->vba.SwathHeightC[k];
    }

    {

        dml32_CalculateVMRowAndSwath(
                mode_lib->vba.NumberOfActiveSurfaces,
                v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.SurfaceParameters,
                v->SurfaceSizeInMALL,
                mode_lib->vba.PTEBufferSizeInRequestsLuma,
                mode_lib->vba.PTEBufferSizeInRequestsChroma,
                mode_lib->vba.DCCMetaBufferSizeBytes,
                mode_lib->vba.UseMALLForStaticScreen,
                mode_lib->vba.UsesMALLForPStateChange,
                mode_lib->vba.MALLAllocatedForDCNFinal,
                v->SwathWidthY,
                v->SwathWidthC,
                mode_lib->vba.GPUVMEnable,
                mode_lib->vba.HostVMEnable,
                mode_lib->vba.HostVMMaxNonCachedPageTableLevels,
                mode_lib->vba.GPUVMMaxPageTableLevels,
                mode_lib->vba.GPUVMMinPageSizeKBytes,
                mode_lib->vba.HostVMMinPageSize,

                /* Output */
                v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.dummy_boolean_array2[0],  // Boolean PTEBufferSizeNotExceeded[]
                v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.dummy_boolean_array2[1],  // Boolean DCCMetaBufferSizeNotExceeded[]
                v->dpte_row_width_luma_ub,
                v->dpte_row_width_chroma_ub,
                v->dpte_row_height,
                v->dpte_row_height_chroma,
                v->dpte_row_height_linear,
                v->dpte_row_height_linear_chroma,
                v->meta_req_width,
                v->meta_req_width_chroma,
                v->meta_req_height,
                v->meta_req_height_chroma,
                v->meta_row_width,
                v->meta_row_width_chroma,
                v->meta_row_height,
                v->meta_row_height_chroma,
                v->vm_group_bytes,
                v->dpte_group_bytes,
                v->PixelPTEReqWidthY,
                v->PixelPTEReqHeightY,
                v->PTERequestSizeY,
                v->PixelPTEReqWidthC,
                v->PixelPTEReqHeightC,
                v->PTERequestSizeC,
                v->dpde0_bytes_per_frame_ub_l,
                v->meta_pte_bytes_per_frame_ub_l,
                v->dpde0_bytes_per_frame_ub_c,
                v->meta_pte_bytes_per_frame_ub_c,
                v->PrefetchSourceLinesY,
                v->PrefetchSourceLinesC,
                v->VInitPreFillY, v->VInitPreFillC,
                v->MaxNumSwathY,
                v->MaxNumSwathC,
                v->meta_row_bw,
                v->dpte_row_bw,
                v->PixelPTEBytesPerRow,
                v->PDEAndMetaPTEBytesFrame,
                v->MetaRowByte,
                v->Use_One_Row_For_Frame,
                v->Use_One_Row_For_Frame_Flip,
                v->UsesMALLForStaticScreen,
                v->PTE_BUFFER_MODE,
                v->BIGK_FRAGMENT_SIZE);
    }

}

// Extra latency, writeback delay, urgent latency/burst factors and VStartup bounds.
static DML32_NOINLINE void dml32_program_latencies(struct display_mode_lib *mode_lib)
{
    struct vba_vars_st *v = &mode_lib->vba;
    unsigned int j, k;

    v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.ReorderBytes = mode_lib->vba.NumberOfChannels
            * dml_max3(mode_lib->vba.UrgentOutOfOrderReturnPerChannelPixelDataOnly,
                    mode_lib->vba.UrgentOutOfOrderReturnPerChannelPixelMixedWithVMData,
                    mode_lib->vba.UrgentOutOfOrderReturnPerChannelVMDataOnly);

    v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.VMDataOnlyReturnBW = dml32_get_return_bw_mbps_vm_only(
            &mode_lib->vba.soc,
            mode_lib->vba.VoltageLevel,
            mode_lib->vba.DCFCLK,
            mode_lib->vba.FabricClock,
            mode_lib->vba.DRAMSpeed);

    v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.HostVMInefficiencyFactor = 1.0;

    if (mode_lib->vba.GPUVMEnable && mode_lib->vba.HostVMEnable)
        v->dummy_vars
            .DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation
            .HostVMInefficiencyFactor =
            mode_lib->vba.ReturnBW / v->dummy_vars
                .DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation
                .VMDataOnlyReturnBW;

    mode_lib->vba.TotalDCCActiveDPP = 0;
    mode_lib->vba.TotalActiveDPP = 0;
    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        mode_lib->vba.TotalActiveDPP = mode_lib->vba.TotalActiveDPP + mode_lib->vba.DPPPerPlane[k];
        if (mode_lib->vba.DCCEnable[k])
            mode_lib->vba.TotalDCCActiveDPP = mode_lib->vba.TotalDCCActiveDPP
                    + mode_lib->vba.DPPPerPlane[k];
    }

    v->UrgentExtraLatency = dml32_CalculateExtraLatency(
            mode_lib->vba.RoundTripPingLatencyCycles,
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.ReorderBytes,
            mode_lib->vba.DCFCLK,
            mode_lib->vba.TotalActiveDPP,
            mode_lib->vba.PixelChunkSizeInKByte,
            mode_lib->vba.TotalDCCActiveDPP,
            mode_lib->vba.MetaChunkSize,
            mode_lib->vba.ReturnBW,
            mode_lib->vba.GPUVMEnable,
            mode_lib->vba.HostVMEnable,
            mode_lib->vba.NumberOfActiveSurfaces,
            dml_uint(mode_lib->vba.DPPPerPlane),
            v->dpte_group_bytes,
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.HostVMInefficiencyFactor,
            mode_lib->vba.HostVMMinPageSize,
            mode_lib->vba.HostVMMaxNonCachedPageTableLevels);

    mode_lib->vba.TCalc = 24.0 / v->DCFCLKDeepSleep;

    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        if (mode_lib->vba.BlendingAndTiming[k] == k) {
            v->WritebackDelay[mode_lib->vba.VoltageLevel][k] = 0;
        }
    }

    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k)
        for (j = 0; j < mode_lib->vba.NumberOfActiveSurfaces; ++j)
            if (mode_lib->vba.BlendingAndTiming[k] == j)
                v->WritebackDelay[mode_lib->vba.VoltageLevel][k] =
                        v->WritebackDelay[mode_lib->vba.VoltageLevel][j];

    v->UrgentLatency = dml32_CalculateUrgentLatency(mode_lib->vba.UrgentLatencyPixelDataOnly,
            mode_lib->vba.UrgentLatencyPixelMixedWithVMData,
            mode_lib->vba.UrgentLatencyVMDataOnly,
            mode_lib->vba.DoUrgentLatencyAdjustment,
            mode_lib->vba.UrgentLatencyAdjustmentFabricClockComponent,
            mode_lib->vba.UrgentLatencyAdjustmentFabricClockReference,
            mode_lib->vba.FabricClock);

    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        dml32_CalculateUrgentBurstFactor(mode_lib->vba.UsesMALLForPStateChange[k],
                v->swath_width_luma_ub[k],
                v->swath_width_chroma_ub[k],
                mode_lib->vba.SwathHeightY[k],
                mode_lib->vba.SwathHeightC[k],
                mode_lib->vba.HTotal[k] / mode_lib->vba.PixelClock[k],
                v->UrgentLatency,
                mode_lib->vba.CursorBufferSize,
                mode_lib->vba.CursorWidth[k][0],
                mode_lib->vba.CursorBPP[k][0],
                mode_lib->vba.VRatio[k],
                mode_lib->vba.VRatioChroma[k],
                v->BytePerPixelDETY[k],
                v->BytePerPixelDETC[k],
                mode_lib->vba.DETBufferSizeY[k],
                mode_lib->vba.DETBufferSizeC[k],

                /* output */
                &v->UrgBurstFactorCursor[k],
                &v->UrgBurstFactorLuma[k],
                &v->UrgBurstFactorChroma[k],
                &v->NoUrgentLatencyHiding[k]);

        v->cursor_bw[k] = mode_lib->vba.NumberOfCursors[k] * mode_lib->vba.CursorWidth[k][0] * mode_lib->vba.CursorBPP[k][0] / 8 / (mode_lib->vba.HTotal[k] / mode_lib->vba.PixelClock[k]) * mode_lib->vba.VRatio[k];
    }

    v->NotEnoughDETSwathFillLatencyHiding = dml32_CalculateDETSwathFillLatencyHiding(
                        mode_lib->vba.NumberOfActiveSurfaces,
                        mode_lib->vba.ReturnBW,
                        v->UrgentLatency,
                        mode_lib->vba.SwathHeightY,
                        mode_lib->vba.SwathHeightC,
                        v->swath_width_luma_ub,
                        v->swath_width_chroma_ub,
                        v->BytePerPixelDETY,
                        v->BytePerPixelDETC,
                        mode_lib->vba.DETBufferSizeY,
                        mode_lib->vba.DETBufferSizeC,
                        dml_uint(mode_lib->vba.DPPPerPlane),
                        mode_lib->vba.HTotal,
                        mode_lib->vba.PixelClock,
                        mode_lib->vba.VRatio,
                        mode_lib->vba.VRatioChroma,
                        mode_lib->vba.UsesMALLForPStateChange,
                        mode_lib->vba.UseUnboundedRequesting);

    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        v->MaxVStartupLines[k] = ((mode_lib->vba.Interlace[k] &&
                !mode_lib->vba.ProgressiveToInterlaceUnitInOPP) ?
                dml_floor((mode_lib->vba.VTotal[k] - mode_lib->vba.VActive[k]) / 2.0, 1.0) :
                mode_lib->vba.VTotal[k] - mode_lib->vba.VActive[k]) - dml_max(1.0,
                dml_ceil(static_cast<double>(v->WritebackDelay[mode_lib->vba.VoltageLevel][k])
                / (mode_lib->vba.HTotal[k] / mode_lib->vba.PixelClock[k]), 1));

        // Clamp to max OTG vstartup register limit
        if (v->MaxVStartupLines[k] > 1023)
            v->MaxVStartupLines[k] = 1023;

    }

}

// Prefetch schedule iteration over VStartup and the prefetch modes.
static DML32_NOINLINE void dml32_program_prefetch(struct display_mode_lib *mode_lib)
{
    struct vba_vars_st *v = &mode_lib->vba;
    unsigned int k;
    bool ImmediateFlipRequirementFinal;
    int iteration;
    double MaxTotalRDBandwidth;
    unsigned int NextPrefetchMode;
    double MaxTotalRDBandwidthNoUrgentBurst = 0.0;
    bool DestinationLineTimesForPrefetchLessThan2 = false;
    bool VRatioPrefetchMoreThanMax = false;
    double TWait;

    v->MaximumMaxVStartupLines = 0;
    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k)
        v->MaximumMaxVStartupLines = dml_max(v->MaximumMaxVStartupLines, v->MaxVStartupLines[k]);

    ImmediateFlipRequirementFinal = false;

    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        ImmediateFlipRequirementFinal = ImmediateFlipRequirementFinal
                || (mode_lib->vba.ImmediateFlipRequirement[k] == dm_immediate_flip_required);
    }
    // ModeProgramming will not repeat the schedule calculation using different prefetch mode,
    //it is just calcualated once with given prefetch mode
    dml32_CalculateMinAndMaxPrefetchMode(
            mode_lib->vba.AllowForPStateChangeOrStutterInVBlankFinal,
            &mode_lib->vba.MinPrefetchMode,
            &mode_lib->vba.MaxPrefetchMode);

    v->VStartupLines = __DML_VBA_MIN_VSTARTUP__;

    iteration = 0;
    MaxTotalRDBandwidth = 0;
    NextPrefetchMode = mode_lib->vba.PrefetchModePerState[mode_lib->vba.VoltageLevel][mode_lib->vba.maxMpcComb];

    do {
        MaxTotalRDBandwidth = 0;
        DestinationLineTimesForPrefetchLessThan2 = false;
        VRatioPrefetchMoreThanMax = false;
        for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
            /* NOTE PerfetchMode variable is invalid in DAL as per the input received.
             * Hence the direction is to use PrefetchModePerState.
             */
            TWait = dml32_CalculateTWait(
                mode_lib->vba.PrefetchModePerState[mode_lib->vba.VoltageLevel][mode_lib->vba.maxMpcComb],
                mode_lib->vba.UsesMALLForPStateChange[k],
                mode_lib->vba.SynchronizeDRRDisplaysForUCLKPStateChangeFinal,
                mode_lib->vba.DRRDisplay[k],
                mode_lib->vba.DRAMClockChangeLatency,
                mode_lib->vba.FCLKChangeLatency, v->UrgentLatency,
                mode_lib->vba.SREnterPlusExitTime);

            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.myPipe = DmlPipe{};

            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.myPipe.Dppclk = mode_lib->vba.DPPCLK[k];
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.myPipe.Dispclk = mode_lib->vba.DISPCLK;
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.myPipe.PixelClock = mode_lib->vba.PixelClock[k];
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.myPipe.DCFClkDeepSleep = v->DCFCLKDeepSleep;
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.myPipe.DPPPerSurface = mode_lib->vba.DPPPerPlane[k];
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.myPipe.ScalerEnabled = mode_lib->vba.ScalerEnabled[k];
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.myPipe.SourceRotation = mode_lib->vba.SourceRotation[k];
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.myPipe.BlockWidth256BytesY = v->BlockWidth256BytesY[k];
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.myPipe.BlockHeight256BytesY = v->BlockHeight256BytesY[k];
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.myPipe.BlockWidth256BytesC = v->BlockWidth256BytesC[k];
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.myPipe.BlockHeight256BytesC = v->BlockHeight256BytesC[k];
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.myPipe.InterlaceEnable = mode_lib->vba.Interlace[k];
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.myPipe.NumberOfCursors = mode_lib->vba.NumberOfCursors[k];
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.myPipe.VBlank = mode_lib->vba.VTotal[k] - mode_lib->vba.VActive[k];
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.myPipe.HTotal = mode_lib->vba.HTotal[k];
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.myPipe.HActive = mode_lib->vba.HActive[k];
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.myPipe.DCCEnable = mode_lib->vba.DCCEnable[k];
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.myPipe.ODMMode = mode_lib->vba.ODMCombineEnabled[k];
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.myPipe.SourcePixelFormat = mode_lib->vba.SourcePixelFormat[k];
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.myPipe.BytePerPixelY = v->BytePerPixelY[k];
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.myPipe.BytePerPixelC = v->BytePerPixelC[k];
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.myPipe.ProgressiveToInterlaceUnitInOPP = mode_lib->vba.ProgressiveToInterlaceUnitInOPP;
            v->ErrorResult[k] = dml32_CalculatePrefetchSchedule(
                    v,
                    k,
                    v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.HostVMInefficiencyFactor,
                    &v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.myPipe,
                    v->DSCDelay[k],
                    static_cast<unsigned int>(v->SwathWidthY[k] / v->HRatio[k]),
                    dml_min(v->VStartupLines, v->MaxVStartupLines[k]),
                    v->MaxVStartupLines[k],
                    v->UrgentLatency,
                    v->UrgentExtraLatency,
                    v->TCalc,
                    v->PDEAndMetaPTEBytesFrame[k],
                    v->MetaRowByte[k],
                    v->PixelPTEBytesPerRow[k],
                    v->PrefetchSourceLinesY[k],
                    v->SwathWidthY[k],
                    v->VInitPreFillY[k],
                    v->MaxNumSwathY[k],
                    v->PrefetchSourceLinesC[k],
                    v->SwathWidthC[k],
                    v->VInitPreFillC[k],
                    v->MaxNumSwathC[k],
                    v->swath_width_luma_ub[k],
                    v->swath_width_chroma_ub[k],
                    v->SwathHeightY[k],
                    v->SwathHeightC[k],
                    TWait,
                    (v->DRAMSpeedPerState[mode_lib->vba.VoltageLevel] <= MEM_STROBE_FREQ_MHZ ||
                        v->DCFCLKPerState[mode_lib->vba.VoltageLevel] <= DCFCLK_FREQ_EXTRA_PREFETCH_REQ_MHZ) ?
                            mode_lib->vba.ip.min_prefetch_in_strobe_us : 0,
                    mode_lib->vba.PrefetchModePerState[mode_lib->vba.VoltageLevel][mode_lib->vba.maxMpcComb] > 0 || mode_lib->vba.DRAMClockChangeRequirementFinal == false,

                    /* Output */
                    &v->DSTXAfterScaler[k],
                    &v->DSTYAfterScaler[k],
                    &v->DestinationLinesForPrefetch[k],
                    &v->PrefetchBandwidth[k],
                    &v->DestinationLinesToRequestVMInVBlank[k],
                    &v->DestinationLinesToRequestRowInVBlank[k],
                    &v->VRatioPrefetchY[k],
                    &v->VRatioPrefetchC[k],
                    &v->RequiredPrefetchPixDataBWLuma[k],
                    &v->RequiredPrefetchPixDataBWChroma[k],
                    &v->NotEnoughTimeForDynamicMetadata[k],
                    &v->Tno_bw[k], &v->prefetch_vmrow_bw[k],
                    &v->Tdmdl_vm[k],
                    &v->Tdmdl[k],
                    &v->TSetup[k],
                    &v->VUpdateOffsetPix[k],
                    &v->VUpdateWidthPix[k],
                    &v->VReadyOffsetPix[k]);

            v->VStartup[k] = dml_min(v->VStartupLines, v->MaxVStartupLines[k]);
        }

        for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
            dml32_CalculateUrgentBurstFactor(mode_lib->vba.UsesMALLForPStateChange[k],
                    v->swath_width_luma_ub[k],
                    v->swath_width_chroma_ub[k],
                    mode_lib->vba.SwathHeightY[k],
                    mode_lib->vba.SwathHeightC[k],
                    mode_lib->vba.HTotal[k] / mode_lib->vba.PixelClock[k],
                    v->UrgentLatency,
                    mode_lib->vba.CursorBufferSize,
                    mode_lib->vba.CursorWidth[k][0],
                    mode_lib->vba.CursorBPP[k][0],
                    v->VRatioPrefetchY[k],
                    v->VRatioPrefetchC[k],
                    v->BytePerPixelDETY[k],
                    v->BytePerPixelDETC[k],
                    mode_lib->vba.DETBufferSizeY[k],
                    mode_lib->vba.DETBufferSizeC[k],
                    /* Output */
                    &v->UrgBurstFactorCursorPre[k],
                    &v->UrgBurstFactorLumaPre[k],
                    &v->UrgBurstFactorChromaPre[k],
                    &v->NoUrgentLatencyHidingPre[k]);

            v->cursor_bw_pre[k] = mode_lib->vba.NumberOfCursors[k] * mode_lib->vba.CursorWidth[k][0] * mode_lib->vba.CursorBPP[k][0] /
                    8.0 / (mode_lib->vba.HTotal[k] / mode_lib->vba.PixelClock[k]) * v->VRatioPrefetchY[k];

            if (v->DestinationLinesForPrefetch[k] < 2)
                DestinationLineTimesForPrefetchLessThan2 = true;

            if (v->VRatioPrefetchY[k] > v->MaxVRatioPre
                    || v->VRatioPrefetchC[k] > v->MaxVRatioPre)
                VRatioPrefetchMoreThanMax = true;

            //bool DestinationLinesToRequestVMInVBlankEqualOrMoreThan32 = false;
            //bool DestinationLinesToRequestRowInVBlankEqualOrMoreThan16 = false;
            //if (v->DestinationLinesToRequestVMInVBlank[k] >= 32) {
            //    DestinationLinesToRequestVMInVBlankEqualOrMoreThan32 = true;
            //}

            //if (v->DestinationLinesToRequestRowInVBlank[k] >= 16) {
            //    DestinationLinesToRequestRowInVBlankEqualOrMoreThan16 = true;
            //}
        }

        v->FractionOfUrgentBandwidth = MaxTotalRDBandwidthNoUrgentBurst / mode_lib->vba.ReturnBW;

        {
            dml32_CalculatePrefetchBandwithSupport(
                    mode_lib->vba.NumberOfActiveSurfaces,
                    mode_lib->vba.ReturnBW,
                    v->NoUrgentLatencyHidingPre,
                    v->ReadBandwidthSurfaceLuma,
                    v->ReadBandwidthSurfaceChroma,
                    v->RequiredPrefetchPixDataBWLuma,
                    v->RequiredPrefetchPixDataBWChroma,
                    v->cursor_bw,
                    v->meta_row_bw,
                    v->dpte_row_bw,
                    v->cursor_bw_pre,
                    v->prefetch_vmrow_bw,
                    dml_uint(mode_lib->vba.DPPPerPlane),
                    v->UrgBurstFactorLuma,
                    v->UrgBurstFactorChroma,
                    v->UrgBurstFactorCursor,
                    v->UrgBurstFactorLumaPre,
                    v->UrgBurstFactorChromaPre,
                    v->UrgBurstFactorCursorPre,
                    v->PrefetchBandwidth,
                    v->VRatio,
                    v->MaxVRatioPre,

                    /* output */
                    &MaxTotalRDBandwidth,
                    &v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.dummy_single[0],
                    &v->PrefetchModeSupported);
        }

        for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k)
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.dummy_unit_vector[k] = 1.0;

        {
            dml32_CalculatePrefetchBandwithSupport(mode_lib->vba.NumberOfActiveSurfaces,
                    mode_lib->vba.ReturnBW,
                    v->NoUrgentLatencyHidingPre,
                    v->ReadBandwidthSurfaceLuma,
                    v->ReadBandwidthSurfaceChroma,
                    v->RequiredPrefetchPixDataBWLuma,
                    v->RequiredPrefetchPixDataBWChroma,
                    v->cursor_bw,
                    v->meta_row_bw,
                    v->dpte_row_bw,
                    v->cursor_bw_pre,
                    v->prefetch_vmrow_bw,
                    dml_uint(mode_lib->vba.DPPPerPlane),
                    v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.dummy_unit_vector,
                    v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.dummy_unit_vector,
                    v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.dummy_unit_vector,
                    v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.dummy_unit_vector,
                    v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.dummy_unit_vector,
                    v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.dummy_unit_vector,
                    v->PrefetchBandwidth,
                    v->VRatio,
                    v->MaxVRatioPre,

                    /* output */
                    &v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.dummy_single[0],
                    &v->FractionOfUrgentBandwidth,
                    &v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.dummy_boolean);
        }

        if (VRatioPrefetchMoreThanMax != false || DestinationLineTimesForPrefetchLessThan2 != false) {
            v->PrefetchModeSupported = false;
        }

        for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
            if (v->ErrorResult[k] == true || v->NotEnoughTimeForDynamicMetadata[k]) {
                v->PrefetchModeSupported = false;
            }
        }

        if (v->PrefetchModeSupported == true && mode_lib->vba.ImmediateFlipSupport == true) {
            mode_lib->vba.BandwidthAvailableForImmediateFlip = dml32_CalculateBandwidthAvailableForImmediateFlip(
                    mode_lib->vba.NumberOfActiveSurfaces,
                    mode_lib->vba.ReturnBW,
                    v->ReadBandwidthSurfaceLuma,
                    v->ReadBandwidthSurfaceChroma,
                    v->RequiredPrefetchPixDataBWLuma,
                    v->RequiredPrefetchPixDataBWChroma,
                    v->cursor_bw,
                    v->cursor_bw_pre,
                    dml_uint(mode_lib->vba.DPPPerPlane),
                    v->UrgBurstFactorLuma,
                    v->UrgBurstFactorChroma,
                    v->UrgBurstFactorCursor,
                    v->UrgBurstFactorLumaPre,
                    v->UrgBurstFactorChromaPre,
                    v->UrgBurstFactorCursorPre);

            mode_lib->vba.TotImmediateFlipBytes = 0;
            for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
                if (mode_lib->vba.ImmediateFlipRequirement[k] != dm_immediate_flip_not_required) {
                    mode_lib->vba.TotImmediateFlipBytes = mode_lib->vba.TotImmediateFlipBytes
                            + mode_lib->vba.DPPPerPlane[k]
                                    * (v->PDEAndMetaPTEBytesFrame[k]
                                            + v->MetaRowByte[k]);
                    if (v->use_one_row_for_frame_flip[k][0][0]) {
                        mode_lib->vba.TotImmediateFlipBytes =
                                mode_lib->vba.TotImmediateFlipBytes
                                        + 2 * v->PixelPTEBytesPerRow[k];
                    } else {
                        mode_lib->vba.TotImmediateFlipBytes =
                                mode_lib->vba.TotImmediateFlipBytes
                                        + v->PixelPTEBytesPerRow[k];
                    }
                }
            }
            for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
                dml32_CalculateFlipSchedule(v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.HostVMInefficiencyFactor,
                        v->UrgentExtraLatency,
                        v->UrgentLatency,
                        mode_lib->vba.GPUVMMaxPageTableLevels,
                        mode_lib->vba.HostVMEnable,
                        mode_lib->vba.HostVMMaxNonCachedPageTableLevels,
                        mode_lib->vba.GPUVMEnable,
                        mode_lib->vba.HostVMMinPageSize,
                        v->PDEAndMetaPTEBytesFrame[k],
                        v->MetaRowByte[k],
                        v->PixelPTEBytesPerRow[k],
                        mode_lib->vba.BandwidthAvailableForImmediateFlip,
                        mode_lib->vba.TotImmediateFlipBytes,
                        mode_lib->vba.SourcePixelFormat[k],
                        mode_lib->vba.HTotal[k] / mode_lib->vba.PixelClock[k],
                        mode_lib->vba.VRatio[k],
                        mode_lib->vba.VRatioChroma[k],
                        v->Tno_bw[k],
                        mode_lib->vba.DCCEnable[k],
                        v->dpte_row_height[k],
                        v->meta_row_height[k],
                        v->dpte_row_height_chroma[k],
                        v->meta_row_height_chroma[k],
                        v->Use_One_Row_For_Frame_Flip[k],

                        /* Output */
                        &v->DestinationLinesToRequestVMInImmediateFlip[k],
                        &v->DestinationLinesToRequestRowInImmediateFlip[k],
                        &v->final_flip_bw[k],
                        &v->ImmediateFlipSupportedForPipe[k]);
            }

            {
                dml32_CalculateImmediateFlipBandwithSupport(mode_lib->vba.NumberOfActiveSurfaces,
                        mode_lib->vba.ReturnBW,
                        mode_lib->vba.ImmediateFlipRequirement,
                        v->final_flip_bw,
                        v->ReadBandwidthSurfaceLuma,
                        v->ReadBandwidthSurfaceChroma,
                        v->RequiredPrefetchPixDataBWLuma,
                        v->RequiredPrefetchPixDataBWChroma,
                        v->cursor_bw,
                        v->meta_row_bw,
                        v->dpte_row_bw,
                        v->cursor_bw_pre,
                        v->prefetch_vmrow_bw,
                        dml_uint(mode_lib->vba.DPPPerPlane),
                        v->UrgBurstFactorLuma,
                        v->UrgBurstFactorChroma,
                        v->UrgBurstFactorCursor,
                        v->UrgBurstFactorLumaPre,
                        v->UrgBurstFactorChromaPre,
                        v->UrgBurstFactorCursorPre,

                        /* output */
                        &v->total_dcn_read_bw_with_flip,    // Single  *TotalBandwidth
                        &v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.dummy_single[0],                        // Single  *FractionOfUrgentBandwidth
                        &v->ImmediateFlipSupported);        // Boolean *ImmediateFlipBandwidthSupport

                dml32_CalculateImmediateFlipBandwithSupport(mode_lib->vba.NumberOfActiveSurfaces,
                        mode_lib->vba.ReturnBW,
                        mode_lib->vba.ImmediateFlipRequirement,
                        v->final_flip_bw,
                        v->ReadBandwidthSurfaceLuma,
                        v->ReadBandwidthSurfaceChroma,
                        v->RequiredPrefetchPixDataBWLuma,
                        v->RequiredPrefetchPixDataBWChroma,
                        v->cursor_bw,
                        v->meta_row_bw,
                        v->dpte_row_bw,
                        v->cursor_bw_pre,
                        v->prefetch_vmrow_bw,
                        dml_uint(mode_lib->vba.DPPPerPlane),
                        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.dummy_unit_vector,
                        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.dummy_unit_vector,
                        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.dummy_unit_vector,
                        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.dummy_unit_vector,
                        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.dummy_unit_vector,
                        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.dummy_unit_vector,

                        /* output */
                        &v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.dummy_single[1],                                // Single  *TotalBandwidth
                        &v->FractionOfUrgentBandwidthImmediateFlip, // Single  *FractionOfUrgentBandwidth
                        &v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.dummy_boolean);                              // Boolean *ImmediateFlipBandwidthSupport
            }

            for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
                if (mode_lib->vba.ImmediateFlipRequirement[k] != dm_immediate_flip_not_required && v->ImmediateFlipSupportedForPipe[k] == false) {
                    v->ImmediateFlipSupported = false;
                }
            }
        } else {
            v->ImmediateFlipSupported = false;
        }

        /* consider flip support is okay if the flip bw is ok or (when user does't require a iflip and there is no host vm) */
        v->PrefetchAndImmediateFlipSupported = (v->PrefetchModeSupported == true &&
                ((!mode_lib->vba.ImmediateFlipSupport && !mode_lib->vba.HostVMEnable && !ImmediateFlipRequirementFinal) ||
                        v->ImmediateFlipSupported)) ? true : false;

        v->VStartupLines = v->VStartupLines + 1;

        if (v->VStartupLines > v->MaximumMaxVStartupLines) {
            break; // VBA_DELTA: Implementation divergence! Gabe is *still* iterating across prefetch modes which we don't care to do
        }
        iteration++;
        if (iteration > 2500) {
        }
    } while (!(v->PrefetchAndImmediateFlipSupported || NextPrefetchMode > mode_lib->vba.MaxPrefetchMode));

    if (v->VStartupLines <= v->MaximumMaxVStartupLines) {
    }

    //Watermarks and NB P-State/DRAM Clock Change Support
}

// Watermarks, delivery times, TTU/meta/VM timing and stutter efficiency.
static DML32_NOINLINE void dml32_program_watermarks_and_stutter(struct display_mode_lib *mode_lib)
{
    struct vba_vars_st *v = &mode_lib->vba;
    unsigned int k;
    double TotalWRBandwidth = 0;
    double WRBandwidth = 0;

    {
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.mmSOCParameters.UrgentLatency = v->UrgentLatency;
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.mmSOCParameters.ExtraLatency = v->UrgentExtraLatency;
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.mmSOCParameters.WritebackLatency = mode_lib->vba.WritebackLatency;
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.mmSOCParameters.DRAMClockChangeLatency = mode_lib->vba.DRAMClockChangeLatency;
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.mmSOCParameters.FCLKChangeLatency = mode_lib->vba.FCLKChangeLatency;
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.mmSOCParameters.SRExitTime = mode_lib->vba.SRExitTime;
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.mmSOCParameters.SREnterPlusExitTime = mode_lib->vba.SREnterPlusExitTime;
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.mmSOCParameters.SRExitZ8Time = mode_lib->vba.SRExitZ8Time;
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.mmSOCParameters.SREnterPlusExitZ8Time = mode_lib->vba.SREnterPlusExitZ8Time;
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.mmSOCParameters.USRRetrainingLatency = mode_lib->vba.USRRetrainingLatency;
        v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.mmSOCParameters.SMNLatency = mode_lib->vba.SMNLatency;

        dml32_CalculateWatermarksMALLUseAndDRAMSpeedChangeSupport(
            v,
            v->PrefetchModePerState[v->VoltageLevel][v->maxMpcComb],
            v->DCFCLK,
            v->ReturnBW,
            v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.mmSOCParameters,
            v->SOCCLK,
            v->DCFCLKDeepSleep,
            v->DETBufferSizeY,
            v->DETBufferSizeC,
            v->SwathHeightY,
            v->SwathHeightC,
            v->SwathWidthY,
            v->SwathWidthC,
            dml_uint(v->DPPPerPlane),
            v->BytePerPixelDETY,
            v->BytePerPixelDETC,
            v->DSTXAfterScaler,
            v->DSTYAfterScaler,
            v->UnboundedRequestEnabled,
            v->CompressedBufferSizeInkByte,

            /* Output */
            &v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.dummy_dramchange_support,
            v->MaxActiveDRAMClockChangeLatencySupported,
            v->SubViewportLinesNeededInMALL,
            &v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.dummy_fclkchange_support,
            &v->MinActiveFCLKChangeLatencySupported,
            &v->dummy_vars.DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation.dummy_USRRetrainingSupport,
            mode_lib->vba.ActiveDRAMClockChangeLatencyMargin);

        /* DCN32 has a new struct Watermarks (typedef) which is used to store
         * calculated WM values. Copy over values from struct to vba varaibles
         * to ensure that the DCN32 getters return the correct value.
         */
        v->UrgentWatermark = v->Watermark.UrgentWatermark;
        v->WritebackUrgentWatermark = v->Watermark.WritebackUrgentWatermark;
        v->DRAMClockChangeWatermark = v->Watermark.DRAMClockChangeWatermark;
        v->WritebackDRAMClockChangeWatermark = v->Watermark.WritebackDRAMClockChangeWatermark;
        v->StutterExitWatermark = v->Watermark.StutterExitWatermark;
        v->StutterEnterPlusExitWatermark = v->Watermark.StutterEnterPlusExitWatermark;
        v->Z8StutterExitWatermark = v->Watermark.Z8StutterExitWatermark;
        v->Z8StutterEnterPlusExitWatermark = v->Watermark.Z8StutterEnterPlusExitWatermark;

        for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
            v->WritebackAllowDRAMClockChangeEndPosition[k] = 0;
            v->WritebackAllowFCLKChangeEndPosition[k] = 0;
        }
    }

    //Display Pipeline Delivery Time in Prefetch, Groups
    dml32_CalculatePixelDeliveryTimes(
            mode_lib->vba.NumberOfActiveSurfaces,
            mode_lib->vba.VRatio,
            mode_lib->vba.VRatioChroma,
            v->VRatioPrefetchY,
            v->VRatioPrefetchC,
            v->swath_width_luma_ub,
            v->swath_width_chroma_ub,
            dml_uint(mode_lib->vba.DPPPerPlane),
            mode_lib->vba.HRatio,
            mode_lib->vba.HRatioChroma,
            mode_lib->vba.PixelClock,
            v->PSCL_THROUGHPUT_LUMA,
            v->PSCL_THROUGHPUT_CHROMA,
            mode_lib->vba.DPPCLK,
            v->BytePerPixelC,
            mode_lib->vba.SourceRotation,
            mode_lib->vba.NumberOfCursors,
            mode_lib->vba.CursorWidth,
            mode_lib->vba.CursorBPP,
            v->BlockWidth256BytesY,
            v->BlockHeight256BytesY,
            v->BlockWidth256BytesC,
            v->BlockHeight256BytesC,

            /* Output */
            v->DisplayPipeLineDeliveryTimeLuma,
            v->DisplayPipeLineDeliveryTimeChroma,
            v->DisplayPipeLineDeliveryTimeLumaPrefetch,
            v->DisplayPipeLineDeliveryTimeChromaPrefetch,
            v->DisplayPipeRequestDeliveryTimeLuma,
            v->DisplayPipeRequestDeliveryTimeChroma,
            v->DisplayPipeRequestDeliveryTimeLumaPrefetch,
            v->DisplayPipeRequestDeliveryTimeChromaPrefetch,
            v->CursorRequestDeliveryTime,
            v->CursorRequestDeliveryTimePrefetch);

    dml32_CalculateMetaAndPTETimes(v->Use_One_Row_For_Frame,
            mode_lib->vba.NumberOfActiveSurfaces,
            mode_lib->vba.GPUVMEnable,
            mode_lib->vba.MetaChunkSize,
            mode_lib->vba.MinMetaChunkSizeBytes,
            mode_lib->vba.HTotal,
            mode_lib->vba.VRatio,
            mode_lib->vba.VRatioChroma,
            v->DestinationLinesToRequestRowInVBlank,
            v->DestinationLinesToRequestRowInImmediateFlip,
            mode_lib->vba.DCCEnable,
            mode_lib->vba.PixelClock,
            v->BytePerPixelY,
            v->BytePerPixelC,
            mode_lib->vba.SourceRotation,
            v->dpte_row_height,
            v->dpte_row_height_chroma,
            v->meta_row_width,
            v->meta_row_width_chroma,
            v->meta_row_height,
            v->meta_row_height_chroma,
            v->meta_req_width,
            v->meta_req_width_chroma,
            v->meta_req_height,
            v->meta_req_height_chroma,
            v->dpte_group_bytes,
            v->PTERequestSizeY,
            v->PTERequestSizeC,
            v->PixelPTEReqWidthY,
            v->PixelPTEReqHeightY,
            v->PixelPTEReqWidthC,
            v->PixelPTEReqHeightC,
            v->dpte_row_width_luma_ub,
            v->dpte_row_width_chroma_ub,

            /* Output */
            v->DST_Y_PER_PTE_ROW_NOM_L,
            v->DST_Y_PER_PTE_ROW_NOM_C,
            v->DST_Y_PER_META_ROW_NOM_L,
            v->DST_Y_PER_META_ROW_NOM_C,
            v->TimePerMetaChunkNominal,
            v->TimePerChromaMetaChunkNominal,
            v->TimePerMetaChunkVBlank,
            v->TimePerChromaMetaChunkVBlank,
            v->TimePerMetaChunkFlip,
            v->TimePerChromaMetaChunkFlip,
            v->time_per_pte_group_nom_luma,
            v->time_per_pte_group_vblank_luma,
            v->time_per_pte_group_flip_luma,
            v->time_per_pte_group_nom_chroma,
            v->time_per_pte_group_vblank_chroma,
            v->time_per_pte_group_flip_chroma);

    dml32_CalculateVMGroupAndRequestTimes(
            mode_lib->vba.NumberOfActiveSurfaces,
            mode_lib->vba.GPUVMEnable,
            mode_lib->vba.GPUVMMaxPageTableLevels,
            mode_lib->vba.HTotal,
            v->BytePerPixelC,
            v->DestinationLinesToRequestVMInVBlank,
            v->DestinationLinesToRequestVMInImmediateFlip,
            mode_lib->vba.DCCEnable,
            mode_lib->vba.PixelClock,
            v->dpte_row_width_luma_ub,
            v->dpte_row_width_chroma_ub,
            v->vm_group_bytes,
            v->dpde0_bytes_per_frame_ub_l,
            v->dpde0_bytes_per_frame_ub_c,
            v->meta_pte_bytes_per_frame_ub_l,
            v->meta_pte_bytes_per_frame_ub_c,

            /* Output */
            v->TimePerVMGroupVBlank,
            v->TimePerVMGroupFlip,
            v->TimePerVMRequestVBlank,
            v->TimePerVMRequestFlip);

    // Min TTUVBlank
    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        if (mode_lib->vba.PrefetchModePerState[mode_lib->vba.VoltageLevel][mode_lib->vba.maxMpcComb] == 0) {
            v->MinTTUVBlank[k] = dml_max4(v->Watermark.DRAMClockChangeWatermark,
                    v->Watermark.FCLKChangeWatermark, v->Watermark.StutterEnterPlusExitWatermark,
                    v->Watermark.UrgentWatermark);
        } else if (mode_lib->vba.PrefetchModePerState[mode_lib->vba.VoltageLevel][mode_lib->vba.maxMpcComb]
                == 1) {
            v->MinTTUVBlank[k] = dml_max3(v->Watermark.FCLKChangeWatermark,
                    v->Watermark.StutterEnterPlusExitWatermark, v->Watermark.UrgentWatermark);
        } else if (mode_lib->vba.PrefetchModePerState[mode_lib->vba.VoltageLevel][mode_lib->vba.maxMpcComb]
                == 2) {
            v->MinTTUVBlank[k] = dml_max(v->Watermark.StutterEnterPlusExitWatermark,
                    v->Watermark.UrgentWatermark);
        } else {
            v->MinTTUVBlank[k] = v->Watermark.UrgentWatermark;
        }
        if (!mode_lib->vba.DynamicMetadataEnable[k])
            v->MinTTUVBlank[k] = mode_lib->vba.TCalc + v->MinTTUVBlank[k];
    }

    // DCC Configuration
    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        dml32_CalculateDCCConfiguration(
                mode_lib->vba.DCCEnable[k],
                mode_lib->vba.DCCProgrammingAssumesScanDirectionUnknownFinal,
                mode_lib->vba.SourcePixelFormat[k], mode_lib->vba.SurfaceWidthY[k],
                mode_lib->vba.SurfaceWidthC[k],
                mode_lib->vba.SurfaceHeightY[k],
                mode_lib->vba.SurfaceHeightC[k],
                mode_lib->vba.nomDETInKByte,
                v->BlockHeight256BytesY[k],
                v->BlockHeight256BytesC[k],
                mode_lib->vba.SurfaceTiling[k],
                v->BytePerPixelY[k],
                v->BytePerPixelC[k],
                v->BytePerPixelDETY[k],
                v->BytePerPixelDETC[k],
                static_cast<enum dm_rotation_angle>(mode_lib->vba.SourceScan[k]),
                /* Output */
                &v->DCCYMaxUncompressedBlock[k],
                &v->DCCCMaxUncompressedBlock[k],
                &v->DCCYMaxCompressedBlock[k],
                &v->DCCCMaxCompressedBlock[k],
                &v->DCCYIndependentBlock[k],
                &v->DCCCIndependentBlock[k]);
    }

    // VStartup Adjustment
    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        bool isInterlaceTiming;
        double Tvstartup_margin = (v->MaxVStartupLines[k] - v->VStartup[k]) * mode_lib->vba.HTotal[k]
                / mode_lib->vba.PixelClock[k];

        v->MinTTUVBlank[k] = v->MinTTUVBlank[k] + Tvstartup_margin;

        v->Tdmdl[k] = v->Tdmdl[k] + Tvstartup_margin;
        if (mode_lib->vba.DynamicMetadataEnable[k] && mode_lib->vba.DynamicMetadataVMEnabled)
            v->Tdmdl_vm[k] = v->Tdmdl_vm[k] + Tvstartup_margin;

        isInterlaceTiming = (mode_lib->vba.Interlace[k] &&
                !mode_lib->vba.ProgressiveToInterlaceUnitInOPP);

        v->MIN_DST_Y_NEXT_START[k] = ((isInterlaceTiming ? dml_floor((mode_lib->vba.VTotal[k] -
                        mode_lib->vba.VFrontPorch[k]) / 2.0, 1.0) :
                        mode_lib->vba.VTotal[k]) - mode_lib->vba.VFrontPorch[k])
                        + dml_max(1.0,
                        dml_ceil(v->WritebackDelay[mode_lib->vba.VoltageLevel][k]
                        / (mode_lib->vba.HTotal[k] / mode_lib->vba.PixelClock[k]), 1.0))
                        + dml_floor(4.0 * v->TSetup[k] / (mode_lib->vba.HTotal[k]
                        / mode_lib->vba.PixelClock[k]), 1.0) / 4.0;

        v->VStartup[k] = (isInterlaceTiming ? (2 * v->MaxVStartupLines[k]) : v->MaxVStartupLines[k]);

        if (((v->VUpdateOffsetPix[k] + v->VUpdateWidthPix[k] + v->VReadyOffsetPix[k])
            / mode_lib->vba.HTotal[k]) <= (isInterlaceTiming ? dml_floor((mode_lib->vba.VTotal[k]
            - mode_lib->vba.VActive[k] - mode_lib->vba.VFrontPorch[k] - v->VStartup[k]) / 2.0, 1.0) :
            static_cast<int>(mode_lib->vba.VTotal[k] - mode_lib->vba.VActive[k]
               - mode_lib->vba.VFrontPorch[k] - v->VStartup[k]))) {
            v->VREADY_AT_OR_AFTER_VSYNC[k] = true;
        } else {
            v->VREADY_AT_OR_AFTER_VSYNC[k] = false;
        }
    }

    {
        //Maximum Bandwidth Used
        for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
            TotalWRBandwidth = TotalWRBandwidth + WRBandwidth;
        }

        v->TotalDataReadBandwidth = 0;
        for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
            v->TotalDataReadBandwidth = v->TotalDataReadBandwidth + v->ReadBandwidthSurfaceLuma[k]
                    + v->ReadBandwidthSurfaceChroma[k];
        }
    }

    // Stutter Efficiency
    dml32_CalculateStutterEfficiency(v->CompressedBufferSizeInkByte,
            mode_lib->vba.UsesMALLForPStateChange,
            v->UnboundedRequestEnabled,
            mode_lib->vba.MetaFIFOSizeInKEntries,
            mode_lib->vba.ZeroSizeBufferEntries,
            mode_lib->vba.PixelChunkSizeInKByte,
            mode_lib->vba.NumberOfActiveSurfaces,
            mode_lib->vba.ROBBufferSizeInKByte,
            v->TotalDataReadBandwidth,
            mode_lib->vba.DCFCLK,
            mode_lib->vba.ReturnBW,
            v->CompbufReservedSpace64B,
            v->CompbufReservedSpaceZs,
            mode_lib->vba.SRExitTime,
            mode_lib->vba.SRExitZ8Time,
            mode_lib->vba.SynchronizeTimingsFinal,
            mode_lib->vba.BlendingAndTiming,
            v->Watermark.StutterEnterPlusExitWatermark,
            v->Watermark.Z8StutterEnterPlusExitWatermark,
            mode_lib->vba.ProgressiveToInterlaceUnitInOPP,
            mode_lib->vba.Interlace,
            v->MinTTUVBlank, dml_uint(mode_lib->vba.DPPPerPlane),
            mode_lib->vba.DETBufferSizeY,
            v->BytePerPixelY,
            v->BytePerPixelDETY,
            v->SwathWidthY,
            mode_lib->vba.SwathHeightY,
            mode_lib->vba.SwathHeightC,
            mode_lib->vba.DCCRateLuma,
            mode_lib->vba.DCCRateChroma,
            mode_lib->vba.DCCFractionOfZeroSizeRequestsLuma,
            mode_lib->vba.DCCFractionOfZeroSizeRequestsChroma,
            mode_lib->vba.HTotal, mode_lib->vba.VTotal,
            mode_lib->vba.PixelClock,
            mode_lib->vba.VRatio,
            mode_lib->vba.SourceRotation,
            v->BlockHeight256BytesY,
            v->BlockWidth256BytesY,
            v->BlockHeight256BytesC,
            v->BlockWidth256BytesC,
            v->DCCYMaxUncompressedBlock,
            v->DCCCMaxUncompressedBlock,
            mode_lib->vba.VActive,
            mode_lib->vba.DCCEnable,
            mode_lib->vba.WritebackEnable,
            v->ReadBandwidthSurfaceLuma,
            v->ReadBandwidthSurfaceChroma,
            v->meta_row_bw,
            v->dpte_row_bw,
            /* Output */
            &v->StutterEfficiencyNotIncludingVBlank,
            &v->StutterEfficiency,
            dml_uint(&v->NumberOfStutterBurstsPerFrame),
            &v->Z8StutterEfficiencyNotIncludingVBlank,
            &v->Z8StutterEfficiency,
            dml_uint(&v->Z8NumberOfStutterBurstsPerFrame),
            &v->StutterPeriod,
            &v->DCHUBBUB_ARB_CSTATE_MAX_CAP_MODE);

    v->Z8StutterEfficiencyNotIncludingVBlankBestCase = v->Z8StutterEfficiencyNotIncludingVBlank;
    v->Z8StutterEfficiencyBestCase = v->Z8StutterEfficiency;
    v->Z8NumberOfStutterBurstsPerFrameBestCase = v->Z8NumberOfStutterBurstsPerFrame;
    v->StutterPeriodBestCase = v->StutterPeriod;
}

static void DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation(
        struct display_mode_lib *mode_lib)
{
    dml32_program_clocks_swath_and_rows(mode_lib);
    dml32_program_latencies(mode_lib);
    dml32_program_prefetch(mode_lib);
    dml32_program_watermarks_and_stutter(mode_lib);
}

} // namespace rx7900::display::dml32

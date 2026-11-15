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
Module: rx7900.display.dcn32_dml_support
Purpose: DML32 mode support across voltage states and MPC combine options.
Created: 2026-10-24 (adaptation; the upstream notice above is preserved)
Architecture: Part of the bounded C++17 port of the Linux v6.12 DML32 model behind
src/display/dcn32_bandwidth (one stream, up to four pipes). Identifiers, statement order and
arithmetic (including the float-typed dml_min/max/floor/ceil helpers and every
int/unsigned conversion) deliberately follow the pinned upstream text so each
function can be compared line by line with work/upstream-dcn-stream; this is a
documented exception to the snake_case rule, as in src/opengl/gfx11_registers.hpp.
Per-plane and per-pipe arrays hold the four DCN 3.2 pipes (DML32_NUM_PLANES; Linux
sizes them DC__NUM_DPP__MAX = 8) and per-state arrays DML32_NUM_STATES entries; debug printing is removed.
Upstream: dml/dcn32/display_mode_vba_32.c (mode_support_configuration, dml32_ModeSupportAndSystemConfigurationFull).
*/
#include "dcn32_dml_util.hpp"

namespace rx7900::display::dml32 {
static void mode_support_configuration(struct vba_vars_st *v,
                  struct display_mode_lib *mode_lib)
{
    int i, j, start_state;

    if (mode_lib->validate_max_state)
        start_state = v->soc.num_states - 1;
    else
        start_state = 0;

    for (i = v->soc.num_states - 1; i >= start_state; i--) {
        for (j = 0; j < 2; j++) {
            if (mode_lib->vba.ScaleRatioAndTapsSupport == true
                && mode_lib->vba.SourceFormatPixelAndScanSupport == true
                && mode_lib->vba.ViewportSizeSupport[i][j] == true
                && !mode_lib->vba.LinkRateDoesNotMatchDPVersion
                && !mode_lib->vba.LinkRateForMultistreamNotIndicated
                && !mode_lib->vba.BPPForMultistreamNotIndicated
                && !mode_lib->vba.MultistreamWithHDMIOreDP
                && !mode_lib->vba.ExceededMultistreamSlots[i]
                && !mode_lib->vba.MSOOrODMSplitWithNonDPLink
                && !mode_lib->vba.NotEnoughLanesForMSO
                && mode_lib->vba.LinkCapacitySupport[i] == true && !mode_lib->vba.P2IWith420
                //&& !mode_lib->vba.DSCOnlyIfNecessaryWithBPP
                && !mode_lib->vba.DSC422NativeNotSupported
                && !mode_lib->vba.MPCCombineMethodIncompatible
                && mode_lib->vba.ODMCombine2To1SupportCheckOK[i] == true
                && mode_lib->vba.ODMCombine4To1SupportCheckOK[i] == true
                && mode_lib->vba.NotEnoughDSCUnits[i] == false
                && !mode_lib->vba.NotEnoughDSCSlices[i]
                && !mode_lib->vba.ImmediateFlipOrHostVMAndPStateWithMALLFullFrameOrPhantomPipe
                && !mode_lib->vba.InvalidCombinationOfMALLUseForPStateAndStaticScreen
                && mode_lib->vba.DSCCLKRequiredMoreThanSupported[i] == false
                && mode_lib->vba.PixelsPerLinePerDSCUnitSupport[i]
                && mode_lib->vba.DTBCLKRequiredMoreThanSupported[i] == false
                && !mode_lib->vba.InvalidCombinationOfMALLUseForPState
                && !mode_lib->vba.ImmediateFlipRequiredButTheRequirementForEachSurfaceIsNotSpecified
                && mode_lib->vba.ROBSupport[i][j] == true
                && mode_lib->vba.DISPCLK_DPPCLK_Support[i][j] == true
                && mode_lib->vba.TotalAvailablePipesSupport[i][j] == true
                && mode_lib->vba.NumberOfOTGSupport == true
                && mode_lib->vba.NumberOfHDMIFRLSupport == true
                && mode_lib->vba.EnoughWritebackUnits == true
                && mode_lib->vba.WritebackLatencySupport == true
                && mode_lib->vba.WritebackScaleRatioAndTapsSupport == true
                && mode_lib->vba.CursorSupport == true && mode_lib->vba.PitchSupport == true
                && mode_lib->vba.ViewportExceedsSurface == false
                && mode_lib->vba.PrefetchSupported[i][j] == true
                && mode_lib->vba.VActiveBandwithSupport[i][j] == true
                && mode_lib->vba.DynamicMetadataSupported[i][j] == true
                && mode_lib->vba.TotalVerticalActiveBandwidthSupport[i][j] == true
                && mode_lib->vba.VRatioInPrefetchSupported[i][j] == true
                && mode_lib->vba.PTEBufferSizeNotExceeded[i][j] == true
                && mode_lib->vba.DCCMetaBufferSizeNotExceeded[i][j] == true
                && mode_lib->vba.NonsupportedDSCInputBPC == false
                && !mode_lib->vba.ExceededMALLSize
                && (mode_lib->vba.NotEnoughDETSwathFillLatencyHidingPerState[i][j] == false
                || static_cast<unsigned int>(i) == v->soc.num_states - 1)
                && ((mode_lib->vba.HostVMEnable == false
                && !mode_lib->vba.ImmediateFlipRequiredFinal)
                || mode_lib->vba.ImmediateFlipSupportedForState[i][j])
                && (!mode_lib->vba.DRAMClockChangeRequirementFinal
                || static_cast<unsigned int>(i) == v->soc.num_states - 1
                || mode_lib->vba.DRAMClockChangeSupport[i][j] != dm_dram_clock_change_unsupported)
                && (!mode_lib->vba.FCLKChangeRequirementFinal || static_cast<unsigned int>(i) == v->soc.num_states - 1
                || mode_lib->vba.FCLKChangeSupport[i][j] != dm_fclock_change_unsupported)
                && (!mode_lib->vba.USRRetrainingRequiredFinal
                || mode_lib->vba.USRRetrainingSupport[i][j])) {
                mode_lib->vba.ModeSupport[i][j] = true;
            } else {
                mode_lib->vba.ModeSupport[i][j] = false;
            }
        }
    }
}


// Scale/format checks and the single-DPP swath and DET configuration.
static DML32_NOINLINE void dml32_support_checks_and_single_dpp_swath(struct display_mode_lib *mode_lib, int /* start_state */)
{
    struct vba_vars_st *v = &mode_lib->vba;
    unsigned int k;

    mode_lib->vba.ScaleRatioAndTapsSupport = true;
    for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
        if (mode_lib->vba.ScalerEnabled[k] == false
                && ((mode_lib->vba.SourcePixelFormat[k] != dm_444_64
                        && mode_lib->vba.SourcePixelFormat[k] != dm_444_32
                        && mode_lib->vba.SourcePixelFormat[k] != dm_444_16
                        && mode_lib->vba.SourcePixelFormat[k] != dm_mono_16
                        && mode_lib->vba.SourcePixelFormat[k] != dm_mono_8
                        && mode_lib->vba.SourcePixelFormat[k] != dm_rgbe
                        && mode_lib->vba.SourcePixelFormat[k] != dm_rgbe_alpha)
                        || mode_lib->vba.HRatio[k] != 1.0 || mode_lib->vba.htaps[k] != 1.0
                        || mode_lib->vba.VRatio[k] != 1.0 || mode_lib->vba.vtaps[k] != 1.0)) {
            mode_lib->vba.ScaleRatioAndTapsSupport = false;
        } else if (mode_lib->vba.vtaps[k] < 1.0 || mode_lib->vba.vtaps[k] > 8.0 || mode_lib->vba.htaps[k] < 1.0
                || mode_lib->vba.htaps[k] > 8.0
                || (mode_lib->vba.htaps[k] > 1.0 && (mode_lib->vba.htaps[k] % 2) == 1)
                || mode_lib->vba.HRatio[k] > mode_lib->vba.MaxHSCLRatio
                || mode_lib->vba.VRatio[k] > mode_lib->vba.MaxVSCLRatio
                || mode_lib->vba.HRatio[k] > mode_lib->vba.htaps[k]
                || mode_lib->vba.VRatio[k] > mode_lib->vba.vtaps[k]
                || (mode_lib->vba.SourcePixelFormat[k] != dm_444_64
                        && mode_lib->vba.SourcePixelFormat[k] != dm_444_32
                        && mode_lib->vba.SourcePixelFormat[k] != dm_444_16
                        && mode_lib->vba.SourcePixelFormat[k] != dm_mono_16
                        && mode_lib->vba.SourcePixelFormat[k] != dm_mono_8
                        && mode_lib->vba.SourcePixelFormat[k] != dm_rgbe
                        && (mode_lib->vba.VTAPsChroma[k] < 1
                                || mode_lib->vba.VTAPsChroma[k] > 8
                                || mode_lib->vba.HTAPsChroma[k] < 1
                                || mode_lib->vba.HTAPsChroma[k] > 8
                                || (mode_lib->vba.HTAPsChroma[k] > 1
                                        && mode_lib->vba.HTAPsChroma[k] % 2
                                                == 1)
                                || mode_lib->vba.HRatioChroma[k]
                                        > mode_lib->vba.MaxHSCLRatio
                                || mode_lib->vba.VRatioChroma[k]
                                        > mode_lib->vba.MaxVSCLRatio
                                || mode_lib->vba.HRatioChroma[k]
                                        > mode_lib->vba.HTAPsChroma[k]
                                || mode_lib->vba.VRatioChroma[k]
                                        > mode_lib->vba.VTAPsChroma[k]))) {
            mode_lib->vba.ScaleRatioAndTapsSupport = false;
        }
    }

    /*Source Format, Pixel Format and Scan Support Check*/
    mode_lib->vba.SourceFormatPixelAndScanSupport = true;
    for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
        if (mode_lib->vba.SurfaceTiling[k] == dm_sw_linear
            && (!(!IsVertical(static_cast<enum dm_rotation_angle>(mode_lib->vba.SourceScan[k])))
                || mode_lib->vba.DCCEnable[k] == true)) {
            mode_lib->vba.SourceFormatPixelAndScanSupport = false;
        }
    }

    for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
        dml32_CalculateBytePerPixelAndBlockSizes(
                mode_lib->vba.SourcePixelFormat[k],
                mode_lib->vba.SurfaceTiling[k],

                /* Output */
                &mode_lib->vba.BytePerPixelY[k],
                &mode_lib->vba.BytePerPixelC[k],
                &mode_lib->vba.BytePerPixelInDETY[k],
                &mode_lib->vba.BytePerPixelInDETC[k],
                &mode_lib->vba.Read256BlockHeightY[k],
                &mode_lib->vba.Read256BlockHeightC[k],
                &mode_lib->vba.Read256BlockWidthY[k],
                &mode_lib->vba.Read256BlockWidthC[k],
                &mode_lib->vba.MacroTileHeightY[k],
                &mode_lib->vba.MacroTileHeightC[k],
                &mode_lib->vba.MacroTileWidthY[k],
                &mode_lib->vba.MacroTileWidthC[k]);
    }

    /*Bandwidth Support Check*/
    for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
        if (!IsVertical(mode_lib->vba.SourceRotation[k])) {
            v->SwathWidthYSingleDPP[k] = mode_lib->vba.ViewportWidth[k];
            v->SwathWidthCSingleDPP[k] = mode_lib->vba.ViewportWidthChroma[k];
        } else {
            v->SwathWidthYSingleDPP[k] = mode_lib->vba.ViewportHeight[k];
            v->SwathWidthCSingleDPP[k] = mode_lib->vba.ViewportHeightChroma[k];
        }
    }
    for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
        v->ReadBandwidthLuma[k] = v->SwathWidthYSingleDPP[k] * dml_ceil(v->BytePerPixelInDETY[k], 1.0)
                / (mode_lib->vba.HTotal[k] / mode_lib->vba.PixelClock[k]) * mode_lib->vba.VRatio[k];
        v->ReadBandwidthChroma[k] = v->SwathWidthYSingleDPP[k] / 2 * dml_ceil(v->BytePerPixelInDETC[k], 2.0)
                / (mode_lib->vba.HTotal[k] / mode_lib->vba.PixelClock[k]) * mode_lib->vba.VRatio[k]
                / 2.0;
    }
    // Writeback is never enabled (dout.wb_enable = 0): no write bandwidth and every
    // writeback check passes.
    for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
        v->WriteBandwidth[k] = 0.0;
    }

    mode_lib->vba.WritebackLatencySupport = true;
    mode_lib->vba.EnoughWritebackUnits = true;
    mode_lib->vba.TotalNumberOfActiveWriteback = 0;
    mode_lib->vba.WritebackScaleRatioAndTapsSupport = true;

    for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
        dml32_CalculateSinglePipeDPPCLKAndSCLThroughput(mode_lib->vba.HRatio[k], mode_lib->vba.HRatioChroma[k],
                mode_lib->vba.VRatio[k], mode_lib->vba.VRatioChroma[k],
                mode_lib->vba.MaxDCHUBToPSCLThroughput, mode_lib->vba.MaxPSCLToLBThroughput,
                mode_lib->vba.PixelClock[k], mode_lib->vba.SourcePixelFormat[k],
                mode_lib->vba.htaps[k], mode_lib->vba.HTAPsChroma[k], mode_lib->vba.vtaps[k],
                mode_lib->vba.VTAPsChroma[k],
                /* Output */
                &mode_lib->vba.PSCL_FACTOR[k], &mode_lib->vba.PSCL_FACTOR_CHROMA[k],
                &mode_lib->vba.MinDPPCLKUsingSingleDPP[k]);
    }

    for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {

        if (mode_lib->vba.SurfaceTiling[k] == dm_sw_linear) {
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.MaximumSwathWidthSupportLuma = 8192;
        } else if (!IsVertical(mode_lib->vba.SourceRotation[k]) && v->BytePerPixelC[k] > 0
                && mode_lib->vba.SourcePixelFormat[k] != dm_rgbe_alpha) {
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.MaximumSwathWidthSupportLuma = 7680;
        } else if (IsVertical(mode_lib->vba.SourceRotation[k]) && v->BytePerPixelC[k] > 0
                && mode_lib->vba.SourcePixelFormat[k] != dm_rgbe_alpha) {
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.MaximumSwathWidthSupportLuma = 4320;
        } else if (mode_lib->vba.SourcePixelFormat[k] == dm_rgbe_alpha) {
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.MaximumSwathWidthSupportLuma = 3840;
        } else if (IsVertical(mode_lib->vba.SourceRotation[k]) && v->BytePerPixelY[k] == 8 &&
                mode_lib->vba.DCCEnable[k] == true) {
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.MaximumSwathWidthSupportLuma = 3072;
        } else {
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.MaximumSwathWidthSupportLuma = 6144;
        }

        if (mode_lib->vba.SourcePixelFormat[k] == dm_420_8 || mode_lib->vba.SourcePixelFormat[k] == dm_420_10
                || mode_lib->vba.SourcePixelFormat[k] == dm_420_12) {
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.MaximumSwathWidthSupportChroma = v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.MaximumSwathWidthSupportLuma / 2.0;
        } else {
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.MaximumSwathWidthSupportChroma = v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.MaximumSwathWidthSupportLuma;
        }
        v->MaximumSwathWidthInLineBufferLuma = mode_lib->vba.LineBufferSizeFinal
                * dml_max(mode_lib->vba.HRatio[k], 1.0) / mode_lib->vba.LBBitPerPixel[k]
                / (mode_lib->vba.vtaps[k] + dml_max(dml_ceil(mode_lib->vba.VRatio[k], 1.0) - 2, 0.0));
        if (v->BytePerPixelC[k] == 0.0) {
            v->MaximumSwathWidthInLineBufferChroma = 0;
        } else {
            v->MaximumSwathWidthInLineBufferChroma = mode_lib->vba.LineBufferSizeFinal
                    * dml_max(mode_lib->vba.HRatioChroma[k], 1.0) / mode_lib->vba.LBBitPerPixel[k]
                    / (mode_lib->vba.VTAPsChroma[k]
                            + dml_max(dml_ceil(mode_lib->vba.VRatioChroma[k], 1.0) - 2,
                                    0.0));
        }
        v->MaximumSwathWidthLuma[k] = dml_min(v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.MaximumSwathWidthSupportLuma,
                v->MaximumSwathWidthInLineBufferLuma);
        v->MaximumSwathWidthChroma[k] = dml_min(v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.MaximumSwathWidthSupportChroma,
                v->MaximumSwathWidthInLineBufferChroma);
    }

    dml32_CalculateSwathAndDETConfiguration(
            mode_lib->vba.DETSizeOverride,
            mode_lib->vba.UsesMALLForPStateChange,
            mode_lib->vba.ConfigReturnBufferSizeInKByte,
            mode_lib->vba.MaxTotalDETInKByte,
            mode_lib->vba.MinCompressedBufferSizeInKByte,
            1, /* ForceSingleDPP */
            mode_lib->vba.NumberOfActiveSurfaces,
            mode_lib->vba.nomDETInKByte,
            mode_lib->vba.UseUnboundedRequesting,
            mode_lib->vba.DisableUnboundRequestIfCompBufReservedSpaceNeedAdjustment,
            mode_lib->vba.ip.pixel_chunk_size_kbytes,
            mode_lib->vba.ip.rob_buffer_size_kbytes,
            mode_lib->vba.CompressedBufferSegmentSizeInkByteFinal,
            mode_lib->vba.Output,
            mode_lib->vba.ReadBandwidthLuma,
            mode_lib->vba.ReadBandwidthChroma,
            mode_lib->vba.MaximumSwathWidthLuma,
            mode_lib->vba.MaximumSwathWidthChroma,
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
            mode_lib->vba.Read256BlockHeightY,
            mode_lib->vba.Read256BlockHeightC,
            mode_lib->vba.Read256BlockWidthY,
            mode_lib->vba.Read256BlockWidthC,
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_odm_mode,
            mode_lib->vba.BlendingAndTiming,
            mode_lib->vba.BytePerPixelY,
            mode_lib->vba.BytePerPixelC,
            mode_lib->vba.BytePerPixelInDETY,
            mode_lib->vba.BytePerPixelInDETC,
            mode_lib->vba.HActive,
            mode_lib->vba.HRatio,
            mode_lib->vba.HRatioChroma,
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[0], /*  Integer DPPPerSurface[] */

            /* Output */
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[1], /* Long            swath_width_luma_ub[] */
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[2], /* Long            swath_width_chroma_ub[]  */
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_double_array[0], /* Long            SwathWidth[]  */
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_double_array[1], /* Long            SwathWidthChroma[]  */
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[3], /* Integer         SwathHeightY[]  */
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[4], /* Integer         SwathHeightC[]  */
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[5], /* Long            DETBufferSizeInKByte[]  */
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[6], /* Long            DETBufferSizeY[]  */
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[7], /* Long            DETBufferSizeC[]  */
            &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_boolean_array[0][0], /* bool           *UnboundedRequestEnabled  */
            &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[0][0], /* Long           *CompressedBufferSizeInkByte  */
            &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[1][0], /* Long           *CompBufReservedSpaceKBytes */
            &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.CompBufReservedSpaceNeedAdjustmentSingleDPP,
            mode_lib->vba.SingleDPPViewportSizeSupportPerSurface,/* bool ViewportSizeSupportPerSurface[] */
            &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_boolean_array[1][0]); /* bool           *ViewportSizeSupport */

    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.MPCCombineMethodAsNeededForPStateChangeAndVoltage = false;
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.MPCCombineMethodAsPossible = false;

    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        if (mode_lib->vba.MPCCombineUse[k] == dm_mpc_reduce_voltage_and_clocks)
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.MPCCombineMethodAsNeededForPStateChangeAndVoltage = true;
        if (mode_lib->vba.MPCCombineUse[k] == dm_mpc_always_when_possible)
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.MPCCombineMethodAsPossible = true;
    }
    mode_lib->vba.MPCCombineMethodIncompatible = v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.MPCCombineMethodAsNeededForPStateChangeAndVoltage
            && v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.MPCCombineMethodAsPossible;

}

// Per-state DISPCLK/DPPCLK, MPC combine and DPP counts, OTG/output counts.
static DML32_NOINLINE void dml32_support_clocks_and_dpp_per_state(struct display_mode_lib *mode_lib, int start_state)
{
    struct vba_vars_st *v = &mode_lib->vba;
    int i, j;
    unsigned int k;
    unsigned int NumberOfNonCombinedSurfaceOfMaximumBandwidth;
    bool CompBufReservedSpaceNeedAdjustment;

    for (i = start_state; static_cast<unsigned int>(i) < v->soc.num_states; i++) {
        for (j = 0; j < 2; j++) {
            mode_lib->vba.TotalNumberOfActiveDPP[i][j] = 0;
            mode_lib->vba.TotalAvailablePipesSupport[i][j] = true;
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.ODMModeNoDSC = dm_odm_combine_mode_disabled;
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.ODMModeDSC = dm_odm_combine_mode_disabled;

            for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
                dml32_CalculateODMMode(
                        mode_lib->vba.MaximumPixelsPerLinePerDSCUnit,
                        mode_lib->vba.HActive[k],
                        mode_lib->vba.OutputFormat[k],
                        mode_lib->vba.Output[k],
                        mode_lib->vba.ODMUse[k],
                        mode_lib->vba.MaxDispclk[i],
                        mode_lib->vba.MaxDispclk[v->soc.num_states - 1],
                        false,
                        mode_lib->vba.TotalNumberOfActiveDPP[i][j],
                        mode_lib->vba.MaxNumDPP,
                        mode_lib->vba.PixelClock[k],
                        mode_lib->vba.DISPCLKDPPCLKDSCCLKDownSpreading,
                        mode_lib->vba.DISPCLKRampingMargin,
                        mode_lib->vba.DISPCLKDPPCLKVCOSpeed,
                        mode_lib->vba.NumberOfDSCSlices[k],

                        /* Output */
                        &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalAvailablePipesSupportNoDSC,
                        &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.NumberOfDPPNoDSC,
                        &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.ODMModeNoDSC,
                        &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.RequiredDISPCLKPerSurfaceNoDSC);

                dml32_CalculateODMMode(
                        mode_lib->vba.MaximumPixelsPerLinePerDSCUnit,
                        mode_lib->vba.HActive[k],
                        mode_lib->vba.OutputFormat[k],
                        mode_lib->vba.Output[k],
                        mode_lib->vba.ODMUse[k],
                        mode_lib->vba.MaxDispclk[i],
                        mode_lib->vba.MaxDispclk[v->soc.num_states - 1],
                        true,
                        mode_lib->vba.TotalNumberOfActiveDPP[i][j],
                        mode_lib->vba.MaxNumDPP,
                        mode_lib->vba.PixelClock[k],
                        mode_lib->vba.DISPCLKDPPCLKDSCCLKDownSpreading,
                        mode_lib->vba.DISPCLKRampingMargin,
                        mode_lib->vba.DISPCLKDPPCLKVCOSpeed,
                        mode_lib->vba.NumberOfDSCSlices[k],

                        /* Output */
                        &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalAvailablePipesSupportDSC,
                        &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.NumberOfDPPDSC,
                        &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.ODMModeDSC,
                        &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.RequiredDISPCLKPerSurfaceDSC);

                dml32_CalculateOutputLink(
                        mode_lib->vba.PHYCLKPerState[i],
                        mode_lib->vba.PHYCLKD18PerState[i],
                        mode_lib->vba.PHYCLKD32PerState[i],
                        mode_lib->vba.Downspreading,
                        (mode_lib->vba.BlendingAndTiming[k] == k),
                        mode_lib->vba.Output[k],
                        mode_lib->vba.OutputFormat[k],
                        mode_lib->vba.HTotal[k],
                        mode_lib->vba.HActive[k],
                        mode_lib->vba.PixelClockBackEnd[k],
                        mode_lib->vba.ForcedOutputLinkBPP[k],
                        mode_lib->vba.DSCInputBitPerComponent[k],
                        mode_lib->vba.NumberOfDSCSlices[k],
                        mode_lib->vba.AudioSampleRate[k],
                        mode_lib->vba.AudioSampleLayout[k],
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.ODMModeNoDSC,
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.ODMModeDSC,
                        mode_lib->vba.DSCEnable[k],
                        mode_lib->vba.OutputLinkDPLanes[k],
                        mode_lib->vba.OutputLinkDPRate[k],

                        /* Output */
                        &mode_lib->vba.RequiresDSC[i][k],
                        &mode_lib->vba.RequiresFEC[i][k],
                        &mode_lib->vba.OutputBppPerState[i][k],
                        &mode_lib->vba.OutputTypePerState[i][k],
                        &mode_lib->vba.OutputRatePerState[i][k],
                        &mode_lib->vba.RequiredSlots[i][k]);

                if (mode_lib->vba.RequiresDSC[i][k] == false) {
                    mode_lib->vba.ODMCombineEnablePerState[i][k] = v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.ODMModeNoDSC;
                    mode_lib->vba.RequiredDISPCLKPerSurface[i][j][k] =
                            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.RequiredDISPCLKPerSurfaceNoDSC;
                    if (!v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalAvailablePipesSupportNoDSC)
                        mode_lib->vba.TotalAvailablePipesSupport[i][j] = false;
                    mode_lib->vba.TotalNumberOfActiveDPP[i][j] =
                            mode_lib->vba.TotalNumberOfActiveDPP[i][j] + v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.NumberOfDPPNoDSC;
                } else {
                    mode_lib->vba.ODMCombineEnablePerState[i][k] = v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.ODMModeDSC;
                    mode_lib->vba.RequiredDISPCLKPerSurface[i][j][k] =
                            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.RequiredDISPCLKPerSurfaceDSC;
                    if (!v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalAvailablePipesSupportDSC)
                        mode_lib->vba.TotalAvailablePipesSupport[i][j] = false;
                    mode_lib->vba.TotalNumberOfActiveDPP[i][j] =
                            mode_lib->vba.TotalNumberOfActiveDPP[i][j] + v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.NumberOfDPPDSC;
                }
            }

            for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
                if (mode_lib->vba.ODMCombineEnablePerState[i][k] == dm_odm_combine_mode_4to1) {
                    mode_lib->vba.MPCCombine[i][j][k] = false;
                    mode_lib->vba.NoOfDPP[i][j][k] = 4;
                } else if (mode_lib->vba.ODMCombineEnablePerState[i][k] == dm_odm_combine_mode_2to1) {
                    mode_lib->vba.MPCCombine[i][j][k] = false;
                    mode_lib->vba.NoOfDPP[i][j][k] = 2;
                } else if (mode_lib->vba.MPCCombineUse[k] == dm_mpc_never) {
                    mode_lib->vba.MPCCombine[i][j][k] = false;
                    mode_lib->vba.NoOfDPP[i][j][k] = 1;
                } else if (dml32_RoundToDFSGranularity(
                        mode_lib->vba.MinDPPCLKUsingSingleDPP[k]
                                * (1 + mode_lib->vba.DISPCLKDPPCLKDSCCLKDownSpreading
                                                / 100), 1,
                        mode_lib->vba.DISPCLKDPPCLKVCOSpeed) <= mode_lib->vba.MaxDppclk[i] &&
                mode_lib->vba.SingleDPPViewportSizeSupportPerSurface[k] == true) {
                    mode_lib->vba.MPCCombine[i][j][k] = false;
                    mode_lib->vba.NoOfDPP[i][j][k] = 1;
                } else if (mode_lib->vba.TotalNumberOfActiveDPP[i][j] < mode_lib->vba.MaxNumDPP) {
                    mode_lib->vba.MPCCombine[i][j][k] = true;
                    mode_lib->vba.NoOfDPP[i][j][k] = 2;
                    mode_lib->vba.TotalNumberOfActiveDPP[i][j] =
                            mode_lib->vba.TotalNumberOfActiveDPP[i][j] + 1;
                } else {
                    mode_lib->vba.MPCCombine[i][j][k] = false;
                    mode_lib->vba.NoOfDPP[i][j][k] = 1;
                    mode_lib->vba.TotalAvailablePipesSupport[i][j] = false;
                }
            }

            mode_lib->vba.TotalNumberOfSingleDPPSurfaces[i][j] = 0;
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.NoChroma = true;

            for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
                if (mode_lib->vba.NoOfDPP[i][j][k] == 1)
                    mode_lib->vba.TotalNumberOfSingleDPPSurfaces[i][j] =
                            mode_lib->vba.TotalNumberOfSingleDPPSurfaces[i][j] + 1;
                if (mode_lib->vba.SourcePixelFormat[k] == dm_420_8
                        || mode_lib->vba.SourcePixelFormat[k] == dm_420_10
                        || mode_lib->vba.SourcePixelFormat[k] == dm_420_12
                        || mode_lib->vba.SourcePixelFormat[k] == dm_rgbe_alpha) {
                    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.NoChroma = false;
                }
            }

            // if TotalNumberOfActiveDPP is > 1, then there should be no unbounded req mode (hw limitation), the comp buf reserved adjustment is not needed regardless
            // if TotalNumberOfActiveDPP is == 1, then will use the SingleDPP version of unbounded_req for the decision
            CompBufReservedSpaceNeedAdjustment = (mode_lib->vba.TotalNumberOfActiveDPP[i][j] > 1) ? 0 : v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.CompBufReservedSpaceNeedAdjustmentSingleDPP;

            if (j == 1 && !dml32_UnboundedRequest(mode_lib->vba.UseUnboundedRequesting,
                    mode_lib->vba.TotalNumberOfActiveDPP[i][j], v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.NoChroma,
                    mode_lib->vba.Output[0],
                    mode_lib->vba.SurfaceTiling[0],
                    CompBufReservedSpaceNeedAdjustment,
                    mode_lib->vba.DisableUnboundRequestIfCompBufReservedSpaceNeedAdjustment)) {
                while (!(mode_lib->vba.TotalNumberOfActiveDPP[i][j] >= mode_lib->vba.MaxNumDPP
                        || mode_lib->vba.TotalNumberOfSingleDPPSurfaces[i][j] == 0)) {
                    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.BWOfNonCombinedSurfaceOfMaximumBandwidth = 0;
                    NumberOfNonCombinedSurfaceOfMaximumBandwidth = 0;

                    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
                        if (mode_lib->vba.MPCCombineUse[k]
                            != dm_mpc_never &&
                            mode_lib->vba.MPCCombineUse[k] != dm_mpc_reduce_voltage &&
                            mode_lib->vba.ReadBandwidthLuma[k] +
                            mode_lib->vba.ReadBandwidthChroma[k] >
                            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.BWOfNonCombinedSurfaceOfMaximumBandwidth &&
                            (mode_lib->vba.ODMCombineEnablePerState[i][k] !=
                            dm_odm_combine_mode_2to1 &&
                            mode_lib->vba.ODMCombineEnablePerState[i][k] !=
                            dm_odm_combine_mode_4to1) &&
                                mode_lib->vba.MPCCombine[i][j][k] == false) {
                            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.BWOfNonCombinedSurfaceOfMaximumBandwidth =
                                mode_lib->vba.ReadBandwidthLuma[k]
                                + mode_lib->vba.ReadBandwidthChroma[k];
                            NumberOfNonCombinedSurfaceOfMaximumBandwidth = k;
                        }
                    }
                    mode_lib->vba.MPCCombine[i][j][NumberOfNonCombinedSurfaceOfMaximumBandwidth] =
                            true;
                    mode_lib->vba.NoOfDPP[i][j][NumberOfNonCombinedSurfaceOfMaximumBandwidth] = 2;
                    mode_lib->vba.TotalNumberOfActiveDPP[i][j] =
                            mode_lib->vba.TotalNumberOfActiveDPP[i][j] + 1;
                    mode_lib->vba.TotalNumberOfSingleDPPSurfaces[i][j] =
                            mode_lib->vba.TotalNumberOfSingleDPPSurfaces[i][j] - 1;
                }
            }

            //DISPCLK/DPPCLK
            mode_lib->vba.WritebackRequiredDISPCLK = 0;

            mode_lib->vba.RequiredDISPCLK[i][j] = mode_lib->vba.WritebackRequiredDISPCLK;
            for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
                mode_lib->vba.RequiredDISPCLK[i][j] = dml_max(mode_lib->vba.RequiredDISPCLK[i][j],
                        mode_lib->vba.RequiredDISPCLKPerSurface[i][j][k]);
            }

            for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k)
                mode_lib->vba.NoOfDPPThisState[k] = mode_lib->vba.NoOfDPP[i][j][k];

            dml32_CalculateDPPCLK(mode_lib->vba.NumberOfActiveSurfaces,
                    mode_lib->vba.DISPCLKDPPCLKDSCCLKDownSpreading,
                    mode_lib->vba.DISPCLKDPPCLKVCOSpeed, mode_lib->vba.MinDPPCLKUsingSingleDPP,
                    dml_uint(mode_lib->vba.NoOfDPPThisState),
                    /* Output */
                    &mode_lib->vba.GlobalDPPCLK, mode_lib->vba.RequiredDPPCLKThisState);

            for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k)
                mode_lib->vba.RequiredDPPCLK[i][j][k] = mode_lib->vba.RequiredDPPCLKThisState[k];

            mode_lib->vba.DISPCLK_DPPCLK_Support[i][j] = !((mode_lib->vba.RequiredDISPCLK[i][j]
                    > mode_lib->vba.MaxDispclk[i])
                    || (mode_lib->vba.GlobalDPPCLK > mode_lib->vba.MaxDppclk[i]));

            if (mode_lib->vba.TotalNumberOfActiveDPP[i][j] > mode_lib->vba.MaxNumDPP)
                mode_lib->vba.TotalAvailablePipesSupport[i][j] = false;
        } // j
    } // i (VOLTAGE_STATE)

    /* Total Available OTG, HDMIFRL, DP Support Check */
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalNumberOfActiveOTG = 0;
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalNumberOfActiveHDMIFRL = 0;
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalNumberOfActiveDP2p0 = 0;
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalNumberOfActiveDP2p0Outputs = 0;

    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        if (mode_lib->vba.BlendingAndTiming[k] == k) {
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalNumberOfActiveOTG = v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalNumberOfActiveOTG + 1;
            if (mode_lib->vba.Output[k] == dm_dp2p0) {
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalNumberOfActiveDP2p0 = v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalNumberOfActiveDP2p0 + 1;
                if (mode_lib->vba.OutputMultistreamId[k]
                        == k || mode_lib->vba.OutputMultistreamEn[k] == false) {
                    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalNumberOfActiveDP2p0Outputs = v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalNumberOfActiveDP2p0Outputs + 1;
                }
            }
        }
    }

    mode_lib->vba.NumberOfOTGSupport = (v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalNumberOfActiveOTG <= mode_lib->vba.MaxNumOTG);
    mode_lib->vba.NumberOfHDMIFRLSupport = (v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalNumberOfActiveHDMIFRL <= mode_lib->vba.MaxNumHDMIFRLOutputs);
    mode_lib->vba.NumberOfDP2p0Support = (v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalNumberOfActiveDP2p0 <= mode_lib->vba.MaxNumDP2p0Streams
            && v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalNumberOfActiveDP2p0Outputs <= mode_lib->vba.MaxNumDP2p0Outputs);

    /* Display IO and DSC Support Check */
    mode_lib->vba.NonsupportedDSCInputBPC = false;
    for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
        if (!(mode_lib->vba.DSCInputBitPerComponent[k] == 12.0
                || mode_lib->vba.DSCInputBitPerComponent[k] == 10.0
                || mode_lib->vba.DSCInputBitPerComponent[k] == 8.0)
                || mode_lib->vba.DSCInputBitPerComponent[k] > mode_lib->vba.MaximumDSCBitsPerComponent) {
            mode_lib->vba.NonsupportedDSCInputBPC = true;
        }
    }

}

// Output link, DSC and per-state swath/DET configuration and MALL size.
static DML32_NOINLINE void dml32_support_output_link_and_swath_per_state(struct display_mode_lib *mode_lib, int start_state)
{
    struct vba_vars_st *v = &mode_lib->vba;
    int i, j;
    unsigned int k, m;
    unsigned int TotalSlots;

    for (i = start_state; static_cast<unsigned int>(i) < v->soc.num_states; ++i) {
        mode_lib->vba.ExceededMultistreamSlots[i] = false;
        for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
            if (mode_lib->vba.OutputMultistreamEn[k] == true && mode_lib->vba.OutputMultistreamId[k] == k) {
                TotalSlots = mode_lib->vba.RequiredSlots[i][k];
                for (j = 0; static_cast<unsigned int>(j) < mode_lib->vba.NumberOfActiveSurfaces; ++j) {
                    if (mode_lib->vba.OutputMultistreamId[j] == k)
                        TotalSlots = TotalSlots + mode_lib->vba.RequiredSlots[i][j];
                }
                if (mode_lib->vba.Output[k] == dm_dp && TotalSlots > 63)
                    mode_lib->vba.ExceededMultistreamSlots[i] = true;
                if (mode_lib->vba.Output[k] == dm_dp2p0 && TotalSlots > 64)
                    mode_lib->vba.ExceededMultistreamSlots[i] = true;
            }
        }
        mode_lib->vba.LinkCapacitySupport[i] = true;
        for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
            if (mode_lib->vba.BlendingAndTiming[k] == k
                && (mode_lib->vba.Output[k] == dm_dp || mode_lib->vba.Output[k] == dm_dp2p0
                    || mode_lib->vba.Output[k] == dm_edp
                    || mode_lib->vba.Output[k] == dm_hdmi)
                && mode_lib->vba.OutputBppPerState[i][k] == 0 &&
                (mode_lib->vba.UsesMALLForPStateChange[k] != dm_use_mall_pstate_change_phantom_pipe)) {
                /* Phantom pipes don't consider DSC in DML, so it could fail link check.
                 * However, we don't care about the link for phantom pipes.
                 */
                mode_lib->vba.LinkCapacitySupport[i] = false;
            }
        }
    }

    mode_lib->vba.P2IWith420 = false;
    mode_lib->vba.DSCOnlyIfNecessaryWithBPP = false;
    mode_lib->vba.DSC422NativeNotSupported = false;
    mode_lib->vba.LinkRateDoesNotMatchDPVersion = false;
    mode_lib->vba.LinkRateForMultistreamNotIndicated = false;
    mode_lib->vba.BPPForMultistreamNotIndicated = false;
    mode_lib->vba.MultistreamWithHDMIOreDP = false;
    mode_lib->vba.MSOOrODMSplitWithNonDPLink = false;
    mode_lib->vba.NotEnoughLanesForMSO = false;

    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        if (mode_lib->vba.BlendingAndTiming[k] == k
                && (mode_lib->vba.Output[k] == dm_dp || mode_lib->vba.Output[k] == dm_dp2p0
                        || mode_lib->vba.Output[k] == dm_edp
                        || mode_lib->vba.Output[k] == dm_hdmi)) {
            if (mode_lib->vba.OutputFormat[k]
                    == dm_420 && mode_lib->vba.Interlace[k] == 1 &&
                    mode_lib->vba.ProgressiveToInterlaceUnitInOPP == true)
                mode_lib->vba.P2IWith420 = true;

            if (mode_lib->vba.DSCEnable[k] && mode_lib->vba.ForcedOutputLinkBPP[k] != 0)
                mode_lib->vba.DSCOnlyIfNecessaryWithBPP = true;
            if (mode_lib->vba.DSCEnable[k] && mode_lib->vba.OutputFormat[k] == dm_n422
                    && !mode_lib->vba.DSC422NativeSupport)
                mode_lib->vba.DSC422NativeNotSupported = true;

            if (((mode_lib->vba.OutputLinkDPRate[k] == dm_dp_rate_hbr
                    || mode_lib->vba.OutputLinkDPRate[k] == dm_dp_rate_hbr2
                    || mode_lib->vba.OutputLinkDPRate[k] == dm_dp_rate_hbr3)
                    && mode_lib->vba.Output[k] != dm_dp && mode_lib->vba.Output[k] != dm_edp)
                    || ((mode_lib->vba.OutputLinkDPRate[k] == dm_dp_rate_uhbr10
                            || mode_lib->vba.OutputLinkDPRate[k] == dm_dp_rate_uhbr13p5
                            || mode_lib->vba.OutputLinkDPRate[k] == dm_dp_rate_uhbr20)
                            && mode_lib->vba.Output[k] != dm_dp2p0))
                mode_lib->vba.LinkRateDoesNotMatchDPVersion = true;

            if (mode_lib->vba.OutputMultistreamEn[k] == true) {
                if (mode_lib->vba.OutputMultistreamId[k] == k
                    && mode_lib->vba.OutputLinkDPRate[k] == dm_dp_rate_na)
                    mode_lib->vba.LinkRateForMultistreamNotIndicated = true;
                if (mode_lib->vba.OutputMultistreamId[k] == k && mode_lib->vba.ForcedOutputLinkBPP[k] == 0)
                    mode_lib->vba.BPPForMultistreamNotIndicated = true;
                for (j = 0; static_cast<unsigned int>(j) < mode_lib->vba.NumberOfActiveSurfaces; ++j) {
                    if (mode_lib->vba.OutputMultistreamId[k] == static_cast<unsigned int>(j)
                        && mode_lib->vba.ForcedOutputLinkBPP[k] == 0)
                        mode_lib->vba.BPPForMultistreamNotIndicated = true;
                }
            }

            if ((mode_lib->vba.Output[k] == dm_edp || mode_lib->vba.Output[k] == dm_hdmi)) {
                if (mode_lib->vba.OutputMultistreamEn[k] == true && mode_lib->vba.OutputMultistreamId[k] == k)
                    mode_lib->vba.MultistreamWithHDMIOreDP = true;
                for (j = 0; static_cast<unsigned int>(j) < mode_lib->vba.NumberOfActiveSurfaces; ++j) {
                    if (mode_lib->vba.OutputMultistreamEn[k] == true && mode_lib->vba.OutputMultistreamId[k] == static_cast<unsigned int>(j))
                        mode_lib->vba.MultistreamWithHDMIOreDP = true;
                }
            }

            if (mode_lib->vba.Output[k] != dm_dp
                    && (mode_lib->vba.ODMUse[k] == dm_odm_split_policy_1to2
                            || mode_lib->vba.ODMUse[k] == dm_odm_mso_policy_1to2
                            || mode_lib->vba.ODMUse[k] == dm_odm_mso_policy_1to4))
                mode_lib->vba.MSOOrODMSplitWithNonDPLink = true;

            if ((mode_lib->vba.ODMUse[k] == dm_odm_mso_policy_1to2
                    && mode_lib->vba.OutputLinkDPLanes[k] < 2)
                    || (mode_lib->vba.ODMUse[k] == dm_odm_mso_policy_1to4
                            && mode_lib->vba.OutputLinkDPLanes[k] < 4))
                mode_lib->vba.NotEnoughLanesForMSO = true;
        }
    }

    for (i = start_state; static_cast<unsigned int>(i) < v->soc.num_states; ++i) {
        mode_lib->vba.DTBCLKRequiredMoreThanSupported[i] = false;
        for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
            if (mode_lib->vba.BlendingAndTiming[k] == k
                    && dml32_RequiredDTBCLK(mode_lib->vba.RequiresDSC[i][k],
                            mode_lib->vba.PixelClockBackEnd[k],
                            mode_lib->vba.OutputFormat[k],
                            mode_lib->vba.OutputBppPerState[i][k],
                            mode_lib->vba.NumberOfDSCSlices[k], mode_lib->vba.HTotal[k],
                            mode_lib->vba.HActive[k], mode_lib->vba.AudioSampleRate[k],
                            mode_lib->vba.AudioSampleLayout[k])
                            > mode_lib->vba.DTBCLKPerState[i]) {
                mode_lib->vba.DTBCLKRequiredMoreThanSupported[i] = true;
            }
        }
    }

    for (i = start_state; static_cast<unsigned int>(i) < v->soc.num_states; ++i) {
        mode_lib->vba.ODMCombine2To1SupportCheckOK[i] = true;
        mode_lib->vba.ODMCombine4To1SupportCheckOK[i] = true;
        for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
            if (mode_lib->vba.BlendingAndTiming[k] == k
                    && mode_lib->vba.ODMCombineEnablePerState[i][k] == dm_odm_combine_mode_2to1
                    && mode_lib->vba.Output[k] == dm_hdmi) {
                mode_lib->vba.ODMCombine2To1SupportCheckOK[i] = false;
            }
            if (mode_lib->vba.BlendingAndTiming[k] == k
                    && mode_lib->vba.ODMCombineEnablePerState[i][k] == dm_odm_combine_mode_4to1
                    && (mode_lib->vba.Output[k] == dm_dp || mode_lib->vba.Output[k] == dm_edp
                            || mode_lib->vba.Output[k] == dm_hdmi)) {
                mode_lib->vba.ODMCombine4To1SupportCheckOK[i] = false;
            }
        }
    }

    for (i = start_state; static_cast<unsigned int>(i) < v->soc.num_states; i++) {
        mode_lib->vba.DSCCLKRequiredMoreThanSupported[i] = false;
        for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
            if (mode_lib->vba.BlendingAndTiming[k] == k) {
                if (mode_lib->vba.Output[k] == dm_dp || mode_lib->vba.Output[k] == dm_dp2p0
                        || mode_lib->vba.Output[k] == dm_edp) {
                    if (mode_lib->vba.OutputFormat[k] == dm_420) {
                        mode_lib->vba.DSCFormatFactor = 2;
                    } else if (mode_lib->vba.OutputFormat[k] == dm_444) {
                        mode_lib->vba.DSCFormatFactor = 1;
                    } else if (mode_lib->vba.OutputFormat[k] == dm_n422) {
                        mode_lib->vba.DSCFormatFactor = 2;
                    } else {
                        mode_lib->vba.DSCFormatFactor = 1;
                    }
                    if (mode_lib->vba.RequiresDSC[i][k] == true) {
                        if (mode_lib->vba.ODMCombineEnablePerState[i][k]
                                == dm_odm_combine_mode_4to1) {
                            if (mode_lib->vba.PixelClockBackEnd[k] / 12.0 / mode_lib->vba.DSCFormatFactor > (1.0 - mode_lib->vba.DISPCLKDPPCLKDSCCLKDownSpreading / 100.0) * mode_lib->vba.MaxDSCCLK[i])
                                mode_lib->vba.DSCCLKRequiredMoreThanSupported[i] = true;
                        } else if (mode_lib->vba.ODMCombineEnablePerState[i][k]
                                == dm_odm_combine_mode_2to1) {
                            if (mode_lib->vba.PixelClockBackEnd[k] / 6.0 / mode_lib->vba.DSCFormatFactor > (1.0 - mode_lib->vba.DISPCLKDPPCLKDSCCLKDownSpreading / 100.0) * mode_lib->vba.MaxDSCCLK[i])
                                mode_lib->vba.DSCCLKRequiredMoreThanSupported[i] = true;
                        } else {
                            if (mode_lib->vba.PixelClockBackEnd[k] / 3.0 / mode_lib->vba.DSCFormatFactor > (1.0 - mode_lib->vba.DISPCLKDPPCLKDSCCLKDownSpreading / 100.0) * mode_lib->vba.MaxDSCCLK[i])
                                mode_lib->vba.DSCCLKRequiredMoreThanSupported[i] = true;
                        }
                    }
                }
            }
        }
    }

    /* Check DSC Unit and Slices Support */
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalDSCUnitsRequired = 0;

    for (i = start_state; static_cast<unsigned int>(i) < v->soc.num_states; ++i) {
        mode_lib->vba.NotEnoughDSCUnits[i] = false;
        mode_lib->vba.NotEnoughDSCSlices[i] = false;
        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalDSCUnitsRequired = 0;
        mode_lib->vba.PixelsPerLinePerDSCUnitSupport[i] = true;
        for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
            if (mode_lib->vba.RequiresDSC[i][k] == true) {
                if (mode_lib->vba.ODMCombineEnablePerState[i][k] == dm_odm_combine_mode_4to1) {
                    if (mode_lib->vba.HActive[k]
                            > 4 * mode_lib->vba.MaximumPixelsPerLinePerDSCUnit)
                        mode_lib->vba.PixelsPerLinePerDSCUnitSupport[i] = false;
                    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalDSCUnitsRequired = v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalDSCUnitsRequired + 4;
                    if (mode_lib->vba.NumberOfDSCSlices[k] > 16)
                        mode_lib->vba.NotEnoughDSCSlices[i] = true;
                } else if (mode_lib->vba.ODMCombineEnablePerState[i][k] == dm_odm_combine_mode_2to1) {
                    if (mode_lib->vba.HActive[k]
                            > 2 * mode_lib->vba.MaximumPixelsPerLinePerDSCUnit)
                        mode_lib->vba.PixelsPerLinePerDSCUnitSupport[i] = false;
                    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalDSCUnitsRequired = v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalDSCUnitsRequired + 2;
                    if (mode_lib->vba.NumberOfDSCSlices[k] > 8)
                        mode_lib->vba.NotEnoughDSCSlices[i] = true;
                } else {
                    if (mode_lib->vba.HActive[k] > mode_lib->vba.MaximumPixelsPerLinePerDSCUnit)
                        mode_lib->vba.PixelsPerLinePerDSCUnitSupport[i] = false;
                    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalDSCUnitsRequired = v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalDSCUnitsRequired + 1;
                    if (mode_lib->vba.NumberOfDSCSlices[k] > 4)
                        mode_lib->vba.NotEnoughDSCSlices[i] = true;
                }
            }
        }
        if (v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.TotalDSCUnitsRequired > mode_lib->vba.NumberOfDSC)
            mode_lib->vba.NotEnoughDSCUnits[i] = true;
    }

    /*DSC Delay per state*/
    for (i = start_state; static_cast<unsigned int>(i) < v->soc.num_states; ++i) {
        for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
            mode_lib->vba.DSCDelayPerState[i][k] = dml32_DSCDelayRequirement(
                    mode_lib->vba.RequiresDSC[i][k], mode_lib->vba.ODMCombineEnablePerState[i][k],
                    mode_lib->vba.DSCInputBitPerComponent[k],
                    mode_lib->vba.OutputBppPerState[i][k], mode_lib->vba.HActive[k],
                    mode_lib->vba.HTotal[k], mode_lib->vba.NumberOfDSCSlices[k],
                    mode_lib->vba.OutputFormat[k], mode_lib->vba.Output[k],
                    mode_lib->vba.PixelClock[k], mode_lib->vba.PixelClockBackEnd[k],
                    mode_lib->vba.ip.dsc_delay_factor_wa);
        }

        for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
            for (m = 0; m <= mode_lib->vba.NumberOfActiveSurfaces - 1; m++) {
                for (j = 0; static_cast<unsigned int>(j) <= mode_lib->vba.NumberOfActiveSurfaces - 1; j++) {
                    if (mode_lib->vba.BlendingAndTiming[k] == m &&
                            mode_lib->vba.RequiresDSC[i][m] == true) {
                        mode_lib->vba.DSCDelayPerState[i][k] =
                            mode_lib->vba.DSCDelayPerState[i][m];
                    }
                }
            }
        }
    }

    //Calculate Swath, DET Configuration, DCFCLKDeepSleep
    //
    for (i = start_state; i < static_cast<int>(v->soc.num_states); ++i) {
        for (j = 0; j <= 1; ++j) {
            for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
                mode_lib->vba.RequiredDPPCLKThisState[k] = mode_lib->vba.RequiredDPPCLK[i][j][k];
                mode_lib->vba.NoOfDPPThisState[k] = mode_lib->vba.NoOfDPP[i][j][k];
                mode_lib->vba.ODMCombineEnableThisState[k] =
                        mode_lib->vba.ODMCombineEnablePerState[i][k];
            }

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
                    mode_lib->vba.Output,
                    mode_lib->vba.ReadBandwidthLuma,
                    mode_lib->vba.ReadBandwidthChroma,
                    mode_lib->vba.MaximumSwathWidthLuma,
                    mode_lib->vba.MaximumSwathWidthChroma,
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
                    mode_lib->vba.Read256BlockHeightY,
                    mode_lib->vba.Read256BlockHeightC,
                    mode_lib->vba.Read256BlockWidthY,
                    mode_lib->vba.Read256BlockWidthC,
                    mode_lib->vba.ODMCombineEnableThisState,
                    mode_lib->vba.BlendingAndTiming,
                    mode_lib->vba.BytePerPixelY,
                    mode_lib->vba.BytePerPixelC,
                    mode_lib->vba.BytePerPixelInDETY,
                    mode_lib->vba.BytePerPixelInDETC,
                    mode_lib->vba.HActive,
                    mode_lib->vba.HRatio,
                    mode_lib->vba.HRatioChroma,
                    dml_uint(mode_lib->vba.NoOfDPPThisState),
                    /* Output */
                    dml_uint(mode_lib->vba.swath_width_luma_ub_this_state),
                    dml_uint(mode_lib->vba.swath_width_chroma_ub_this_state),
                    mode_lib->vba.SwathWidthYThisState,
                    mode_lib->vba.SwathWidthCThisState,
                    mode_lib->vba.SwathHeightYThisState,
                    mode_lib->vba.SwathHeightCThisState,
                    mode_lib->vba.DETBufferSizeInKByteThisState,
                    mode_lib->vba.DETBufferSizeYThisState,
                    mode_lib->vba.DETBufferSizeCThisState,
                    &mode_lib->vba.UnboundedRequestEnabledThisState,
                    &mode_lib->vba.CompressedBufferSizeInkByteThisState,
                    &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer[0], /* Long CompBufReservedSpaceKBytes */
                    &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_boolean[0], /* bool CompBufReservedSpaceNeedAdjustment */
                    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_boolean_array[0],
                    &mode_lib->vba.ViewportSizeSupport[i][j]);

            for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
                mode_lib->vba.swath_width_luma_ub_all_states[i][j][k] =
                        mode_lib->vba.swath_width_luma_ub_this_state[k];
                mode_lib->vba.swath_width_chroma_ub_all_states[i][j][k] =
                        mode_lib->vba.swath_width_chroma_ub_this_state[k];
                mode_lib->vba.SwathWidthYAllStates[i][j][k] = mode_lib->vba.SwathWidthYThisState[k];
                mode_lib->vba.SwathWidthCAllStates[i][j][k] = mode_lib->vba.SwathWidthCThisState[k];
                mode_lib->vba.SwathHeightYAllStates[i][j][k] = mode_lib->vba.SwathHeightYThisState[k];
                mode_lib->vba.SwathHeightCAllStates[i][j][k] = mode_lib->vba.SwathHeightCThisState[k];
                mode_lib->vba.UnboundedRequestEnabledAllStates[i][j] =
                        mode_lib->vba.UnboundedRequestEnabledThisState;
                mode_lib->vba.CompressedBufferSizeInkByteAllStates[i][j] =
                        mode_lib->vba.CompressedBufferSizeInkByteThisState;
                mode_lib->vba.DETBufferSizeInKByteAllStates[i][j][k] =
                        mode_lib->vba.DETBufferSizeInKByteThisState[k];
                mode_lib->vba.DETBufferSizeYAllStates[i][j][k] =
                        mode_lib->vba.DETBufferSizeYThisState[k];
                mode_lib->vba.DETBufferSizeCAllStates[i][j][k] =
                        mode_lib->vba.DETBufferSizeCThisState[k];
            }
        }
    }

    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        mode_lib->vba.cursor_bw[k] = mode_lib->vba.NumberOfCursors[k] * mode_lib->vba.CursorWidth[k][0]
                * mode_lib->vba.CursorBPP[k][0] / 8.0
                / (mode_lib->vba.HTotal[k] / mode_lib->vba.PixelClock[k]) * mode_lib->vba.VRatio[k];
    }

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
            mode_lib->vba.BytePerPixelY,
            mode_lib->vba.ViewportWidthChroma,
            mode_lib->vba.ViewportHeightChroma,
            mode_lib->vba.BytePerPixelC,
            mode_lib->vba.SurfaceWidthY,
            mode_lib->vba.SurfaceWidthC,
            mode_lib->vba.SurfaceHeightY,
            mode_lib->vba.SurfaceHeightC,
            mode_lib->vba.Read256BlockWidthY,
            mode_lib->vba.Read256BlockWidthC,
            mode_lib->vba.Read256BlockHeightY,
            mode_lib->vba.Read256BlockHeightC,
            mode_lib->vba.MacroTileWidthY,
            mode_lib->vba.MacroTileWidthC,
            mode_lib->vba.MacroTileHeightY,
            mode_lib->vba.MacroTileHeightC,
            mode_lib->vba.DCCMetaPitchY,
            mode_lib->vba.DCCMetaPitchC,

            /* Output */
            mode_lib->vba.SurfaceSizeInMALL,
            &mode_lib->vba.ExceededMALLSize);

}

// Per-state PTE rows, return bandwidth, urgent latency and DCFCLK states.
static DML32_NOINLINE void dml32_support_rows_and_return_bandwidth(struct display_mode_lib *mode_lib, int start_state)
{
    struct vba_vars_st *v = &mode_lib->vba;
    int i, j;
    unsigned int k, m;

    for (i = start_state; static_cast<unsigned int>(i) < v->soc.num_states; i++) {
        for (j = 0; j < 2; j++) {
            for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
                mode_lib->vba.swath_width_luma_ub_this_state[k] =
                        mode_lib->vba.swath_width_luma_ub_all_states[i][j][k];
                mode_lib->vba.swath_width_chroma_ub_this_state[k] =
                        mode_lib->vba.swath_width_chroma_ub_all_states[i][j][k];
                mode_lib->vba.SwathWidthYThisState[k] = mode_lib->vba.SwathWidthYAllStates[i][j][k];
                mode_lib->vba.SwathWidthCThisState[k] = mode_lib->vba.SwathWidthCAllStates[i][j][k];
                mode_lib->vba.SwathHeightYThisState[k] = mode_lib->vba.SwathHeightYAllStates[i][j][k];
                mode_lib->vba.SwathHeightCThisState[k] = mode_lib->vba.SwathHeightCAllStates[i][j][k];
                mode_lib->vba.DETBufferSizeInKByteThisState[k] =
                        mode_lib->vba.DETBufferSizeInKByteAllStates[i][j][k];
                mode_lib->vba.DETBufferSizeYThisState[k] =
                        mode_lib->vba.DETBufferSizeYAllStates[i][j][k];
                mode_lib->vba.DETBufferSizeCThisState[k] =
                        mode_lib->vba.DETBufferSizeCAllStates[i][j][k];
                mode_lib->vba.RequiredDPPCLKThisState[k] = mode_lib->vba.RequiredDPPCLK[i][j][k];
                mode_lib->vba.NoOfDPPThisState[k] = mode_lib->vba.NoOfDPP[i][j][k];
            }

            mode_lib->vba.TotalNumberOfDCCActiveDPP[i][j] = 0;
            for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
                if (mode_lib->vba.DCCEnable[k] == true) {
                    mode_lib->vba.TotalNumberOfDCCActiveDPP[i][j] =
                            mode_lib->vba.TotalNumberOfDCCActiveDPP[i][j]
                                    + mode_lib->vba.NoOfDPP[i][j][k];
                }
            }

            for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].PixelClock = mode_lib->vba.PixelClock[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].DPPPerSurface = mode_lib->vba.NoOfDPP[i][j][k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].SourceRotation = mode_lib->vba.SourceRotation[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].ViewportHeight = mode_lib->vba.ViewportHeight[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].ViewportHeightChroma = mode_lib->vba.ViewportHeightChroma[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].BlockWidth256BytesY = mode_lib->vba.Read256BlockWidthY[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].BlockHeight256BytesY = mode_lib->vba.Read256BlockHeightY[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].BlockWidth256BytesC = mode_lib->vba.Read256BlockWidthC[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].BlockHeight256BytesC = mode_lib->vba.Read256BlockHeightC[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].BlockWidthY = mode_lib->vba.MacroTileWidthY[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].BlockHeightY = mode_lib->vba.MacroTileHeightY[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].BlockWidthC = mode_lib->vba.MacroTileWidthC[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].BlockHeightC = mode_lib->vba.MacroTileHeightC[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].InterlaceEnable = mode_lib->vba.Interlace[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].HTotal = mode_lib->vba.HTotal[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].DCCEnable = mode_lib->vba.DCCEnable[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].SourcePixelFormat = mode_lib->vba.SourcePixelFormat[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].SurfaceTiling = mode_lib->vba.SurfaceTiling[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].BytePerPixelY = mode_lib->vba.BytePerPixelY[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].BytePerPixelC = mode_lib->vba.BytePerPixelC[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].ProgressiveToInterlaceUnitInOPP =
                mode_lib->vba.ProgressiveToInterlaceUnitInOPP;
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].VRatio = mode_lib->vba.VRatio[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].VRatioChroma = mode_lib->vba.VRatioChroma[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].VTaps = mode_lib->vba.vtaps[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].VTapsChroma = mode_lib->vba.VTAPsChroma[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].PitchY = mode_lib->vba.PitchY[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].DCCMetaPitchY = mode_lib->vba.DCCMetaPitchY[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].PitchC = mode_lib->vba.PitchC[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].DCCMetaPitchC = mode_lib->vba.DCCMetaPitchC[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].ViewportStationary = mode_lib->vba.ViewportStationary[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].ViewportXStart = mode_lib->vba.ViewportXStartY[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].ViewportYStart = mode_lib->vba.ViewportYStartY[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].ViewportXStartC = mode_lib->vba.ViewportXStartC[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].ViewportYStartC = mode_lib->vba.ViewportYStartC[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].FORCE_ONE_ROW_FOR_FRAME = mode_lib->vba.ForceOneRowForFrame[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].SwathHeightY = mode_lib->vba.SwathHeightYThisState[k];
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters[k].SwathHeightC = mode_lib->vba.SwathHeightCThisState[k];
            }

            {
                dml32_CalculateVMRowAndSwath(
                        mode_lib->vba.NumberOfActiveSurfaces,
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SurfParameters,
                        mode_lib->vba.SurfaceSizeInMALL,
                        mode_lib->vba.PTEBufferSizeInRequestsLuma,
                        mode_lib->vba.PTEBufferSizeInRequestsChroma,
                        mode_lib->vba.DCCMetaBufferSizeBytes,
                        mode_lib->vba.UseMALLForStaticScreen,
                        mode_lib->vba.UsesMALLForPStateChange,
                        mode_lib->vba.MALLAllocatedForDCNFinal,
                        mode_lib->vba.SwathWidthYThisState,
                        mode_lib->vba.SwathWidthCThisState,
                        mode_lib->vba.GPUVMEnable,
                        mode_lib->vba.HostVMEnable,
                        mode_lib->vba.HostVMMaxNonCachedPageTableLevels,
                        mode_lib->vba.GPUVMMaxPageTableLevels,
                        mode_lib->vba.GPUVMMinPageSizeKBytes,
                        mode_lib->vba.HostVMMinPageSize,

                        /* Output */
                        mode_lib->vba.PTEBufferSizeNotExceededPerState,
                        mode_lib->vba.DCCMetaBufferSizeNotExceededPerState,
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[0],
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[1],
                        mode_lib->vba.dpte_row_height,
                        mode_lib->vba.dpte_row_height_chroma,
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[2],
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[3],
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[4],
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[5],
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[6],
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[7],
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[8],
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[9],
                        mode_lib->vba.meta_row_height,
                        mode_lib->vba.meta_row_height_chroma,
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[10],
                        mode_lib->vba.dpte_group_bytes,
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[11],
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[12],
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[13],
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[14],
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[15],
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[16],
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[17],
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[18],
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[19],
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[20],
                        mode_lib->vba.PrefetchLinesYThisState,
                        mode_lib->vba.PrefetchLinesCThisState,
                        mode_lib->vba.PrefillY,
                        mode_lib->vba.PrefillC,
                        mode_lib->vba.MaxNumSwY,
                        mode_lib->vba.MaxNumSwC,
                        mode_lib->vba.meta_row_bandwidth_this_state,
                        mode_lib->vba.dpte_row_bandwidth_this_state,
                        mode_lib->vba.DPTEBytesPerRowThisState,
                        mode_lib->vba.PDEAndMetaPTEBytesPerFrameThisState,
                        mode_lib->vba.MetaRowBytesThisState,
                        mode_lib->vba.use_one_row_for_frame_this_state,
                        mode_lib->vba.use_one_row_for_frame_flip_this_state,
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_boolean_array[0], // Boolean UsesMALLForStaticScreen[]
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_boolean_array[1], // Boolean PTE_BUFFER_MODE[]
                        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[21]); // Long BIGK_FRAGMENT_SIZE[]
            }

            for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
                mode_lib->vba.PrefetchLinesY[i][j][k] = mode_lib->vba.PrefetchLinesYThisState[k];
                mode_lib->vba.PrefetchLinesC[i][j][k] = mode_lib->vba.PrefetchLinesCThisState[k];
                mode_lib->vba.meta_row_bandwidth[i][j][k] =
                        mode_lib->vba.meta_row_bandwidth_this_state[k];
                mode_lib->vba.dpte_row_bandwidth[i][j][k] =
                        mode_lib->vba.dpte_row_bandwidth_this_state[k];
                mode_lib->vba.DPTEBytesPerRow[i][j][k] = mode_lib->vba.DPTEBytesPerRowThisState[k];
                mode_lib->vba.PDEAndMetaPTEBytesPerFrame[i][j][k] =
                        mode_lib->vba.PDEAndMetaPTEBytesPerFrameThisState[k];
                mode_lib->vba.MetaRowBytes[i][j][k] = mode_lib->vba.MetaRowBytesThisState[k];
                mode_lib->vba.use_one_row_for_frame[i][j][k] =
                        mode_lib->vba.use_one_row_for_frame_this_state[k];
                mode_lib->vba.use_one_row_for_frame_flip[i][j][k] =
                        mode_lib->vba.use_one_row_for_frame_flip_this_state[k];
            }

            mode_lib->vba.PTEBufferSizeNotExceeded[i][j] = true;
            for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
                if (mode_lib->vba.PTEBufferSizeNotExceededPerState[k] == false)
                    mode_lib->vba.PTEBufferSizeNotExceeded[i][j] = false;
            }

            mode_lib->vba.DCCMetaBufferSizeNotExceeded[i][j] = true;
            for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
                if (mode_lib->vba.DCCMetaBufferSizeNotExceededPerState[k] == false)
                    mode_lib->vba.DCCMetaBufferSizeNotExceeded[i][j] = false;
            }

            mode_lib->vba.UrgLatency[i] = dml32_CalculateUrgentLatency(
                    mode_lib->vba.UrgentLatencyPixelDataOnly,
                    mode_lib->vba.UrgentLatencyPixelMixedWithVMData,
                    mode_lib->vba.UrgentLatencyVMDataOnly, mode_lib->vba.DoUrgentLatencyAdjustment,
                    mode_lib->vba.UrgentLatencyAdjustmentFabricClockComponent,
                    mode_lib->vba.UrgentLatencyAdjustmentFabricClockReference,
                    mode_lib->vba.FabricClockPerState[i]);

            //bool   NotUrgentLatencyHiding[DML32_NUM_PLANES];
            for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
                dml32_CalculateUrgentBurstFactor(
                        mode_lib->vba.UsesMALLForPStateChange[k],
                        mode_lib->vba.swath_width_luma_ub_this_state[k],
                        mode_lib->vba.swath_width_chroma_ub_this_state[k],
                        mode_lib->vba.SwathHeightYThisState[k],
                        mode_lib->vba.SwathHeightCThisState[k],
                        static_cast<double>(mode_lib->vba.HTotal[k]) / mode_lib->vba.PixelClock[k],
                        mode_lib->vba.UrgLatency[i],
                        mode_lib->vba.CursorBufferSize,
                        mode_lib->vba.CursorWidth[k][0],
                        mode_lib->vba.CursorBPP[k][0],
                        mode_lib->vba.VRatio[k],
                        mode_lib->vba.VRatioChroma[k],
                        mode_lib->vba.BytePerPixelInDETY[k],
                        mode_lib->vba.BytePerPixelInDETC[k],
                        mode_lib->vba.DETBufferSizeYThisState[k],
                        mode_lib->vba.DETBufferSizeCThisState[k],
                        /* Output */
                        &mode_lib->vba.UrgentBurstFactorCursor[k],
                        &mode_lib->vba.UrgentBurstFactorLuma[k],
                        &mode_lib->vba.UrgentBurstFactorChroma[k],
                        &mode_lib->vba.NoUrgentLatencyHiding[k]);
            }

            dml32_CalculateDCFCLKDeepSleep(
                    mode_lib->vba.NumberOfActiveSurfaces,
                    mode_lib->vba.BytePerPixelY,
                    mode_lib->vba.BytePerPixelC,
                    mode_lib->vba.VRatio,
                    mode_lib->vba.VRatioChroma,
                    mode_lib->vba.SwathWidthYThisState,
                    mode_lib->vba.SwathWidthCThisState,
                    dml_uint(mode_lib->vba.NoOfDPPThisState),
                    mode_lib->vba.HRatio,
                    mode_lib->vba.HRatioChroma,
                    mode_lib->vba.PixelClock,
                    mode_lib->vba.PSCL_FACTOR,
                    mode_lib->vba.PSCL_FACTOR_CHROMA,
                    mode_lib->vba.RequiredDPPCLKThisState,
                    mode_lib->vba.ReadBandwidthLuma,
                    mode_lib->vba.ReadBandwidthChroma,
                    mode_lib->vba.ReturnBusWidth,

                    /* Output */
                    &mode_lib->vba.ProjectedDCFCLKDeepSleep[i][j]);
        }
    }

    //Calculate Return BW
    for (i = start_state; i < static_cast<int>(v->soc.num_states); ++i) {
        for (j = 0; j <= 1; ++j) {
            for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
                if (mode_lib->vba.BlendingAndTiming[k] == k) {
                    mode_lib->vba.WritebackDelayTime[k] = 0.0;
                }
            }
            for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
                for (m = 0; m <= mode_lib->vba.NumberOfActiveSurfaces - 1; m++) {
                    if (mode_lib->vba.BlendingAndTiming[k] == m) {
                        mode_lib->vba.WritebackDelayTime[k] =
                                mode_lib->vba.WritebackDelayTime[m];
                    }
                }
            }
            mode_lib->vba.MaxMaxVStartup[i][j] = 0;
            for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
                mode_lib->vba.MaximumVStartup[i][j][k] = ((mode_lib->vba.Interlace[k] &&
                                !mode_lib->vba.ProgressiveToInterlaceUnitInOPP) ?
                                dml_floor((mode_lib->vba.VTotal[k] -
                                    mode_lib->vba.VActive[k]) / 2.0, 1.0) :
                                mode_lib->vba.VTotal[k] - mode_lib->vba.VActive[k])
                                - dml_max(1.0, dml_ceil(1.0 *
                                    mode_lib->vba.WritebackDelayTime[k] /
                                    (mode_lib->vba.HTotal[k] /
                                    mode_lib->vba.PixelClock[k]), 1.0));

                // Clamp to max OTG vstartup register limit
                if (mode_lib->vba.MaximumVStartup[i][j][k] > 1023)
                    mode_lib->vba.MaximumVStartup[i][j][k] = 1023;

                mode_lib->vba.MaxMaxVStartup[i][j] = dml_max(mode_lib->vba.MaxMaxVStartup[i][j],
                        mode_lib->vba.MaximumVStartup[i][j][k]);
            }
        }
    }

    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.ReorderingBytes = mode_lib->vba.NumberOfChannels
            * dml_max3(mode_lib->vba.UrgentOutOfOrderReturnPerChannelPixelDataOnly,
                    mode_lib->vba.UrgentOutOfOrderReturnPerChannelPixelMixedWithVMData,
                    mode_lib->vba.UrgentOutOfOrderReturnPerChannelVMDataOnly);

    dml32_CalculateMinAndMaxPrefetchMode(mode_lib->vba.AllowForPStateChangeOrStutterInVBlankFinal,
            &mode_lib->vba.MinPrefetchMode,
            &mode_lib->vba.MaxPrefetchMode);

    for (i = start_state; i < static_cast<int>(v->soc.num_states); ++i) {
        for (j = 0; j <= 1; ++j)
            mode_lib->vba.DCFCLKState[i][j] = mode_lib->vba.DCFCLKPerState[i];
    }

    /* Immediate Flip and MALL parameters */
    mode_lib->vba.ImmediateFlipRequiredFinal = false;
    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        mode_lib->vba.ImmediateFlipRequiredFinal = mode_lib->vba.ImmediateFlipRequiredFinal
                || (mode_lib->vba.ImmediateFlipRequirement[k] == dm_immediate_flip_required);
    }

    mode_lib->vba.ImmediateFlipRequiredButTheRequirementForEachSurfaceIsNotSpecified = false;
    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        mode_lib->vba.ImmediateFlipRequiredButTheRequirementForEachSurfaceIsNotSpecified =
                mode_lib->vba.ImmediateFlipRequiredButTheRequirementForEachSurfaceIsNotSpecified
                        || ((mode_lib->vba.ImmediateFlipRequirement[k]
                                != dm_immediate_flip_required)
                                && (mode_lib->vba.ImmediateFlipRequirement[k]
                                        != dm_immediate_flip_not_required));
    }
    mode_lib->vba.ImmediateFlipRequiredButTheRequirementForEachSurfaceIsNotSpecified =
            mode_lib->vba.ImmediateFlipRequiredButTheRequirementForEachSurfaceIsNotSpecified
                    && mode_lib->vba.ImmediateFlipRequiredFinal;

    mode_lib->vba.ImmediateFlipOrHostVMAndPStateWithMALLFullFrameOrPhantomPipe = false;
    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        mode_lib->vba.ImmediateFlipOrHostVMAndPStateWithMALLFullFrameOrPhantomPipe =
            mode_lib->vba.ImmediateFlipOrHostVMAndPStateWithMALLFullFrameOrPhantomPipe ||
            ((mode_lib->vba.HostVMEnable == true || mode_lib->vba.ImmediateFlipRequirement[k] !=
                    dm_immediate_flip_not_required) &&
            (mode_lib->vba.UsesMALLForPStateChange[k] == dm_use_mall_pstate_change_full_frame ||
            mode_lib->vba.UsesMALLForPStateChange[k] == dm_use_mall_pstate_change_phantom_pipe));
    }

    mode_lib->vba.InvalidCombinationOfMALLUseForPStateAndStaticScreen = false;
    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        mode_lib->vba.InvalidCombinationOfMALLUseForPStateAndStaticScreen =
            mode_lib->vba.InvalidCombinationOfMALLUseForPStateAndStaticScreen
            || ((mode_lib->vba.UseMALLForStaticScreen[k] == dm_use_mall_static_screen_enable
            || mode_lib->vba.UseMALLForStaticScreen[k] == dm_use_mall_static_screen_optimize)
            && (mode_lib->vba.UsesMALLForPStateChange[k] == dm_use_mall_pstate_change_phantom_pipe))
            || ((mode_lib->vba.UseMALLForStaticScreen[k] == dm_use_mall_static_screen_disable
            || mode_lib->vba.UseMALLForStaticScreen[k] == dm_use_mall_static_screen_optimize)
            && (mode_lib->vba.UsesMALLForPStateChange[k] == dm_use_mall_pstate_change_full_frame));
    }

    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.FullFrameMALLPStateMethod = false;
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SubViewportMALLPStateMethod = false;
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.PhantomPipeMALLPStateMethod = false;

    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        if (mode_lib->vba.UsesMALLForPStateChange[k] == dm_use_mall_pstate_change_full_frame)
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.FullFrameMALLPStateMethod = true;
        if (mode_lib->vba.UsesMALLForPStateChange[k] == dm_use_mall_pstate_change_sub_viewport)
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SubViewportMALLPStateMethod = true;
        if (mode_lib->vba.UsesMALLForPStateChange[k] == dm_use_mall_pstate_change_phantom_pipe)
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.PhantomPipeMALLPStateMethod = true;
    }
    mode_lib->vba.InvalidCombinationOfMALLUseForPState = (v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SubViewportMALLPStateMethod
            != v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.PhantomPipeMALLPStateMethod) || (v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.SubViewportMALLPStateMethod && v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.FullFrameMALLPStateMethod);

    // ip.use_min_dcfclk is 0 on DCN3.2, so dml32_UseMinimumDCFCLK never runs.

    for (i = start_state; i < static_cast<int>(v->soc.num_states); ++i) {
        for (j = 0; j <= 1; ++j) {
            mode_lib->vba.ReturnBWPerState[i][j] = dml32_get_return_bw_mbps(&mode_lib->vba.soc, i,
                    mode_lib->vba.HostVMEnable, mode_lib->vba.DCFCLKState[i][j],
                    mode_lib->vba.FabricClockPerState[i], mode_lib->vba.DRAMSpeedPerState[i]);
        }
    }

    //Re-ordering Buffer Support Check
    for (i = start_state; i < static_cast<int>(v->soc.num_states); ++i) {
        for (j = 0; j <= 1; ++j) {
            if ((mode_lib->vba.ROBBufferSizeInKByte - mode_lib->vba.PixelChunkSizeInKByte) * 1024
                    / mode_lib->vba.ReturnBWPerState[i][j]
                    > (mode_lib->vba.RoundTripPingLatencyCycles + 32)
                            / mode_lib->vba.DCFCLKState[i][j]
                            + v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.ReorderingBytes / mode_lib->vba.ReturnBWPerState[i][j]) {
                mode_lib->vba.ROBSupport[i][j] = true;
            } else {
                mode_lib->vba.ROBSupport[i][j] = false;
            }
        }
    }

    //Vertical Active BW support check
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.MaxTotalVActiveRDBandwidth = 0;

    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.MaxTotalVActiveRDBandwidth += mode_lib->vba.ReadBandwidthLuma[k]
                + mode_lib->vba.ReadBandwidthChroma[k];
    }

    for (i = start_state; i < static_cast<int>(v->soc.num_states); ++i) {
        for (j = 0; j <= 1; ++j) {
            mode_lib->vba.MaxTotalVerticalActiveAvailableBandwidth[i][j] =
                dml_min3(mode_lib->vba.ReturnBusWidth * mode_lib->vba.DCFCLKState[i][j]
                    * mode_lib->vba.MaxAveragePercentOfIdealSDPPortBWDisplayCanUseInNormalSystemOperation / 100,
                    mode_lib->vba.FabricClockPerState[i]
                    * mode_lib->vba.FabricDatapathToDCNDataReturn
                    * mode_lib->vba.MaxAveragePercentOfIdealFabricBWDisplayCanUseInNormalSystemOperation / 100,
                    mode_lib->vba.DRAMSpeedPerState[i]
                    * mode_lib->vba.NumberOfChannels
                    * mode_lib->vba.DRAMChannelWidth
                    * (i < 2 ? mode_lib->vba.MaxAveragePercentOfIdealDRAMBWDisplayCanUseInNormalSystemOperationSTROBE : mode_lib->vba.MaxAveragePercentOfIdealDRAMBWDisplayCanUseInNormalSystemOperation) / 100);

            if (v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.MaxTotalVActiveRDBandwidth
                    <= mode_lib->vba.MaxTotalVerticalActiveAvailableBandwidth[i][j]) {
                mode_lib->vba.TotalVerticalActiveBandwidthSupport[i][j] = true;
            } else {
                mode_lib->vba.TotalVerticalActiveBandwidthSupport[i][j] = false;
            }
        }
    }
}

// One pass of the per-state prefetch iteration (the body of the upstream do/while
// over VStartup and the prefetch modes).
/* One surface of the prefetch-attempt loop, split out unchanged so the
 * caller's frame does not carry this call's outgoing arguments. */
static DML32_NOINLINE void dml32_support_prefetch_schedule_surface(struct display_mode_lib *mode_lib, int i, int j,
        unsigned int k)
{
    struct vba_vars_st *v = &mode_lib->vba;

    mode_lib->vba.TWait = dml32_CalculateTWait(
            mode_lib->vba.PrefetchModePerState[i][j],
            mode_lib->vba.UsesMALLForPStateChange[k],
            mode_lib->vba.SynchronizeDRRDisplaysForUCLKPStateChangeFinal,
            mode_lib->vba.DRRDisplay[k],
            mode_lib->vba.DRAMClockChangeLatency,
            mode_lib->vba.FCLKChangeLatency, mode_lib->vba.UrgLatency[i],
            mode_lib->vba.SREnterPlusExitTime);

    /* Linux zeroes sizeof(DmlPipe) = 208 bytes from the start of this dummy struct,
     * whose rows hold DC__NUM_DPP__MAX = 8 entries: rows 0..5 and entries 0..3 of
     * row 6 of dummy_integer_array, not myPipe. */
    for (unsigned int row = 0; row < 7; ++row)
        for (unsigned int entry = 0; entry < (row < 6 ? 8u : 4u) && entry < DML32_NUM_PLANES; ++entry)
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer_array[row][entry] = 0;
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.myPipe.Dppclk = mode_lib->vba.RequiredDPPCLK[i][j][k];
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.myPipe.Dispclk = mode_lib->vba.RequiredDISPCLK[i][j];
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.myPipe.PixelClock = mode_lib->vba.PixelClock[k];
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.myPipe.DCFClkDeepSleep = mode_lib->vba.ProjectedDCFCLKDeepSleep[i][j];
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.myPipe.DPPPerSurface = mode_lib->vba.NoOfDPP[i][j][k];
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.myPipe.ScalerEnabled = mode_lib->vba.ScalerEnabled[k];
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.myPipe.SourceRotation = mode_lib->vba.SourceRotation[k];
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.myPipe.BlockWidth256BytesY = mode_lib->vba.Read256BlockWidthY[k];
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.myPipe.BlockHeight256BytesY = mode_lib->vba.Read256BlockHeightY[k];
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.myPipe.BlockWidth256BytesC = mode_lib->vba.Read256BlockWidthC[k];
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.myPipe.BlockHeight256BytesC = mode_lib->vba.Read256BlockHeightC[k];
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.myPipe.InterlaceEnable = mode_lib->vba.Interlace[k];
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.myPipe.NumberOfCursors = mode_lib->vba.NumberOfCursors[k];
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.myPipe.VBlank = mode_lib->vba.VTotal[k] - mode_lib->vba.VActive[k];
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.myPipe.HTotal = mode_lib->vba.HTotal[k];
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.myPipe.HActive = mode_lib->vba.HActive[k];
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.myPipe.DCCEnable = mode_lib->vba.DCCEnable[k];
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.myPipe.ODMMode = mode_lib->vba.ODMCombineEnablePerState[i][k];
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.myPipe.SourcePixelFormat = mode_lib->vba.SourcePixelFormat[k];
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.myPipe.BytePerPixelY = mode_lib->vba.BytePerPixelY[k];
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.myPipe.BytePerPixelC = mode_lib->vba.BytePerPixelC[k];
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.myPipe.ProgressiveToInterlaceUnitInOPP =
            mode_lib->vba.ProgressiveToInterlaceUnitInOPP;

    mode_lib->vba.NoTimeForPrefetch[i][j][k] =
        dml32_CalculatePrefetchSchedule(
            v,
            k,
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.HostVMInefficiencyFactor,
            &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.myPipe,
            v->DSCDelayPerState[i][k],
            v->SwathWidthYThisState[k] / v->HRatio[k],
            dml_min(v->MaxVStartup, v->MaximumVStartup[i][j][k]),
            v->MaximumVStartup[i][j][k],
            v->UrgLatency[i],
            v->ExtraLatency,
            v->TimeCalc,
            v->PDEAndMetaPTEBytesPerFrame[i][j][k],
            v->MetaRowBytes[i][j][k],
            v->DPTEBytesPerRow[i][j][k],
            v->PrefetchLinesY[i][j][k],
            v->SwathWidthYThisState[k],
            v->PrefillY[k],
            v->MaxNumSwY[k],
            v->PrefetchLinesC[i][j][k],
            v->SwathWidthCThisState[k],
            v->PrefillC[k],
            v->MaxNumSwC[k],
            v->swath_width_luma_ub_this_state[k],
            v->swath_width_chroma_ub_this_state[k],
            v->SwathHeightYThisState[k],
            v->SwathHeightCThisState[k], v->TWait,
            (v->DRAMSpeedPerState[i] <= MEM_STROBE_FREQ_MHZ || v->DCFCLKState[i][j] <= DCFCLK_FREQ_EXTRA_PREFETCH_REQ_MHZ) ?
                    mode_lib->vba.ip.min_prefetch_in_strobe_us : 0,
            mode_lib->vba.PrefetchModePerState[i][j] > 0 || mode_lib->vba.DRAMClockChangeRequirementFinal == false,

            /* Output */
            &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.DSTXAfterScaler[k],
            &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.DSTYAfterScaler[k],
            &v->LineTimesForPrefetch[k],
            &v->PrefetchBW[k],
            &v->LinesForMetaPTE[k],
            &v->LinesForMetaAndDPTERow[k],
            &v->VRatioPreY[i][j][k],
            &v->VRatioPreC[i][j][k],
            &v->RequiredPrefetchPixelDataBWLuma[0][0][k],
            &v->RequiredPrefetchPixelDataBWChroma[0][0][k],
            &v->NoTimeForDynamicMetadata[i][j][k],
            &v->Tno_bw[k],
            &v->prefetch_vmrow_bw[k],
            &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_single[0],         // double *Tdmdl_vm
            &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_single[1],         // double *Tdmdl
            &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_single[2],         // double *TSetup
            &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer[0],                                        // unsigned int   *VUpdateOffsetPix
            &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_single[3],         // unsigned int   *VUpdateWidthPix
            &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_single[4]);        // unsigned int   *VReadyOffsetPix
}

/* Urgent burst factors and cursor prefetch bandwidth of one prefetch attempt. */
static DML32_NOINLINE void dml32_support_prefetch_urgent_burst(struct display_mode_lib *mode_lib, int i, int j)
{
    struct vba_vars_st *v = &mode_lib->vba;
    unsigned int k;

    for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
        dml32_CalculateUrgentBurstFactor(
                mode_lib->vba.UsesMALLForPStateChange[k],
                mode_lib->vba.swath_width_luma_ub_this_state[k],
                mode_lib->vba.swath_width_chroma_ub_this_state[k],
                mode_lib->vba.SwathHeightYThisState[k],
                mode_lib->vba.SwathHeightCThisState[k],
                mode_lib->vba.HTotal[k] / mode_lib->vba.PixelClock[k],
                mode_lib->vba.UrgLatency[i], mode_lib->vba.CursorBufferSize,
                mode_lib->vba.CursorWidth[k][0], mode_lib->vba.CursorBPP[k][0],
                mode_lib->vba.VRatioPreY[i][j][k],
                mode_lib->vba.VRatioPreC[i][j][k],
                mode_lib->vba.BytePerPixelInDETY[k],
                mode_lib->vba.BytePerPixelInDETC[k],
                mode_lib->vba.DETBufferSizeYThisState[k],
                mode_lib->vba.DETBufferSizeCThisState[k],
                /* Output */
                &mode_lib->vba.UrgentBurstFactorCursorPre[k],
                &mode_lib->vba.UrgentBurstFactorLumaPre[k],
                &mode_lib->vba.UrgentBurstFactorChromaPre[k],
                &mode_lib->vba.NotUrgentLatencyHidingPre[k]);

        v->cursor_bw_pre[k] = mode_lib->vba.NumberOfCursors[k] * mode_lib->vba.CursorWidth[k][0] * mode_lib->vba.CursorBPP[k][0] /
                8.0 / (mode_lib->vba.HTotal[k] / mode_lib->vba.PixelClock[k]) * v->VRatioPreY[i][j][k];
    }
}

/* Prefetch bandwidth support of one prefetch attempt. */
static DML32_NOINLINE void dml32_support_prefetch_bandwidth(struct display_mode_lib *mode_lib, int i, int j)
{
    struct vba_vars_st *v = &mode_lib->vba;

    {
        dml32_CalculatePrefetchBandwithSupport(
                mode_lib->vba.NumberOfActiveSurfaces,
                mode_lib->vba.ReturnBWPerState[i][j],
                mode_lib->vba.NotUrgentLatencyHidingPre,
                mode_lib->vba.ReadBandwidthLuma,
                mode_lib->vba.ReadBandwidthChroma,
                mode_lib->vba.RequiredPrefetchPixelDataBWLuma[0][0],
                mode_lib->vba.RequiredPrefetchPixelDataBWChroma[0][0],
                mode_lib->vba.cursor_bw,
                mode_lib->vba.meta_row_bandwidth_this_state,
                mode_lib->vba.dpte_row_bandwidth_this_state,
                mode_lib->vba.cursor_bw_pre,
                mode_lib->vba.prefetch_vmrow_bw,
                dml_uint(mode_lib->vba.NoOfDPPThisState),
                mode_lib->vba.UrgentBurstFactorLuma,
                mode_lib->vba.UrgentBurstFactorChroma,
                mode_lib->vba.UrgentBurstFactorCursor,
                mode_lib->vba.UrgentBurstFactorLumaPre,
                mode_lib->vba.UrgentBurstFactorChromaPre,
                mode_lib->vba.UrgentBurstFactorCursorPre,
                v->PrefetchBW,
                v->VRatio,
                v->MaxVRatioPre,

                /* output */
                &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_single[0],   // Single  *PrefetchBandwidth
                &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_single[1],   // Single  *FractionOfUrgentBandwidth
                &mode_lib->vba.PrefetchSupported[i][j]);
    }
}

/* Immediate-flip support of one prefetch attempt. */
static DML32_NOINLINE void dml32_support_prefetch_immediate_flip(struct display_mode_lib *mode_lib, int i, int j)
{
    struct vba_vars_st *v = &mode_lib->vba;
    unsigned int k;

    if (mode_lib->vba.PrefetchSupported[i][j] == true
            && mode_lib->vba.VRatioInPrefetchSupported[i][j] == true) {
        mode_lib->vba.BandwidthAvailableForImmediateFlip =
                dml32_CalculateBandwidthAvailableForImmediateFlip(
                mode_lib->vba.NumberOfActiveSurfaces,
                mode_lib->vba.ReturnBWPerState[i][j],
                mode_lib->vba.ReadBandwidthLuma,
                mode_lib->vba.ReadBandwidthChroma,
                mode_lib->vba.RequiredPrefetchPixelDataBWLuma[0][0],
                mode_lib->vba.RequiredPrefetchPixelDataBWChroma[0][0],
                mode_lib->vba.cursor_bw,
                mode_lib->vba.cursor_bw_pre,
                dml_uint(mode_lib->vba.NoOfDPPThisState),
                mode_lib->vba.UrgentBurstFactorLuma,
                mode_lib->vba.UrgentBurstFactorChroma,
                mode_lib->vba.UrgentBurstFactorCursor,
                mode_lib->vba.UrgentBurstFactorLumaPre,
                mode_lib->vba.UrgentBurstFactorChromaPre,
                mode_lib->vba.UrgentBurstFactorCursorPre);

        mode_lib->vba.TotImmediateFlipBytes = 0.0;
        for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
            if (!(mode_lib->vba.ImmediateFlipRequirement[k] ==
                    dm_immediate_flip_not_required)) {
                mode_lib->vba.TotImmediateFlipBytes =
                        mode_lib->vba.TotImmediateFlipBytes
                    + mode_lib->vba.NoOfDPP[i][j][k]
                    * mode_lib->vba.PDEAndMetaPTEBytesPerFrame[i][j][k]
                    + mode_lib->vba.MetaRowBytes[i][j][k];
                if (mode_lib->vba.use_one_row_for_frame_flip[i][j][k]) {
                    mode_lib->vba.TotImmediateFlipBytes =
                        mode_lib->vba.TotImmediateFlipBytes + 2
                    * mode_lib->vba.DPTEBytesPerRow[i][j][k];
                } else {
                    mode_lib->vba.TotImmediateFlipBytes =
                        mode_lib->vba.TotImmediateFlipBytes
                    + mode_lib->vba.DPTEBytesPerRow[i][j][k];
                }
            }
        }

        for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
            dml32_CalculateFlipSchedule(v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.HostVMInefficiencyFactor,
                mode_lib->vba.ExtraLatency,
                mode_lib->vba.UrgLatency[i],
                mode_lib->vba.GPUVMMaxPageTableLevels,
                mode_lib->vba.HostVMEnable,
                mode_lib->vba.HostVMMaxNonCachedPageTableLevels,
                mode_lib->vba.GPUVMEnable,
                mode_lib->vba.HostVMMinPageSize,
                mode_lib->vba.PDEAndMetaPTEBytesPerFrame[i][j][k],
                mode_lib->vba.MetaRowBytes[i][j][k],
                mode_lib->vba.DPTEBytesPerRow[i][j][k],
                mode_lib->vba.BandwidthAvailableForImmediateFlip,
                mode_lib->vba.TotImmediateFlipBytes,
                mode_lib->vba.SourcePixelFormat[k],
                (mode_lib->vba.HTotal[k] / mode_lib->vba.PixelClock[k]),
                mode_lib->vba.VRatio[k],
                mode_lib->vba.VRatioChroma[k],
                mode_lib->vba.Tno_bw[k],
                    mode_lib->vba.DCCEnable[k],
                mode_lib->vba.dpte_row_height[k],
                mode_lib->vba.meta_row_height[k],
                mode_lib->vba.dpte_row_height_chroma[k],
                mode_lib->vba.meta_row_height_chroma[k],
                mode_lib->vba.use_one_row_for_frame_flip[i][j][k], // 24

                /* Output */
                &mode_lib->vba.DestinationLinesToRequestVMInImmediateFlip[k],
                &mode_lib->vba.DestinationLinesToRequestRowInImmediateFlip[k],
                &mode_lib->vba.final_flip_bw[k],
                &mode_lib->vba.ImmediateFlipSupportedForPipe[k]);
        }

        {
            dml32_CalculateImmediateFlipBandwithSupport(mode_lib->vba.NumberOfActiveSurfaces,
                    mode_lib->vba.ReturnBWPerState[i][j],
                    mode_lib->vba.ImmediateFlipRequirement,
                    mode_lib->vba.final_flip_bw,
                    mode_lib->vba.ReadBandwidthLuma,
                    mode_lib->vba.ReadBandwidthChroma,
                    mode_lib->vba.RequiredPrefetchPixelDataBWLuma[0][0],
                    mode_lib->vba.RequiredPrefetchPixelDataBWChroma[0][0],
                    mode_lib->vba.cursor_bw,
                    mode_lib->vba.meta_row_bandwidth_this_state,
                    mode_lib->vba.dpte_row_bandwidth_this_state,
                    mode_lib->vba.cursor_bw_pre,
                    mode_lib->vba.prefetch_vmrow_bw,
                    dml_uint(mode_lib->vba.DPPPerPlane),
                    mode_lib->vba.UrgentBurstFactorLuma,
                    mode_lib->vba.UrgentBurstFactorChroma,
                    mode_lib->vba.UrgentBurstFactorCursor,
                    mode_lib->vba.UrgentBurstFactorLumaPre,
                    mode_lib->vba.UrgentBurstFactorChromaPre,
                    mode_lib->vba.UrgentBurstFactorCursorPre,

                    /* output */
                    &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_single[0], //  Single  *TotalBandwidth
                    &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_single[1], //  Single  *FractionOfUrgentBandwidth
                    &mode_lib->vba.ImmediateFlipSupportedForState[i][j]); // Boolean *ImmediateFlipBandwidthSupport
        }

        for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
            if (!(mode_lib->vba.ImmediateFlipRequirement[k]
                    == dm_immediate_flip_not_required)
                    && (mode_lib->vba.ImmediateFlipSupportedForPipe[k]
                            == false))
                mode_lib->vba.ImmediateFlipSupportedForState[i][j] = false;
        }
    } else { // if prefetch not support, assume iflip not supported
        mode_lib->vba.ImmediateFlipSupportedForState[i][j] = false;
    }
}

static DML32_NOINLINE void dml32_support_prefetch_attempt(struct display_mode_lib *mode_lib, int i, int j)
{
    struct vba_vars_st *v = &mode_lib->vba;
    unsigned int k;

    mode_lib->vba.PrefetchModePerState[i][j] = v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.NextPrefetchModeState;
    mode_lib->vba.MaxVStartup = mode_lib->vba.NextMaxVStartup;

    for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
        dml32_support_prefetch_schedule_surface(mode_lib, i, j, k);
    }

    dml32_support_prefetch_urgent_burst(mode_lib, i, j);


    dml32_support_prefetch_bandwidth(mode_lib, i, j);

    for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
        if (mode_lib->vba.LineTimesForPrefetch[k]
                < 2.0 || mode_lib->vba.LinesForMetaPTE[k] >= 32.0
                || mode_lib->vba.LinesForMetaAndDPTERow[k] >= 16.0
                || mode_lib->vba.NoTimeForPrefetch[i][j][k] == true) {
            mode_lib->vba.PrefetchSupported[i][j] = false;
        }
    }

    mode_lib->vba.DynamicMetadataSupported[i][j] = true;
    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        if (mode_lib->vba.NoTimeForDynamicMetadata[i][j][k] == true)
            mode_lib->vba.DynamicMetadataSupported[i][j] = false;
    }

    mode_lib->vba.VRatioInPrefetchSupported[i][j] = true;
    for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
        if (mode_lib->vba.VRatioPreY[i][j][k] > mode_lib->vba.MaxVRatioPre
                || mode_lib->vba.VRatioPreC[i][j][k] > mode_lib->vba.MaxVRatioPre
                || mode_lib->vba.NoTimeForPrefetch[i][j][k] == true) {
            mode_lib->vba.VRatioInPrefetchSupported[i][j] = false;
        }
    }
    mode_lib->vba.AnyLinesForVMOrRowTooLarge = false;
    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        if (mode_lib->vba.LinesForMetaAndDPTERow[k] >= 16
                || mode_lib->vba.LinesForMetaPTE[k] >= 32) {
            mode_lib->vba.AnyLinesForVMOrRowTooLarge = true;
        }
    }

    dml32_support_prefetch_immediate_flip(mode_lib, i, j);

    if (mode_lib->vba.MaxVStartup <= __DML_VBA_MIN_VSTARTUP__
            || mode_lib->vba.AnyLinesForVMOrRowTooLarge == false) {
        mode_lib->vba.NextMaxVStartup = mode_lib->vba.MaxMaxVStartup[i][j];
        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.NextPrefetchModeState = v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.NextPrefetchModeState + 1;
    } else {
        mode_lib->vba.NextMaxVStartup = mode_lib->vba.NextMaxVStartup - 1;
    }
}

// One voltage state and MPC combine option of the prefetch check (the body of the
// upstream i/j loops).
static DML32_NOINLINE void dml32_support_prefetch_state(struct display_mode_lib *mode_lib, int i, int j)
{
    struct vba_vars_st *v = &mode_lib->vba;
    unsigned int k;


    mode_lib->vba.TimeCalc = 24 / mode_lib->vba.ProjectedDCFCLKDeepSleep[i][j];

    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        mode_lib->vba.NoOfDPPThisState[k] = mode_lib->vba.NoOfDPP[i][j][k];
        mode_lib->vba.swath_width_luma_ub_this_state[k] =
                mode_lib->vba.swath_width_luma_ub_all_states[i][j][k];
        mode_lib->vba.swath_width_chroma_ub_this_state[k] =
                mode_lib->vba.swath_width_chroma_ub_all_states[i][j][k];
        mode_lib->vba.SwathWidthYThisState[k] = mode_lib->vba.SwathWidthYAllStates[i][j][k];
        mode_lib->vba.SwathWidthCThisState[k] = mode_lib->vba.SwathWidthCAllStates[i][j][k];
        mode_lib->vba.SwathHeightYThisState[k] = mode_lib->vba.SwathHeightYAllStates[i][j][k];
        mode_lib->vba.SwathHeightCThisState[k] = mode_lib->vba.SwathHeightCAllStates[i][j][k];
        mode_lib->vba.UnboundedRequestEnabledThisState =
                mode_lib->vba.UnboundedRequestEnabledAllStates[i][j];
        mode_lib->vba.CompressedBufferSizeInkByteThisState =
                mode_lib->vba.CompressedBufferSizeInkByteAllStates[i][j];
        mode_lib->vba.DETBufferSizeInKByteThisState[k] =
                mode_lib->vba.DETBufferSizeInKByteAllStates[i][j][k];
        mode_lib->vba.DETBufferSizeYThisState[k] =
                mode_lib->vba.DETBufferSizeYAllStates[i][j][k];
        mode_lib->vba.DETBufferSizeCThisState[k] =
                mode_lib->vba.DETBufferSizeCAllStates[i][j][k];
    }

    mode_lib->vba.VActiveBandwithSupport[i][j] = dml32_CalculateVActiveBandwithSupport(
            mode_lib->vba.NumberOfActiveSurfaces,
            mode_lib->vba.ReturnBWPerState[i][j],
            mode_lib->vba.NoUrgentLatencyHiding,
            mode_lib->vba.ReadBandwidthLuma,
            mode_lib->vba.ReadBandwidthChroma,
            mode_lib->vba.cursor_bw,
            mode_lib->vba.meta_row_bandwidth_this_state,
            mode_lib->vba.dpte_row_bandwidth_this_state,
            dml_uint(mode_lib->vba.NoOfDPPThisState),
            mode_lib->vba.UrgentBurstFactorLuma,
            mode_lib->vba.UrgentBurstFactorChroma,
            mode_lib->vba.UrgentBurstFactorCursor);

    mode_lib->vba.NotEnoughDETSwathFillLatencyHidingPerState[i][j] = dml32_CalculateDETSwathFillLatencyHiding(
            mode_lib->vba.NumberOfActiveSurfaces,
            mode_lib->vba.ReturnBWPerState[i][j],
            mode_lib->vba.UrgLatency[i],
            mode_lib->vba.SwathHeightYThisState,
            mode_lib->vba.SwathHeightCThisState,
            dml_uint(mode_lib->vba.swath_width_luma_ub_this_state),
            dml_uint(mode_lib->vba.swath_width_chroma_ub_this_state),
            mode_lib->vba.BytePerPixelInDETY,
            mode_lib->vba.BytePerPixelInDETC,
            mode_lib->vba.DETBufferSizeYThisState,
            mode_lib->vba.DETBufferSizeCThisState,
            dml_uint(mode_lib->vba.NoOfDPPThisState),
            mode_lib->vba.HTotal,
            mode_lib->vba.PixelClock,
            mode_lib->vba.VRatio,
            mode_lib->vba.VRatioChroma,
            mode_lib->vba.UsesMALLForPStateChange,
            mode_lib->vba.UseUnboundedRequesting);

    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.VMDataOnlyReturnBWPerState = dml32_get_return_bw_mbps_vm_only(&mode_lib->vba.soc, i,
            mode_lib->vba.DCFCLKState[i][j], mode_lib->vba.FabricClockPerState[i],
            mode_lib->vba.DRAMSpeedPerState[i]);
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.HostVMInefficiencyFactor = 1;

    if (mode_lib->vba.GPUVMEnable && mode_lib->vba.HostVMEnable)
        v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.HostVMInefficiencyFactor = mode_lib->vba.ReturnBWPerState[i][j]
                / v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.VMDataOnlyReturnBWPerState;

    mode_lib->vba.ExtraLatency = dml32_CalculateExtraLatency(
            mode_lib->vba.RoundTripPingLatencyCycles, v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.ReorderingBytes,
            mode_lib->vba.DCFCLKState[i][j], mode_lib->vba.TotalNumberOfActiveDPP[i][j],
            mode_lib->vba.PixelChunkSizeInKByte,
            mode_lib->vba.TotalNumberOfDCCActiveDPP[i][j], mode_lib->vba.MetaChunkSize,
            mode_lib->vba.ReturnBWPerState[i][j], mode_lib->vba.GPUVMEnable,
            mode_lib->vba.HostVMEnable, mode_lib->vba.NumberOfActiveSurfaces,
            dml_uint(mode_lib->vba.NoOfDPPThisState), mode_lib->vba.dpte_group_bytes,
            v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.HostVMInefficiencyFactor, mode_lib->vba.HostVMMinPageSize,
            mode_lib->vba.HostVMMaxNonCachedPageTableLevels);

    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.NextPrefetchModeState = mode_lib->vba.MinPrefetchMode;

    mode_lib->vba.NextMaxVStartup = mode_lib->vba.MaxMaxVStartup[i][j];

    do {
        dml32_support_prefetch_attempt(mode_lib, i, j);
    } while (!((mode_lib->vba.PrefetchSupported[i][j] == true
            && mode_lib->vba.DynamicMetadataSupported[i][j] == true
            && mode_lib->vba.VRatioInPrefetchSupported[i][j] == true &&
            // consider flip support is okay if when there is no hostvm and the
            // user does't require a iflip OR the flip bw is ok
            // If there is hostvm, DCN needs to support iflip for invalidation
            ((mode_lib->vba.HostVMEnable == false
                    && !mode_lib->vba.ImmediateFlipRequiredFinal)
                    || mode_lib->vba.ImmediateFlipSupportedForState[i][j] == true))
            || (mode_lib->vba.NextMaxVStartup == mode_lib->vba.MaxMaxVStartup[i][j]
                    && v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.NextPrefetchModeState > mode_lib->vba.MaxPrefetchMode)));

    for (k = 0; k < mode_lib->vba.NumberOfActiveSurfaces; ++k) {
        mode_lib->vba.use_one_row_for_frame_this_state[k] =
                mode_lib->vba.use_one_row_for_frame[i][j][k];
    }

    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.mSOCParameters.UrgentLatency = mode_lib->vba.UrgLatency[i];
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.mSOCParameters.ExtraLatency = mode_lib->vba.ExtraLatency;
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.mSOCParameters.WritebackLatency = mode_lib->vba.WritebackLatency;
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.mSOCParameters.DRAMClockChangeLatency = mode_lib->vba.DRAMClockChangeLatency;
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.mSOCParameters.FCLKChangeLatency = mode_lib->vba.FCLKChangeLatency;
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.mSOCParameters.SRExitTime = mode_lib->vba.SRExitTime;
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.mSOCParameters.SREnterPlusExitTime = mode_lib->vba.SREnterPlusExitTime;
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.mSOCParameters.SRExitZ8Time = mode_lib->vba.SRExitZ8Time;
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.mSOCParameters.SREnterPlusExitZ8Time = mode_lib->vba.SREnterPlusExitZ8Time;
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.mSOCParameters.USRRetrainingLatency = mode_lib->vba.USRRetrainingLatency;
    v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.mSOCParameters.SMNLatency = mode_lib->vba.SMNLatency;

    {
        dml32_CalculateWatermarksMALLUseAndDRAMSpeedChangeSupport(
                v,
                v->PrefetchModePerState[i][j],
                v->DCFCLKState[i][j],
                v->ReturnBWPerState[i][j],
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.mSOCParameters,
                v->SOCCLKPerState[i],
                v->ProjectedDCFCLKDeepSleep[i][j],
                v->DETBufferSizeYThisState,
                v->DETBufferSizeCThisState,
                v->SwathHeightYThisState,
                v->SwathHeightCThisState,
                v->SwathWidthYThisState, // 24
                v->SwathWidthCThisState,
                dml_uint(v->NoOfDPPThisState),
                v->BytePerPixelInDETY,
                v->BytePerPixelInDETC,
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.DSTXAfterScaler,
                v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.DSTYAfterScaler,
                v->UnboundedRequestEnabledThisState,
                v->CompressedBufferSizeInkByteThisState,

                /* Output */
                &v->DRAMClockChangeSupport[i][j],
                &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_single2[0], // double *MaxActiveDRAMClockChangeLatencySupported
                &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_integer[0], // Long SubViewportLinesNeededInMALL[]
                &v->FCLKChangeSupport[i][j],
                &v->dummy_vars.dml32_ModeSupportAndSystemConfigurationFull.dummy_single2[1], // double *MinActiveFCLKChangeLatencySupported
                &mode_lib->vba.USRRetrainingSupport[i][j],
                mode_lib->vba.ActiveDRAMClockChangeLatencyMarginPerState[i][j]);
    }
}

/* Prefetch Check */
// Per-state prefetch schedule, immediate flip and watermark checks.
static DML32_NOINLINE void dml32_support_prefetch_per_state(struct display_mode_lib *mode_lib, int start_state)
{
    struct vba_vars_st *v = &mode_lib->vba;
    int i, j;

    for (i = start_state; i < static_cast<int>(v->soc.num_states); ++i) {
        for (j = 0; j <= 1; ++j) {
            dml32_support_prefetch_state(mode_lib, i, j);
        }
    } // End of Prefetch Check

    /*Cursor Support Check*/
}

// Cursor/pitch/viewport checks, mode support per state and level selection.
static DML32_NOINLINE void dml32_support_select_level(struct display_mode_lib *mode_lib, int start_state)
{
    struct vba_vars_st *v = &mode_lib->vba;
    int i;
    unsigned int k;
    unsigned int MaximumMPCCombine;

    mode_lib->vba.CursorSupport = true;
    for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
        if (mode_lib->vba.CursorWidth[k][0] > 0.0) {
            if (mode_lib->vba.CursorBPP[k][0] == 64 && mode_lib->vba.Cursor64BppSupport == false)
                mode_lib->vba.CursorSupport = false;
        }
    }

    /*Valid Pitch Check*/
    mode_lib->vba.PitchSupport = true;
    for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
        mode_lib->vba.AlignedYPitch[k] = dml_ceil(
                dml_max(mode_lib->vba.PitchY[k], mode_lib->vba.SurfaceWidthY[k]),
                mode_lib->vba.MacroTileWidthY[k]);
        if (mode_lib->vba.DCCEnable[k] == true) {
            mode_lib->vba.AlignedDCCMetaPitchY[k] = dml_ceil(
                    dml_max(mode_lib->vba.DCCMetaPitchY[k], mode_lib->vba.SurfaceWidthY[k]),
                    64.0 * mode_lib->vba.Read256BlockWidthY[k]);
        } else {
            mode_lib->vba.AlignedDCCMetaPitchY[k] = mode_lib->vba.DCCMetaPitchY[k];
        }
        if (mode_lib->vba.SourcePixelFormat[k] != dm_444_64 && mode_lib->vba.SourcePixelFormat[k] != dm_444_32
                && mode_lib->vba.SourcePixelFormat[k] != dm_444_16
                && mode_lib->vba.SourcePixelFormat[k] != dm_mono_16
                && mode_lib->vba.SourcePixelFormat[k] != dm_rgbe
                && mode_lib->vba.SourcePixelFormat[k] != dm_mono_8) {
            mode_lib->vba.AlignedCPitch[k] = dml_ceil(
                    dml_max(mode_lib->vba.PitchC[k], mode_lib->vba.SurfaceWidthC[k]),
                    mode_lib->vba.MacroTileWidthC[k]);
            if (mode_lib->vba.DCCEnable[k] == true) {
                mode_lib->vba.AlignedDCCMetaPitchC[k] = dml_ceil(
                        dml_max(mode_lib->vba.DCCMetaPitchC[k],
                                mode_lib->vba.SurfaceWidthC[k]),
                        64.0 * mode_lib->vba.Read256BlockWidthC[k]);
            } else {
                mode_lib->vba.AlignedDCCMetaPitchC[k] = mode_lib->vba.DCCMetaPitchC[k];
            }
        } else {
            mode_lib->vba.AlignedCPitch[k] = mode_lib->vba.PitchC[k];
            mode_lib->vba.AlignedDCCMetaPitchC[k] = mode_lib->vba.DCCMetaPitchC[k];
        }
        if (mode_lib->vba.AlignedYPitch[k] > mode_lib->vba.PitchY[k]
                || mode_lib->vba.AlignedCPitch[k] > mode_lib->vba.PitchC[k]
                || mode_lib->vba.AlignedDCCMetaPitchY[k] > mode_lib->vba.DCCMetaPitchY[k]
                || mode_lib->vba.AlignedDCCMetaPitchC[k] > mode_lib->vba.DCCMetaPitchC[k]) {
            mode_lib->vba.PitchSupport = false;
        }
    }

    mode_lib->vba.ViewportExceedsSurface = false;
    for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
        if (mode_lib->vba.ViewportWidth[k] > mode_lib->vba.SurfaceWidthY[k]
                || mode_lib->vba.ViewportHeight[k] > mode_lib->vba.SurfaceHeightY[k]) {
            mode_lib->vba.ViewportExceedsSurface = true;
            if (mode_lib->vba.SourcePixelFormat[k] != dm_444_64
                    && mode_lib->vba.SourcePixelFormat[k] != dm_444_32
                    && mode_lib->vba.SourcePixelFormat[k] != dm_444_16
                    && mode_lib->vba.SourcePixelFormat[k] != dm_444_8
                    && mode_lib->vba.SourcePixelFormat[k] != dm_rgbe) {
                if (mode_lib->vba.ViewportWidthChroma[k] > mode_lib->vba.SurfaceWidthC[k]
                        || mode_lib->vba.ViewportHeightChroma[k]
                                > mode_lib->vba.SurfaceHeightC[k]) {
                    mode_lib->vba.ViewportExceedsSurface = true;
                }
            }
        }
    }

    /*Mode Support, Voltage State and SOC Configuration*/
    mode_support_configuration(v, mode_lib);

    MaximumMPCCombine = 0;

    for (i = v->soc.num_states; i >= start_state; i--) {
        if (static_cast<unsigned int>(i) == v->soc.num_states || mode_lib->vba.ModeSupport[i][0] == true ||
                mode_lib->vba.ModeSupport[i][1] == true) {
            mode_lib->vba.VoltageLevel = i;
            mode_lib->vba.ModeIsSupported = mode_lib->vba.ModeSupport[i][0] == true
                    || mode_lib->vba.ModeSupport[i][1] == true;

            if (mode_lib->vba.ModeSupport[i][0] == true)
                MaximumMPCCombine = 0;
            else
                MaximumMPCCombine = 1;
        }
    }

    mode_lib->vba.ImmediateFlipSupport =
            mode_lib->vba.ImmediateFlipSupportedForState[mode_lib->vba.VoltageLevel][MaximumMPCCombine];
    mode_lib->vba.UnboundedRequestEnabled =
            mode_lib->vba.UnboundedRequestEnabledAllStates[mode_lib->vba.VoltageLevel][MaximumMPCCombine];
    mode_lib->vba.CompressedBufferSizeInkByte =
            mode_lib->vba.CompressedBufferSizeInkByteAllStates[mode_lib->vba.VoltageLevel][MaximumMPCCombine]; // Not used, informational

    for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
        mode_lib->vba.MPCCombineEnable[k] =
                mode_lib->vba.MPCCombine[mode_lib->vba.VoltageLevel][MaximumMPCCombine][k];
        mode_lib->vba.DPPPerPlane[k] = mode_lib->vba.NoOfDPP[mode_lib->vba.VoltageLevel][MaximumMPCCombine][k];
        mode_lib->vba.SwathHeightY[k] =
                mode_lib->vba.SwathHeightYAllStates[mode_lib->vba.VoltageLevel][MaximumMPCCombine][k];
        mode_lib->vba.SwathHeightC[k] =
                mode_lib->vba.SwathHeightCAllStates[mode_lib->vba.VoltageLevel][MaximumMPCCombine][k];
        mode_lib->vba.DETBufferSizeInKByte[k] =
            mode_lib->vba.DETBufferSizeInKByteAllStates[mode_lib->vba.VoltageLevel][MaximumMPCCombine][k];
        mode_lib->vba.DETBufferSizeY[k] =
                mode_lib->vba.DETBufferSizeYAllStates[mode_lib->vba.VoltageLevel][MaximumMPCCombine][k];
        mode_lib->vba.DETBufferSizeC[k] =
                mode_lib->vba.DETBufferSizeCAllStates[mode_lib->vba.VoltageLevel][MaximumMPCCombine][k];
        mode_lib->vba.OutputType[k] = mode_lib->vba.OutputTypePerState[mode_lib->vba.VoltageLevel][k];
        mode_lib->vba.OutputRate[k] = mode_lib->vba.OutputRatePerState[mode_lib->vba.VoltageLevel][k];
    }

    mode_lib->vba.DCFCLK = mode_lib->vba.DCFCLKState[mode_lib->vba.VoltageLevel][MaximumMPCCombine];
    mode_lib->vba.DRAMSpeed = mode_lib->vba.DRAMSpeedPerState[mode_lib->vba.VoltageLevel];
    mode_lib->vba.FabricClock = mode_lib->vba.FabricClockPerState[mode_lib->vba.VoltageLevel];
    mode_lib->vba.SOCCLK = mode_lib->vba.SOCCLKPerState[mode_lib->vba.VoltageLevel];
    mode_lib->vba.ReturnBW = mode_lib->vba.ReturnBWPerState[mode_lib->vba.VoltageLevel][MaximumMPCCombine];
    mode_lib->vba.DISPCLK = mode_lib->vba.RequiredDISPCLK[mode_lib->vba.VoltageLevel][MaximumMPCCombine];
    mode_lib->vba.maxMpcComb = MaximumMPCCombine;

    for (k = 0; k <= mode_lib->vba.NumberOfActiveSurfaces - 1; k++) {
        if (mode_lib->vba.BlendingAndTiming[k] == k) {
            mode_lib->vba.ODMCombineEnabled[k] =
                    mode_lib->vba.ODMCombineEnablePerState[mode_lib->vba.VoltageLevel][k];
        } else {
            mode_lib->vba.ODMCombineEnabled[k] = dm_odm_combine_mode_disabled;
        }

        mode_lib->vba.DSCEnabled[k] = mode_lib->vba.RequiresDSC[mode_lib->vba.VoltageLevel][k];
        mode_lib->vba.FECEnable[k] = mode_lib->vba.RequiresFEC[mode_lib->vba.VoltageLevel][k];
        mode_lib->vba.OutputBpp[k] = mode_lib->vba.OutputBppPerState[mode_lib->vba.VoltageLevel][k];
    }

    mode_lib->vba.UrgentWatermark = mode_lib->vba.Watermark.UrgentWatermark;
    mode_lib->vba.StutterEnterPlusExitWatermark = mode_lib->vba.Watermark.StutterEnterPlusExitWatermark;
    mode_lib->vba.StutterExitWatermark = mode_lib->vba.Watermark.StutterExitWatermark;
    mode_lib->vba.WritebackDRAMClockChangeWatermark = mode_lib->vba.Watermark.WritebackDRAMClockChangeWatermark;
    mode_lib->vba.DRAMClockChangeWatermark = mode_lib->vba.Watermark.DRAMClockChangeWatermark;
    mode_lib->vba.UrgentLatency = mode_lib->vba.UrgLatency[mode_lib->vba.VoltageLevel];
    mode_lib->vba.DCFCLKDeepSleep = mode_lib->vba.ProjectedDCFCLKDeepSleep[mode_lib->vba.VoltageLevel][MaximumMPCCombine];

    /* VBA has Error type to Error Msg output here, but not necessary for DML-C */
}

void dml32_ModeSupportAndSystemConfigurationFull(struct display_mode_lib *mode_lib)
{
    struct vba_vars_st *v = &mode_lib->vba;
    int start_state;

    /*MODE SUPPORT, VOLTAGE STATE AND SOC CONFIGURATION*/
    if (mode_lib->validate_max_state)
        start_state = v->soc.num_states - 1;
    else
        start_state = 0;

    /*Scale Ratio, taps Support Check*/


    dml32_support_checks_and_single_dpp_swath(mode_lib, start_state);
    dml32_support_clocks_and_dpp_per_state(mode_lib, start_state);
    dml32_support_output_link_and_swath_per_state(mode_lib, start_state);
    dml32_support_rows_and_return_bandwidth(mode_lib, start_state);
    dml32_support_prefetch_per_state(mode_lib, start_state);
    dml32_support_select_level(mode_lib, start_state);
} // ModeSupportAndSystemConfigurationFull

} // namespace rx7900::display::dml32

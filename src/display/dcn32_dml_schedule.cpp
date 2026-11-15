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
Module: rx7900.display.dcn32_dml_schedule
Purpose: DML32 prefetch/flip schedules, watermarks, delivery times and stutter.
Created: 2026-10-24 (adaptation; the upstream notice above is preserved)
Architecture: Part of the bounded C++17 port of the Linux v6.12 DML32 model behind
src/display/dcn32_bandwidth (one stream, up to four pipes). Identifiers, statement order and
arithmetic (including the float-typed dml_min/max/floor/ceil helpers and every
int/unsigned conversion) deliberately follow the pinned upstream text so each
function can be compared line by line with work/upstream-dcn-stream; this is a
documented exception to the snake_case rule, as in src/opengl/gfx11_registers.hpp.
Per-plane and per-pipe arrays hold the four DCN 3.2 pipes (DML32_NUM_PLANES; Linux
sizes them DC__NUM_DPP__MAX = 8) and per-state arrays DML32_NUM_STATES entries; debug printing is removed.
Upstream: dml/dcn32/display_mode_vba_util_32.c (third part).
*/
#include "dcn32_dml_util.hpp"

namespace rx7900::display::dml32 {
bool dml32_CalculatePrefetchSchedule(
        struct vba_vars_st *v,
        unsigned int k,
        double HostVMInefficiencyFactor,
        DmlPipe *myPipe,
        unsigned int DSCDelay,
        unsigned int DPP_RECOUT_WIDTH,
        unsigned int VStartup,
        unsigned int MaxVStartup,
        double UrgentLatency,
        double UrgentExtraLatency,
        double TCalc,
        unsigned int PDEAndMetaPTEBytesFrame,
        unsigned int MetaRowByte,
        unsigned int PixelPTEBytesPerRow,
        double PrefetchSourceLinesY,
        unsigned int /* SwathWidthY */,
        unsigned int VInitPreFillY,
        unsigned int MaxNumSwathY,
        double PrefetchSourceLinesC,
        unsigned int /* SwathWidthC */,
        unsigned int VInitPreFillC,
        unsigned int MaxNumSwathC,
        unsigned int swath_width_luma_ub,
        unsigned int swath_width_chroma_ub,
        unsigned int SwathHeightY,
        unsigned int SwathHeightC,
        double TWait,
        double TPreReq,
        bool ExtendPrefetchIfPossible,
        /* Output */
        double   *DSTXAfterScaler,
        double   *DSTYAfterScaler,
        double *DestinationLinesForPrefetch,
        double *PrefetchBandwidth,
        double *DestinationLinesToRequestVMInVBlank,
        double *DestinationLinesToRequestRowInVBlank,
        double *VRatioPrefetchY,
        double *VRatioPrefetchC,
        double *RequiredPrefetchPixDataBWLuma,
        double *RequiredPrefetchPixDataBWChroma,
        bool   *NotEnoughTimeForDynamicMetadata,
        double *Tno_bw,
        double *prefetch_vmrow_bw,
        double *Tdmdl_vm,
        double *Tdmdl,
        double *TSetup,
        unsigned int   *VUpdateOffsetPix,
        double   *VUpdateWidthPix,
        double   *VReadyOffsetPix)
{
    double DPPCLKDelaySubtotalPlusCNVCFormater = v->DPPCLKDelaySubtotal + v->DPPCLKDelayCNVCFormater;
    bool MyError = false;
    unsigned int DPPCycles, DISPCLKCycles;
    double DSTTotalPixelsAfterScaler;
    double LineTime;
    double dst_y_prefetch_equ;
    double prefetch_bw_oto;
    double Tvm_oto;
    double Tr0_oto;
    double Tvm_oto_lines;
    double Tr0_oto_lines;
    double dst_y_prefetch_oto;
    double TimeForFetchingMetaPTE = 0;
    double TimeForFetchingRowInVBlank = 0;
    double LinesToRequestPrefetchPixelData = 0;
    double LinesForPrefetchBandwidth = 0;
    unsigned int HostVMDynamicLevelsTrips;
    double  trip_to_mem;
    double  Tvm_trips;
    double  Tr0_trips;
    double  Tvm_trips_rounded;
    double  Tr0_trips_rounded;
    double  Lsw_oto;
    double  Tpre_rounded;
    double  prefetch_bw_equ;
    double  Tvm_equ;
    double  Tr0_equ;
    double  Tdmbf;
    double  Tdmec;
    double  Tdmsks;
    double  prefetch_sw_bytes;
    double  bytes_pp;
    double  dep_bytes;
    unsigned int max_vratio_pre = v->MaxVRatioPre;
    double  min_Lsw;
    double  Tsw_est1 = 0;
    double  Tsw_est3 = 0;

    if (v->GPUVMEnable == true && v->HostVMEnable == true)
        HostVMDynamicLevelsTrips = v->HostVMMaxNonCachedPageTableLevels;
    else
        HostVMDynamicLevelsTrips = 0;
    dml32_CalculateVUpdateAndDynamicMetadataParameters(
            v->MaxInterDCNTileRepeaters,
            myPipe->Dppclk,
            myPipe->Dispclk,
            myPipe->DCFClkDeepSleep,
            myPipe->PixelClock,
            myPipe->HTotal,
            myPipe->VBlank,
            v->DynamicMetadataTransmittedBytes[k],
            v->DynamicMetadataLinesBeforeActiveRequired[k],
            myPipe->InterlaceEnable,
            myPipe->ProgressiveToInterlaceUnitInOPP,
            TSetup,

            /* output */
            &Tdmbf,
            &Tdmec,
            &Tdmsks,
            VUpdateOffsetPix,
            VUpdateWidthPix,
            VReadyOffsetPix);

    LineTime = myPipe->HTotal / myPipe->PixelClock;
    trip_to_mem = UrgentLatency;
    Tvm_trips = UrgentExtraLatency + trip_to_mem * (v->GPUVMMaxPageTableLevels * (HostVMDynamicLevelsTrips + 1) - 1);

    if (v->DynamicMetadataVMEnabled == true)
        *Tdmdl = TWait + Tvm_trips + trip_to_mem;
    else
        *Tdmdl = TWait + UrgentExtraLatency;

    if (v->DynamicMetadataEnable[k] == true) {
        if (VStartup * LineTime < *TSetup + *Tdmdl + Tdmbf + Tdmec + Tdmsks) {
            *NotEnoughTimeForDynamicMetadata = true;
        } else {
            *NotEnoughTimeForDynamicMetadata = false;
        }
    } else {
        *NotEnoughTimeForDynamicMetadata = false;
    }

    *Tdmdl_vm =  (v->DynamicMetadataEnable[k] == true && v->DynamicMetadataVMEnabled == true &&
            v->GPUVMEnable == true ? TWait + Tvm_trips : 0);

    if (myPipe->ScalerEnabled)
        DPPCycles = DPPCLKDelaySubtotalPlusCNVCFormater + v->DPPCLKDelaySCL;
    else
        DPPCycles = DPPCLKDelaySubtotalPlusCNVCFormater + v->DPPCLKDelaySCLLBOnly;

    DPPCycles = DPPCycles + myPipe->NumberOfCursors * v->DPPCLKDelayCNVCCursor;

    DISPCLKCycles = v->DISPCLKDelaySubtotal;

    if (myPipe->Dppclk == 0.0 || myPipe->Dispclk == 0.0)
        return true;

    *DSTXAfterScaler = DPPCycles * myPipe->PixelClock / myPipe->Dppclk + DISPCLKCycles *
            myPipe->PixelClock / myPipe->Dispclk + DSCDelay;

    *DSTXAfterScaler = *DSTXAfterScaler + (myPipe->ODMMode != dm_odm_combine_mode_disabled ? 18 : 0)
            + (myPipe->DPPPerSurface - 1) * DPP_RECOUT_WIDTH
            + ((myPipe->ODMMode == dm_odm_split_mode_1to2 || myPipe->ODMMode == dm_odm_mode_mso_1to2) ?
                    myPipe->HActive / 2 : 0)
            + ((myPipe->ODMMode == dm_odm_mode_mso_1to4) ? myPipe->HActive * 3 / 4 : 0);

    if (v->OutputFormat[k] == dm_420 || (myPipe->InterlaceEnable && myPipe->ProgressiveToInterlaceUnitInOPP))
        *DSTYAfterScaler = 1;
    else
        *DSTYAfterScaler = 0;

    DSTTotalPixelsAfterScaler = *DSTYAfterScaler * myPipe->HTotal + *DSTXAfterScaler;
    *DSTYAfterScaler = dml_floor(DSTTotalPixelsAfterScaler / myPipe->HTotal, 1);
    *DSTXAfterScaler = DSTTotalPixelsAfterScaler - (static_cast<double>(*DSTYAfterScaler * myPipe->HTotal));

    MyError = false;

    Tr0_trips = trip_to_mem * (HostVMDynamicLevelsTrips + 1);

    if (v->GPUVMEnable == true) {
        Tvm_trips_rounded = dml_ceil(4.0 * Tvm_trips / LineTime, 1.0) / 4.0 * LineTime;
        Tr0_trips_rounded = dml_ceil(4.0 * Tr0_trips / LineTime, 1.0) / 4.0 * LineTime;
        if (v->GPUVMMaxPageTableLevels >= 3) {
            *Tno_bw = UrgentExtraLatency + trip_to_mem *
                    static_cast<double>((v->GPUVMMaxPageTableLevels - 2) * (HostVMDynamicLevelsTrips + 1) - 1);
        } else if (v->GPUVMMaxPageTableLevels == 1 && myPipe->DCCEnable != true) {
            Tr0_trips_rounded = dml_ceil(4.0 * UrgentExtraLatency / LineTime, 1.0) /
                    4.0 * LineTime; // VBA_ERROR
            *Tno_bw = UrgentExtraLatency;
        } else {
            *Tno_bw = 0;
        }
    } else if (myPipe->DCCEnable == true) {
        Tvm_trips_rounded = LineTime / 4.0;
        Tr0_trips_rounded = dml_ceil(4.0 * Tr0_trips / LineTime, 1.0) / 4.0 * LineTime;
        *Tno_bw = 0;
    } else {
        Tvm_trips_rounded = LineTime / 4.0;
        Tr0_trips_rounded = LineTime / 2.0;
        *Tno_bw = 0;
    }
    Tvm_trips_rounded = dml_max(Tvm_trips_rounded, LineTime / 4.0);
    Tr0_trips_rounded = dml_max(Tr0_trips_rounded, LineTime / 4.0);

    if (myPipe->SourcePixelFormat == dm_420_8 || myPipe->SourcePixelFormat == dm_420_10
            || myPipe->SourcePixelFormat == dm_420_12) {
        bytes_pp = myPipe->BytePerPixelY + myPipe->BytePerPixelC / 4;
    } else {
        bytes_pp = myPipe->BytePerPixelY + myPipe->BytePerPixelC;
    }

    prefetch_sw_bytes = PrefetchSourceLinesY * swath_width_luma_ub * myPipe->BytePerPixelY
            + PrefetchSourceLinesC * swath_width_chroma_ub * myPipe->BytePerPixelC;
    prefetch_bw_oto = dml_max(bytes_pp * myPipe->PixelClock / myPipe->DPPPerSurface,
            prefetch_sw_bytes / (dml_max(PrefetchSourceLinesY, PrefetchSourceLinesC) * LineTime));

    min_Lsw = dml_max(PrefetchSourceLinesY, PrefetchSourceLinesC) / max_vratio_pre;
    min_Lsw = dml_max(min_Lsw, 1.0);
    Lsw_oto = dml_ceil(4.0 * dml_max(prefetch_sw_bytes / prefetch_bw_oto / LineTime, min_Lsw), 1.0) / 4.0;

    if (v->GPUVMEnable == true) {
        Tvm_oto = dml_max3(
                Tvm_trips,
                *Tno_bw + PDEAndMetaPTEBytesFrame * HostVMInefficiencyFactor / prefetch_bw_oto,
                LineTime / 4.0);
    } else
        Tvm_oto = LineTime / 4.0;

    if ((v->GPUVMEnable == true || myPipe->DCCEnable == true)) {
        Tr0_oto = dml_max4(
                Tr0_trips,
                (MetaRowByte + PixelPTEBytesPerRow * HostVMInefficiencyFactor) / prefetch_bw_oto,
                (LineTime - Tvm_oto)/2.0,
                LineTime / 4.0);
    } else
        Tr0_oto = (LineTime - Tvm_oto) / 2.0;

    Tvm_oto_lines = dml_ceil(4.0 * Tvm_oto / LineTime, 1) / 4.0;
    Tr0_oto_lines = dml_ceil(4.0 * Tr0_oto / LineTime, 1) / 4.0;
    dst_y_prefetch_oto = Tvm_oto_lines + 2 * Tr0_oto_lines + Lsw_oto;

    dst_y_prefetch_equ = VStartup - (*TSetup + dml_max(TWait + TCalc, *Tdmdl)) / LineTime -
            (*DSTYAfterScaler + static_cast<double>(*DSTXAfterScaler) / static_cast<double>(myPipe->HTotal));

    dst_y_prefetch_equ = dml_min(dst_y_prefetch_equ, __DML_VBA_MAX_DST_Y_PRE__);

    dst_y_prefetch_equ = dml_floor(4.0 * (dst_y_prefetch_equ + 0.125), 1) / 4.0;
    Tpre_rounded = dst_y_prefetch_equ * LineTime;
    dep_bytes = dml_max(PDEAndMetaPTEBytesFrame * HostVMInefficiencyFactor,
            MetaRowByte + PixelPTEBytesPerRow * HostVMInefficiencyFactor);

    if (prefetch_sw_bytes < dep_bytes)
        prefetch_sw_bytes = 2 * dep_bytes;

    *PrefetchBandwidth = 0;
    *DestinationLinesToRequestVMInVBlank = 0;
    *DestinationLinesToRequestRowInVBlank = 0;
    *VRatioPrefetchY = 0;
    *VRatioPrefetchC = 0;
    *RequiredPrefetchPixDataBWLuma = 0;
    if (dst_y_prefetch_equ > 1 &&
            (Tpre_rounded >= TPreReq || dst_y_prefetch_equ == __DML_VBA_MAX_DST_Y_PRE__)) {
        double PrefetchBandwidth1;
        double PrefetchBandwidth2;
        double PrefetchBandwidth3;
        double PrefetchBandwidth4;

        if (Tpre_rounded - *Tno_bw > 0) {
            PrefetchBandwidth1 = (PDEAndMetaPTEBytesFrame * HostVMInefficiencyFactor + 2 * MetaRowByte
                    + 2 * PixelPTEBytesPerRow * HostVMInefficiencyFactor
                    + prefetch_sw_bytes) / (Tpre_rounded - *Tno_bw);
            Tsw_est1 = prefetch_sw_bytes / PrefetchBandwidth1;
        } else
            PrefetchBandwidth1 = 0;

        if (VStartup == MaxVStartup && (Tsw_est1 / LineTime < min_Lsw)
                && Tpre_rounded - min_Lsw * LineTime - 0.75 * LineTime - *Tno_bw > 0) {
            PrefetchBandwidth1 = (PDEAndMetaPTEBytesFrame * HostVMInefficiencyFactor + 2 * MetaRowByte
                    + 2 * PixelPTEBytesPerRow * HostVMInefficiencyFactor)
                    / (Tpre_rounded - min_Lsw * LineTime - 0.75 * LineTime - *Tno_bw);
        }

        if (Tpre_rounded - *Tno_bw - 2 * Tr0_trips_rounded > 0)
            PrefetchBandwidth2 = (PDEAndMetaPTEBytesFrame * HostVMInefficiencyFactor + prefetch_sw_bytes) /
            (Tpre_rounded - *Tno_bw - 2 * Tr0_trips_rounded);
        else
            PrefetchBandwidth2 = 0;

        if (Tpre_rounded - Tvm_trips_rounded > 0) {
            PrefetchBandwidth3 = (2 * MetaRowByte + 2 * PixelPTEBytesPerRow * HostVMInefficiencyFactor
                    + prefetch_sw_bytes) / (Tpre_rounded - Tvm_trips_rounded);
            Tsw_est3 = prefetch_sw_bytes / PrefetchBandwidth3;
        } else
            PrefetchBandwidth3 = 0;

        if (VStartup == MaxVStartup &&
                (Tsw_est3 / LineTime < min_Lsw) && Tpre_rounded - min_Lsw * LineTime - 0.75 *
                LineTime - Tvm_trips_rounded > 0) {
            PrefetchBandwidth3 = (2 * MetaRowByte + 2 * PixelPTEBytesPerRow * HostVMInefficiencyFactor)
                    / (Tpre_rounded - min_Lsw * LineTime - 0.75 * LineTime - Tvm_trips_rounded);
        }

        if (Tpre_rounded - Tvm_trips_rounded - 2 * Tr0_trips_rounded > 0) {
            PrefetchBandwidth4 = prefetch_sw_bytes /
                    (Tpre_rounded - Tvm_trips_rounded - 2 * Tr0_trips_rounded);
        } else {
            PrefetchBandwidth4 = 0;
        }

        {
            bool Case1OK;
            bool Case2OK;
            bool Case3OK;

            if (PrefetchBandwidth1 > 0) {
                if (*Tno_bw + PDEAndMetaPTEBytesFrame * HostVMInefficiencyFactor / PrefetchBandwidth1
                        >= Tvm_trips_rounded
                        && (MetaRowByte + PixelPTEBytesPerRow * HostVMInefficiencyFactor)
                                / PrefetchBandwidth1 >= Tr0_trips_rounded) {
                    Case1OK = true;
                } else {
                    Case1OK = false;
                }
            } else {
                Case1OK = false;
            }

            if (PrefetchBandwidth2 > 0) {
                if (*Tno_bw + PDEAndMetaPTEBytesFrame * HostVMInefficiencyFactor / PrefetchBandwidth2
                        >= Tvm_trips_rounded
                        && (MetaRowByte + PixelPTEBytesPerRow * HostVMInefficiencyFactor)
                        / PrefetchBandwidth2 < Tr0_trips_rounded) {
                    Case2OK = true;
                } else {
                    Case2OK = false;
                }
            } else {
                Case2OK = false;
            }

            if (PrefetchBandwidth3 > 0) {
                if (*Tno_bw + PDEAndMetaPTEBytesFrame * HostVMInefficiencyFactor / PrefetchBandwidth3 <
                        Tvm_trips_rounded && (MetaRowByte + PixelPTEBytesPerRow *
                                HostVMInefficiencyFactor) / PrefetchBandwidth3 >=
                                Tr0_trips_rounded) {
                    Case3OK = true;
                } else {
                    Case3OK = false;
                }
            } else {
                Case3OK = false;
            }

            if (Case1OK)
                prefetch_bw_equ = PrefetchBandwidth1;
            else if (Case2OK)
                prefetch_bw_equ = PrefetchBandwidth2;
            else if (Case3OK)
                prefetch_bw_equ = PrefetchBandwidth3;
            else
                prefetch_bw_equ = PrefetchBandwidth4;

            if (prefetch_bw_equ > 0) {
                if (v->GPUVMEnable == true) {
                    Tvm_equ = dml_max3(*Tno_bw + PDEAndMetaPTEBytesFrame *
                            HostVMInefficiencyFactor / prefetch_bw_equ,
                            Tvm_trips, LineTime / 4);
                } else {
                    Tvm_equ = LineTime / 4;
                }

                if ((v->GPUVMEnable == true || myPipe->DCCEnable == true)) {
                    Tr0_equ = dml_max4((MetaRowByte + PixelPTEBytesPerRow *
                            HostVMInefficiencyFactor) / prefetch_bw_equ, Tr0_trips,
                            (LineTime - Tvm_equ) / 2, LineTime / 4);
                } else {
                    Tr0_equ = (LineTime - Tvm_equ) / 2;
                }
            } else {
                Tvm_equ = 0;
                Tr0_equ = 0;
            }
        }

        if (dst_y_prefetch_oto < dst_y_prefetch_equ) {
            if (dst_y_prefetch_oto * LineTime < TPreReq) {
                *DestinationLinesForPrefetch = dst_y_prefetch_equ;
            } else {
                *DestinationLinesForPrefetch = dst_y_prefetch_oto;
            }
            TimeForFetchingMetaPTE = Tvm_oto;
            TimeForFetchingRowInVBlank = Tr0_oto;
            *PrefetchBandwidth = prefetch_bw_oto;
            /* Clamp to oto for bandwidth calculation */
            LinesForPrefetchBandwidth = dst_y_prefetch_oto;
        } else {
            /* For mode programming we want to extend the prefetch as much as possible
             * (up to oto, or as long as we can for equ) if we're not already applying
             * the 60us prefetch requirement. This is to avoid intermittent underflow
             * issues during prefetch.
             *
             * The prefetch extension is applied under the following scenarios:
             * 1. We're in prefetch mode > 0 (i.e. we don't support MCLK switch in blank)
             * 2. We're using subvp or drr methods of p-state switch, in which case we
             *    we don't care if prefetch takes up more of the blanking time
             *
             * Mode programming typically chooses the smallest prefetch time possible
             * (i.e. highest bandwidth during prefetch) presumably to create margin between
             * p-states / c-states that happen in vblank and prefetch. Therefore we only
             * apply this prefetch extension when p-state in vblank is not required (UCLK
             * p-states take up the most vblank time).
             */
            if (ExtendPrefetchIfPossible && TPreReq == 0 && VStartup < MaxVStartup) {
                MyError = true;
            } else {
                *DestinationLinesForPrefetch = dst_y_prefetch_equ;
                TimeForFetchingMetaPTE = Tvm_equ;
                TimeForFetchingRowInVBlank = Tr0_equ;
                *PrefetchBandwidth = prefetch_bw_equ;
                /* Clamp to equ for bandwidth calculation */
                LinesForPrefetchBandwidth = dst_y_prefetch_equ;
            }
        }

        *DestinationLinesToRequestVMInVBlank = dml_ceil(4.0 * TimeForFetchingMetaPTE / LineTime, 1.0) / 4.0;

        *DestinationLinesToRequestRowInVBlank =
                dml_ceil(4.0 * TimeForFetchingRowInVBlank / LineTime, 1.0) / 4.0;

        LinesToRequestPrefetchPixelData = LinesForPrefetchBandwidth -
                *DestinationLinesToRequestVMInVBlank - 2 * *DestinationLinesToRequestRowInVBlank;

        if (LinesToRequestPrefetchPixelData >= 1 && prefetch_bw_equ > 0) {
            *VRatioPrefetchY = static_cast<double>(PrefetchSourceLinesY) / LinesToRequestPrefetchPixelData;
            *VRatioPrefetchY = dml_max(*VRatioPrefetchY, 1.0);
            if ((SwathHeightY > 4) && (VInitPreFillY > 3)) {
                if (LinesToRequestPrefetchPixelData > (VInitPreFillY - 3.0) / 2.0) {
                    *VRatioPrefetchY =
                            dml_max(static_cast<double>(PrefetchSourceLinesY) /
                                    LinesToRequestPrefetchPixelData,
                                    static_cast<double>(MaxNumSwathY) * SwathHeightY /
                                    (LinesToRequestPrefetchPixelData -
                                    (VInitPreFillY - 3.0) / 2.0));
                    *VRatioPrefetchY = dml_max(*VRatioPrefetchY, 1.0);
                } else {
                    MyError = true;
                    *VRatioPrefetchY = 0;
                }
            }

            *VRatioPrefetchC = static_cast<double>(PrefetchSourceLinesC) / LinesToRequestPrefetchPixelData;
            *VRatioPrefetchC = dml_max(*VRatioPrefetchC, 1.0);

            if ((SwathHeightC > 4)) {
                if (LinesToRequestPrefetchPixelData > (VInitPreFillC - 3.0) / 2.0) {
                    *VRatioPrefetchC =
                        dml_max(*VRatioPrefetchC,
                            static_cast<double>(MaxNumSwathC) * SwathHeightC /
                            (LinesToRequestPrefetchPixelData -
                            (VInitPreFillC - 3.0) / 2.0));
                    *VRatioPrefetchC = dml_max(*VRatioPrefetchC, 1.0);
                } else {
                    MyError = true;
                    *VRatioPrefetchC = 0;
                }
            }

            *RequiredPrefetchPixDataBWLuma = static_cast<double>(PrefetchSourceLinesY)
                    / LinesToRequestPrefetchPixelData * myPipe->BytePerPixelY * swath_width_luma_ub
                    / LineTime;

            *RequiredPrefetchPixDataBWChroma = static_cast<double>(PrefetchSourceLinesC) /
                    LinesToRequestPrefetchPixelData
                    * myPipe->BytePerPixelC
                    * swath_width_chroma_ub / LineTime;
        } else {
            MyError = true;
            *VRatioPrefetchY = 0;
            *VRatioPrefetchC = 0;
            *RequiredPrefetchPixDataBWLuma = 0;
            *RequiredPrefetchPixDataBWChroma = 0;
        }
    } else {
        MyError = true;
    }

    {
        double prefetch_vm_bw;
        double prefetch_row_bw;

        if (PDEAndMetaPTEBytesFrame == 0) {
            prefetch_vm_bw = 0;
        } else if (*DestinationLinesToRequestVMInVBlank > 0) {
            prefetch_vm_bw = PDEAndMetaPTEBytesFrame * HostVMInefficiencyFactor /
                    (*DestinationLinesToRequestVMInVBlank * LineTime);
        } else {
            prefetch_vm_bw = 0;
            MyError = true;
        }

        if (MetaRowByte + PixelPTEBytesPerRow == 0) {
            prefetch_row_bw = 0;
        } else if (*DestinationLinesToRequestRowInVBlank > 0) {
            prefetch_row_bw = (MetaRowByte + PixelPTEBytesPerRow * HostVMInefficiencyFactor) /
                    (*DestinationLinesToRequestRowInVBlank * LineTime);

        } else {
            prefetch_row_bw = 0;
            MyError = true;
        }

        *prefetch_vmrow_bw = dml_max(prefetch_vm_bw, prefetch_row_bw);
    }

    if (MyError) {
        *PrefetchBandwidth = 0;
        TimeForFetchingMetaPTE = 0;
        TimeForFetchingRowInVBlank = 0;
        *DestinationLinesToRequestVMInVBlank = 0;
        *DestinationLinesToRequestRowInVBlank = 0;
        *DestinationLinesForPrefetch = 0;
        LinesToRequestPrefetchPixelData = 0;
        *VRatioPrefetchY = 0;
        *VRatioPrefetchC = 0;
        *RequiredPrefetchPixDataBWLuma = 0;
        *RequiredPrefetchPixDataBWChroma = 0;
    }

    return MyError;
} // CalculatePrefetchSchedule

void dml32_CalculateFlipSchedule(
        double HostVMInefficiencyFactor,
        double UrgentExtraLatency,
        double UrgentLatency,
        unsigned int GPUVMMaxPageTableLevels,
        bool HostVMEnable,
        unsigned int HostVMMaxNonCachedPageTableLevels,
        bool GPUVMEnable,
        double /* HostVMMinPageSize */,
        double PDEAndMetaPTEBytesPerFrame,
        double MetaRowBytes,
        double DPTEBytesPerRow,
        double BandwidthAvailableForImmediateFlip,
        unsigned int TotImmediateFlipBytes,
        enum source_format_class SourcePixelFormat,
        double LineTime,
        double VRatio,
        double VRatioChroma,
        double Tno_bw,
        bool DCCEnable,
        unsigned int dpte_row_height,
        unsigned int meta_row_height,
        unsigned int dpte_row_height_chroma,
        unsigned int meta_row_height_chroma,
        bool    use_one_row_for_frame_flip,

        /* Output */
        double *DestinationLinesToRequestVMInImmediateFlip,
        double *DestinationLinesToRequestRowInImmediateFlip,
        double *final_flip_bw,
        bool *ImmediateFlipSupportedForPipe)
{
    double min_row_time = 0.0;
    unsigned int HostVMDynamicLevelsTrips;
    double TimeForFetchingMetaPTEImmediateFlip;
    double TimeForFetchingRowInVBlankImmediateFlip;
    double ImmediateFlipBW = 1.0;

    if (GPUVMEnable == true && HostVMEnable == true)
        HostVMDynamicLevelsTrips = HostVMMaxNonCachedPageTableLevels;
    else
        HostVMDynamicLevelsTrips = 0;

    if (TotImmediateFlipBytes > 0) {
        if (use_one_row_for_frame_flip) {
            ImmediateFlipBW = (PDEAndMetaPTEBytesPerFrame + MetaRowBytes + 2 * DPTEBytesPerRow) *
                    BandwidthAvailableForImmediateFlip / TotImmediateFlipBytes;
        } else {
            ImmediateFlipBW = (PDEAndMetaPTEBytesPerFrame + MetaRowBytes + DPTEBytesPerRow) *
                    BandwidthAvailableForImmediateFlip / TotImmediateFlipBytes;
        }
        if (GPUVMEnable == true) {
            TimeForFetchingMetaPTEImmediateFlip = dml_max3(Tno_bw + PDEAndMetaPTEBytesPerFrame *
                    HostVMInefficiencyFactor / ImmediateFlipBW,
                    UrgentExtraLatency + UrgentLatency *
                    (GPUVMMaxPageTableLevels * (HostVMDynamicLevelsTrips + 1) - 1),
                    LineTime / 4.0);
        } else {
            TimeForFetchingMetaPTEImmediateFlip = 0;
        }
        if ((GPUVMEnable == true || DCCEnable == true)) {
            TimeForFetchingRowInVBlankImmediateFlip = dml_max3(
                    (MetaRowBytes + DPTEBytesPerRow * HostVMInefficiencyFactor) / ImmediateFlipBW,
                    UrgentLatency * (HostVMDynamicLevelsTrips + 1), LineTime / 4.0);
        } else {
            TimeForFetchingRowInVBlankImmediateFlip = 0;
        }

        *DestinationLinesToRequestVMInImmediateFlip =
                dml_ceil(4.0 * (TimeForFetchingMetaPTEImmediateFlip / LineTime), 1.0) / 4.0;
        *DestinationLinesToRequestRowInImmediateFlip =
                dml_ceil(4.0 * (TimeForFetchingRowInVBlankImmediateFlip / LineTime), 1.0) / 4.0;

        if (GPUVMEnable == true) {
            *final_flip_bw = dml_max(PDEAndMetaPTEBytesPerFrame * HostVMInefficiencyFactor /
                    (*DestinationLinesToRequestVMInImmediateFlip * LineTime),
                    (MetaRowBytes + DPTEBytesPerRow * HostVMInefficiencyFactor) /
                    (*DestinationLinesToRequestRowInImmediateFlip * LineTime));
        } else if ((GPUVMEnable == true || DCCEnable == true)) {
            *final_flip_bw = (MetaRowBytes + DPTEBytesPerRow * HostVMInefficiencyFactor) /
                    (*DestinationLinesToRequestRowInImmediateFlip * LineTime);
        } else {
            *final_flip_bw = 0;
        }
    } else {
        TimeForFetchingMetaPTEImmediateFlip = 0;
        TimeForFetchingRowInVBlankImmediateFlip = 0;
        *DestinationLinesToRequestVMInImmediateFlip = 0;
        *DestinationLinesToRequestRowInImmediateFlip = 0;
        *final_flip_bw = 0;
    }

    if (SourcePixelFormat == dm_420_8 || SourcePixelFormat == dm_420_10 || SourcePixelFormat == dm_rgbe_alpha) {
        if (GPUVMEnable == true && DCCEnable != true) {
            min_row_time = dml_min(dpte_row_height *
                    LineTime / VRatio, dpte_row_height_chroma * LineTime / VRatioChroma);
        } else if (GPUVMEnable != true && DCCEnable == true) {
            min_row_time = dml_min(meta_row_height *
                    LineTime / VRatio, meta_row_height_chroma * LineTime / VRatioChroma);
        } else {
            min_row_time = dml_min4(dpte_row_height * LineTime / VRatio, meta_row_height *
                    LineTime / VRatio, dpte_row_height_chroma * LineTime /
                    VRatioChroma, meta_row_height_chroma * LineTime / VRatioChroma);
        }
    } else {
        if (GPUVMEnable == true && DCCEnable != true) {
            min_row_time = dpte_row_height * LineTime / VRatio;
        } else if (GPUVMEnable != true && DCCEnable == true) {
            min_row_time = meta_row_height * LineTime / VRatio;
        } else {
            min_row_time =
                dml_min(dpte_row_height * LineTime / VRatio, meta_row_height * LineTime / VRatio);
        }
    }

    if (*DestinationLinesToRequestVMInImmediateFlip >= 32 || *DestinationLinesToRequestRowInImmediateFlip >= 16
            || TimeForFetchingMetaPTEImmediateFlip + 2 * TimeForFetchingRowInVBlankImmediateFlip
                    > min_row_time) {
        *ImmediateFlipSupportedForPipe = false;
    } else {
        *ImmediateFlipSupportedForPipe = true;
    }

} // CalculateFlipSchedule

void dml32_CalculateWatermarksMALLUseAndDRAMSpeedChangeSupport(
        struct vba_vars_st *v,
        unsigned int PrefetchMode,
        double /* DCFCLK */,
        double /* ReturnBW */,
        SOCParametersList mmSOCParameters,
        double SOCCLK,
        double DCFClkDeepSleep,
        unsigned int DETBufferSizeY[],
        unsigned int DETBufferSizeC[],
        unsigned int SwathHeightY[],
        unsigned int SwathHeightC[],
        double SwathWidthY[],
        double SwathWidthC[],
        unsigned int DPPPerSurface[],
        double BytePerPixelDETY[],
        double BytePerPixelDETC[],
        double DSTXAfterScaler[],
        double DSTYAfterScaler[],
        bool UnboundedRequestEnabled,
        unsigned int CompressedBufferSizeInkByte,

        /* Output */
        enum clock_change_support *DRAMClockChangeSupport,
        double MaxActiveDRAMClockChangeLatencySupported[],
        unsigned int SubViewportLinesNeededInMALL[],
        enum dm_fclock_change_support *FCLKChangeSupport,
        double *MinActiveFCLKChangeLatencySupported,
        bool *USRRetrainingSupport,
        double ActiveDRAMClockChangeLatencyMargin[])
{
    unsigned int i, j, k;
    unsigned int SurfaceWithMinActiveFCLKChangeMargin = 0;
    unsigned int DRAMClockChangeSupportNumber = 0;
    unsigned int LastSurfaceWithoutMargin = 0;
    unsigned int DRAMClockChangeMethod = 0;
    bool FoundFirstSurfaceWithMinActiveFCLKChangeMargin = false;
    double MinActiveFCLKChangeMargin = 0.;
    double SecondMinActiveFCLKChangeMarginOneDisplayInVBLank = 0.;
    double ActiveClockChangeLatencyHidingY;
    double ActiveClockChangeLatencyHidingC;
    double ActiveClockChangeLatencyHiding;
    double EffectiveDETBufferSizeY;
    double     ActiveFCLKChangeLatencyMargin[DML32_NUM_PLANES]{};
    double     USRRetrainingLatencyMargin[DML32_NUM_PLANES]{};
    double TotalPixelBW = 0.0;
    bool    SynchronizedSurfaces[DML32_NUM_PLANES][DML32_NUM_PLANES];
    double     EffectiveLBLatencyHidingY;
    double     EffectiveLBLatencyHidingC;
    double     LinesInDETY[DML32_NUM_PLANES];
    double     LinesInDETC[DML32_NUM_PLANES];
    unsigned int    LinesInDETYRoundedDownToSwath[DML32_NUM_PLANES];
    unsigned int    LinesInDETCRoundedDownToSwath[DML32_NUM_PLANES];
    double     FullDETBufferingTimeY;
    double     FullDETBufferingTimeC;
    double     WritebackDRAMClockChangeLatencyMargin;
    double     WritebackFCLKChangeLatencyMargin;
    double     WritebackLatencyHiding;
    bool    SameTimingForFCLKChange;

    unsigned int    TotalActiveWriteback = 0;
    unsigned int LBLatencyHidingSourceLinesY[DML32_NUM_PLANES]{}; // these arrays are written before read
    unsigned int LBLatencyHidingSourceLinesC[DML32_NUM_PLANES]{};

    v->Watermark.UrgentWatermark = mmSOCParameters.UrgentLatency + mmSOCParameters.ExtraLatency;
    v->Watermark.USRRetrainingWatermark = mmSOCParameters.UrgentLatency + mmSOCParameters.ExtraLatency
            + mmSOCParameters.USRRetrainingLatency + mmSOCParameters.SMNLatency;
    v->Watermark.DRAMClockChangeWatermark = mmSOCParameters.DRAMClockChangeLatency + v->Watermark.UrgentWatermark;
    v->Watermark.FCLKChangeWatermark = mmSOCParameters.FCLKChangeLatency + v->Watermark.UrgentWatermark;
    v->Watermark.StutterExitWatermark = mmSOCParameters.SRExitTime + mmSOCParameters.ExtraLatency
            + 10 / DCFClkDeepSleep;
    v->Watermark.StutterEnterPlusExitWatermark = mmSOCParameters.SREnterPlusExitTime + mmSOCParameters.ExtraLatency
            + 10 / DCFClkDeepSleep;
    v->Watermark.Z8StutterExitWatermark = mmSOCParameters.SRExitZ8Time + mmSOCParameters.ExtraLatency
            + 10 / DCFClkDeepSleep;
    v->Watermark.Z8StutterEnterPlusExitWatermark = mmSOCParameters.SREnterPlusExitZ8Time
            + mmSOCParameters.ExtraLatency + 10 / DCFClkDeepSleep;

    TotalActiveWriteback = 0;
    for (k = 0; k < v->NumberOfActiveSurfaces; ++k) {
        if (v->WritebackEnable[k] == true)
            TotalActiveWriteback = TotalActiveWriteback + 1;
    }

    if (TotalActiveWriteback <= 1) {
        v->Watermark.WritebackUrgentWatermark = mmSOCParameters.WritebackLatency;
    } else {
        v->Watermark.WritebackUrgentWatermark = mmSOCParameters.WritebackLatency
                + v->WritebackChunkSize * 1024.0 / 32.0 / SOCCLK;
    }
    if (v->USRRetrainingRequiredFinal)
        v->Watermark.WritebackDRAMClockChangeWatermark = v->Watermark.WritebackDRAMClockChangeWatermark
                + mmSOCParameters.USRRetrainingLatency;

    if (TotalActiveWriteback <= 1) {
        v->Watermark.WritebackDRAMClockChangeWatermark = mmSOCParameters.DRAMClockChangeLatency
                + mmSOCParameters.WritebackLatency;
        v->Watermark.WritebackFCLKChangeWatermark = mmSOCParameters.FCLKChangeLatency
                + mmSOCParameters.WritebackLatency;
    } else {
        v->Watermark.WritebackDRAMClockChangeWatermark = mmSOCParameters.DRAMClockChangeLatency
                + mmSOCParameters.WritebackLatency + v->WritebackChunkSize * 1024.0 / 32.0 / SOCCLK;
        v->Watermark.WritebackFCLKChangeWatermark = mmSOCParameters.FCLKChangeLatency
                + mmSOCParameters.WritebackLatency + v->WritebackChunkSize * 1024 / 32 / SOCCLK;
    }

    if (v->USRRetrainingRequiredFinal)
        v->Watermark.WritebackDRAMClockChangeWatermark = v->Watermark.WritebackDRAMClockChangeWatermark
                + mmSOCParameters.USRRetrainingLatency;

    if (v->USRRetrainingRequiredFinal)
        v->Watermark.WritebackFCLKChangeWatermark = v->Watermark.WritebackFCLKChangeWatermark
                + mmSOCParameters.USRRetrainingLatency;

    for (k = 0; k < v->NumberOfActiveSurfaces; ++k) {
        TotalPixelBW = TotalPixelBW + DPPPerSurface[k] * (SwathWidthY[k] * BytePerPixelDETY[k] * v->VRatio[k] +
                SwathWidthC[k] * BytePerPixelDETC[k] * v->VRatioChroma[k]) / (v->HTotal[k] / v->PixelClock[k]);
    }

    for (k = 0; k < v->NumberOfActiveSurfaces; ++k) {

        LBLatencyHidingSourceLinesY[k] = dml_min(static_cast<double>(v->MaxLineBufferLines), dml_floor(v->LineBufferSizeFinal / v->LBBitPerPixel[k] / (SwathWidthY[k] / dml_max(v->HRatio[k], 1.0)), 1)) - (v->vtaps[k] - 1);
        LBLatencyHidingSourceLinesC[k] = dml_min(static_cast<double>(v->MaxLineBufferLines), dml_floor(v->LineBufferSizeFinal / v->LBBitPerPixel[k] / (SwathWidthC[k] / dml_max(v->HRatioChroma[k], 1.0)), 1)) - (v->VTAPsChroma[k] - 1);

        EffectiveLBLatencyHidingY = LBLatencyHidingSourceLinesY[k] / v->VRatio[k] * (v->HTotal[k] / v->PixelClock[k]);
        EffectiveLBLatencyHidingC = LBLatencyHidingSourceLinesC[k] / v->VRatioChroma[k] * (v->HTotal[k] / v->PixelClock[k]);
        EffectiveDETBufferSizeY = DETBufferSizeY[k];

        if (UnboundedRequestEnabled) {
            EffectiveDETBufferSizeY = EffectiveDETBufferSizeY
                    + CompressedBufferSizeInkByte * 1024
                            * (SwathWidthY[k] * BytePerPixelDETY[k] * v->VRatio[k])
                            / (v->HTotal[k] / v->PixelClock[k]) / TotalPixelBW;
        }

        LinesInDETY[k] = static_cast<double>(EffectiveDETBufferSizeY) / BytePerPixelDETY[k] / SwathWidthY[k];
        LinesInDETYRoundedDownToSwath[k] = dml_floor(LinesInDETY[k], SwathHeightY[k]);
        FullDETBufferingTimeY = LinesInDETYRoundedDownToSwath[k] * (v->HTotal[k] / v->PixelClock[k]) / v->VRatio[k];

        ActiveClockChangeLatencyHidingY = EffectiveLBLatencyHidingY + FullDETBufferingTimeY
                - (DSTXAfterScaler[k] / v->HTotal[k] + DSTYAfterScaler[k]) * v->HTotal[k] / v->PixelClock[k];

        if (v->NumberOfActiveSurfaces > 1) {
            ActiveClockChangeLatencyHidingY = ActiveClockChangeLatencyHidingY
                    - (1.0 - 1.0 / v->NumberOfActiveSurfaces) * SwathHeightY[k] * v->HTotal[k]
                            / v->PixelClock[k] / v->VRatio[k];
        }

        if (BytePerPixelDETC[k] > 0) {
            LinesInDETC[k] = DETBufferSizeC[k] / BytePerPixelDETC[k] / SwathWidthC[k];
            LinesInDETCRoundedDownToSwath[k] = dml_floor(LinesInDETC[k], SwathHeightC[k]);
            FullDETBufferingTimeC = LinesInDETCRoundedDownToSwath[k] * (v->HTotal[k] / v->PixelClock[k])
                    / v->VRatioChroma[k];
            ActiveClockChangeLatencyHidingC = EffectiveLBLatencyHidingC + FullDETBufferingTimeC
                    - (DSTXAfterScaler[k] / v->HTotal[k] + DSTYAfterScaler[k]) * v->HTotal[k]
                            / v->PixelClock[k];
            if (v->NumberOfActiveSurfaces > 1) {
                ActiveClockChangeLatencyHidingC = ActiveClockChangeLatencyHidingC
                        - (1 - 1 / v->NumberOfActiveSurfaces) * SwathHeightC[k] * v->HTotal[k]
                                / v->PixelClock[k] / v->VRatioChroma[k];
            }
            ActiveClockChangeLatencyHiding = dml_min(ActiveClockChangeLatencyHidingY,
                    ActiveClockChangeLatencyHidingC);
        } else {
            ActiveClockChangeLatencyHiding = ActiveClockChangeLatencyHidingY;
        }

        ActiveDRAMClockChangeLatencyMargin[k] = ActiveClockChangeLatencyHiding - v->Watermark.UrgentWatermark
                - v->Watermark.DRAMClockChangeWatermark;
        ActiveFCLKChangeLatencyMargin[k] = ActiveClockChangeLatencyHiding - v->Watermark.UrgentWatermark
                - v->Watermark.FCLKChangeWatermark;
        USRRetrainingLatencyMargin[k] = ActiveClockChangeLatencyHiding - v->Watermark.USRRetrainingWatermark;

        if (v->WritebackEnable[k]) {
            WritebackLatencyHiding = v->WritebackInterfaceBufferSize * 1024
                    / (v->WritebackDestinationWidth[k] * v->WritebackDestinationHeight[k]
                            / (v->WritebackSourceHeight[k] * v->HTotal[k] / v->PixelClock[k]) * 4);
            if (v->WritebackPixelFormat[k] == dm_444_64)
                WritebackLatencyHiding = WritebackLatencyHiding / 2;

            WritebackDRAMClockChangeLatencyMargin = WritebackLatencyHiding
                    - v->Watermark.WritebackDRAMClockChangeWatermark;

            WritebackFCLKChangeLatencyMargin = WritebackLatencyHiding
                    - v->Watermark.WritebackFCLKChangeWatermark;

            ActiveDRAMClockChangeLatencyMargin[k] = dml_min(ActiveDRAMClockChangeLatencyMargin[k],
                    WritebackFCLKChangeLatencyMargin);
            ActiveFCLKChangeLatencyMargin[k] = dml_min(ActiveFCLKChangeLatencyMargin[k],
                    WritebackDRAMClockChangeLatencyMargin);
        }
        MaxActiveDRAMClockChangeLatencySupported[k] =
                (v->UsesMALLForPStateChange[k] == dm_use_mall_pstate_change_phantom_pipe) ?
                        0 :
                        (ActiveDRAMClockChangeLatencyMargin[k]
                                + mmSOCParameters.DRAMClockChangeLatency);
    }

    for (i = 0; i < v->NumberOfActiveSurfaces; ++i) {
        for (j = 0; j < v->NumberOfActiveSurfaces; ++j) {
            if (i == j ||
                    (v->BlendingAndTiming[i] == i && v->BlendingAndTiming[j] == i) ||
                    (v->BlendingAndTiming[j] == j && v->BlendingAndTiming[i] == j) ||
                    (v->BlendingAndTiming[i] == v->BlendingAndTiming[j] && v->BlendingAndTiming[i] != i) ||
                    (v->SynchronizeTimingsFinal && v->PixelClock[i] == v->PixelClock[j] &&
                    v->HTotal[i] == v->HTotal[j] && v->VTotal[i] == v->VTotal[j] &&
                    v->VActive[i] == v->VActive[j]) || (v->SynchronizeDRRDisplaysForUCLKPStateChangeFinal &&
                    (v->DRRDisplay[i] || v->DRRDisplay[j]))) {
                SynchronizedSurfaces[i][j] = true;
            } else {
                SynchronizedSurfaces[i][j] = false;
            }
        }
    }

    for (k = 0; k < v->NumberOfActiveSurfaces; ++k) {
        if ((v->UsesMALLForPStateChange[k] != dm_use_mall_pstate_change_phantom_pipe) &&
                (!FoundFirstSurfaceWithMinActiveFCLKChangeMargin ||
                ActiveFCLKChangeLatencyMargin[k] < MinActiveFCLKChangeMargin)) {
            FoundFirstSurfaceWithMinActiveFCLKChangeMargin = true;
            MinActiveFCLKChangeMargin = ActiveFCLKChangeLatencyMargin[k];
            SurfaceWithMinActiveFCLKChangeMargin = k;
        }
    }

    *MinActiveFCLKChangeLatencySupported = MinActiveFCLKChangeMargin + mmSOCParameters.FCLKChangeLatency;

    SameTimingForFCLKChange = true;
    for (k = 0; k < v->NumberOfActiveSurfaces; ++k) {
        if (!SynchronizedSurfaces[k][SurfaceWithMinActiveFCLKChangeMargin]) {
            if ((v->UsesMALLForPStateChange[k] != dm_use_mall_pstate_change_phantom_pipe) &&
                    (SameTimingForFCLKChange ||
                    ActiveFCLKChangeLatencyMargin[k] <
                    SecondMinActiveFCLKChangeMarginOneDisplayInVBLank)) {
                SecondMinActiveFCLKChangeMarginOneDisplayInVBLank = ActiveFCLKChangeLatencyMargin[k];
            }
            SameTimingForFCLKChange = false;
        }
    }

    if (MinActiveFCLKChangeMargin > 0) {
        *FCLKChangeSupport = dm_fclock_change_vactive;
    } else if ((SameTimingForFCLKChange || SecondMinActiveFCLKChangeMarginOneDisplayInVBLank > 0) &&
            (PrefetchMode <= 1)) {
        *FCLKChangeSupport = dm_fclock_change_vblank;
    } else {
        *FCLKChangeSupport = dm_fclock_change_unsupported;
    }

    *USRRetrainingSupport = true;
    for (k = 0; k < v->NumberOfActiveSurfaces; ++k) {
        if ((v->UsesMALLForPStateChange[k] != dm_use_mall_pstate_change_phantom_pipe) &&
                (USRRetrainingLatencyMargin[k] < 0)) {
            *USRRetrainingSupport = false;
        }
    }

    for (k = 0; k < v->NumberOfActiveSurfaces; ++k) {
        if (v->UsesMALLForPStateChange[k] != dm_use_mall_pstate_change_full_frame &&
                v->UsesMALLForPStateChange[k] != dm_use_mall_pstate_change_sub_viewport &&
                v->UsesMALLForPStateChange[k] != dm_use_mall_pstate_change_phantom_pipe &&
                ActiveDRAMClockChangeLatencyMargin[k] < 0) {
            if (PrefetchMode > 0) {
                DRAMClockChangeSupportNumber = 2;
            } else if (DRAMClockChangeSupportNumber == 0) {
                DRAMClockChangeSupportNumber = 1;
                LastSurfaceWithoutMargin = k;
            } else if (DRAMClockChangeSupportNumber == 1 &&
                    !SynchronizedSurfaces[LastSurfaceWithoutMargin][k]) {
                DRAMClockChangeSupportNumber = 2;
            }
        }
    }

    for (k = 0; k < v->NumberOfActiveSurfaces; ++k) {
        if (v->UsesMALLForPStateChange[k] == dm_use_mall_pstate_change_full_frame)
            DRAMClockChangeMethod = 1;
        else if (v->UsesMALLForPStateChange[k] == dm_use_mall_pstate_change_sub_viewport)
            DRAMClockChangeMethod = 2;
    }

    if (DRAMClockChangeMethod == 0) {
        if (DRAMClockChangeSupportNumber == 0)
            *DRAMClockChangeSupport = dm_dram_clock_change_vactive;
        else if (DRAMClockChangeSupportNumber == 1)
            *DRAMClockChangeSupport = dm_dram_clock_change_vblank;
        else
            *DRAMClockChangeSupport = dm_dram_clock_change_unsupported;
    } else if (DRAMClockChangeMethod == 1) {
        if (DRAMClockChangeSupportNumber == 0)
            *DRAMClockChangeSupport = dm_dram_clock_change_vactive_w_mall_full_frame;
        else if (DRAMClockChangeSupportNumber == 1)
            *DRAMClockChangeSupport = dm_dram_clock_change_vblank_w_mall_full_frame;
        else
            *DRAMClockChangeSupport = dm_dram_clock_change_unsupported;
    } else {
        if (DRAMClockChangeSupportNumber == 0)
            *DRAMClockChangeSupport = dm_dram_clock_change_vactive_w_mall_sub_vp;
        else if (DRAMClockChangeSupportNumber == 1)
            *DRAMClockChangeSupport = dm_dram_clock_change_vblank_w_mall_sub_vp;
        else
            *DRAMClockChangeSupport = dm_dram_clock_change_unsupported;
    }

    for (k = 0; k < v->NumberOfActiveSurfaces; ++k) {
        unsigned int dst_y_pstate;
        unsigned int src_y_pstate_l;
        unsigned int src_y_pstate_c;
        unsigned int src_y_ahead_l, src_y_ahead_c, sub_vp_lines_l, sub_vp_lines_c;

        dst_y_pstate = dml_ceil((mmSOCParameters.DRAMClockChangeLatency + mmSOCParameters.UrgentLatency) / (v->HTotal[k] / v->PixelClock[k]), 1);
        src_y_pstate_l = dml_ceil(dst_y_pstate * v->VRatio[k], SwathHeightY[k]);
        src_y_ahead_l = dml_floor(DETBufferSizeY[k] / BytePerPixelDETY[k] / SwathWidthY[k], SwathHeightY[k]) + LBLatencyHidingSourceLinesY[k];
        sub_vp_lines_l = src_y_pstate_l + src_y_ahead_l + v->meta_row_height[k];

        SubViewportLinesNeededInMALL[k] = sub_vp_lines_l;

        if (BytePerPixelDETC[k] > 0) {
            src_y_pstate_c = dml_ceil(dst_y_pstate * v->VRatioChroma[k], SwathHeightC[k]);
            src_y_ahead_c = dml_floor(DETBufferSizeC[k] / BytePerPixelDETC[k] / SwathWidthC[k], SwathHeightC[k]) + LBLatencyHidingSourceLinesC[k];
            sub_vp_lines_c = src_y_pstate_c + src_y_ahead_c + v->meta_row_height_chroma[k];
            SubViewportLinesNeededInMALL[k] = dml_max(sub_vp_lines_l, sub_vp_lines_c);

        }
    }
} // CalculateWatermarksMALLUseAndDRAMSpeedChangeSupport

void dml32_CalculateMinAndMaxPrefetchMode(
        enum dm_prefetch_modes   AllowForPStateChangeOrStutterInVBlankFinal,
        unsigned int             *MinPrefetchMode,
        unsigned int             *MaxPrefetchMode)
{
    if (AllowForPStateChangeOrStutterInVBlankFinal == dm_prefetch_support_none) {
        *MinPrefetchMode = 3;
        *MaxPrefetchMode = 3;
    } else if (AllowForPStateChangeOrStutterInVBlankFinal == dm_prefetch_support_stutter) {
        *MinPrefetchMode = 2;
        *MaxPrefetchMode = 2;
    } else if (AllowForPStateChangeOrStutterInVBlankFinal == dm_prefetch_support_fclk_and_stutter) {
        *MinPrefetchMode = 1;
        *MaxPrefetchMode = 1;
    } else if (AllowForPStateChangeOrStutterInVBlankFinal == dm_prefetch_support_uclk_fclk_and_stutter) {
        *MinPrefetchMode = 0;
        *MaxPrefetchMode = 0;
    } else {
        *MinPrefetchMode = 0;
        *MaxPrefetchMode = 3;
    }
} // CalculateMinAndMaxPrefetchMode

void dml32_CalculatePixelDeliveryTimes(
        unsigned int             NumberOfActiveSurfaces,
        double              VRatio[],
        double              VRatioChroma[],
        double              VRatioPrefetchY[],
        double              VRatioPrefetchC[],
        unsigned int             swath_width_luma_ub[],
        unsigned int             swath_width_chroma_ub[],
        unsigned int             DPPPerSurface[],
        double              HRatio[],
        double              HRatioChroma[],
        double              PixelClock[],
        double              PSCL_THROUGHPUT[],
        double              PSCL_THROUGHPUT_CHROMA[],
        double              Dppclk[],
        unsigned int             BytePerPixelC[],
        enum dm_rotation_angle   SourceRotation[],
        unsigned int             NumberOfCursors[],
        unsigned int             CursorWidth[][DC__NUM_CURSOR__MAX],
        unsigned int             CursorBPP[][DC__NUM_CURSOR__MAX],
        unsigned int             BlockWidth256BytesY[],
        unsigned int             BlockHeight256BytesY[],
        unsigned int             BlockWidth256BytesC[],
        unsigned int             BlockHeight256BytesC[],

        /* Output */
        double              DisplayPipeLineDeliveryTimeLuma[],
        double              DisplayPipeLineDeliveryTimeChroma[],
        double              DisplayPipeLineDeliveryTimeLumaPrefetch[],
        double              DisplayPipeLineDeliveryTimeChromaPrefetch[],
        double              DisplayPipeRequestDeliveryTimeLuma[],
        double              DisplayPipeRequestDeliveryTimeChroma[],
        double              DisplayPipeRequestDeliveryTimeLumaPrefetch[],
        double              DisplayPipeRequestDeliveryTimeChromaPrefetch[],
        double              CursorRequestDeliveryTime[],
        double              CursorRequestDeliveryTimePrefetch[])
{
    double   req_per_swath_ub;
    unsigned int k;

    for (k = 0; k < NumberOfActiveSurfaces; ++k) {

        if (VRatio[k] <= 1) {
            DisplayPipeLineDeliveryTimeLuma[k] =
                    swath_width_luma_ub[k] * DPPPerSurface[k] / HRatio[k] / PixelClock[k];
        } else {
            DisplayPipeLineDeliveryTimeLuma[k] = swath_width_luma_ub[k] / PSCL_THROUGHPUT[k] / Dppclk[k];
        }

        if (BytePerPixelC[k] == 0) {
            DisplayPipeLineDeliveryTimeChroma[k] = 0;
        } else {
            if (VRatioChroma[k] <= 1) {
                DisplayPipeLineDeliveryTimeChroma[k] =
                    swath_width_chroma_ub[k] * DPPPerSurface[k] / HRatioChroma[k] / PixelClock[k];
            } else {
                DisplayPipeLineDeliveryTimeChroma[k] =
                    swath_width_chroma_ub[k] / PSCL_THROUGHPUT_CHROMA[k] / Dppclk[k];
            }
        }

        if (VRatioPrefetchY[k] <= 1) {
            DisplayPipeLineDeliveryTimeLumaPrefetch[k] =
                    swath_width_luma_ub[k] * DPPPerSurface[k] / HRatio[k] / PixelClock[k];
        } else {
            DisplayPipeLineDeliveryTimeLumaPrefetch[k] =
                    swath_width_luma_ub[k] / PSCL_THROUGHPUT[k] / Dppclk[k];
        }

        if (BytePerPixelC[k] == 0) {
            DisplayPipeLineDeliveryTimeChromaPrefetch[k] = 0;
        } else {
            if (VRatioPrefetchC[k] <= 1) {
                DisplayPipeLineDeliveryTimeChromaPrefetch[k] = swath_width_chroma_ub[k] *
                        DPPPerSurface[k] / HRatioChroma[k] / PixelClock[k];
            } else {
                DisplayPipeLineDeliveryTimeChromaPrefetch[k] =
                        swath_width_chroma_ub[k] / PSCL_THROUGHPUT_CHROMA[k] / Dppclk[k];
            }
        }
    }

    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        if (!IsVertical(SourceRotation[k]))
            req_per_swath_ub = swath_width_luma_ub[k] / BlockWidth256BytesY[k];
        else
            req_per_swath_ub = swath_width_luma_ub[k] / BlockHeight256BytesY[k];

        DisplayPipeRequestDeliveryTimeLuma[k] = DisplayPipeLineDeliveryTimeLuma[k] / req_per_swath_ub;
        DisplayPipeRequestDeliveryTimeLumaPrefetch[k] =
                DisplayPipeLineDeliveryTimeLumaPrefetch[k] / req_per_swath_ub;
        if (BytePerPixelC[k] == 0) {
            DisplayPipeRequestDeliveryTimeChroma[k] = 0;
            DisplayPipeRequestDeliveryTimeChromaPrefetch[k] = 0;
        } else {
            if (!IsVertical(SourceRotation[k]))
                req_per_swath_ub = swath_width_chroma_ub[k] / BlockWidth256BytesC[k];
            else
                req_per_swath_ub = swath_width_chroma_ub[k] / BlockHeight256BytesC[k];
            DisplayPipeRequestDeliveryTimeChroma[k] =
                    DisplayPipeLineDeliveryTimeChroma[k] / req_per_swath_ub;
            DisplayPipeRequestDeliveryTimeChromaPrefetch[k] =
                    DisplayPipeLineDeliveryTimeChromaPrefetch[k] / req_per_swath_ub;
        }
    }

    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        unsigned int cursor_req_per_width;

        cursor_req_per_width = dml_ceil(static_cast<double>(CursorWidth[k][0]) * static_cast<double>(CursorBPP[k][0]) /
                256.0 / 8.0, 1.0);
        if (NumberOfCursors[k] > 0) {
            if (VRatio[k] <= 1) {
                CursorRequestDeliveryTime[k] = static_cast<double>(CursorWidth[k][0]) /
                        HRatio[k] / PixelClock[k] / cursor_req_per_width;
            } else {
                CursorRequestDeliveryTime[k] = static_cast<double>(CursorWidth[k][0]) /
                        PSCL_THROUGHPUT[k] / Dppclk[k] / cursor_req_per_width;
            }
            if (VRatioPrefetchY[k] <= 1) {
                CursorRequestDeliveryTimePrefetch[k] = static_cast<double>(CursorWidth[k][0]) /
                        HRatio[k] / PixelClock[k] / cursor_req_per_width;
            } else {
                CursorRequestDeliveryTimePrefetch[k] = static_cast<double>(CursorWidth[k][0]) /
                        PSCL_THROUGHPUT[k] / Dppclk[k] / cursor_req_per_width;
            }
        } else {
            CursorRequestDeliveryTime[k] = 0;
            CursorRequestDeliveryTimePrefetch[k] = 0;
        }
    }
} // CalculatePixelDeliveryTimes

void dml32_CalculateMetaAndPTETimes(
        bool use_one_row_for_frame[],
        unsigned int NumberOfActiveSurfaces,
        bool GPUVMEnable,
        unsigned int MetaChunkSize,
        unsigned int MinMetaChunkSizeBytes,
        unsigned int    HTotal[],
        double  VRatio[],
        double  VRatioChroma[],
        double  DestinationLinesToRequestRowInVBlank[],
        double  DestinationLinesToRequestRowInImmediateFlip[],
        bool DCCEnable[],
        double  PixelClock[],
        unsigned int BytePerPixelY[],
        unsigned int BytePerPixelC[],
        enum dm_rotation_angle SourceRotation[],
        unsigned int dpte_row_height[],
        unsigned int dpte_row_height_chroma[],
        unsigned int meta_row_width[],
        unsigned int meta_row_width_chroma[],
        unsigned int meta_row_height[],
        unsigned int meta_row_height_chroma[],
        unsigned int meta_req_width[],
        unsigned int meta_req_width_chroma[],
        unsigned int meta_req_height[],
        unsigned int meta_req_height_chroma[],
        unsigned int dpte_group_bytes[],
        unsigned int    PTERequestSizeY[],
        unsigned int    PTERequestSizeC[],
        unsigned int    PixelPTEReqWidthY[],
        unsigned int    PixelPTEReqHeightY[],
        unsigned int    PixelPTEReqWidthC[],
        unsigned int    PixelPTEReqHeightC[],
        unsigned int    dpte_row_width_luma_ub[],
        unsigned int    dpte_row_width_chroma_ub[],

        /* Output */
        double DST_Y_PER_PTE_ROW_NOM_L[],
        double DST_Y_PER_PTE_ROW_NOM_C[],
        double DST_Y_PER_META_ROW_NOM_L[],
        double DST_Y_PER_META_ROW_NOM_C[],
        double TimePerMetaChunkNominal[],
        double TimePerChromaMetaChunkNominal[],
        double TimePerMetaChunkVBlank[],
        double TimePerChromaMetaChunkVBlank[],
        double TimePerMetaChunkFlip[],
        double TimePerChromaMetaChunkFlip[],
        double time_per_pte_group_nom_luma[],
        double time_per_pte_group_vblank_luma[],
        double time_per_pte_group_flip_luma[],
        double time_per_pte_group_nom_chroma[],
        double time_per_pte_group_vblank_chroma[],
        double time_per_pte_group_flip_chroma[])
{
    unsigned int   meta_chunk_width;
    unsigned int   min_meta_chunk_width;
    unsigned int   meta_chunk_per_row_int;
    unsigned int   meta_row_remainder;
    unsigned int   meta_chunk_threshold;
    unsigned int   meta_chunks_per_row_ub;
    unsigned int   meta_chunk_width_chroma;
    unsigned int   min_meta_chunk_width_chroma;
    unsigned int   meta_chunk_per_row_int_chroma;
    unsigned int   meta_row_remainder_chroma;
    unsigned int   meta_chunk_threshold_chroma;
    unsigned int   meta_chunks_per_row_ub_chroma;
    unsigned int   dpte_group_width_luma;
    unsigned int   dpte_groups_per_row_luma_ub;
    unsigned int   dpte_group_width_chroma;
    unsigned int   dpte_groups_per_row_chroma_ub;
    unsigned int k;

    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        DST_Y_PER_PTE_ROW_NOM_L[k] = dpte_row_height[k] / VRatio[k];
        if (BytePerPixelC[k] == 0)
            DST_Y_PER_PTE_ROW_NOM_C[k] = 0;
        else
            DST_Y_PER_PTE_ROW_NOM_C[k] = dpte_row_height_chroma[k] / VRatioChroma[k];
        DST_Y_PER_META_ROW_NOM_L[k] = meta_row_height[k] / VRatio[k];
        if (BytePerPixelC[k] == 0)
            DST_Y_PER_META_ROW_NOM_C[k] = 0;
        else
            DST_Y_PER_META_ROW_NOM_C[k] = meta_row_height_chroma[k] / VRatioChroma[k];
    }

    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        if (DCCEnable[k] == true) {
            meta_chunk_width = MetaChunkSize * 1024 * 256 / BytePerPixelY[k] / meta_row_height[k];
            min_meta_chunk_width = MinMetaChunkSizeBytes * 256 / BytePerPixelY[k] / meta_row_height[k];
            meta_chunk_per_row_int = meta_row_width[k] / meta_chunk_width;
            meta_row_remainder = meta_row_width[k] % meta_chunk_width;
            if (!IsVertical(SourceRotation[k]))
                meta_chunk_threshold = 2 * min_meta_chunk_width - meta_req_width[k];
            else
                meta_chunk_threshold = 2 * min_meta_chunk_width - meta_req_height[k];

            if (meta_row_remainder <= meta_chunk_threshold)
                meta_chunks_per_row_ub = meta_chunk_per_row_int + 1;
            else
                meta_chunks_per_row_ub = meta_chunk_per_row_int + 2;

            TimePerMetaChunkNominal[k] = meta_row_height[k] / VRatio[k] *
                    HTotal[k] / PixelClock[k] / meta_chunks_per_row_ub;
            TimePerMetaChunkVBlank[k] = DestinationLinesToRequestRowInVBlank[k] *
                    HTotal[k] / PixelClock[k] / meta_chunks_per_row_ub;
            TimePerMetaChunkFlip[k] = DestinationLinesToRequestRowInImmediateFlip[k] *
                    HTotal[k] / PixelClock[k] / meta_chunks_per_row_ub;
            if (BytePerPixelC[k] == 0) {
                TimePerChromaMetaChunkNominal[k] = 0;
                TimePerChromaMetaChunkVBlank[k] = 0;
                TimePerChromaMetaChunkFlip[k] = 0;
            } else {
                meta_chunk_width_chroma = MetaChunkSize * 1024 * 256 / BytePerPixelC[k] /
                        meta_row_height_chroma[k];
                min_meta_chunk_width_chroma = MinMetaChunkSizeBytes * 256 / BytePerPixelC[k] /
                        meta_row_height_chroma[k];
                meta_chunk_per_row_int_chroma = static_cast<double>(meta_row_width_chroma[k]) /
                        meta_chunk_width_chroma;
                meta_row_remainder_chroma = meta_row_width_chroma[k] % meta_chunk_width_chroma;
                if (!IsVertical(SourceRotation[k])) {
                    meta_chunk_threshold_chroma = 2 * min_meta_chunk_width_chroma -
                            meta_req_width_chroma[k];
                } else {
                    meta_chunk_threshold_chroma = 2 * min_meta_chunk_width_chroma -
                            meta_req_height_chroma[k];
                }
                if (meta_row_remainder_chroma <= meta_chunk_threshold_chroma)
                    meta_chunks_per_row_ub_chroma = meta_chunk_per_row_int_chroma + 1;
                else
                    meta_chunks_per_row_ub_chroma = meta_chunk_per_row_int_chroma + 2;

                TimePerChromaMetaChunkNominal[k] = meta_row_height_chroma[k] / VRatioChroma[k] *
                        HTotal[k] / PixelClock[k] / meta_chunks_per_row_ub_chroma;
                TimePerChromaMetaChunkVBlank[k] = DestinationLinesToRequestRowInVBlank[k] *
                        HTotal[k] / PixelClock[k] / meta_chunks_per_row_ub_chroma;
                TimePerChromaMetaChunkFlip[k] = DestinationLinesToRequestRowInImmediateFlip[k] *
                        HTotal[k] / PixelClock[k] / meta_chunks_per_row_ub_chroma;
            }
        } else {
            TimePerMetaChunkNominal[k] = 0;
            TimePerMetaChunkVBlank[k] = 0;
            TimePerMetaChunkFlip[k] = 0;
            TimePerChromaMetaChunkNominal[k] = 0;
            TimePerChromaMetaChunkVBlank[k] = 0;
            TimePerChromaMetaChunkFlip[k] = 0;
        }
    }

    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        if (GPUVMEnable == true) {
            if (!IsVertical(SourceRotation[k])) {
                dpte_group_width_luma = static_cast<double>(dpte_group_bytes[k]) /
                        static_cast<double>(PTERequestSizeY[k]) * PixelPTEReqWidthY[k];
            } else {
                dpte_group_width_luma = static_cast<double>(dpte_group_bytes[k]) /
                        static_cast<double>(PTERequestSizeY[k]) * PixelPTEReqHeightY[k];
            }

            if (use_one_row_for_frame[k]) {
                dpte_groups_per_row_luma_ub = dml_ceil(static_cast<double>(dpte_row_width_luma_ub[k]) /
                        static_cast<double>(dpte_group_width_luma) / 2.0, 1.0);
            } else {
                dpte_groups_per_row_luma_ub = dml_ceil(static_cast<double>(dpte_row_width_luma_ub[k]) /
                        static_cast<double>(dpte_group_width_luma), 1.0);
            }

            time_per_pte_group_nom_luma[k] = DST_Y_PER_PTE_ROW_NOM_L[k] *
                    HTotal[k] / PixelClock[k] / dpte_groups_per_row_luma_ub;
            time_per_pte_group_vblank_luma[k] = DestinationLinesToRequestRowInVBlank[k] *
                    HTotal[k] / PixelClock[k] / dpte_groups_per_row_luma_ub;
            time_per_pte_group_flip_luma[k] = DestinationLinesToRequestRowInImmediateFlip[k] *
                    HTotal[k] / PixelClock[k] / dpte_groups_per_row_luma_ub;
            if (BytePerPixelC[k] == 0) {
                time_per_pte_group_nom_chroma[k] = 0;
                time_per_pte_group_vblank_chroma[k] = 0;
                time_per_pte_group_flip_chroma[k] = 0;
            } else {
                if (!IsVertical(SourceRotation[k])) {
                    dpte_group_width_chroma = static_cast<double>(dpte_group_bytes[k]) /
                            static_cast<double>(PTERequestSizeC[k]) * PixelPTEReqWidthC[k];
                } else {
                    dpte_group_width_chroma = static_cast<double>(dpte_group_bytes[k]) /
                            static_cast<double>(PTERequestSizeC[k]) * PixelPTEReqHeightC[k];
                }

                if (use_one_row_for_frame[k]) {
                    dpte_groups_per_row_chroma_ub = dml_ceil(static_cast<double>(dpte_row_width_chroma_ub[k]) /
                            static_cast<double>(dpte_group_width_chroma) / 2.0, 1.0);
                } else {
                    dpte_groups_per_row_chroma_ub = dml_ceil(static_cast<double>(dpte_row_width_chroma_ub[k]) /
                            static_cast<double>(dpte_group_width_chroma), 1.0);
                }
                time_per_pte_group_nom_chroma[k] = DST_Y_PER_PTE_ROW_NOM_C[k] *
                        HTotal[k] / PixelClock[k] / dpte_groups_per_row_chroma_ub;
                time_per_pte_group_vblank_chroma[k] = DestinationLinesToRequestRowInVBlank[k] *
                        HTotal[k] / PixelClock[k] / dpte_groups_per_row_chroma_ub;
                time_per_pte_group_flip_chroma[k] = DestinationLinesToRequestRowInImmediateFlip[k] *
                        HTotal[k] / PixelClock[k] / dpte_groups_per_row_chroma_ub;
            }
        } else {
            time_per_pte_group_nom_luma[k] = 0;
            time_per_pte_group_vblank_luma[k] = 0;
            time_per_pte_group_flip_luma[k] = 0;
            time_per_pte_group_nom_chroma[k] = 0;
            time_per_pte_group_vblank_chroma[k] = 0;
            time_per_pte_group_flip_chroma[k] = 0;
        }
    }
} // CalculateMetaAndPTETimes

void dml32_CalculateVMGroupAndRequestTimes(
        unsigned int     NumberOfActiveSurfaces,
        bool     GPUVMEnable,
        unsigned int     GPUVMMaxPageTableLevels,
        unsigned int     HTotal[],
        unsigned int     BytePerPixelC[],
        double      DestinationLinesToRequestVMInVBlank[],
        double      DestinationLinesToRequestVMInImmediateFlip[],
        bool     DCCEnable[],
        double      PixelClock[],
        unsigned int        /* dpte_row_width_luma_ub */[],
        unsigned int        /* dpte_row_width_chroma_ub */[],
        unsigned int     vm_group_bytes[],
        unsigned int     dpde0_bytes_per_frame_ub_l[],
        unsigned int     dpde0_bytes_per_frame_ub_c[],
        unsigned int        meta_pte_bytes_per_frame_ub_l[],
        unsigned int        meta_pte_bytes_per_frame_ub_c[],

        /* Output */
        double      TimePerVMGroupVBlank[],
        double      TimePerVMGroupFlip[],
        double      TimePerVMRequestVBlank[],
        double      TimePerVMRequestFlip[])
{
    unsigned int k;
    unsigned int   num_group_per_lower_vm_stage;
    unsigned int   num_req_per_lower_vm_stage;

    for (k = 0; k < NumberOfActiveSurfaces; ++k) {

        if (GPUVMEnable == true && (DCCEnable[k] == true || GPUVMMaxPageTableLevels > 1)) {
            if (DCCEnable[k] == false) {
                if (BytePerPixelC[k] > 0) {
                    num_group_per_lower_vm_stage = dml_ceil(
                            static_cast<double>(dpde0_bytes_per_frame_ub_l[k]) /
                            static_cast<double>(vm_group_bytes[k]), 1.0) +
                            dml_ceil(static_cast<double>(dpde0_bytes_per_frame_ub_c[k]) /
                            static_cast<double>(vm_group_bytes[k]), 1.0);
                } else {
                    num_group_per_lower_vm_stage = dml_ceil(
                            static_cast<double>(dpde0_bytes_per_frame_ub_l[k]) /
                            static_cast<double>(vm_group_bytes[k]), 1.0);
                }
            } else {
                if (GPUVMMaxPageTableLevels == 1) {
                    if (BytePerPixelC[k] > 0) {
                        num_group_per_lower_vm_stage = dml_ceil(
                            static_cast<double>(meta_pte_bytes_per_frame_ub_l[k]) /
                            static_cast<double>(vm_group_bytes[k]), 1.0) +
                            dml_ceil(static_cast<double>(meta_pte_bytes_per_frame_ub_c[k]) /
                            static_cast<double>(vm_group_bytes[k]), 1.0);
                    } else {
                        num_group_per_lower_vm_stage = dml_ceil(
                                static_cast<double>(meta_pte_bytes_per_frame_ub_l[k]) /
                                static_cast<double>(vm_group_bytes[k]), 1.0);
                    }
                } else {
                    if (BytePerPixelC[k] > 0) {
                        num_group_per_lower_vm_stage = 2 + dml_ceil(
                            static_cast<double>(dpde0_bytes_per_frame_ub_l[k]) /
                            static_cast<double>(vm_group_bytes[k]), 1) +
                            dml_ceil(static_cast<double>(dpde0_bytes_per_frame_ub_c[k]) /
                            static_cast<double>(vm_group_bytes[k]), 1) +
                            dml_ceil(static_cast<double>(meta_pte_bytes_per_frame_ub_l[k]) /
                            static_cast<double>(vm_group_bytes[k]), 1) +
                            dml_ceil(static_cast<double>(meta_pte_bytes_per_frame_ub_c[k]) /
                            static_cast<double>(vm_group_bytes[k]), 1);
                    } else {
                        num_group_per_lower_vm_stage = 1 + dml_ceil(
                            static_cast<double>(dpde0_bytes_per_frame_ub_l[k]) /
                            static_cast<double>(vm_group_bytes[k]), 1) + dml_ceil(
                            static_cast<double>(meta_pte_bytes_per_frame_ub_l[k]) /
                            static_cast<double>(vm_group_bytes[k]), 1);
                    }
                }
            }

            if (DCCEnable[k] == false) {
                if (BytePerPixelC[k] > 0) {
                    num_req_per_lower_vm_stage = dpde0_bytes_per_frame_ub_l[k] / 64 +
                            dpde0_bytes_per_frame_ub_c[k] / 64;
                } else {
                    num_req_per_lower_vm_stage = dpde0_bytes_per_frame_ub_l[k] / 64;
                }
            } else {
                if (GPUVMMaxPageTableLevels == 1) {
                    if (BytePerPixelC[k] > 0) {
                        num_req_per_lower_vm_stage = meta_pte_bytes_per_frame_ub_l[k] / 64 +
                                meta_pte_bytes_per_frame_ub_c[k] / 64;
                    } else {
                        num_req_per_lower_vm_stage = meta_pte_bytes_per_frame_ub_l[k] / 64;
                    }
                } else {
                    if (BytePerPixelC[k] > 0) {
                        num_req_per_lower_vm_stage = dpde0_bytes_per_frame_ub_l[k] /
                                64 + dpde0_bytes_per_frame_ub_c[k] / 64 +
                                meta_pte_bytes_per_frame_ub_l[k] / 64 +
                                meta_pte_bytes_per_frame_ub_c[k] / 64;
                    } else {
                        num_req_per_lower_vm_stage = dpde0_bytes_per_frame_ub_l[k] /
                                64 + meta_pte_bytes_per_frame_ub_l[k] / 64;
                    }
                }
            }

            TimePerVMGroupVBlank[k] = DestinationLinesToRequestVMInVBlank[k] *
                    HTotal[k] / PixelClock[k] / num_group_per_lower_vm_stage;
            TimePerVMGroupFlip[k] = DestinationLinesToRequestVMInImmediateFlip[k] *
                    HTotal[k] / PixelClock[k] / num_group_per_lower_vm_stage;
            TimePerVMRequestVBlank[k] = DestinationLinesToRequestVMInVBlank[k] *
                    HTotal[k] / PixelClock[k] / num_req_per_lower_vm_stage;
            TimePerVMRequestFlip[k] = DestinationLinesToRequestVMInImmediateFlip[k] *
                    HTotal[k] / PixelClock[k] / num_req_per_lower_vm_stage;

            if (GPUVMMaxPageTableLevels > 2) {
                TimePerVMGroupVBlank[k]    = TimePerVMGroupVBlank[k] / 2;
                TimePerVMGroupFlip[k]      = TimePerVMGroupFlip[k] / 2;
                TimePerVMRequestVBlank[k]  = TimePerVMRequestVBlank[k] / 2;
                TimePerVMRequestFlip[k]    = TimePerVMRequestFlip[k] / 2;
            }

        } else {
            TimePerVMGroupVBlank[k] = 0;
            TimePerVMGroupFlip[k] = 0;
            TimePerVMRequestVBlank[k] = 0;
            TimePerVMRequestFlip[k] = 0;
        }

    }
} // CalculateVMGroupAndRequestTimes

void dml32_CalculateDCCConfiguration(
        bool             DCCEnabled,
        bool             DCCProgrammingAssumesScanDirectionUnknown,
        enum source_format_class SourcePixelFormat,
        unsigned int             SurfaceWidthLuma,
        unsigned int             /* SurfaceWidthChroma */,
        unsigned int             SurfaceHeightLuma,
        unsigned int             /* SurfaceHeightChroma */,
        unsigned int                nomDETInKByte,
        unsigned int             RequestHeight256ByteLuma,
        unsigned int             RequestHeight256ByteChroma,
        enum dm_swizzle_mode     /* TilingFormat */,
        unsigned int             BytePerPixelY,
        unsigned int             BytePerPixelC,
        double              /* BytePerPixelDETY */,
        double              /* BytePerPixelDETC */,
        enum dm_rotation_angle   SourceRotation,
        /* Output */
        unsigned int        *MaxUncompressedBlockLuma,
        unsigned int        *MaxUncompressedBlockChroma,
        unsigned int        *MaxCompressedBlockLuma,
        unsigned int        *MaxCompressedBlockChroma,
        unsigned int        *IndependentBlockLuma,
        unsigned int        *IndependentBlockChroma)
{
    typedef enum {
        REQ_256Bytes,
        REQ_128BytesNonContiguous,
        REQ_128BytesContiguous,
        REQ_NA
    } RequestType;

    RequestType   RequestLuma;
    RequestType   RequestChroma;

    unsigned int   segment_order_horz_contiguous_luma;
    unsigned int   segment_order_horz_contiguous_chroma;
    unsigned int   segment_order_vert_contiguous_luma;
    unsigned int   segment_order_vert_contiguous_chroma;
    unsigned int req128_horz_wc_l;
    unsigned int req128_horz_wc_c;
    unsigned int req128_vert_wc_l;
    unsigned int req128_vert_wc_c;
    unsigned int MAS_vp_horz_limit;
    unsigned int MAS_vp_vert_limit;
    unsigned int max_vp_horz_width;
    unsigned int max_vp_vert_height;
    unsigned int eff_surf_width_l;
    unsigned int eff_surf_width_c;
    unsigned int eff_surf_height_l;
    unsigned int eff_surf_height_c;
    unsigned int full_swath_bytes_horz_wc_l;
    unsigned int full_swath_bytes_horz_wc_c;
    unsigned int full_swath_bytes_vert_wc_l;
    unsigned int full_swath_bytes_vert_wc_c;
    unsigned int DETBufferSizeForDCC = nomDETInKByte * 1024;

    unsigned int   yuv420;
    unsigned int   horz_div_l;
    unsigned int   horz_div_c;
    unsigned int   vert_div_l;
    unsigned int   vert_div_c;

    unsigned int     swath_buf_size;
    double   detile_buf_vp_horz_limit;
    double   detile_buf_vp_vert_limit;

    yuv420 = ((SourcePixelFormat == dm_420_8 || SourcePixelFormat == dm_420_10 ||
            SourcePixelFormat == dm_420_12) ? 1 : 0);
    horz_div_l = 1;
    horz_div_c = 1;
    vert_div_l = 1;
    vert_div_c = 1;

    if (BytePerPixelY == 1)
        vert_div_l = 0;
    if (BytePerPixelC == 1)
        vert_div_c = 0;

    if (BytePerPixelC == 0) {
        swath_buf_size = DETBufferSizeForDCC / 2 - 2 * 256;
        detile_buf_vp_horz_limit = static_cast<double>(swath_buf_size) / (static_cast<double>(RequestHeight256ByteLuma) *
                BytePerPixelY / (1 + horz_div_l));
        detile_buf_vp_vert_limit = static_cast<double>(swath_buf_size) / (256.0 / RequestHeight256ByteLuma /
                (1 + vert_div_l));
    } else {
        swath_buf_size = DETBufferSizeForDCC / 2 - 2 * 2 * 256;
        detile_buf_vp_horz_limit = static_cast<double>(swath_buf_size) / (static_cast<double>(RequestHeight256ByteLuma) *
                BytePerPixelY / (1 + horz_div_l) + static_cast<double>(RequestHeight256ByteChroma) *
                BytePerPixelC / (1 + horz_div_c) / (1 + yuv420));
        detile_buf_vp_vert_limit = static_cast<double>(swath_buf_size) / (256.0 / RequestHeight256ByteLuma /
                (1 + vert_div_l) + 256.0 / RequestHeight256ByteChroma /
                (1 + vert_div_c) / (1 + yuv420));
    }

    if (SourcePixelFormat == dm_420_10) {
        detile_buf_vp_horz_limit = 1.5 * detile_buf_vp_horz_limit;
        detile_buf_vp_vert_limit = 1.5 * detile_buf_vp_vert_limit;
    }

    detile_buf_vp_horz_limit = dml_floor(detile_buf_vp_horz_limit - 1, 16);
    detile_buf_vp_vert_limit = dml_floor(detile_buf_vp_vert_limit - 1, 16);

    MAS_vp_horz_limit = SourcePixelFormat == dm_rgbe_alpha ? 3840 : 6144;
    MAS_vp_vert_limit = SourcePixelFormat == dm_rgbe_alpha ? 3840 : (BytePerPixelY == 8 ? 3072 : 6144);
    max_vp_horz_width = dml_min(static_cast<double>(MAS_vp_horz_limit), detile_buf_vp_horz_limit);
    max_vp_vert_height = dml_min(static_cast<double>(MAS_vp_vert_limit), detile_buf_vp_vert_limit);
    eff_surf_width_l =  (SurfaceWidthLuma > max_vp_horz_width ? max_vp_horz_width : SurfaceWidthLuma);
    eff_surf_width_c =  eff_surf_width_l / (1 + yuv420);
    eff_surf_height_l =  (SurfaceHeightLuma > max_vp_vert_height ? max_vp_vert_height : SurfaceHeightLuma);
    eff_surf_height_c =  eff_surf_height_l / (1 + yuv420);

    full_swath_bytes_horz_wc_l = eff_surf_width_l * RequestHeight256ByteLuma * BytePerPixelY;
    full_swath_bytes_vert_wc_l = eff_surf_height_l * 256 / RequestHeight256ByteLuma;
    if (BytePerPixelC > 0) {
        full_swath_bytes_horz_wc_c = eff_surf_width_c * RequestHeight256ByteChroma * BytePerPixelC;
        full_swath_bytes_vert_wc_c = eff_surf_height_c * 256 / RequestHeight256ByteChroma;
    } else {
        full_swath_bytes_horz_wc_c = 0;
        full_swath_bytes_vert_wc_c = 0;
    }

    if (SourcePixelFormat == dm_420_10) {
        full_swath_bytes_horz_wc_l = dml_ceil(static_cast<double>(full_swath_bytes_horz_wc_l) * 2.0 / 3.0, 256.0);
        full_swath_bytes_horz_wc_c = dml_ceil(static_cast<double>(full_swath_bytes_horz_wc_c) * 2.0 / 3.0, 256.0);
        full_swath_bytes_vert_wc_l = dml_ceil(static_cast<double>(full_swath_bytes_vert_wc_l) * 2.0 / 3.0, 256.0);
        full_swath_bytes_vert_wc_c = dml_ceil(static_cast<double>(full_swath_bytes_vert_wc_c) * 2.0 / 3.0, 256.0);
    }

    if (2 * full_swath_bytes_horz_wc_l + 2 * full_swath_bytes_horz_wc_c <= DETBufferSizeForDCC) {
        req128_horz_wc_l = 0;
        req128_horz_wc_c = 0;
    } else if (full_swath_bytes_horz_wc_l < 1.5 * full_swath_bytes_horz_wc_c && 2 * full_swath_bytes_horz_wc_l +
            full_swath_bytes_horz_wc_c <= DETBufferSizeForDCC) {
        req128_horz_wc_l = 0;
        req128_horz_wc_c = 1;
    } else if (full_swath_bytes_horz_wc_l >= 1.5 * full_swath_bytes_horz_wc_c && full_swath_bytes_horz_wc_l + 2 *
            full_swath_bytes_horz_wc_c <= DETBufferSizeForDCC) {
        req128_horz_wc_l = 1;
        req128_horz_wc_c = 0;
    } else {
        req128_horz_wc_l = 1;
        req128_horz_wc_c = 1;
    }

    if (2 * full_swath_bytes_vert_wc_l + 2 * full_swath_bytes_vert_wc_c <= DETBufferSizeForDCC) {
        req128_vert_wc_l = 0;
        req128_vert_wc_c = 0;
    } else if (full_swath_bytes_vert_wc_l < 1.5 * full_swath_bytes_vert_wc_c && 2 *
            full_swath_bytes_vert_wc_l + full_swath_bytes_vert_wc_c <= DETBufferSizeForDCC) {
        req128_vert_wc_l = 0;
        req128_vert_wc_c = 1;
    } else if (full_swath_bytes_vert_wc_l >= 1.5 * full_swath_bytes_vert_wc_c &&
            full_swath_bytes_vert_wc_l + 2 * full_swath_bytes_vert_wc_c <= DETBufferSizeForDCC) {
        req128_vert_wc_l = 1;
        req128_vert_wc_c = 0;
    } else {
        req128_vert_wc_l = 1;
        req128_vert_wc_c = 1;
    }

    if (BytePerPixelY == 2) {
        segment_order_horz_contiguous_luma = 0;
        segment_order_vert_contiguous_luma = 1;
    } else {
        segment_order_horz_contiguous_luma = 1;
        segment_order_vert_contiguous_luma = 0;
    }

    if (BytePerPixelC == 2) {
        segment_order_horz_contiguous_chroma = 0;
        segment_order_vert_contiguous_chroma = 1;
    } else {
        segment_order_horz_contiguous_chroma = 1;
        segment_order_vert_contiguous_chroma = 0;
    }

    if (DCCProgrammingAssumesScanDirectionUnknown == true) {
        if (req128_horz_wc_l == 0 && req128_vert_wc_l == 0)
            RequestLuma = REQ_256Bytes;
        else if ((req128_horz_wc_l == 1 && segment_order_horz_contiguous_luma == 0) ||
                (req128_vert_wc_l == 1 && segment_order_vert_contiguous_luma == 0))
            RequestLuma = REQ_128BytesNonContiguous;
        else
            RequestLuma = REQ_128BytesContiguous;

        if (req128_horz_wc_c == 0 && req128_vert_wc_c == 0)
            RequestChroma = REQ_256Bytes;
        else if ((req128_horz_wc_c == 1 && segment_order_horz_contiguous_chroma == 0) ||
                (req128_vert_wc_c == 1 && segment_order_vert_contiguous_chroma == 0))
            RequestChroma = REQ_128BytesNonContiguous;
        else
            RequestChroma = REQ_128BytesContiguous;

    } else if (!IsVertical(SourceRotation)) {
        if (req128_horz_wc_l == 0)
            RequestLuma = REQ_256Bytes;
        else if (segment_order_horz_contiguous_luma == 0)
            RequestLuma = REQ_128BytesNonContiguous;
        else
            RequestLuma = REQ_128BytesContiguous;

        if (req128_horz_wc_c == 0)
            RequestChroma = REQ_256Bytes;
        else if (segment_order_horz_contiguous_chroma == 0)
            RequestChroma = REQ_128BytesNonContiguous;
        else
            RequestChroma = REQ_128BytesContiguous;

    } else {
        if (req128_vert_wc_l == 0)
            RequestLuma = REQ_256Bytes;
        else if (segment_order_vert_contiguous_luma == 0)
            RequestLuma = REQ_128BytesNonContiguous;
        else
            RequestLuma = REQ_128BytesContiguous;

        if (req128_vert_wc_c == 0)
            RequestChroma = REQ_256Bytes;
        else if (segment_order_vert_contiguous_chroma == 0)
            RequestChroma = REQ_128BytesNonContiguous;
        else
            RequestChroma = REQ_128BytesContiguous;
    }

    if (RequestLuma == REQ_256Bytes) {
        *MaxUncompressedBlockLuma = 256;
        *MaxCompressedBlockLuma = 256;
        *IndependentBlockLuma = 0;
    } else if (RequestLuma == REQ_128BytesContiguous) {
        *MaxUncompressedBlockLuma = 256;
        *MaxCompressedBlockLuma = 128;
        *IndependentBlockLuma = 128;
    } else {
        *MaxUncompressedBlockLuma = 256;
        *MaxCompressedBlockLuma = 64;
        *IndependentBlockLuma = 64;
    }

    if (RequestChroma == REQ_256Bytes) {
        *MaxUncompressedBlockChroma = 256;
        *MaxCompressedBlockChroma = 256;
        *IndependentBlockChroma = 0;
    } else if (RequestChroma == REQ_128BytesContiguous) {
        *MaxUncompressedBlockChroma = 256;
        *MaxCompressedBlockChroma = 128;
        *IndependentBlockChroma = 128;
    } else {
        *MaxUncompressedBlockChroma = 256;
        *MaxCompressedBlockChroma = 64;
        *IndependentBlockChroma = 64;
    }

    if (DCCEnabled != true || BytePerPixelC == 0) {
        *MaxUncompressedBlockChroma = 0;
        *MaxCompressedBlockChroma = 0;
        *IndependentBlockChroma = 0;
    }

    if (DCCEnabled != true) {
        *MaxUncompressedBlockLuma = 0;
        *MaxCompressedBlockLuma = 0;
        *IndependentBlockLuma = 0;
    }

} // CalculateDCCConfiguration

void dml32_CalculateStutterEfficiency(
        unsigned int      CompressedBufferSizeInkByte,
        enum dm_use_mall_for_pstate_change_mode UseMALLForPStateChange[],
        bool   UnboundedRequestEnabled,
        unsigned int      MetaFIFOSizeInKEntries,
        unsigned int      ZeroSizeBufferEntries,
        unsigned int      PixelChunkSizeInKByte,
        unsigned int   NumberOfActiveSurfaces,
        unsigned int      ROBBufferSizeInKByte,
        double    TotalDataReadBandwidth,
        double    DCFCLK,
        double    ReturnBW,
        unsigned int      CompbufReservedSpace64B,
        unsigned int      CompbufReservedSpaceZs,
        double    SRExitTime,
        double    SRExitZ8Time,
        bool   SynchronizeTimingsFinal,
        unsigned int   BlendingAndTiming[],
        double    StutterEnterPlusExitWatermark,
        double    Z8StutterEnterPlusExitWatermark,
        bool   ProgressiveToInterlaceUnitInOPP,
        bool   Interlace[],
        double    MinTTUVBlank[],
        unsigned int   DPPPerSurface[],
        unsigned int      DETBufferSizeY[],
        unsigned int   BytePerPixelY[],
        double    BytePerPixelDETY[],
        double      SwathWidthY[],
        unsigned int   SwathHeightY[],
        unsigned int   SwathHeightC[],
        double    NetDCCRateLuma[],
        double    NetDCCRateChroma[],
        double    DCCFractionOfZeroSizeRequestsLuma[],
        double    DCCFractionOfZeroSizeRequestsChroma[],
        unsigned int      HTotal[],
        unsigned int      VTotal[],
        double    PixelClock[],
        double    VRatio[],
        enum dm_rotation_angle SourceRotation[],
        unsigned int   BlockHeight256BytesY[],
        unsigned int   BlockWidth256BytesY[],
        unsigned int   BlockHeight256BytesC[],
        unsigned int   BlockWidth256BytesC[],
        unsigned int   DCCYMaxUncompressedBlock[],
        unsigned int   DCCCMaxUncompressedBlock[],
        unsigned int      VActive[],
        bool   DCCEnable[],
        bool   WritebackEnable[],
        double    ReadBandwidthSurfaceLuma[],
        double    ReadBandwidthSurfaceChroma[],
        double    meta_row_bw[],
        double    dpte_row_bw[],

        /* Output */
        double   *StutterEfficiencyNotIncludingVBlank,
        double   *StutterEfficiency,
        unsigned int     *NumberOfStutterBurstsPerFrame,
        double   *Z8StutterEfficiencyNotIncludingVBlank,
        double   *Z8StutterEfficiency,
        unsigned int     *Z8NumberOfStutterBurstsPerFrame,
        double   *StutterPeriod,
        bool  *DCHUBBUB_ARB_CSTATE_MAX_CAP_MODE)
{

    bool FoundCriticalSurface = false;
    unsigned int SwathSizeCriticalSurface = 0;
    unsigned int LastChunkOfSwathSize;
    unsigned int MissingPartOfLastSwathOfDETSize;
    double LastZ8StutterPeriod = 0.0;
    double LastStutterPeriod = 0.0;
    unsigned int TotalNumberOfActiveOTG = 0;
    double doublePixelClock = 0;
    unsigned int doubleHTotal = 0;
    unsigned int doubleVTotal = 0;
    bool SameTiming = true;
    double DETBufferingTimeY;
    double SwathWidthYCriticalSurface = 0.0;
    double SwathHeightYCriticalSurface = 0.0;
    double VActiveTimeCriticalSurface = 0.0;
    double FrameTimeCriticalSurface = 0.0;
    unsigned int BytePerPixelYCriticalSurface = 0;
    double LinesToFinishSwathTransferStutterCriticalSurface = 0.0;
    unsigned int DETBufferSizeYCriticalSurface = 0;
    double MinTTUVBlankCriticalSurface = 0.0;
    unsigned int BlockWidth256BytesYCriticalSurface = 0;
    bool doublePlaneCriticalSurface = 0;
    bool doublePipeCriticalSurface = 0;
    double TotalCompressedReadBandwidth;
    double TotalRowReadBandwidth;
    double AverageDCCCompressionRate;
    double EffectiveCompressedBufferSize;
    double PartOfUncompressedPixelBurstThatFitsInROBAndCompressedBuffer;
    double StutterBurstTime;
    unsigned int TotalActiveWriteback;
    double LinesInDETY;
    double LinesInDETYRoundedDownToSwath;
    double MaximumEffectiveCompressionLuma;
    double MaximumEffectiveCompressionChroma;
    double TotalZeroSizeRequestReadBandwidth;
    double TotalZeroSizeCompressedReadBandwidth;
    double AverageDCCZeroSizeFraction;
    double AverageZeroSizeCompressionRate;
    unsigned int k;

    TotalZeroSizeRequestReadBandwidth = 0;
    TotalZeroSizeCompressedReadBandwidth = 0;
    TotalRowReadBandwidth = 0;
    TotalCompressedReadBandwidth = 0;

    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        if (UseMALLForPStateChange[k] != dm_use_mall_pstate_change_phantom_pipe) {
            if (DCCEnable[k] == true) {
                if ((IsVertical(SourceRotation[k]) && BlockWidth256BytesY[k] > SwathHeightY[k])
                        || (!IsVertical(SourceRotation[k])
                                && BlockHeight256BytesY[k] > SwathHeightY[k])
                        || DCCYMaxUncompressedBlock[k] < 256) {
                    MaximumEffectiveCompressionLuma = 2;
                } else {
                    MaximumEffectiveCompressionLuma = 4;
                }
                TotalCompressedReadBandwidth = TotalCompressedReadBandwidth
                        + ReadBandwidthSurfaceLuma[k]
                                / dml_min(NetDCCRateLuma[k],
                                        MaximumEffectiveCompressionLuma);
                TotalZeroSizeRequestReadBandwidth = TotalZeroSizeRequestReadBandwidth
                        + ReadBandwidthSurfaceLuma[k] * DCCFractionOfZeroSizeRequestsLuma[k];
                TotalZeroSizeCompressedReadBandwidth = TotalZeroSizeCompressedReadBandwidth
                        + ReadBandwidthSurfaceLuma[k] * DCCFractionOfZeroSizeRequestsLuma[k]
                                / MaximumEffectiveCompressionLuma;

                if (ReadBandwidthSurfaceChroma[k] > 0) {
                    if ((IsVertical(SourceRotation[k]) && BlockWidth256BytesC[k] > SwathHeightC[k])
                            || (!IsVertical(SourceRotation[k])
                                    && BlockHeight256BytesC[k] > SwathHeightC[k])
                            || DCCCMaxUncompressedBlock[k] < 256) {
                        MaximumEffectiveCompressionChroma = 2;
                    } else {
                        MaximumEffectiveCompressionChroma = 4;
                    }
                    TotalCompressedReadBandwidth =
                            TotalCompressedReadBandwidth
                            + ReadBandwidthSurfaceChroma[k]
                            / dml_min(NetDCCRateChroma[k],
                            MaximumEffectiveCompressionChroma);
                    TotalZeroSizeRequestReadBandwidth = TotalZeroSizeRequestReadBandwidth
                            + ReadBandwidthSurfaceChroma[k]
                                    * DCCFractionOfZeroSizeRequestsChroma[k];
                    TotalZeroSizeCompressedReadBandwidth = TotalZeroSizeCompressedReadBandwidth
                            + ReadBandwidthSurfaceChroma[k]
                                    * DCCFractionOfZeroSizeRequestsChroma[k]
                                    / MaximumEffectiveCompressionChroma;
                }
            } else {
                TotalCompressedReadBandwidth = TotalCompressedReadBandwidth
                        + ReadBandwidthSurfaceLuma[k] + ReadBandwidthSurfaceChroma[k];
            }
            TotalRowReadBandwidth = TotalRowReadBandwidth
                    + DPPPerSurface[k] * (meta_row_bw[k] + dpte_row_bw[k]);
        }
    }

    AverageDCCCompressionRate = TotalDataReadBandwidth / TotalCompressedReadBandwidth;
    AverageDCCZeroSizeFraction = TotalZeroSizeRequestReadBandwidth / TotalDataReadBandwidth;

    if (AverageDCCZeroSizeFraction == 1) {
        AverageZeroSizeCompressionRate = TotalZeroSizeRequestReadBandwidth
                / TotalZeroSizeCompressedReadBandwidth;
        EffectiveCompressedBufferSize = static_cast<double>(MetaFIFOSizeInKEntries) * 1024 * 64
                * AverageZeroSizeCompressionRate
                + (static_cast<double>(ZeroSizeBufferEntries) - CompbufReservedSpaceZs) * 64
                        * AverageZeroSizeCompressionRate;
    } else if (AverageDCCZeroSizeFraction > 0) {
        AverageZeroSizeCompressionRate = TotalZeroSizeRequestReadBandwidth
                / TotalZeroSizeCompressedReadBandwidth;
        EffectiveCompressedBufferSize = dml_min(
                static_cast<double>(CompressedBufferSizeInkByte) * 1024 * AverageDCCCompressionRate,
                static_cast<double>(MetaFIFOSizeInKEntries) * 1024 * 64
                    / (AverageDCCZeroSizeFraction / AverageZeroSizeCompressionRate
                    + 1 / AverageDCCCompressionRate))
                    + dml_min((static_cast<double>(ROBBufferSizeInKByte) * 1024 - CompbufReservedSpace64B * 64)
                    * AverageDCCCompressionRate,
                    (static_cast<double>(ZeroSizeBufferEntries) - CompbufReservedSpaceZs) * 64
                    / (AverageDCCZeroSizeFraction / AverageZeroSizeCompressionRate));

    } else {
        EffectiveCompressedBufferSize = dml_min(
                static_cast<double>(CompressedBufferSizeInkByte) * 1024 * AverageDCCCompressionRate,
                static_cast<double>(MetaFIFOSizeInKEntries) * 1024 * 64 * AverageDCCCompressionRate)
                + (static_cast<double>(ROBBufferSizeInKByte) * 1024 - CompbufReservedSpace64B * 64)
                        * AverageDCCCompressionRate;

    }

    *StutterPeriod = 0;

    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        if (UseMALLForPStateChange[k] != dm_use_mall_pstate_change_phantom_pipe) {
            LinesInDETY = (static_cast<double>(DETBufferSizeY[k])
                    + (UnboundedRequestEnabled == true ? EffectiveCompressedBufferSize : 0)
                            * ReadBandwidthSurfaceLuma[k] / TotalDataReadBandwidth)
                    / BytePerPixelDETY[k] / SwathWidthY[k];
            LinesInDETYRoundedDownToSwath = dml_floor(LinesInDETY, SwathHeightY[k]);
            DETBufferingTimeY = LinesInDETYRoundedDownToSwath * (static_cast<double>(HTotal[k]) / PixelClock[k])
                    / VRatio[k];

            if (!FoundCriticalSurface || DETBufferingTimeY < *StutterPeriod) {
                bool isInterlaceTiming = Interlace[k] && !ProgressiveToInterlaceUnitInOPP;

                FoundCriticalSurface = true;
                *StutterPeriod = DETBufferingTimeY;
                FrameTimeCriticalSurface = (
                        isInterlaceTiming ?
                                dml_floor(static_cast<double>(VTotal[k]) / 2.0, 1.0) : VTotal[k])
                        * static_cast<double>(HTotal[k]) / PixelClock[k];
                VActiveTimeCriticalSurface = (
                        isInterlaceTiming ?
                                dml_floor(static_cast<double>(VActive[k]) / 2.0, 1.0) : VActive[k])
                        * static_cast<double>(HTotal[k]) / PixelClock[k];
                BytePerPixelYCriticalSurface = BytePerPixelY[k];
                SwathWidthYCriticalSurface = SwathWidthY[k];
                SwathHeightYCriticalSurface = SwathHeightY[k];
                BlockWidth256BytesYCriticalSurface = BlockWidth256BytesY[k];
                LinesToFinishSwathTransferStutterCriticalSurface = SwathHeightY[k]
                        - (LinesInDETY - LinesInDETYRoundedDownToSwath);
                DETBufferSizeYCriticalSurface = DETBufferSizeY[k];
                MinTTUVBlankCriticalSurface = MinTTUVBlank[k];
                doublePlaneCriticalSurface = (ReadBandwidthSurfaceChroma[k] == 0);
                doublePipeCriticalSurface = (DPPPerSurface[k] == 1);

            }
        }
    }

    PartOfUncompressedPixelBurstThatFitsInROBAndCompressedBuffer = dml_min(*StutterPeriod * TotalDataReadBandwidth,
            EffectiveCompressedBufferSize);

    StutterBurstTime = PartOfUncompressedPixelBurstThatFitsInROBAndCompressedBuffer / AverageDCCCompressionRate
            / ReturnBW
            + (*StutterPeriod * TotalDataReadBandwidth
                    - PartOfUncompressedPixelBurstThatFitsInROBAndCompressedBuffer) / (DCFCLK * 64)
            + *StutterPeriod * TotalRowReadBandwidth / ReturnBW;
    StutterBurstTime = dml_max(StutterBurstTime,
            LinesToFinishSwathTransferStutterCriticalSurface * BytePerPixelYCriticalSurface
                    * SwathWidthYCriticalSurface / ReturnBW);

    TotalActiveWriteback = 0;
    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        if (WritebackEnable[k])
            TotalActiveWriteback = TotalActiveWriteback + 1;
    }

    if (TotalActiveWriteback == 0) {
        *StutterEfficiencyNotIncludingVBlank = dml_max(0.,
                1 - (SRExitTime + StutterBurstTime) / *StutterPeriod) * 100;
        *Z8StutterEfficiencyNotIncludingVBlank = dml_max(0.,
                1 - (SRExitZ8Time + StutterBurstTime) / *StutterPeriod) * 100;
        *NumberOfStutterBurstsPerFrame = (
                *StutterEfficiencyNotIncludingVBlank > 0 ?
                        dml_ceil(VActiveTimeCriticalSurface / *StutterPeriod, 1) : 0);
        *Z8NumberOfStutterBurstsPerFrame = (
                *Z8StutterEfficiencyNotIncludingVBlank > 0 ?
                        dml_ceil(VActiveTimeCriticalSurface / *StutterPeriod, 1) : 0);
    } else {
        *StutterEfficiencyNotIncludingVBlank = 0.;
        *Z8StutterEfficiencyNotIncludingVBlank = 0.;
        *NumberOfStutterBurstsPerFrame = 0;
        *Z8NumberOfStutterBurstsPerFrame = 0;
    }

    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        if (UseMALLForPStateChange[k] != dm_use_mall_pstate_change_phantom_pipe) {
            if (BlendingAndTiming[k] == k) {
                if (TotalNumberOfActiveOTG == 0) {
                    doublePixelClock = PixelClock[k];
                    doubleHTotal = HTotal[k];
                    doubleVTotal = VTotal[k];
                } else if (doublePixelClock != PixelClock[k] || doubleHTotal != HTotal[k]
                        || doubleVTotal != VTotal[k]) {
                    SameTiming = false;
                }
                TotalNumberOfActiveOTG = TotalNumberOfActiveOTG + 1;
            }
        }
    }

    if (*StutterEfficiencyNotIncludingVBlank > 0) {
        LastStutterPeriod = VActiveTimeCriticalSurface - (*NumberOfStutterBurstsPerFrame - 1) * *StutterPeriod;

        if ((SynchronizeTimingsFinal || TotalNumberOfActiveOTG == 1) && SameTiming
                && LastStutterPeriod + MinTTUVBlankCriticalSurface > StutterEnterPlusExitWatermark) {
            *StutterEfficiency = (1 - (*NumberOfStutterBurstsPerFrame * SRExitTime
                        + StutterBurstTime * VActiveTimeCriticalSurface
                        / *StutterPeriod) / FrameTimeCriticalSurface) * 100;
        } else {
            *StutterEfficiency = *StutterEfficiencyNotIncludingVBlank;
        }
    } else {
        *StutterEfficiency = 0;
    }

    if (*Z8StutterEfficiencyNotIncludingVBlank > 0) {
        LastZ8StutterPeriod = VActiveTimeCriticalSurface
                - (*NumberOfStutterBurstsPerFrame - 1) * *StutterPeriod;
        if ((SynchronizeTimingsFinal || TotalNumberOfActiveOTG == 1) && SameTiming && LastZ8StutterPeriod +
                MinTTUVBlankCriticalSurface > Z8StutterEnterPlusExitWatermark) {
            *Z8StutterEfficiency = (1 - (*NumberOfStutterBurstsPerFrame * SRExitZ8Time + StutterBurstTime
                * VActiveTimeCriticalSurface / *StutterPeriod) / FrameTimeCriticalSurface) * 100;
        } else {
            *Z8StutterEfficiency = *Z8StutterEfficiencyNotIncludingVBlank;
        }
    } else {
        *Z8StutterEfficiency = 0.;
    }

    SwathSizeCriticalSurface = BytePerPixelYCriticalSurface * SwathHeightYCriticalSurface
            * dml_ceil(SwathWidthYCriticalSurface, BlockWidth256BytesYCriticalSurface);
    LastChunkOfSwathSize = SwathSizeCriticalSurface % (PixelChunkSizeInKByte * 1024);
    MissingPartOfLastSwathOfDETSize = dml_ceil(DETBufferSizeYCriticalSurface, SwathSizeCriticalSurface)
            - DETBufferSizeYCriticalSurface;

    *DCHUBBUB_ARB_CSTATE_MAX_CAP_MODE = !(!UnboundedRequestEnabled && (NumberOfActiveSurfaces == 1)
            && doublePlaneCriticalSurface && doublePipeCriticalSurface && (LastChunkOfSwathSize > 0)
            && (LastChunkOfSwathSize <= 4096) && (MissingPartOfLastSwathOfDETSize > 0)
            && (MissingPartOfLastSwathOfDETSize <= LastChunkOfSwathSize));

} // CalculateStutterEfficiency

void dml32_CalculateMaxDETAndMinCompressedBufferSize(
        unsigned int    ConfigReturnBufferSizeInKByte,
        unsigned int    ROBBufferSizeInKByte,
        unsigned int MaxNumDPP,
        bool nomDETInKByteOverrideEnable, // VBA_DELTA, allow DV to override default DET size
        unsigned int nomDETInKByteOverrideValue,  // VBA_DELTA

        /* Output */
        unsigned int *MaxTotalDETInKByte,
        unsigned int *nomDETInKByte,
        unsigned int *MinCompressedBufferSizeInKByte)
{
    bool     det_buff_size_override_en  = nomDETInKByteOverrideEnable;
    unsigned int        det_buff_size_override_val = nomDETInKByteOverrideValue;

    *MaxTotalDETInKByte = dml_ceil((static_cast<double>(ConfigReturnBufferSizeInKByte) +
            static_cast<double>(ROBBufferSizeInKByte)) * 4.0 / 5.0, 64);
    *nomDETInKByte = dml_floor(static_cast<double>(*MaxTotalDETInKByte) / static_cast<double>(MaxNumDPP), 64);
    *MinCompressedBufferSizeInKByte = ConfigReturnBufferSizeInKByte - *MaxTotalDETInKByte;

    if (det_buff_size_override_en) {
        *nomDETInKByte = det_buff_size_override_val;
    }
} // CalculateMaxDETAndMinCompressedBufferSize

bool dml32_CalculateVActiveBandwithSupport(unsigned int NumberOfActiveSurfaces,
        double ReturnBW,
        bool NotUrgentLatencyHiding[],
        double ReadBandwidthLuma[],
        double ReadBandwidthChroma[],
        double cursor_bw[],
        double meta_row_bandwidth[],
        double dpte_row_bandwidth[],
        unsigned int NumberOfDPP[],
        double UrgentBurstFactorLuma[],
        double UrgentBurstFactorChroma[],
        double UrgentBurstFactorCursor[])
{
    unsigned int k;
    bool NotEnoughUrgentLatencyHiding = false;
    bool CalculateVActiveBandwithSupport_val = false;
    double VActiveBandwith = 0;

    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        if (NotUrgentLatencyHiding[k]) {
            NotEnoughUrgentLatencyHiding = true;
        }
    }

    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        VActiveBandwith = VActiveBandwith + ReadBandwidthLuma[k] * UrgentBurstFactorLuma[k] + ReadBandwidthChroma[k] * UrgentBurstFactorChroma[k] + cursor_bw[k] * UrgentBurstFactorCursor[k] + NumberOfDPP[k] * meta_row_bandwidth[k] + NumberOfDPP[k] * dpte_row_bandwidth[k];
    }

    CalculateVActiveBandwithSupport_val = (VActiveBandwith <= ReturnBW) && !NotEnoughUrgentLatencyHiding;

    return CalculateVActiveBandwithSupport_val;
}

void dml32_CalculatePrefetchBandwithSupport(unsigned int NumberOfActiveSurfaces,
        double ReturnBW,
        bool NotUrgentLatencyHiding[],
        double ReadBandwidthLuma[],
        double ReadBandwidthChroma[],
        double PrefetchBandwidthLuma[],
        double PrefetchBandwidthChroma[],
        double cursor_bw[],
        double meta_row_bandwidth[],
        double dpte_row_bandwidth[],
        double cursor_bw_pre[],
        double prefetch_vmrow_bw[],
        unsigned int NumberOfDPP[],
        double UrgentBurstFactorLuma[],
        double UrgentBurstFactorChroma[],
        double UrgentBurstFactorCursor[],
        double UrgentBurstFactorLumaPre[],
        double UrgentBurstFactorChromaPre[],
        double UrgentBurstFactorCursorPre[],
        double PrefetchBW[],
        double VRatio[],
        double MaxVRatioPre,

        /* output */
        double  *MaxPrefetchBandwidth,
        double  *FractionOfUrgentBandwidth,
        bool *PrefetchBandwidthSupport)
{
    unsigned int k;
    double ActiveBandwidthPerSurface;
    bool NotEnoughUrgentLatencyHiding = false;
    double TotalActiveBandwidth = 0;
    double TotalPrefetchBandwidth = 0;

    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        if (NotUrgentLatencyHiding[k]) {
            NotEnoughUrgentLatencyHiding = true;
        }
    }

    *MaxPrefetchBandwidth = 0;
    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        ActiveBandwidthPerSurface = ReadBandwidthLuma[k] * UrgentBurstFactorLuma[k] + ReadBandwidthChroma[k] * UrgentBurstFactorChroma[k] + cursor_bw[k] * UrgentBurstFactorCursor[k] + NumberOfDPP[k] * (meta_row_bandwidth[k] + dpte_row_bandwidth[k]);

        TotalActiveBandwidth += ActiveBandwidthPerSurface;

        TotalPrefetchBandwidth = TotalPrefetchBandwidth + PrefetchBW[k] * VRatio[k];

        *MaxPrefetchBandwidth = *MaxPrefetchBandwidth + dml_max3(NumberOfDPP[k] * prefetch_vmrow_bw[k],
                ActiveBandwidthPerSurface,
                NumberOfDPP[k] * (PrefetchBandwidthLuma[k] * UrgentBurstFactorLumaPre[k] + PrefetchBandwidthChroma[k] * UrgentBurstFactorChromaPre[k]) + cursor_bw_pre[k] * UrgentBurstFactorCursorPre[k]);
    }

    if (MaxVRatioPre == __DML_MAX_VRATIO_PRE__)
        *PrefetchBandwidthSupport = (*MaxPrefetchBandwidth <= ReturnBW) && (TotalPrefetchBandwidth <= TotalActiveBandwidth * __DML_MAX_BW_RATIO_PRE__) && !NotEnoughUrgentLatencyHiding;
    else
        *PrefetchBandwidthSupport = (*MaxPrefetchBandwidth <= ReturnBW) && !NotEnoughUrgentLatencyHiding;

    *FractionOfUrgentBandwidth = *MaxPrefetchBandwidth / ReturnBW;
}

double dml32_CalculateBandwidthAvailableForImmediateFlip(unsigned int NumberOfActiveSurfaces,
        double ReturnBW,
        double ReadBandwidthLuma[],
        double ReadBandwidthChroma[],
        double PrefetchBandwidthLuma[],
        double PrefetchBandwidthChroma[],
        double cursor_bw[],
        double cursor_bw_pre[],
        unsigned int NumberOfDPP[],
        double UrgentBurstFactorLuma[],
        double UrgentBurstFactorChroma[],
        double UrgentBurstFactorCursor[],
        double UrgentBurstFactorLumaPre[],
        double UrgentBurstFactorChromaPre[],
        double UrgentBurstFactorCursorPre[])
{
    unsigned int k;
    double CalculateBandwidthAvailableForImmediateFlip_val = ReturnBW;

    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        CalculateBandwidthAvailableForImmediateFlip_val = CalculateBandwidthAvailableForImmediateFlip_val - dml_max(ReadBandwidthLuma[k] * UrgentBurstFactorLuma[k] + ReadBandwidthChroma[k] * UrgentBurstFactorChroma[k] + cursor_bw[k] * UrgentBurstFactorCursor[k],
                NumberOfDPP[k] * (PrefetchBandwidthLuma[k] * UrgentBurstFactorLumaPre[k] + PrefetchBandwidthChroma[k] * UrgentBurstFactorChromaPre[k]) + cursor_bw_pre[k] * UrgentBurstFactorCursorPre[k]);
    }

    return CalculateBandwidthAvailableForImmediateFlip_val;
}

void dml32_CalculateImmediateFlipBandwithSupport(unsigned int NumberOfActiveSurfaces,
        double ReturnBW,
        enum immediate_flip_requirement ImmediateFlipRequirement[],
        double final_flip_bw[],
        double ReadBandwidthLuma[],
        double ReadBandwidthChroma[],
        double PrefetchBandwidthLuma[],
        double PrefetchBandwidthChroma[],
        double cursor_bw[],
        double meta_row_bandwidth[],
        double dpte_row_bandwidth[],
        double cursor_bw_pre[],
        double prefetch_vmrow_bw[],
        unsigned int NumberOfDPP[],
        double UrgentBurstFactorLuma[],
        double UrgentBurstFactorChroma[],
        double UrgentBurstFactorCursor[],
        double UrgentBurstFactorLumaPre[],
        double UrgentBurstFactorChromaPre[],
        double UrgentBurstFactorCursorPre[],

        /* output */
        double  *TotalBandwidth,
        double  *FractionOfUrgentBandwidth,
        bool *ImmediateFlipBandwidthSupport)
{
    unsigned int k;
    *TotalBandwidth = 0;
    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        if (ImmediateFlipRequirement[k] != dm_immediate_flip_not_required) {
            *TotalBandwidth = *TotalBandwidth + dml_max3(NumberOfDPP[k] * prefetch_vmrow_bw[k],
                    NumberOfDPP[k] * final_flip_bw[k] + ReadBandwidthLuma[k] * UrgentBurstFactorLuma[k] + ReadBandwidthChroma[k] * UrgentBurstFactorChroma[k] + cursor_bw[k] * UrgentBurstFactorCursor[k],
                    NumberOfDPP[k] * (final_flip_bw[k] + PrefetchBandwidthLuma[k] * UrgentBurstFactorLumaPre[k] + PrefetchBandwidthChroma[k] * UrgentBurstFactorChromaPre[k]) + cursor_bw_pre[k] * UrgentBurstFactorCursorPre[k]);
        } else {
            *TotalBandwidth = *TotalBandwidth + dml_max3(NumberOfDPP[k] * prefetch_vmrow_bw[k],
                    NumberOfDPP[k] * (meta_row_bandwidth[k] + dpte_row_bandwidth[k]) + ReadBandwidthLuma[k] * UrgentBurstFactorLuma[k] + ReadBandwidthChroma[k] * UrgentBurstFactorChroma[k] + cursor_bw[k] * UrgentBurstFactorCursor[k],
                    NumberOfDPP[k] * (PrefetchBandwidthLuma[k] * UrgentBurstFactorLumaPre[k] + PrefetchBandwidthChroma[k] * UrgentBurstFactorChromaPre[k]) + cursor_bw_pre[k] * UrgentBurstFactorCursorPre[k]);
        }
    }
    *ImmediateFlipBandwidthSupport = (*TotalBandwidth <= ReturnBW);
    *FractionOfUrgentBandwidth = *TotalBandwidth / ReturnBW;
}

bool dml32_CalculateDETSwathFillLatencyHiding(unsigned int NumberOfActiveSurfaces,
        double ReturnBW,
        double UrgentLatency,
        unsigned int SwathHeightY[],
        unsigned int SwathHeightC[],
        unsigned int SwathWidthY[],
        unsigned int SwathWidthC[],
        double  BytePerPixelInDETY[],
        double  BytePerPixelInDETC[],
        unsigned int    DETBufferSizeY[],
        unsigned int    DETBufferSizeC[],
        unsigned int    NumOfDPP[],
        unsigned int    HTotal[],
        double  PixelClock[],
        double  VRatioY[],
        double  VRatioC[],
        enum dm_use_mall_for_pstate_change_mode UsesMALLForPStateChange[],
        enum unbounded_requesting_policy UseUnboundedRequesting)
{
    int k;
    double SwathSizeAllSurfaces = 0;
    double SwathSizeAllSurfacesInFetchTimeUs;
    double DETSwathLatencyHidingUs;
    double DETSwathLatencyHidingYUs;
    double DETSwathLatencyHidingCUs;
    double SwathSizePerSurfaceY[DML32_NUM_PLANES];
    double SwathSizePerSurfaceC[DML32_NUM_PLANES];
    bool NotEnoughDETSwathFillLatencyHiding = false;

    if (UseUnboundedRequesting == dm_unbounded_requesting)
        return false;

    /* calculate sum of single swath size for all pipes in bytes */
    for (k = 0; static_cast<unsigned int>(k) < NumberOfActiveSurfaces; k++) {
        SwathSizePerSurfaceY[k] = SwathHeightY[k] * SwathWidthY[k] * BytePerPixelInDETY[k] * NumOfDPP[k];

        if (SwathHeightC[k] != 0)
            SwathSizePerSurfaceC[k] = SwathHeightC[k] * SwathWidthC[k] * BytePerPixelInDETC[k] * NumOfDPP[k];
        else
            SwathSizePerSurfaceC[k] = 0;

        SwathSizeAllSurfaces += SwathSizePerSurfaceY[k] + SwathSizePerSurfaceC[k];
    }

    SwathSizeAllSurfacesInFetchTimeUs = SwathSizeAllSurfaces / ReturnBW + UrgentLatency;

    /* ensure all DET - 1 swath can hide a fetch for all surfaces */
    for (k = 0; static_cast<unsigned int>(k) < NumberOfActiveSurfaces; k++) {
        double LineTime = HTotal[k] / PixelClock[k];

        /* only care if surface is not phantom */
        if (UsesMALLForPStateChange[k] != dm_use_mall_pstate_change_phantom_pipe) {
            DETSwathLatencyHidingYUs = (dml_floor(DETBufferSizeY[k] / BytePerPixelInDETY[k] / SwathWidthY[k], 1.0) - SwathHeightY[k]) / VRatioY[k] * LineTime;

            if (SwathHeightC[k] != 0) {
                DETSwathLatencyHidingCUs = (dml_floor(DETBufferSizeC[k] / BytePerPixelInDETC[k] / SwathWidthC[k], 1.0) - SwathHeightC[k]) / VRatioC[k] * LineTime;

                DETSwathLatencyHidingUs = dml_min(DETSwathLatencyHidingYUs, DETSwathLatencyHidingCUs);
            } else {
                DETSwathLatencyHidingUs = DETSwathLatencyHidingYUs;
            }

            /* DET must be able to hide time to fetch 1 swath for each surface */
            if (DETSwathLatencyHidingUs < SwathSizeAllSurfacesInFetchTimeUs) {
                NotEnoughDETSwathFillLatencyHiding = true;
                break;
            }
        }
    }

    return NotEnoughDETSwathFillLatencyHiding;
}

} // namespace rx7900::display::dml32

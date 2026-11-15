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
Module: rx7900.display.dcn32_dml_latency
Purpose: DML32 PTE/meta row, bandwidth, latency and DCFCLK deep-sleep utilities.
Created: 2026-10-24 (adaptation; the upstream notice above is preserved)
Architecture: Part of the bounded C++17 port of the Linux v6.12 DML32 model behind
src/display/dcn32_bandwidth (one stream, up to four pipes). Identifiers, statement order and
arithmetic (including the float-typed dml_min/max/floor/ceil helpers and every
int/unsigned conversion) deliberately follow the pinned upstream text so each
function can be compared line by line with work/upstream-dcn-stream; this is a
documented exception to the snake_case rule, as in src/opengl/gfx11_registers.hpp.
Per-plane and per-pipe arrays hold the four DCN 3.2 pipes (DML32_NUM_PLANES; Linux
sizes them DC__NUM_DPP__MAX = 8) and per-state arrays DML32_NUM_STATES entries; debug printing is removed.
Upstream: dml/dcn32/display_mode_vba_util_32.c (second part).
*/
#include "dcn32_dml_util.hpp"

namespace rx7900::display::dml32 {
void dml32_CalculateVMRowAndSwath(
        unsigned int NumberOfActiveSurfaces,
        DmlPipe myPipe[],
        unsigned int SurfaceSizeInMALL[],
        unsigned int PTEBufferSizeInRequestsLuma,
        unsigned int PTEBufferSizeInRequestsChroma,
        unsigned int DCCMetaBufferSizeBytes,
        enum dm_use_mall_for_static_screen_mode UseMALLForStaticScreen[],
        enum dm_use_mall_for_pstate_change_mode UseMALLForPStateChange[],
        unsigned int MALLAllocatedForDCN,
        double SwathWidthY[],
        double SwathWidthC[],
        bool GPUVMEnable,
        bool HostVMEnable,
        unsigned int HostVMMaxNonCachedPageTableLevels,
        unsigned int GPUVMMaxPageTableLevels,
        unsigned int GPUVMMinPageSizeKBytes[],
        unsigned int HostVMMinPageSize,

        /* Output */
        bool PTEBufferSizeNotExceeded[],
        bool DCCMetaBufferSizeNotExceeded[],
        unsigned int dpte_row_width_luma_ub[],
        unsigned int dpte_row_width_chroma_ub[],
        unsigned int dpte_row_height_luma[],
        unsigned int dpte_row_height_chroma[],
        unsigned int dpte_row_height_linear_luma[],     // VBA_DELTA
        unsigned int dpte_row_height_linear_chroma[],   // VBA_DELTA
        unsigned int meta_req_width[],
        unsigned int meta_req_width_chroma[],
        unsigned int meta_req_height[],
        unsigned int meta_req_height_chroma[],
        unsigned int meta_row_width[],
        unsigned int meta_row_width_chroma[],
        unsigned int meta_row_height[],
        unsigned int meta_row_height_chroma[],
        unsigned int vm_group_bytes[],
        unsigned int dpte_group_bytes[],
        unsigned int PixelPTEReqWidthY[],
        unsigned int PixelPTEReqHeightY[],
        unsigned int PTERequestSizeY[],
        unsigned int PixelPTEReqWidthC[],
        unsigned int PixelPTEReqHeightC[],
        unsigned int PTERequestSizeC[],
        unsigned int dpde0_bytes_per_frame_ub_l[],
        unsigned int meta_pte_bytes_per_frame_ub_l[],
        unsigned int dpde0_bytes_per_frame_ub_c[],
        unsigned int meta_pte_bytes_per_frame_ub_c[],
        double PrefetchSourceLinesY[],
        double PrefetchSourceLinesC[],
        double VInitPreFillY[],
        double VInitPreFillC[],
        unsigned int MaxNumSwathY[],
        unsigned int MaxNumSwathC[],
        double meta_row_bw[],
        double dpte_row_bw[],
        double PixelPTEBytesPerRow[],
        double PDEAndMetaPTEBytesFrame[],
        double MetaRowByte[],
        bool use_one_row_for_frame[],
        bool use_one_row_for_frame_flip[],
        bool UsesMALLForStaticScreen[],
        bool PTE_BUFFER_MODE[],
        unsigned int BIGK_FRAGMENT_SIZE[])
{
    unsigned int k;
    unsigned int PTEBufferSizeInRequestsForLuma[DML32_NUM_PLANES];
    unsigned int PTEBufferSizeInRequestsForChroma[DML32_NUM_PLANES];
    unsigned int PDEAndMetaPTEBytesFrameY;
    unsigned int PDEAndMetaPTEBytesFrameC;
    unsigned int MetaRowByteY[DML32_NUM_PLANES] = {0};
    unsigned int MetaRowByteC[DML32_NUM_PLANES] = {0};
    unsigned int PixelPTEBytesPerRowY[DML32_NUM_PLANES];
    unsigned int PixelPTEBytesPerRowC[DML32_NUM_PLANES];
    unsigned int PixelPTEBytesPerRowY_one_row_per_frame[DML32_NUM_PLANES];
    unsigned int PixelPTEBytesPerRowC_one_row_per_frame[DML32_NUM_PLANES];
    unsigned int dpte_row_width_luma_ub_one_row_per_frame[DML32_NUM_PLANES];
    unsigned int dpte_row_height_luma_one_row_per_frame[DML32_NUM_PLANES];
    unsigned int dpte_row_width_chroma_ub_one_row_per_frame[DML32_NUM_PLANES];
    unsigned int dpte_row_height_chroma_one_row_per_frame[DML32_NUM_PLANES];
    bool one_row_per_frame_fits_in_buffer[DML32_NUM_PLANES];

    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        if (HostVMEnable == true) {
            vm_group_bytes[k] = 512;
            dpte_group_bytes[k] = 512;
        } else if (GPUVMEnable == true) {
            vm_group_bytes[k] = 2048;
            if (GPUVMMinPageSizeKBytes[k] >= 64 && IsVertical(myPipe[k].SourceRotation))
                dpte_group_bytes[k] = 512;
            else
                dpte_group_bytes[k] = 2048;
        } else {
            vm_group_bytes[k] = 0;
            dpte_group_bytes[k] = 0;
        }

        if (myPipe[k].SourcePixelFormat == dm_420_8 || myPipe[k].SourcePixelFormat == dm_420_10 ||
                myPipe[k].SourcePixelFormat == dm_420_12 ||
                myPipe[k].SourcePixelFormat == dm_rgbe_alpha) {
            if ((myPipe[k].SourcePixelFormat == dm_420_10 || myPipe[k].SourcePixelFormat == dm_420_12) &&
                    !IsVertical(myPipe[k].SourceRotation)) {
                PTEBufferSizeInRequestsForLuma[k] =
                        (PTEBufferSizeInRequestsLuma + PTEBufferSizeInRequestsChroma) / 2;
                PTEBufferSizeInRequestsForChroma[k] = PTEBufferSizeInRequestsForLuma[k];
            } else {
                PTEBufferSizeInRequestsForLuma[k] = PTEBufferSizeInRequestsLuma;
                PTEBufferSizeInRequestsForChroma[k] = PTEBufferSizeInRequestsChroma;
            }

            PDEAndMetaPTEBytesFrameC = dml32_CalculateVMAndRowBytes(
                    myPipe[k].ViewportStationary,
                    myPipe[k].DCCEnable,
                    myPipe[k].DPPPerSurface,
                    myPipe[k].BlockHeight256BytesC,
                    myPipe[k].BlockWidth256BytesC,
                    myPipe[k].SourcePixelFormat,
                    myPipe[k].SurfaceTiling,
                    myPipe[k].BytePerPixelC,
                    myPipe[k].SourceRotation,
                    SwathWidthC[k],
                    myPipe[k].ViewportHeightChroma,
                    myPipe[k].ViewportXStartC,
                    myPipe[k].ViewportYStartC,
                    GPUVMEnable,
                    HostVMEnable,
                    HostVMMaxNonCachedPageTableLevels,
                    GPUVMMaxPageTableLevels,
                    GPUVMMinPageSizeKBytes[k],
                    HostVMMinPageSize,
                    PTEBufferSizeInRequestsForChroma[k],
                    myPipe[k].PitchC,
                    myPipe[k].DCCMetaPitchC,
                    myPipe[k].BlockWidthC,
                    myPipe[k].BlockHeightC,

                    /* Output */
                    &MetaRowByteC[k],
                    &PixelPTEBytesPerRowC[k],
                    &dpte_row_width_chroma_ub[k],
                    &dpte_row_height_chroma[k],
                    &dpte_row_height_linear_chroma[k],
                    &PixelPTEBytesPerRowC_one_row_per_frame[k],
                    &dpte_row_width_chroma_ub_one_row_per_frame[k],
                    &dpte_row_height_chroma_one_row_per_frame[k],
                    &meta_req_width_chroma[k],
                    &meta_req_height_chroma[k],
                    &meta_row_width_chroma[k],
                    &meta_row_height_chroma[k],
                    &PixelPTEReqWidthC[k],
                    &PixelPTEReqHeightC[k],
                    &PTERequestSizeC[k],
                    &dpde0_bytes_per_frame_ub_c[k],
                    &meta_pte_bytes_per_frame_ub_c[k]);

            PrefetchSourceLinesC[k] = dml32_CalculatePrefetchSourceLines(
                    myPipe[k].VRatioChroma,
                    myPipe[k].VTapsChroma,
                    myPipe[k].InterlaceEnable,
                    myPipe[k].ProgressiveToInterlaceUnitInOPP,
                    myPipe[k].SwathHeightC,
                    myPipe[k].SourceRotation,
                    myPipe[k].ViewportStationary,
                    SwathWidthC[k],
                    myPipe[k].ViewportHeightChroma,
                    myPipe[k].ViewportXStartC,
                    myPipe[k].ViewportYStartC,

                    /* Output */
                    &VInitPreFillC[k],
                    &MaxNumSwathC[k]);
        } else {
            PTEBufferSizeInRequestsForLuma[k] = PTEBufferSizeInRequestsLuma + PTEBufferSizeInRequestsChroma;
            PTEBufferSizeInRequestsForChroma[k] = 0;
            PixelPTEBytesPerRowC[k] = 0;
            PDEAndMetaPTEBytesFrameC = 0;
            MetaRowByteC[k] = 0;
            MaxNumSwathC[k] = 0;
            PrefetchSourceLinesC[k] = 0;
            dpte_row_height_chroma_one_row_per_frame[k] = 0;
            dpte_row_width_chroma_ub_one_row_per_frame[k] = 0;
            PixelPTEBytesPerRowC_one_row_per_frame[k] = 0;
        }

        PDEAndMetaPTEBytesFrameY = dml32_CalculateVMAndRowBytes(
                myPipe[k].ViewportStationary,
                myPipe[k].DCCEnable,
                myPipe[k].DPPPerSurface,
                myPipe[k].BlockHeight256BytesY,
                myPipe[k].BlockWidth256BytesY,
                myPipe[k].SourcePixelFormat,
                myPipe[k].SurfaceTiling,
                myPipe[k].BytePerPixelY,
                myPipe[k].SourceRotation,
                SwathWidthY[k],
                myPipe[k].ViewportHeight,
                myPipe[k].ViewportXStart,
                myPipe[k].ViewportYStart,
                GPUVMEnable,
                HostVMEnable,
                HostVMMaxNonCachedPageTableLevels,
                GPUVMMaxPageTableLevels,
                GPUVMMinPageSizeKBytes[k],
                HostVMMinPageSize,
                PTEBufferSizeInRequestsForLuma[k],
                myPipe[k].PitchY,
                myPipe[k].DCCMetaPitchY,
                myPipe[k].BlockWidthY,
                myPipe[k].BlockHeightY,

                /* Output */
                &MetaRowByteY[k],
                &PixelPTEBytesPerRowY[k],
                &dpte_row_width_luma_ub[k],
                &dpte_row_height_luma[k],
                &dpte_row_height_linear_luma[k],
                &PixelPTEBytesPerRowY_one_row_per_frame[k],
                &dpte_row_width_luma_ub_one_row_per_frame[k],
                &dpte_row_height_luma_one_row_per_frame[k],
                &meta_req_width[k],
                &meta_req_height[k],
                &meta_row_width[k],
                &meta_row_height[k],
                &PixelPTEReqWidthY[k],
                &PixelPTEReqHeightY[k],
                &PTERequestSizeY[k],
                &dpde0_bytes_per_frame_ub_l[k],
                &meta_pte_bytes_per_frame_ub_l[k]);

        PrefetchSourceLinesY[k] = dml32_CalculatePrefetchSourceLines(
                myPipe[k].VRatio,
                myPipe[k].VTaps,
                myPipe[k].InterlaceEnable,
                myPipe[k].ProgressiveToInterlaceUnitInOPP,
                myPipe[k].SwathHeightY,
                myPipe[k].SourceRotation,
                myPipe[k].ViewportStationary,
                SwathWidthY[k],
                myPipe[k].ViewportHeight,
                myPipe[k].ViewportXStart,
                myPipe[k].ViewportYStart,

                /* Output */
                &VInitPreFillY[k],
                &MaxNumSwathY[k]);

        PDEAndMetaPTEBytesFrame[k] = PDEAndMetaPTEBytesFrameY + PDEAndMetaPTEBytesFrameC;
        MetaRowByte[k] = MetaRowByteY[k] + MetaRowByteC[k];

        if (PixelPTEBytesPerRowY[k] <= 64 * PTEBufferSizeInRequestsForLuma[k] &&
                PixelPTEBytesPerRowC[k] <= 64 * PTEBufferSizeInRequestsForChroma[k]) {
            PTEBufferSizeNotExceeded[k] = true;
        } else {
            PTEBufferSizeNotExceeded[k] = false;
        }

        one_row_per_frame_fits_in_buffer[k] = (PixelPTEBytesPerRowY_one_row_per_frame[k] <= 64 * 2 *
            PTEBufferSizeInRequestsForLuma[k] &&
            PixelPTEBytesPerRowC_one_row_per_frame[k] <= 64 * 2 * PTEBufferSizeInRequestsForChroma[k]);
    }

    dml32_CalculateMALLUseForStaticScreen(
            NumberOfActiveSurfaces,
            MALLAllocatedForDCN,
            UseMALLForStaticScreen,   // mode
            SurfaceSizeInMALL,
            one_row_per_frame_fits_in_buffer,
            /* Output */
            UsesMALLForStaticScreen); // boolen

    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        PTE_BUFFER_MODE[k] = myPipe[k].FORCE_ONE_ROW_FOR_FRAME || UsesMALLForStaticScreen[k] ||
                (UseMALLForPStateChange[k] == dm_use_mall_pstate_change_sub_viewport) ||
                (UseMALLForPStateChange[k] == dm_use_mall_pstate_change_phantom_pipe) ||
                (GPUVMMinPageSizeKBytes[k] > 64);
        BIGK_FRAGMENT_SIZE[k] = dml_log2(GPUVMMinPageSizeKBytes[k] * 1024) - 12;
    }

    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        use_one_row_for_frame[k] = myPipe[k].FORCE_ONE_ROW_FOR_FRAME || UsesMALLForStaticScreen[k] ||
                (UseMALLForPStateChange[k] == dm_use_mall_pstate_change_sub_viewport) ||
                (UseMALLForPStateChange[k] == dm_use_mall_pstate_change_phantom_pipe) ||
                (GPUVMMinPageSizeKBytes[k] > 64 && IsVertical(myPipe[k].SourceRotation));

        use_one_row_for_frame_flip[k] = use_one_row_for_frame[k] &&
                !(UseMALLForPStateChange[k] == dm_use_mall_pstate_change_full_frame);

        if (use_one_row_for_frame[k]) {
            dpte_row_height_luma[k] = dpte_row_height_luma_one_row_per_frame[k];
            dpte_row_width_luma_ub[k] = dpte_row_width_luma_ub_one_row_per_frame[k];
            PixelPTEBytesPerRowY[k] = PixelPTEBytesPerRowY_one_row_per_frame[k];
            dpte_row_height_chroma[k] = dpte_row_height_chroma_one_row_per_frame[k];
            dpte_row_width_chroma_ub[k] = dpte_row_width_chroma_ub_one_row_per_frame[k];
            PixelPTEBytesPerRowC[k] = PixelPTEBytesPerRowC_one_row_per_frame[k];
            PTEBufferSizeNotExceeded[k] = one_row_per_frame_fits_in_buffer[k];
        }

        if (MetaRowByte[k] <= DCCMetaBufferSizeBytes)
            DCCMetaBufferSizeNotExceeded[k] = true;
        else
            DCCMetaBufferSizeNotExceeded[k] = false;

        PixelPTEBytesPerRow[k] = PixelPTEBytesPerRowY[k] + PixelPTEBytesPerRowC[k];
        if (use_one_row_for_frame[k])
            PixelPTEBytesPerRow[k] = PixelPTEBytesPerRow[k] / 2;

        dml32_CalculateRowBandwidth(
                GPUVMEnable,
                myPipe[k].SourcePixelFormat,
                myPipe[k].VRatio,
                myPipe[k].VRatioChroma,
                myPipe[k].DCCEnable,
                myPipe[k].HTotal / myPipe[k].PixelClock,
                MetaRowByteY[k], MetaRowByteC[k],
                meta_row_height[k],
                meta_row_height_chroma[k],
                PixelPTEBytesPerRowY[k],
                PixelPTEBytesPerRowC[k],
                dpte_row_height_luma[k],
                dpte_row_height_chroma[k],

                /* Output */
                &meta_row_bw[k],
                &dpte_row_bw[k]);
    }
} // CalculateVMRowAndSwath

unsigned int dml32_CalculateVMAndRowBytes(
        bool ViewportStationary,
        bool DCCEnable,
        unsigned int NumberOfDPPs,
        unsigned int BlockHeight256Bytes,
        unsigned int BlockWidth256Bytes,
        enum source_format_class /* SourcePixelFormat */,
        unsigned int SurfaceTiling,
        unsigned int BytePerPixel,
        enum dm_rotation_angle SourceRotation,
        double SwathWidth,
        unsigned int ViewportHeight,
        unsigned int    ViewportXStart,
        unsigned int    ViewportYStart,
        bool GPUVMEnable,
        bool HostVMEnable,
        unsigned int HostVMMaxNonCachedPageTableLevels,
        unsigned int GPUVMMaxPageTableLevels,
        unsigned int GPUVMMinPageSizeKBytes,
        unsigned int HostVMMinPageSize,
        unsigned int PTEBufferSizeInRequests,
        unsigned int Pitch,
        unsigned int DCCMetaPitch,
        unsigned int MacroTileWidth,
        unsigned int MacroTileHeight,

        /* Output */
        unsigned int *MetaRowByte,
        unsigned int *PixelPTEBytesPerRow,
        unsigned int    *dpte_row_width_ub,
        unsigned int *dpte_row_height,
        unsigned int *dpte_row_height_linear,
        unsigned int    *PixelPTEBytesPerRow_one_row_per_frame,
        unsigned int    *dpte_row_width_ub_one_row_per_frame,
        unsigned int    *dpte_row_height_one_row_per_frame,
        unsigned int *MetaRequestWidth,
        unsigned int *MetaRequestHeight,
        unsigned int *meta_row_width,
        unsigned int *meta_row_height,
        unsigned int *PixelPTEReqWidth,
        unsigned int *PixelPTEReqHeight,
        unsigned int *PTERequestSize,
        unsigned int    *DPDE0BytesFrame,
        unsigned int    *MetaPTEBytesFrame)
{
    unsigned int MPDEBytesFrame;
    unsigned int DCCMetaSurfaceBytes;
    unsigned int ExtraDPDEBytesFrame;
    unsigned int PDEAndMetaPTEBytesFrame;
    unsigned int HostVMDynamicLevels = 0;
    unsigned int    MacroTileSizeBytes;
    unsigned int    vp_height_meta_ub;
    unsigned int    vp_height_dpte_ub;
    unsigned int PixelPTEReqWidth_linear = 0; // VBA_DELTA. VBA doesn't calculate this

    if (GPUVMEnable == true && HostVMEnable == true) {
        if (HostVMMinPageSize < 2048)
            HostVMDynamicLevels = HostVMMaxNonCachedPageTableLevels;
        else if (HostVMMinPageSize >= 2048 && HostVMMinPageSize < 1048576)
            HostVMDynamicLevels = dml_max(0, static_cast<int>(HostVMMaxNonCachedPageTableLevels) - 1);
        else
            HostVMDynamicLevels = dml_max(0, static_cast<int>(HostVMMaxNonCachedPageTableLevels) - 2);
    }

    *MetaRequestHeight = 8 * BlockHeight256Bytes;
    *MetaRequestWidth = 8 * BlockWidth256Bytes;
    if (SurfaceTiling == dm_sw_linear) {
        *meta_row_height = 32;
        *meta_row_width = dml_floor(ViewportXStart + SwathWidth + *MetaRequestWidth - 1, *MetaRequestWidth)
                - dml_floor(ViewportXStart, *MetaRequestWidth);
    } else if (!IsVertical(SourceRotation)) {
        *meta_row_height = *MetaRequestHeight;
        if (ViewportStationary && NumberOfDPPs == 1) {
            *meta_row_width = dml_floor(ViewportXStart + SwathWidth + *MetaRequestWidth - 1,
                    *MetaRequestWidth) - dml_floor(ViewportXStart, *MetaRequestWidth);
        } else {
            *meta_row_width = dml_ceil(SwathWidth - 1, *MetaRequestWidth) + *MetaRequestWidth;
        }
        *MetaRowByte = *meta_row_width * *MetaRequestHeight * BytePerPixel / 256.0;
    } else {
        *meta_row_height = *MetaRequestWidth;
        if (ViewportStationary && NumberOfDPPs == 1) {
            *meta_row_width = dml_floor(ViewportYStart + ViewportHeight + *MetaRequestHeight - 1,
                    *MetaRequestHeight) - dml_floor(ViewportYStart, *MetaRequestHeight);
        } else {
            *meta_row_width = dml_ceil(SwathWidth - 1, *MetaRequestHeight) + *MetaRequestHeight;
        }
        *MetaRowByte = *meta_row_width * *MetaRequestWidth * BytePerPixel / 256.0;
    }

    if (ViewportStationary && (NumberOfDPPs == 1 || !IsVertical(SourceRotation))) {
        vp_height_meta_ub = dml_floor(ViewportYStart + ViewportHeight + 64 * BlockHeight256Bytes - 1,
                64 * BlockHeight256Bytes) - dml_floor(ViewportYStart, 64 * BlockHeight256Bytes);
    } else if (!IsVertical(SourceRotation)) {
        vp_height_meta_ub = dml_ceil(ViewportHeight - 1, 64 * BlockHeight256Bytes) + 64 * BlockHeight256Bytes;
    } else {
        vp_height_meta_ub = dml_ceil(SwathWidth - 1, 64 * BlockHeight256Bytes) + 64 * BlockHeight256Bytes;
    }

    DCCMetaSurfaceBytes = DCCMetaPitch * vp_height_meta_ub * BytePerPixel / 256.0;

    if (GPUVMEnable == true) {
        *MetaPTEBytesFrame = (dml_ceil(static_cast<double>(DCCMetaSurfaceBytes - 4.0 * 1024.0) /
                (8 * 4.0 * 1024), 1) + 1) * 64;
        MPDEBytesFrame = 128 * (GPUVMMaxPageTableLevels - 1);
    } else {
        *MetaPTEBytesFrame = 0;
        MPDEBytesFrame = 0;
    }

    if (DCCEnable != true) {
        *MetaPTEBytesFrame = 0;
        MPDEBytesFrame = 0;
        *MetaRowByte = 0;
    }

    MacroTileSizeBytes = MacroTileWidth * BytePerPixel * MacroTileHeight;

    if (GPUVMEnable == true && GPUVMMaxPageTableLevels > 1) {
        if (ViewportStationary && (NumberOfDPPs == 1 || !IsVertical(SourceRotation))) {
            vp_height_dpte_ub = dml_floor(ViewportYStart + ViewportHeight +
                    MacroTileHeight - 1, MacroTileHeight) -
                    dml_floor(ViewportYStart, MacroTileHeight);
        } else if (!IsVertical(SourceRotation)) {
            vp_height_dpte_ub = dml_ceil(ViewportHeight - 1, MacroTileHeight) + MacroTileHeight;
        } else {
            vp_height_dpte_ub = dml_ceil(SwathWidth - 1, MacroTileHeight) + MacroTileHeight;
        }
        *DPDE0BytesFrame = 64 * (dml_ceil((Pitch * vp_height_dpte_ub * BytePerPixel - MacroTileSizeBytes) /
                (8 * 2097152), 1) + 1);
        ExtraDPDEBytesFrame = 128 * (GPUVMMaxPageTableLevels - 2);
    } else {
        *DPDE0BytesFrame = 0;
        ExtraDPDEBytesFrame = 0;
        vp_height_dpte_ub = 0;
    }

    PDEAndMetaPTEBytesFrame = *MetaPTEBytesFrame + MPDEBytesFrame + *DPDE0BytesFrame + ExtraDPDEBytesFrame;

    if (HostVMEnable == true)
        PDEAndMetaPTEBytesFrame = PDEAndMetaPTEBytesFrame * (1 + 8 * HostVMDynamicLevels);

    if (SurfaceTiling == dm_sw_linear) {
        *PixelPTEReqHeight = 1;
        *PixelPTEReqWidth = GPUVMMinPageSizeKBytes * 1024 * 8 / BytePerPixel;
        PixelPTEReqWidth_linear = GPUVMMinPageSizeKBytes * 1024 * 8 / BytePerPixel;
        *PTERequestSize = 64;
    } else if (GPUVMMinPageSizeKBytes == 4) {
        *PixelPTEReqHeight = 16 * BlockHeight256Bytes;
        *PixelPTEReqWidth = 16 * BlockWidth256Bytes;
        *PTERequestSize = 128;
    } else {
        *PixelPTEReqHeight = MacroTileHeight;
        *PixelPTEReqWidth = 8 *  1024 * GPUVMMinPageSizeKBytes / (MacroTileHeight * BytePerPixel);
        *PTERequestSize = 64;
    }

    *dpte_row_height_one_row_per_frame = vp_height_dpte_ub;
    *dpte_row_width_ub_one_row_per_frame = (dml_ceil((static_cast<double>(Pitch) * static_cast<double>(*dpte_row_height_one_row_per_frame) /
            static_cast<double>(*PixelPTEReqHeight) - 1) / static_cast<double>(*PixelPTEReqWidth), 1) + 1) *
                    static_cast<double>(*PixelPTEReqWidth);
    *PixelPTEBytesPerRow_one_row_per_frame = *dpte_row_width_ub_one_row_per_frame / *PixelPTEReqWidth *
            *PTERequestSize;

    if (SurfaceTiling == dm_sw_linear) {
        *dpte_row_height = dml_min(128, 1 << static_cast<unsigned int>(dml_floor(dml_log2(PTEBufferSizeInRequests *
                *PixelPTEReqWidth / Pitch), 1)));
        *dpte_row_width_ub = dml_ceil((static_cast<double>(Pitch) * static_cast<double>(*dpte_row_height) - 1),
                static_cast<double>(*PixelPTEReqWidth)) + *PixelPTEReqWidth;
        *PixelPTEBytesPerRow = *dpte_row_width_ub / static_cast<double>(*PixelPTEReqWidth) * static_cast<double>(*PTERequestSize);

        // VBA_DELTA, VBA doesn't have programming value for pte row height linear.
        *dpte_row_height_linear = 1 << static_cast<unsigned int>(dml_floor(dml_log2(PTEBufferSizeInRequests *
                PixelPTEReqWidth_linear / Pitch), 1));
        if (*dpte_row_height_linear > 128)
            *dpte_row_height_linear = 128;

    } else if (!IsVertical(SourceRotation)) {
        *dpte_row_height = *PixelPTEReqHeight;

        if (GPUVMMinPageSizeKBytes > 64) {
            *dpte_row_width_ub = (dml_ceil((Pitch * *dpte_row_height / *PixelPTEReqHeight - 1) /
                    *PixelPTEReqWidth, 1) + 1) * *PixelPTEReqWidth;
        } else if (ViewportStationary && (NumberOfDPPs == 1)) {
            *dpte_row_width_ub = dml_floor(ViewportXStart + SwathWidth +
                    *PixelPTEReqWidth - 1, *PixelPTEReqWidth) -
                    dml_floor(ViewportXStart, *PixelPTEReqWidth);
        } else {
            *dpte_row_width_ub = (dml_ceil((SwathWidth - 1) / *PixelPTEReqWidth, 1) + 1) *
                    *PixelPTEReqWidth;
        }

        *PixelPTEBytesPerRow = *dpte_row_width_ub / *PixelPTEReqWidth * *PTERequestSize;
    } else {
        *dpte_row_height = dml_min(*PixelPTEReqWidth, MacroTileWidth);

        if (ViewportStationary && (NumberOfDPPs == 1)) {
            *dpte_row_width_ub = dml_floor(ViewportYStart + ViewportHeight + *PixelPTEReqHeight - 1,
                    *PixelPTEReqHeight) - dml_floor(ViewportYStart, *PixelPTEReqHeight);
        } else {
            *dpte_row_width_ub = (dml_ceil((SwathWidth - 1) / *PixelPTEReqHeight, 1) + 1)
                    * *PixelPTEReqHeight;
        }

        *PixelPTEBytesPerRow = *dpte_row_width_ub / *PixelPTEReqHeight * *PTERequestSize;
    }

    if (GPUVMEnable != true)
        *PixelPTEBytesPerRow = 0;
    if (HostVMEnable == true)
        *PixelPTEBytesPerRow = *PixelPTEBytesPerRow * (1 + 8 * HostVMDynamicLevels);

    return PDEAndMetaPTEBytesFrame;
} // CalculateVMAndRowBytes

double dml32_CalculatePrefetchSourceLines(
        double VRatio,
        unsigned int VTaps,
        bool Interlace,
        bool ProgressiveToInterlaceUnitInOPP,
        unsigned int SwathHeight,
        enum dm_rotation_angle SourceRotation,
        bool ViewportStationary,
        double SwathWidth,
        unsigned int ViewportHeight,
        unsigned int ViewportXStart,
        unsigned int ViewportYStart,

        /* Output */
        double *VInitPreFill,
        unsigned int *MaxNumSwath)
{

    unsigned int vp_start_rot;
    unsigned int sw0_tmp;
    unsigned int MaxPartialSwath;
    double numLines;

    if (ProgressiveToInterlaceUnitInOPP)
        *VInitPreFill = dml_floor((VRatio + static_cast<double>(VTaps) + 1) / 2.0, 1);
    else
        *VInitPreFill = dml_floor((VRatio + static_cast<double>(VTaps) + 1 + Interlace * 0.5 * VRatio) / 2.0, 1);

    if (ViewportStationary) {
        if (SourceRotation == dm_rotation_180 || SourceRotation == dm_rotation_180m) {
            vp_start_rot = SwathHeight -
                    ((static_cast<unsigned int>(ViewportYStart + ViewportHeight - 1) % SwathHeight) + 1);
        } else if (SourceRotation == dm_rotation_270 || SourceRotation == dm_rotation_90m) {
            vp_start_rot = ViewportXStart;
        } else if (SourceRotation == dm_rotation_90 || SourceRotation == dm_rotation_270m) {
            vp_start_rot = SwathHeight -
                    ((static_cast<unsigned int>(ViewportYStart + SwathWidth - 1) % SwathHeight) + 1);
        } else {
            vp_start_rot = ViewportYStart;
        }
        sw0_tmp = SwathHeight - (vp_start_rot % SwathHeight);
        if (sw0_tmp < *VInitPreFill)
            *MaxNumSwath = dml_ceil((*VInitPreFill - sw0_tmp) / SwathHeight, 1) + 1;
        else
            *MaxNumSwath = 1;
        MaxPartialSwath = dml_max(1, static_cast<unsigned int>(vp_start_rot + *VInitPreFill - 1) % SwathHeight);
    } else {
        *MaxNumSwath = dml_ceil((*VInitPreFill - 1.0) / SwathHeight, 1) + 1;
        if (*VInitPreFill > 1)
            MaxPartialSwath = dml_max(1, static_cast<unsigned int>(*VInitPreFill - 2) % SwathHeight);
        else
            MaxPartialSwath = dml_max(1, static_cast<unsigned int>(*VInitPreFill + SwathHeight - 2) % SwathHeight);
    }
    numLines = *MaxNumSwath * SwathHeight + MaxPartialSwath;

    return numLines;

} // CalculatePrefetchSourceLines

void dml32_CalculateMALLUseForStaticScreen(
        unsigned int NumberOfActiveSurfaces,
        unsigned int MALLAllocatedForDCNFinal,
        enum dm_use_mall_for_static_screen_mode *UseMALLForStaticScreen,
        unsigned int SurfaceSizeInMALL[],
        bool one_row_per_frame_fits_in_buffer[],

        /* output */
        bool UsesMALLForStaticScreen[])
{
    unsigned int k;
    unsigned int SurfaceToAddToMALL;
    bool CanAddAnotherSurfaceToMALL;
    unsigned int TotalSurfaceSizeInMALL;

    TotalSurfaceSizeInMALL = 0;
    for (k = 0; k < NumberOfActiveSurfaces; ++k) {
        UsesMALLForStaticScreen[k] = (UseMALLForStaticScreen[k] == dm_use_mall_static_screen_enable);
        if (UsesMALLForStaticScreen[k])
            TotalSurfaceSizeInMALL = TotalSurfaceSizeInMALL + SurfaceSizeInMALL[k];
    }

    SurfaceToAddToMALL = 0;
    CanAddAnotherSurfaceToMALL = true;
    while (CanAddAnotherSurfaceToMALL) {
        CanAddAnotherSurfaceToMALL = false;
        for (k = 0; k < NumberOfActiveSurfaces; ++k) {
            if (TotalSurfaceSizeInMALL + SurfaceSizeInMALL[k] <= MALLAllocatedForDCNFinal * 1024 * 1024 &&
                    !UsesMALLForStaticScreen[k] &&
                    UseMALLForStaticScreen[k] != dm_use_mall_static_screen_disable &&
                    one_row_per_frame_fits_in_buffer[k] &&
                    (!CanAddAnotherSurfaceToMALL ||
                    SurfaceSizeInMALL[k] < SurfaceSizeInMALL[SurfaceToAddToMALL])) {
                CanAddAnotherSurfaceToMALL = true;
                SurfaceToAddToMALL = k;
            }
        }
        if (CanAddAnotherSurfaceToMALL) {
            UsesMALLForStaticScreen[SurfaceToAddToMALL] = true;
            TotalSurfaceSizeInMALL = TotalSurfaceSizeInMALL + SurfaceSizeInMALL[SurfaceToAddToMALL];

        }
    }
}

void dml32_CalculateRowBandwidth(
        bool GPUVMEnable,
        enum source_format_class SourcePixelFormat,
        double VRatio,
        double VRatioChroma,
        bool DCCEnable,
        double LineTime,
        unsigned int MetaRowByteLuma,
        unsigned int MetaRowByteChroma,
        unsigned int meta_row_height_luma,
        unsigned int meta_row_height_chroma,
        unsigned int PixelPTEBytesPerRowLuma,
        unsigned int PixelPTEBytesPerRowChroma,
        unsigned int dpte_row_height_luma,
        unsigned int dpte_row_height_chroma,
        /* Output */
        double *meta_row_bw,
        double *dpte_row_bw)
{
    if (DCCEnable != true) {
        *meta_row_bw = 0;
    } else if (SourcePixelFormat == dm_420_8 || SourcePixelFormat == dm_420_10 || SourcePixelFormat == dm_420_12 ||
            SourcePixelFormat == dm_rgbe_alpha) {
        *meta_row_bw = VRatio * MetaRowByteLuma / (meta_row_height_luma * LineTime) + VRatioChroma *
                MetaRowByteChroma / (meta_row_height_chroma * LineTime);
    } else {
        *meta_row_bw = VRatio * MetaRowByteLuma / (meta_row_height_luma * LineTime);
    }

    if (GPUVMEnable != true) {
        *dpte_row_bw = 0;
    } else if (SourcePixelFormat == dm_420_8 || SourcePixelFormat == dm_420_10 || SourcePixelFormat == dm_420_12 ||
            SourcePixelFormat == dm_rgbe_alpha) {
        *dpte_row_bw = VRatio * PixelPTEBytesPerRowLuma / (dpte_row_height_luma * LineTime) +
                VRatioChroma * PixelPTEBytesPerRowChroma / (dpte_row_height_chroma * LineTime);
    } else {
        *dpte_row_bw = VRatio * PixelPTEBytesPerRowLuma / (dpte_row_height_luma * LineTime);
    }
}

double dml32_CalculateUrgentLatency(
        double UrgentLatencyPixelDataOnly,
        double UrgentLatencyPixelMixedWithVMData,
        double UrgentLatencyVMDataOnly,
        bool   DoUrgentLatencyAdjustment,
        double UrgentLatencyAdjustmentFabricClockComponent,
        double UrgentLatencyAdjustmentFabricClockReference,
        double FabricClock)
{
    double   ret;

    ret = dml_max3(UrgentLatencyPixelDataOnly, UrgentLatencyPixelMixedWithVMData, UrgentLatencyVMDataOnly);
    if (DoUrgentLatencyAdjustment == true) {
        ret = ret + UrgentLatencyAdjustmentFabricClockComponent *
                (UrgentLatencyAdjustmentFabricClockReference / FabricClock - 1);
    }
    return ret;
}

void dml32_CalculateUrgentBurstFactor(
        enum dm_use_mall_for_pstate_change_mode UseMALLForPStateChange,
        unsigned int    swath_width_luma_ub,
        unsigned int    swath_width_chroma_ub,
        unsigned int SwathHeightY,
        unsigned int SwathHeightC,
        double  LineTime,
        double  UrgentLatency,
        double  CursorBufferSize,
        unsigned int CursorWidth,
        unsigned int CursorBPP,
        double  VRatio,
        double  /* VRatioC */,
        double  BytePerPixelInDETY,
        double  BytePerPixelInDETC,
        unsigned int    DETBufferSizeY,
        unsigned int    DETBufferSizeC,
        /* Output */
        double *UrgentBurstFactorCursor,
        double *UrgentBurstFactorLuma,
        double *UrgentBurstFactorChroma,
        bool   *NotEnoughUrgentLatencyHiding)
{
    double       LinesInDETLuma;
    double       LinesInDETChroma;
    unsigned int LinesInCursorBuffer;
    double       CursorBufferSizeInTime;
    double       DETBufferSizeInTimeLuma;
    double       DETBufferSizeInTimeChroma;

    *NotEnoughUrgentLatencyHiding = 0;

    if (CursorWidth > 0) {
        LinesInCursorBuffer = 1 << static_cast<unsigned int>(dml_floor(dml_log2(CursorBufferSize * 1024.0 /
                (CursorWidth * CursorBPP / 8.0)), 1.0));
        if (VRatio > 0) {
            CursorBufferSizeInTime = LinesInCursorBuffer * LineTime / VRatio;
            if (CursorBufferSizeInTime - UrgentLatency <= 0) {
                *NotEnoughUrgentLatencyHiding = 1;
                *UrgentBurstFactorCursor = 0;
            } else {
                *UrgentBurstFactorCursor = CursorBufferSizeInTime /
                        (CursorBufferSizeInTime - UrgentLatency);
            }
        } else {
            *UrgentBurstFactorCursor = 1;
        }
    }

    LinesInDETLuma = (UseMALLForPStateChange == dm_use_mall_pstate_change_phantom_pipe ? 1024*1024 :
            DETBufferSizeY) / BytePerPixelInDETY / swath_width_luma_ub;

    if (VRatio > 0) {
        DETBufferSizeInTimeLuma = dml_floor(LinesInDETLuma, SwathHeightY) * LineTime / VRatio;
        if (DETBufferSizeInTimeLuma - UrgentLatency <= 0) {
            *NotEnoughUrgentLatencyHiding = 1;
            *UrgentBurstFactorLuma = 0;
        } else {
            *UrgentBurstFactorLuma = DETBufferSizeInTimeLuma / (DETBufferSizeInTimeLuma - UrgentLatency);
        }
    } else {
        *UrgentBurstFactorLuma = 1;
    }

    if (BytePerPixelInDETC > 0) {
        LinesInDETChroma = (UseMALLForPStateChange == dm_use_mall_pstate_change_phantom_pipe ?
                    1024 * 1024 : DETBufferSizeC) / BytePerPixelInDETC
                    / swath_width_chroma_ub;

        if (VRatio > 0) {
            DETBufferSizeInTimeChroma = dml_floor(LinesInDETChroma, SwathHeightC) * LineTime / VRatio;
            if (DETBufferSizeInTimeChroma - UrgentLatency <= 0) {
                *NotEnoughUrgentLatencyHiding = 1;
                *UrgentBurstFactorChroma = 0;
            } else {
                *UrgentBurstFactorChroma = DETBufferSizeInTimeChroma
                        / (DETBufferSizeInTimeChroma - UrgentLatency);
            }
        } else {
            *UrgentBurstFactorChroma = 1;
        }
    }
} // CalculateUrgentBurstFactor

void dml32_CalculateDCFCLKDeepSleep(
        unsigned int NumberOfActiveSurfaces,
        unsigned int BytePerPixelY[],
        unsigned int BytePerPixelC[],
        double VRatio[],
        double VRatioChroma[],
        double SwathWidthY[],
        double SwathWidthC[],
        unsigned int DPPPerSurface[],
        double HRatio[],
        double HRatioChroma[],
        double PixelClock[],
        double PSCL_THROUGHPUT[],
        double PSCL_THROUGHPUT_CHROMA[],
        double Dppclk[],
        double ReadBandwidthLuma[],
        double ReadBandwidthChroma[],
        unsigned int ReturnBusWidth,

        /* Output */
        double *DCFClkDeepSleep)
{
    unsigned int k;
    double   DisplayPipeLineDeliveryTimeLuma;
    double   DisplayPipeLineDeliveryTimeChroma;
    double   DCFClkDeepSleepPerSurface[DML32_NUM_PLANES]{}; // written before read; initialized for -Wmaybe-uninitialized
    double ReadBandwidth = 0.0;

    for (k = 0; k < NumberOfActiveSurfaces; ++k) {

        if (VRatio[k] <= 1) {
            DisplayPipeLineDeliveryTimeLuma = SwathWidthY[k] * DPPPerSurface[k] / HRatio[k]
                    / PixelClock[k];
        } else {
            DisplayPipeLineDeliveryTimeLuma = SwathWidthY[k] / PSCL_THROUGHPUT[k] / Dppclk[k];
        }
        if (BytePerPixelC[k] == 0) {
            DisplayPipeLineDeliveryTimeChroma = 0;
        } else {
            if (VRatioChroma[k] <= 1) {
                DisplayPipeLineDeliveryTimeChroma = SwathWidthC[k] *
                        DPPPerSurface[k] / HRatioChroma[k] / PixelClock[k];
            } else {
                DisplayPipeLineDeliveryTimeChroma = SwathWidthC[k] / PSCL_THROUGHPUT_CHROMA[k]
                        / Dppclk[k];
            }
        }

        if (BytePerPixelC[k] > 0) {
            DCFClkDeepSleepPerSurface[k] = dml_max(__DML_MIN_DCFCLK_FACTOR__ * SwathWidthY[k] *
                    BytePerPixelY[k] / 32.0 / DisplayPipeLineDeliveryTimeLuma,
                    __DML_MIN_DCFCLK_FACTOR__ * SwathWidthC[k] * BytePerPixelC[k] /
                    32.0 / DisplayPipeLineDeliveryTimeChroma);
        } else {
            DCFClkDeepSleepPerSurface[k] = __DML_MIN_DCFCLK_FACTOR__ * SwathWidthY[k] * BytePerPixelY[k] /
                    64.0 / DisplayPipeLineDeliveryTimeLuma;
        }
        DCFClkDeepSleepPerSurface[k] = dml_max(DCFClkDeepSleepPerSurface[k], PixelClock[k] / 16);

    }

    for (k = 0; k < NumberOfActiveSurfaces; ++k)
        ReadBandwidth = ReadBandwidth + ReadBandwidthLuma[k] + ReadBandwidthChroma[k];

    *DCFClkDeepSleep = dml_max(8.0, __DML_MIN_DCFCLK_FACTOR__ * ReadBandwidth / static_cast<double>(ReturnBusWidth));

    for (k = 0; k < NumberOfActiveSurfaces; ++k)
        *DCFClkDeepSleep = dml_max(*DCFClkDeepSleep, DCFClkDeepSleepPerSurface[k]);
} // CalculateDCFCLKDeepSleep

unsigned int dml32_CalculateExtraLatencyBytes(unsigned int ReorderingBytes,
        unsigned int TotalNumberOfActiveDPP,
        unsigned int PixelChunkSizeInKByte,
        unsigned int TotalNumberOfDCCActiveDPP,
        unsigned int MetaChunkSize,
        bool GPUVMEnable,
        bool HostVMEnable,
        unsigned int NumberOfActiveSurfaces,
        unsigned int NumberOfDPP[],
        unsigned int dpte_group_bytes[],
        double HostVMInefficiencyFactor,
        double HostVMMinPageSize,
        unsigned int HostVMMaxNonCachedPageTableLevels)
{
    unsigned int k;
    double   ret;
    unsigned int  HostVMDynamicLevels;

    if (GPUVMEnable == true && HostVMEnable == true) {
        if (HostVMMinPageSize < 2048)
            HostVMDynamicLevels = HostVMMaxNonCachedPageTableLevels;
        else if (HostVMMinPageSize >= 2048 && HostVMMinPageSize < 1048576)
            HostVMDynamicLevels = dml_max(0, static_cast<int>(HostVMMaxNonCachedPageTableLevels) - 1);
        else
            HostVMDynamicLevels = dml_max(0, static_cast<int>(HostVMMaxNonCachedPageTableLevels) - 2);
    } else {
        HostVMDynamicLevels = 0;
    }

    ret = ReorderingBytes + (TotalNumberOfActiveDPP * PixelChunkSizeInKByte +
            TotalNumberOfDCCActiveDPP * MetaChunkSize) * 1024.0;

    if (GPUVMEnable == true) {
        for (k = 0; k < NumberOfActiveSurfaces; ++k) {
            ret = ret + NumberOfDPP[k] * dpte_group_bytes[k] *
                    (1 + 8 * HostVMDynamicLevels) * HostVMInefficiencyFactor;
        }
    }
    return ret;
}

void dml32_CalculateVUpdateAndDynamicMetadataParameters(
        unsigned int MaxInterDCNTileRepeaters,
        double Dppclk,
        double Dispclk,
        double DCFClkDeepSleep,
        double PixelClock,
        unsigned int HTotal,
        unsigned int VBlank,
        unsigned int DynamicMetadataTransmittedBytes,
        unsigned int DynamicMetadataLinesBeforeActiveRequired,
        unsigned int InterlaceEnable,
        bool ProgressiveToInterlaceUnitInOPP,

        /* output */
        double *TSetup,
        double *Tdmbf,
        double *Tdmec,
        double *Tdmsks,
        unsigned int *VUpdateOffsetPix,
        double *VUpdateWidthPix,
        double *VReadyOffsetPix)
{
    double TotalRepeaterDelayTime;

    TotalRepeaterDelayTime = MaxInterDCNTileRepeaters * (2 / Dppclk + 3 / Dispclk);
    *VUpdateWidthPix  =
            dml_ceil((14.0 / DCFClkDeepSleep + 12.0 / Dppclk + TotalRepeaterDelayTime) * PixelClock, 1.0);
    *VReadyOffsetPix  = dml_ceil(dml_max(150.0 / Dppclk,
            TotalRepeaterDelayTime + 20.0 / DCFClkDeepSleep + 10.0 / Dppclk) * PixelClock, 1.0);
    *VUpdateOffsetPix = dml_ceil(HTotal / 4.0, 1.0);
    *TSetup = (*VUpdateOffsetPix + *VUpdateWidthPix + *VReadyOffsetPix) / PixelClock;
    *Tdmbf = DynamicMetadataTransmittedBytes / 4.0 / Dispclk;
    *Tdmec = HTotal / PixelClock;

    if (DynamicMetadataLinesBeforeActiveRequired == 0)
        *Tdmsks = VBlank * HTotal / PixelClock / 2.0;
    else
        *Tdmsks = DynamicMetadataLinesBeforeActiveRequired * HTotal / PixelClock;

    if (InterlaceEnable == 1 && ProgressiveToInterlaceUnitInOPP == false)
        *Tdmsks = *Tdmsks / 2;
}

double dml32_CalculateTWait(
        unsigned int PrefetchMode,
        enum dm_use_mall_for_pstate_change_mode UseMALLForPStateChange,
        bool SynchronizeDRRDisplaysForUCLKPStateChangeFinal,
        bool DRRDisplay,
        double DRAMClockChangeLatency,
        double FCLKChangeLatency,
        double UrgentLatency,
        double SREnterPlusExitTime)
{
    double TWait = 0.0;

    if (PrefetchMode == 0 &&
            !(UseMALLForPStateChange == dm_use_mall_pstate_change_full_frame) &&
            !(UseMALLForPStateChange == dm_use_mall_pstate_change_sub_viewport) &&
            !(UseMALLForPStateChange == dm_use_mall_pstate_change_phantom_pipe) &&
            !(SynchronizeDRRDisplaysForUCLKPStateChangeFinal && DRRDisplay)) {
        TWait = dml_max3(DRAMClockChangeLatency + UrgentLatency, SREnterPlusExitTime, UrgentLatency);
    } else if (PrefetchMode <= 1 && !(UseMALLForPStateChange == dm_use_mall_pstate_change_phantom_pipe)) {
        TWait = dml_max3(FCLKChangeLatency + UrgentLatency, SREnterPlusExitTime, UrgentLatency);
    } else if (PrefetchMode <= 2 && !(UseMALLForPStateChange == dm_use_mall_pstate_change_phantom_pipe)) {
        TWait = dml_max(SREnterPlusExitTime, UrgentLatency);
    } else {
        TWait = UrgentLatency;
    }

    return TWait;
} // CalculateTWait

// Function: get_return_bw_mbps
// Megabyte per second
double dml32_get_return_bw_mbps(const soc_bounding_box_st *soc,
        const int VoltageLevel,
        const bool HostVMEnable,
        const double DCFCLK,
        const double FabricClock,
        const double DRAMSpeed)
{
    double ReturnBW = 0.;
    double IdealSDPPortBandwidth    = soc->return_bus_width_bytes /*mode_lib->vba.ReturnBusWidth*/ * DCFCLK;
    double IdealFabricBandwidth     = FabricClock * soc->fabric_datapath_to_dcn_data_return_bytes;
    double IdealDRAMBandwidth       = DRAMSpeed * soc->num_chans * soc->dram_channel_width_bytes;
    double PixelDataOnlyReturnBW    = dml_min3(IdealSDPPortBandwidth * soc->pct_ideal_sdp_bw_after_urgent / 100,
            IdealFabricBandwidth * soc->pct_ideal_fabric_bw_after_urgent / 100,
            IdealDRAMBandwidth * (VoltageLevel < 2 ? soc->pct_ideal_dram_bw_after_urgent_strobe  :
                    soc->pct_ideal_dram_sdp_bw_after_urgent_pixel_only) / 100);
    double PixelMixedWithVMDataReturnBW = dml_min3(IdealSDPPortBandwidth * soc->pct_ideal_sdp_bw_after_urgent / 100,
            IdealFabricBandwidth * soc->pct_ideal_fabric_bw_after_urgent / 100,
            IdealDRAMBandwidth * (VoltageLevel < 2 ? soc->pct_ideal_dram_bw_after_urgent_strobe :
                    soc->pct_ideal_dram_sdp_bw_after_urgent_pixel_only) / 100);

    if (HostVMEnable != true)
        ReturnBW = PixelDataOnlyReturnBW;
    else
        ReturnBW = PixelMixedWithVMDataReturnBW;

    return ReturnBW;
}

// Function: get_return_bw_mbps_vm_only
// Megabyte per second
double dml32_get_return_bw_mbps_vm_only(const soc_bounding_box_st *soc,
        const int VoltageLevel,
        const double DCFCLK,
        const double FabricClock,
        const double DRAMSpeed)
{
    double VMDataOnlyReturnBW = dml_min3(
            soc->return_bus_width_bytes * DCFCLK * soc->pct_ideal_sdp_bw_after_urgent / 100.0,
            FabricClock * soc->fabric_datapath_to_dcn_data_return_bytes
                    * soc->pct_ideal_sdp_bw_after_urgent / 100.0,
            DRAMSpeed * soc->num_chans * soc->dram_channel_width_bytes
                    * (VoltageLevel < 2 ?
                            soc->pct_ideal_dram_bw_after_urgent_strobe :
                            soc->pct_ideal_dram_sdp_bw_after_urgent_vm_only) / 100.0);
    return VMDataOnlyReturnBW;
}

double dml32_CalculateExtraLatency(
        unsigned int RoundTripPingLatencyCycles,
        unsigned int ReorderingBytes,
        double DCFCLK,
        unsigned int TotalNumberOfActiveDPP,
        unsigned int PixelChunkSizeInKByte,
        unsigned int TotalNumberOfDCCActiveDPP,
        unsigned int MetaChunkSize,
        double ReturnBW,
        bool GPUVMEnable,
        bool HostVMEnable,
        unsigned int NumberOfActiveSurfaces,
        unsigned int NumberOfDPP[],
        unsigned int dpte_group_bytes[],
        double HostVMInefficiencyFactor,
        double HostVMMinPageSize,
        unsigned int HostVMMaxNonCachedPageTableLevels)
{
    double ExtraLatencyBytes;
    double ExtraLatency;

    ExtraLatencyBytes = dml32_CalculateExtraLatencyBytes(
            ReorderingBytes,
            TotalNumberOfActiveDPP,
            PixelChunkSizeInKByte,
            TotalNumberOfDCCActiveDPP,
            MetaChunkSize,
            GPUVMEnable,
            HostVMEnable,
            NumberOfActiveSurfaces,
            NumberOfDPP,
            dpte_group_bytes,
            HostVMInefficiencyFactor,
            HostVMMinPageSize,
            HostVMMaxNonCachedPageTableLevels);

    ExtraLatency = (RoundTripPingLatencyCycles + __DML_ARB_TO_RET_DELAY__) / DCFCLK + ExtraLatencyBytes / ReturnBW;

    return ExtraLatency;
} // CalculateExtraLatency

} // namespace rx7900::display::dml32

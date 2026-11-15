/*
Module: rx7900.hardware.vmhub_layout
Purpose: Define the MMHUB3.0.0 and GFXHUB3.0 (GC11.0.0) VM-hub register layouts once.
Created: 2026-10-21
Architecture: Header-only constants shared by gmc11 and the memory GFXHUB protocol.
Copyright 2021 Advanced Micro Devices, Inc. Values extracted from Linux v6.12
mmhub_3_0_0_offset.h/_sh_mask.h, gc_11_0_0_offset.h/_sh_mask.h/_default.h and
mmhub_v3_0.c defaults. Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#pragma once

#include <stdint.h>

namespace rx7900::hardware {

// Dword register indices relative to the hub's discovered segment0 base (every
// listed register has BASE_IDX 0 in the pinned headers).
struct VmHubLayout final {
    uint32_t context0_base_low;
    uint32_t context0_base_high;
    uint32_t context0_start_low;
    uint32_t context0_start_high;
    uint32_t context0_end_low;
    uint32_t context0_end_high;
    uint32_t context1_base_low;
    uint32_t agp_base;
    uint32_t agp_bot;
    uint32_t agp_top;
    uint32_t system_aperture_low;
    uint32_t system_aperture_high;
    uint32_t default_address_lsb;
    uint32_t default_address_msb;
    uint32_t fault_default_low;
    uint32_t fault_default_high;
    uint32_t fault_control;
    uint32_t fault_control2;
    uint32_t fault_status;
    uint32_t l1_tlb_control;
    uint32_t l2_control;
    uint32_t l2_control2;
    uint32_t l2_control3;
    uint32_t l2_control4;
    uint32_t l2_control5;
    uint32_t context0_control;
    uint32_t context1_control;
    uint32_t context1_start_low;
    uint32_t context1_start_high;
    uint32_t context1_end_low;
    uint32_t context1_end_high;
    uint32_t identity_low_low;
    uint32_t identity_low_high;
    uint32_t identity_high_low;
    uint32_t identity_high_high;
    uint32_t identity_offset_low;
    uint32_t identity_offset_high;
    uint32_t invalidate0_semaphore;
    uint32_t invalidate0_request;
    uint32_t invalidate0_ack;
    uint32_t invalidate1_request;
    uint32_t invalidate0_range_low;
    uint32_t invalidate0_range_high;
    uint32_t invalidate1_range_low;
    uint32_t bank_select_reserved_cid2;
    uint32_t contexts_disable;
    uint32_t fb_location_base;
    uint32_t fb_location_top;
    uint32_t fb_offset;
    uint32_t l2_control3_default;
    uint32_t l2_control4_default;
    uint32_t l2_control5_default;
};

constexpr VmHubLayout mmhub_3_0_0_layout{
    0x7ab, 0x7ac, 0x7cb, 0x7cc, 0x7eb, 0x7ec, 0x7ad, 0x8f0, 0x8ef, 0x8ee, 0x8f1, 0x8f2,
    0x8d8, 0x8d9, 0x70f, 0x710, 0x708, 0x709, 0x70c, 0x8f3, 0x700, 0x701, 0x702, 0x718,
    0x71e, 0x740, 0x741, 0x7cd, 0x7ce, 0x7ed, 0x7ee, 0x712, 0x713, 0x714, 0x715, 0x716,
    0x717, 0x751, 0x763, 0x775, 0x764, 0x787, 0x788, 0x789, 0x71b, 0x750,
    0x8ec, 0x8ed, 0x8d7, 0x80100007, 0x000000c1, 0x00003fe0};

constexpr VmHubLayout gfxhub_3_0_layout{
    0x16f3, 0x16f4, 0x1713, 0x1714, 0x1733, 0x1734, 0x16f5, 0x167c, 0x167b, 0x167a, 0x167d,
    0x167e, 0x15a8, 0x15a9, 0x15cb, 0x15cc, 0x15c4, 0x15c5, 0x15c8, 0x167f, 0x15bc, 0x15bd,
    0x15be, 0x15d4, 0x15da, 0x1688, 0x1689, 0x1715, 0x1716, 0x1735, 0x1736, 0x15ce, 0x15cf,
    0x15d0, 0x15d1, 0x15d2, 0x15d3, 0x1699, 0x16ab, 0x16bd, 0x16ac, 0x16cf, 0x16d0, 0x16d1,
    0x15d7, 0x1698, 0x1678, 0x1679, 0x15a7, 0x80120007, 0x000000c1, 0x00003fe0};

// GC11.0.0 CP_DEBUG and CPG_UTCL1_ERROR_HALT_DISABLE (GFXHUB fault defaults).
constexpr uint32_t gc_cp_debug = 0x1e1f;
constexpr uint32_t gc_cp_debug_utcl1_error_halt_disable = 0x8000;

// MC_VM_FB_LOCATION_BASE/TOP and MC_VM_FB_OFFSET fields (24-bit, 16 MiB units).
constexpr uint32_t vmhub_fb_field_mask = 0x00ffffff;

// Identical field layouts in both hubs (mask, shift) from the pinned sh_mask headers.
namespace vmhub_field {
constexpr uint32_t l1_enable_tlb = 0x1;
constexpr uint32_t l1_system_access_mode_shift = 3;
constexpr uint32_t l1_system_access_mode = 0x18;
constexpr uint32_t l1_unmapped_access = 0x20;
constexpr uint32_t l1_advanced_driver_model = 0x40;
constexpr uint32_t l1_eco_bits = 0x780;
constexpr uint32_t l1_mtype_shift = 11;
constexpr uint32_t l1_mtype = 0x3800;
constexpr uint32_t l2_enable_cache = 0x1;
constexpr uint32_t l2_fragment_processing = 0x2;
constexpr uint32_t l2_pde0_tag_generation = 0x100;
constexpr uint32_t l2_default_page_out_to_system = 0x800;
constexpr uint32_t l2_pde_fault_classification = 0x40000;
constexpr uint32_t l2_context1_identity_mode_shift = 19;
constexpr uint32_t l2_context1_identity_mode = 0x180000;
constexpr uint32_t l2_identity_fragment_size = 0x3e00000;
constexpr uint32_t l2_invalidate_all_l1_tlbs = 0x1;
constexpr uint32_t l2_invalidate_l2_cache = 0x2;
constexpr uint32_t l3_bank_select = 0x3f;
constexpr uint32_t l3_bigk_fragment_shift = 15;
constexpr uint32_t l3_bigk_fragment = 0xf8000;
constexpr uint32_t l4_pde_request_physical = 0x40;
constexpr uint32_t l4_pte_request_physical = 0x80;
constexpr uint32_t l5_smallk_fragment = 0x1f;
constexpr uint32_t context_enable = 0x1;
constexpr uint32_t context_depth_shift = 1;
constexpr uint32_t context_depth = 0x6;
constexpr uint32_t context_block_size_shift = 3;
constexpr uint32_t context_block_size = 0x78;
constexpr uint32_t context_retry_fault = 0x80;
// RANGE, DUMMY_PAGE, PDE0, VALID, READ, WRITE, EXECUTE fault enable defaults.
constexpr uint32_t context1_fault_defaults = 0x400 | 0x1000 | 0x4000 | 0x10000 | 0x40000 |
                                             0x100000 | 0x400000;
// RANGE..EXECUTE fault enable defaults in L2_PROTECTION_FAULT_CNTL (bits 2..12).
constexpr uint32_t fault_enable_defaults = 0x1ffc;
constexpr uint32_t fault_crash_on_no_retry = 0x40000000;
constexpr uint32_t fault_crash_on_retry = 0x80000000;
constexpr uint32_t fault2_active_page_migration_retry = 0x40000;
constexpr uint32_t invalidate_l2_ptes = 0x80000;
constexpr uint32_t invalidate_l2_pde0 = 0x100000;
constexpr uint32_t invalidate_l2_pde1 = 0x200000;
constexpr uint32_t invalidate_l2_pde2 = 0x400000;
constexpr uint32_t invalidate_l1_ptes = 0x800000;
constexpr uint32_t invalidate_flush_type_shift = 16;
// MMVM_L2_BANK_SELECT_RESERVED_CID2 bit 25: RESERVED_CACHE_PRIVATE_INVALIDATION.
constexpr uint32_t private_invalidation = 1u << 25;
} // namespace vmhub_field

} // namespace rx7900::hardware

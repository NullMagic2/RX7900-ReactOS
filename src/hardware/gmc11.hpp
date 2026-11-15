/*
Module: rx7900.hardware.gmc11
Purpose: Initialize the GMC11 GART and the MMHUB3.0/GFXHUB3.0 VM hubs as Linux does.
Created: 2026-10-21
Architecture: One table-driven hub sequence over vmhub_layout; Nbio HDP flushes,
Manager leases and the shared GFX11 PTE encoder are reused.
*/
#pragma once

#include "nbio.hpp"
#include "vmhub_layout.hpp"
#include "page_table.hpp"

namespace rx7900::hardware {

// gmc_v11_0_mc_init default gart_size and amdgpu_gart_init table size.
constexpr uint64_t gart_bytes = 512ULL << 20;
constexpr uint32_t gart_table_bytes = static_cast<uint32_t>(gart_bytes / 4096 * 8);
// Linux "use register 17 for GART" in gmc_v11_0_flush_gpu_tlb.
constexpr uint32_t gart_invalidate_engine = 17;

enum class VmHub { mmhub, gfxhub };

enum class GmcStatus {
    ready,
    prepared,
    enabled,
    disabled,
    flushed,
    mapped,
    released,
    unavailable,
    unsupported_ip,
    invalid_memory,
    invalid_argument,
    busy,
    mmio_failure,
    timeout,
    quarantined
};

// amdgpu_gmc_gart_location with AMDGPU_GART_PLACEMENT_HIGH: the top of the 47-bit
// MC space below AMDGPU_GMC_HOLE_START, aligned down to 4 GiB.
struct GartLayout final {
    uint64_t start;
    uint64_t end;
};

bool compute_gart_layout(const VramLocation& vram, GartLayout& output) noexcept;

// Only validate_gmc_discovery creates a valid token: unique MMHUB instance0 3.0.0
// and GC instance0 11.0.0, every used segment0 register inside BAR5.
class GmcRegisters final {
public:
    GmcRegisters() noexcept = default;
    bool valid() const noexcept { return mmhub_.valid() && gc_.valid() && aperture_bytes_ != 0; }
    uint32_t base(VmHub hub) const noexcept {
        return hub == VmHub::mmhub ? mmhub_.segment(0) : gc_.segment(0);
    }
    uint32_t aperture_bytes() const noexcept { return aperture_bytes_; }

private:
    friend DiscoveryStatus validate_gmc_discovery(ByteView, uint32_t, GmcRegisters&) noexcept;
    DiscoveredIp mmhub_{};
    DiscoveredIp gc_{};
    uint32_t aperture_bytes_ = 0;
};

DiscoveryStatus validate_gmc_discovery(ByteView binary, uint32_t register_aperture_bytes,
                                       GmcRegisters& output) noexcept;

struct GmcPolicy final {
    // Linux adev->usec_timeout polls with 1 us delays for semaphore and ACK.
    uint32_t poll_iterations;
};

struct GmcRecovery final {
    void* context;
    bool (*confirm_reset_quiescence)(void*) noexcept;
};

struct GmcReport final {
    GmcStatus status = GmcStatus::unavailable;
    GartLayout gart{};
    uint64_t table_mc = 0;
    uint64_t page_directory = 0;
    uint64_t scratch_physical = 0;
    uint64_t dummy_page_bus = 0;
    bool mmhub_enabled = false;
    bool gfxhub_enabled = false;
    bool semaphore_timeout = false;
    uint32_t flushes = 0;
    bool memory_retained = false;
    // Completed invalidations issued through MES (flush_tlb_firmware).
    uint32_t firmware_flushes = 0;
};

// gfxhub_v3_0/mmhub_v3_0 get_invalidate_req (identical field layout on both
// hubs): PER_VMID_INVALIDATE_REQ = 1 << vmid, FLUSH_TYPE, INVALIDATE_L2_PTES,
// L2_PDE0/1/2 and L1_PTES, CLEAR_PROTECTION_FAULT_STATUS_ADDR 0. vmid 0..15,
// flush_type 0..gmc_max_flush_type (Linux uses 0 legacy, 1 lightweight, 2
// heavyweight). Callers validate the ranges; out-of-range bits are masked.
constexpr uint32_t gmc_max_flush_type = 7;
uint32_t gmc11_invalidate_request(uint32_t vmid, uint32_t flush_type) noexcept;

// mes_v11_api_def.h union MESAPI__MISC: 64-dword frame, MES_API_TYPE_SCHEDULER,
// MES_SCH_API_MISC, api_status at byte 8 and the WAIT_REG_MEM payload at byte 24
// (offsets measured from the pinned header; the size equals mes11_api_frame_bytes).
constexpr uint32_t gmc_mes_frame_bytes = 256;
constexpr uint32_t gmc_mes_misc_api_status_offset = 8;

// amdgpu_mes_reg_write_reg_wait -> mes_v11_0_misc_op(MES_MISC_OP_WRM_REG_WR_WAIT):
// opcode MESAPI_MISC__WAIT_REG_MEM, op WRM_OPERATION__WR_WAIT_WR_REG, reference,
// mask, reg_offset1 = reg0 (written with reference) and reg_offset2 = reg1
// (waited on with mask). reg0/reg1 are absolute dword register offsets
// (SOC15_REG_OFFSET). Every other byte is zero; the submitter fills api_status.
// False (nothing written) when frame is smaller than gmc_mes_frame_bytes.
bool encode_mes_reg_write_reg_wait(uint32_t reg0, uint32_t reg1, uint32_t reference,
                                   uint32_t mask, sdma::CommandBuffer frame) noexcept;

// The MES path of gmc_v11_0_flush_gpu_tlb (amdgpu_gmc_fw_reg_write_reg_wait once
// adev->mes.ring[0].sched.ready; Linux v6.12 never uses the KIQ after MES is up).
// submit sends one MESAPI frame of gmc_mes_frame_bytes through the MES scheduler
// ring (mes_v11_0_submit_pkt_and_poll_completion: the submitter places the API
// completion fence at api_status_offset, commits and polls) and returns true only
// when MES reported completion. Serialized PASSIVE_LEVEL use with Mes11.
struct GmcFirmwareInvalidation final {
    void* context;
    bool (*submit)(void*, ByteView frame, uint32_t api_status_offset) noexcept;
};

// gmc_v11_0_emit_flush_gpu_tlb for a kernel ring that owns invalidation engine
// engine (amdgpu_gmc_allocate_vm_inv_eng) and a driver-grabbed VMID: semaphore
// wait (MMHUB only), CONTEXT[vmid] PAGE_TABLE_BASE_ADDR_LO32/HI32 = pd_addr
// (amdgpu_gmc_pd_addr), REQ write with ACK wait, semaphore release (MMHUB only).
// Register fields are absolute dword offsets; the ring owner encodes them with
// its engine's WAIT_REG_MEM/WRITE_DATA packets. MES user queues never use this
// (MES programs their VMID itself).
enum class GmcRingOpKind : uint32_t { reg_wait, write, write_wait };

struct GmcRingOp final {
    GmcRingOpKind kind;
    uint32_t reg0;
    uint32_t reg1;
    uint32_t value;
    uint32_t mask;
};

struct GmcRingFlush final {
    GmcRingOp ops[5];
    uint32_t count;
};

// False for an invalid token, vmid above 15, engine above 17 or a pd_addr with
// bits outside the 48-bit PDE address plus low flag field; output then unchanged.
bool gmc11_emit_flush_gpu_tlb(const GmcRegisters& registers, VmHub hub, uint32_t vmid,
                              uint64_t pd_addr, uint32_t engine, GmcRingFlush& output) noexcept;

// Serialized PASSIVE_LEVEL use by the exclusive device owner. The GART table and
// scratch page are local-VRAM allocations (CPU-mapped uncached for the table);
// the dummy page is a pinned system page DMA bus address owned by the native
// caller for the device lifetime. Nbio, Manager and the mapping outlive this.
// After any hub was enabled, the table and scratch page stay leased until a
// confirmed reset: hubs may walk them at any time.
class Gmc11 final {
public:
    Gmc11() noexcept = default;
    Gmc11(const Gmc11&) = delete;
    Gmc11& operator=(const Gmc11&) = delete;

    GmcStatus initialize(GmcRegisters registers, RegisterIo io, Nbio& nbio, VramLocation vram,
                         GmcPolicy policy, GmcRecovery recovery) noexcept;

    // amdgpu_gart_table_vram_alloc, amdgpu_device_mem_scratch_init and the
    // amdgpu_gart_unbind entry pattern (dummy page, no flags) for every entry.
    GmcStatus prepare_gart(memory::Manager& manager, memory::Handle table,
                           memory::Handle scratch, uint64_t dummy_page_bus) noexcept;

    // gmc_v11_0_gart_enable: MMHUB gart_enable, HDP flush, fault defaults on
    // (amdgpu_vm_fault_stop default NEVER) and the GART VMID0 TLB flush.
    GmcStatus enable_mmhub() noexcept;
    // gfx_v11_0_gfxhub_enable; only after RLC autoload completed (gfx powered on).
    GmcStatus enable_gfxhub() noexcept;
    // gfxhub_v3_0_gart_disable (gfx_v11_0_hw_fini) and mmhub_v3_0_gart_disable
    // (gmc_v11_0_hw_fini): all 16 contexts off, L1 TLB and advanced driver model
    // off, L2 cache off and L2_CNTL3 cleared. Leases stay retained: in-flight
    // translations are only ruled out by a later confirmed reset. Allowed from
    // the quarantined state so a fault still stops the hub.
    GmcStatus disable_hub(VmHub hub) noexcept;

    // gmc_v11_0_flush_gpu_tlb register path (before KIQ/MES exist), engine 17:
    // HDP flush, MMHUB semaphore, REQ, ACK bit vmid, semaphore release, MMHUB
    // private invalidation. flush_type 0 keeps the VMID0 GART behaviour; vmid
    // 1..15 and flush_type 1..7 serve process VMs. A missing ACK quarantines.
    // GFXHUB register access needs GFX powered: with GFXOFF allowed Linux uses
    // the MES path instead (see flush_tlb_firmware).
    GmcStatus flush_tlb(VmHub hub, uint32_t vmid, uint32_t flush_type = 0) noexcept;

    // The MES branch of gmc_v11_0_flush_gpu_tlb: HDP flush, then one
    // amdgpu_mes_reg_write_reg_wait(REQ17, ACK17, get_invalidate_req, 1 << vmid);
    // no semaphore and no private MMHUB invalidation, exactly as Linux. Linux only
    // logs a failed MES frame; here it quarantines (translation state unknown).
    GmcStatus flush_tlb_firmware(VmHub hub, uint32_t vmid, uint32_t flush_type,
                                 const GmcFirmwareInvalidation& firmware) noexcept;

    // amdgpu_gart_bind: encode system-page PTEs at a GART offset with the shared
    // encoder, publish through HDP and flush every enabled hub.
    // gmc_v11_0_flush_gpu_tlb routing: once the MES ring is ready, Linux sends
    // every engine-17 flush (both hubs) through amdgpu_mes_reg_write_reg_wait;
    // before that, and again after MES stops, the register path. A null submit
    // selects the register path.
    void set_firmware_invalidation(GmcFirmwareInvalidation firmware) noexcept { firmware_ = firmware; }
    GmcStatus map_system_pages(uint64_t gart_offset, ByteView dma_page_addresses,
                               memory::Access access) noexcept;
    // amdgpu_gart_unbind: point the entries back at the dummy page (no flags) and
    // invalidate. Only after confirmed reset or engine retirement. Returns flushed.
    GmcStatus unmap_system_pages(uint64_t gart_offset, uint64_t pages) noexcept;
    // Resume after power loss (gmc_v11_0_resume -> gart_enable): the VRAM table
    // lost its contents, so before amdgpu_gtt_mgr_recover rebinds every live GTT
    // allocation through map_system_pages, every entry is pointed at the dummy
    // page again (the prepare_gart pattern) and published with an HDP flush.
    // Linux leaves unbound entries as whatever VRAM holds after S3; writing the
    // dummy page instead is a fail-closed divergence. Both hubs must be disabled
    // and the memory retained; returns prepared.
    GmcStatus reset_gart_table() noexcept;
    // GART aperture in the VMID0 MC space (valid after prepare_gart).
    GartLayout gart() const noexcept { return report_.gart; }

    GmcStatus release_after_reset() noexcept;
    GmcReport report() const noexcept { return report_; }
    // The discovery token initialize bound (kernel-ring VM flush encoding).
    const GmcRegisters& registers() const noexcept { return registers_; }

private:
    bool read(VmHub hub, uint32_t reg, uint32_t& value) noexcept;
    bool write(VmHub hub, uint32_t reg, uint32_t value) noexcept;
    bool modify(VmHub hub, uint32_t reg, uint32_t clear, uint32_t set) noexcept;
    GmcStatus gart_enable(VmHub hub) noexcept;
    GmcStatus fault_defaults(VmHub hub) noexcept;
    GmcStatus invalidate_gart() noexcept;
    GmcStatus finish(GmcStatus status) noexcept;
    const VmHubLayout& layout(VmHub hub) const noexcept {
        return hub == VmHub::mmhub ? mmhub_3_0_0_layout : gfxhub_3_0_layout;
    }

    GmcRegisters registers_{};
    RegisterIo io_{};
    Nbio* nbio_ = nullptr;
    VramLocation vram_{};
    GmcPolicy policy_{};
    GmcRecovery recovery_{};
    GmcReport report_{};
    memory::Manager* manager_ = nullptr;
    memory::Handle table_{};
    memory::Handle scratch_{};
    uint8_t* table_cpu_ = nullptr;
    bool initialized_ = false;
    bool faulted_ = false;
    GmcFirmwareInvalidation firmware_{};
};

} // namespace rx7900::hardware

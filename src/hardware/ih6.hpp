/*
Module: rx7900.hardware.ih6
Purpose: Initialize, enable, disable and service the OSSSYS 6.0.0 (IH6) rings.
Created: 2026-10-21
Architecture: Discovery-validated OSSSYS token, Nbio for NBIO4.3 IH control and
doorbell range, Manager leases on local-VRAM rings; entry decoding stays in
src/acceleration/interrupt.
Copyright 2021 Advanced Micro Devices, Inc.; Copyright 2014 Advanced Micro Devices, Inc.
Adapted Linux v6.12 ih_v6_0.c, amdgpu_ih.h, amdgpu_doorbell.h and soc15_hw_ip.h.
Notice and provenance: licenses/AMDGPU-SDMA.txt.
*/
#pragma once

#include "nbio.hpp"

namespace rx7900::hardware {

// soc15_hw_ip.h OSSSYS_HWID; discovery.hpp does not own this identifier.
constexpr uint16_t discovery_osssys_hardware_id = 40;

// Linux amdgpu_ih.h IH_RING_SIZE, used for both hardware rings on Navi31.
constexpr uint32_t ih6_default_ring_bytes = 256 * 1024;
// IH_RB_WPTR.OFFSET covers bits 17:2, so a ring larger than 256 KiB cannot be
// addressed. The lower bound is one 4 KiB page (Linux BO alignment).
constexpr uint32_t ih6_maximum_ring_bytes = 256 * 1024;
constexpr uint32_t ih6_minimum_ring_bytes = 4096;
constexpr uint32_t ih6_ring_alignment = 4096;
constexpr uint32_t ih6_entry_bytes = 32;
// One dword: IH writes IH_RB_WPTR for ring 0 here (IH_RB_WPTR_ADDR_LO bits 31:2).
constexpr uint32_t ih6_writeback_bytes = 4;
constexpr uint32_t ih6_writeback_alignment = 4;
// soc21_init_doorbell_index: AMDGPU_NAVI10_DOORBELL_IH (0x178) in 64-bit units,
// shifted to dword doorbell indices exactly as ih_v6_0_sw_init.
constexpr uint32_t ih6_ring0_doorbell_index = 0x178u << 1;
constexpr uint32_t ih6_ring1_doorbell_index = (0x178u + 1) << 1;
// ih_v6_0_irq_disable: mdelay(1) after the rings are disabled.
constexpr uint32_t ih6_disable_settle_microseconds = 1000;

enum class Ih6Ring : uint32_t {
    primary = 0, // IH ring 0: every client, MSI source, wptr writeback
    page_fault = 1 // IH ring 1: VMC (client 0xa) faults; wptr via self-interrupt
};

constexpr uint32_t ih6_ring_count = 2;

enum class Ih6Status {
    ready,
    enabled,
    disabled,
    released,
    overflow,
    unavailable,
    unsupported_ip,
    invalid_argument,
    invalid_memory,
    mmio_failure,
    platform_failure,
    busy,
    quarantined
};

// Only validate_ih6_discovery creates a valid token: unique OSSSYS instance0
// 6.0.0 whose highest register used here (IH_CHICKEN 0x18d) fits BAR5.
class Ih6Registers final {
public:
    Ih6Registers() noexcept = default;
    bool valid() const noexcept { return ip_.valid() && aperture_bytes_ != 0; }
    uint32_t segment0_dword_base() const noexcept { return ip_.segment(0); }
    uint32_t aperture_bytes() const noexcept { return aperture_bytes_; }

private:
    friend DiscoveryStatus validate_ih6_discovery(ByteView, uint32_t, Ih6Registers&) noexcept;
    DiscoveredIp ip_{};
    uint32_t aperture_bytes_ = 0;
};

DiscoveryStatus validate_ih6_discovery(ByteView binary, uint32_t register_aperture_bytes,
                                       Ih6Registers& output) noexcept;

struct Ih6Config final {
    // Power of two in [ih6_minimum_ring_bytes, ih6_maximum_ring_bytes]; Linux
    // uses ih6_default_ring_bytes for both rings.
    uint32_t ring_bytes;
    // IH_RB_CNTL.RPTR_REARM on ring 0 is Linux adev->irq.msi_enabled.
    bool msi_enabled;
    // Linux adev->dummy_page_addr for BIF_BX0_INTERRUPT_CNTL2 (bits 39:8): the
    // DMA bus address of a page the native DMA owner keeps mapped for the
    // device lifetime. Nonzero, 4 KiB aligned and below 2^40. It is only read
    // by BIF dummy reads, which NBIO disables while MSI is in use.
    uint64_t dummy_page_bus_address;
};

// Native services that this protocol cannot provide itself. All callbacks run
// at PASSIVE_LEVEL except write_doorbell32, which set_rptr may call at the
// interrupt-servicing IRQL chosen by the owner. context outlives this object.
struct Ih6Platform final {
    void* context;
    // Linux pci_set_master. Required; false quarantines the protocol.
    bool (*enable_bus_master)(void*) noexcept;
    // Linux WDOORBELL32 into the mapped doorbell BAR (dword index). Null selects
    // register mode: IH_DOORBELL_RPTR.ENABLE=0, NBIO doorbell range size 0 and
    // RPTR acknowledgement through IH_RB_RPTR, which is the ih_v6_0_set_rptr
    // non-doorbell path. A non-null callback requires the doorbell aperture
    // (soc21_common_hw_init enable_doorbell_aperture) to be owned and enabled.
    bool (*write_doorbell32)(void*, uint32_t dword_index, uint32_t value) noexcept;
    // True only after reset proves IH no longer writes either ring or the
    // writeback dword. Never inferred from a failed or timed-out access.
    bool (*confirm_reset_quiescence)(void*) noexcept;
};

struct Ih6Report final {
    Ih6Status status = Ih6Status::unavailable;
    uint64_t ring_mc[ih6_ring_count]{};
    uint64_t writeback_mc = 0;
    uint32_t ring_bytes = 0;
    uint32_t rb_cntl[ih6_ring_count]{};
    uint32_t doorbell_rptr[ih6_ring_count]{};
    uint32_t doorbell_range = 0;
    uint32_t interrupt_cntl = 0;
    uint32_t rptr[ih6_ring_count]{};
    uint32_t overflows = 0;
    bool doorbell_mode = false;
    bool enabled = false;
    bool memory_retained = false;
    bool clock_gated = false;
};

// Serialized PASSIVE_LEVEL use by the exclusive device owner for initialize,
// hw_init, set_clockgating, disable, release and release_after_reset: these use
// Nbio's shared SMN pair and Manager. The ISR/DPC must be disconnected or
// excluded by the owner's interrupt lock while they run.
//
// read_wptr, set_rptr, record_self_interrupt and ring_view touch only OSSSYS
// MMIO, already retained CPU mappings and the doorbell callback, never Nbio or
// Manager; the owner may call them at DIRQL/DISPATCH_LEVEL under its interrupt
// lock while the protocol is enabled.
//
// Linux allocates both rings in GTT (system memory through VMID0). This project
// places them, and the writeback dword, in CPU-visible local VRAM (uncached BAR0
// window) and programs their MC addresses; MC_SPACE stays 4 (VMID0) because
// VMID0 maps the FB aperture directly. Any uncertain register store after the
// first hardware write quarantines the protocol and keeps every lease until
// release_after_reset confirms the IH engine is quiescent.
class Ih6 final {
public:
    Ih6() noexcept = default;
    Ih6(const Ih6&) = delete;
    Ih6& operator=(const Ih6&) = delete;

    // nbio_registers is the token nbio was initialized with; it supplies the
    // NBIF segment 2 (BIF_BX0_INTERRUPT_CNTL) and 3 (S2A doorbell) bases.
    Ih6Status initialize(Ih6Registers registers, NbioRegisters nbio_registers, RegisterIo io,
                         Nbio& nbio, VramLocation vram, Ih6Config config,
                         Ih6Platform platform) noexcept;

    // ih_v6_0_hw_init (ih_v6_0_irq_init) for ring 0 and ring 1 (Navi31 is not
    // an APU, so Linux always programs ring 1). From ready it retains one lease
    // on each local-VRAM allocation (rings CPU-visible >= ring_bytes, 4 KiB
    // aligned; writeback CPU-visible, 4-byte aligned), zeroes them and flushes
    // HDP. Leases are unchanged when validation fails before any register write.
    Ih6Status hw_init(memory::Manager& manager, memory::Handle ring0, memory::Handle ring1,
                      memory::Handle writeback) noexcept;
    // ih_v6_0_resume after a confirmed disable: zero the retained rings and
    // writeback again, flush HDP and repeat ih_v6_0_irq_init.
    Ih6Status resume() noexcept;

    // ih_v6_0_hw_fini (ih_v6_0_irq_disable): self-interrupt wptr update off,
    // both rings disabled, RPTR/WPTR zeroed, then a 1 ms settle delay. Leases
    // stay retained until release or release_after_reset.
    Ih6Status disable() noexcept;

    // ih_v6_0_update_clockgating_state with AMD_CG_SUPPORT_IH_CG (present in
    // the soc21 IP 11.0.0 cg_flags). Linux calls it from late init with gate and
    // before fini/suspend with ungate; it is not part of hw_init.
    Ih6Status set_clockgating(bool gate) noexcept;

    // amdgpu_ih_ring_fini after a confirmed disable: unretain all leases.
    Ih6Status release() noexcept;
    // After confirmed reset quiescence from any state: unretain all leases.
    Ih6Status release_after_reset() noexcept;

    // ih_v6_0_get_wptr: ring 0 reads the writeback dword, ring 1 the latest
    // self-interrupt value. A confirmed overflow follows Linux (RPTR moves to
    // wptr + 32, WPTR_OVERFLOW_CLEAR pulsed) but returns overflow so the caller
    // latches event loss instead of silently continuing. wptr is a byte offset.
    Ih6Status read_wptr(Ih6Ring ring, uint32_t& wptr) noexcept;
    // ih_v6_0_set_rptr: rptr is a 32-byte-aligned byte offset below ring_bytes.
    Ih6Status set_rptr(Ih6Ring ring, uint32_t rptr) noexcept;
    // ih_v6_0_self_irq (SOC21_IH_CLIENTID_IH, source 0) with the decoded IV ring
    // id and source data word 0. Only ring id 1 updates state; returns whether it did.
    bool record_self_interrupt(uint32_t ring_id, uint32_t source_data0) noexcept;
    // Borrowed CPU view of an enabled ring for sdma::decode_interrupt; it is
    // valid only until disable, release or quarantine.
    bool ring_view(Ih6Ring ring, ByteView& view) const noexcept;
    uint32_t rptr(Ih6Ring ring) const noexcept;

    Ih6Report report() const noexcept { return report_; }

private:
    enum class Phase { uninitialized, ready, enabled, disabled, faulted };

    struct RingRegisters final {
        uint32_t base;
        uint32_t base_hi;
        uint32_t cntl;
        uint32_t wptr;
        uint32_t rptr;
        uint32_t doorbell_rptr;
    };

    static RingRegisters ring_registers(uint32_t index) noexcept;
    bool read(uint32_t reg, uint32_t& value) noexcept;
    bool write(uint32_t reg, uint32_t value) noexcept;
    bool modify(uint32_t reg, uint32_t clear, uint32_t set) noexcept;
    bool toggle_ring(uint32_t index, bool enable) noexcept;
    bool toggle_interrupts(bool enable) noexcept;
    bool enable_ring(uint32_t index) noexcept;
    bool force_wptr_update(bool enable) noexcept;
    bool nbio_ih_control() noexcept;
    bool nbio_doorbell_range() noexcept;
    bool clear_memory() noexcept;
    Ih6Status irq_init() noexcept;
    Ih6Status quarantine(Ih6Status reason) noexcept;
    bool retain_memory(memory::Manager& manager, memory::Handle ring0, memory::Handle ring1,
                       memory::Handle writeback) noexcept;
    void release_leases() noexcept;

    Ih6Registers registers_{};
    NbioRegisters nbio_registers_{};
    RegisterIo io_{};
    Nbio* nbio_ = nullptr;
    VramLocation vram_{};
    Ih6Config config_{};
    Ih6Platform platform_{};
    Ih6Report report_{};
    Phase phase_ = Phase::uninitialized;
    memory::Manager* manager_ = nullptr;
    memory::Handle handles_[ih6_ring_count + 1]{};
    uint8_t* ring_cpu_[ih6_ring_count]{};
    uint8_t* writeback_cpu_ = nullptr;
    uint32_t ring1_wptr_ = 0;
};

} // namespace rx7900::hardware

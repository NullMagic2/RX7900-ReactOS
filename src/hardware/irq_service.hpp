/*
Module: rx7900.hardware.irq_service
Purpose: Service the Navi31 IH6 rings (amdgpu_ih_process) and enable the sources
Linux enables by the end of initialization.
Created: 2026-10-21
Architecture: Consumes the enabled Ih6 owner (wptr/rptr/ring views only), reuses
sdma::decode_interrupt for amdgpu_ih_decode_iv_helper and the shared GMC/SMU
discovery tokens; native code supplies MMIO, the interrupt lock and the DPC.
Copyright 2014, 2018, 2021, 2022 Advanced Micro Devices, Inc.
Adapted Linux v6.12 amdgpu_irq.c/.h, amdgpu_ih.c/.h, ih_v6_0.c, gfx_v11_0.c,
sdma_v6_0.c, gmc_v11_0.c, amdgpu_gmc.c, gfxhub_v3_0.c, mmhub_v3_0.c, amdgpu_fence.c,
smu_v13_0.c, smu_cmn.h, amdgpu_mes_ctx.h, soc15_ih_clientid.h, irqsrcs_gfx_11_0_0.h,
irqsrcs_vmc_1_0.h, irqsrcs_nbif_7_4.h and the gc_11_0_0, mmhub_3_0_0 and
mp_13_0_0 register headers. Notice and provenance: licenses/AMDGPU-SDMA.txt.
*/
#pragma once

#include "gmc11.hpp"
#include "ih6.hpp"
#include "interrupt.hpp"
#include "smu13.hpp"

namespace rx7900::hardware {

// AMDGPU_IH_MAX_NUM_IVS: Linux handles at most this many vectors per restart.
// A drain decodes at most this many entries from each ring per call.
constexpr uint32_t irq_drain_batch_entries = 32;
// amdgpu_ih_process re-reads WPTR after acknowledging and restarts while it
// moved. The ISR bounds that loop; a later IV re-asserts the line.
constexpr uint32_t irq_isr_max_passes = 2;
// SOC15_IH_CLIENTID_MAX (AMDGPU_IRQ_CLIENTID_MAX).
constexpr uint32_t irq_client_count = 0x20;
// gfx_v11_0_sw_init for GC 11.0.0: ME0 has two pipe registers (RING0/RING1),
// MEC1/MEC2 four pipes each; the IV ring_id queue field is three bits wide.
constexpr uint32_t irq_gfx_pipes = 2;
constexpr uint32_t irq_mec_count = 2;
constexpr uint32_t irq_mec_pipes = 4;
constexpr uint32_t irq_mec_queues = 8;
constexpr uint32_t irq_sdma_instances = 2;
// amdgpu_gmc_set_vm_fault_masks walks 16 VM contexts per hub.
constexpr uint32_t irq_vm_contexts = 16;
// SMU_IH_INTERRUPT_CONTEXT_ID_* values are below this bound (smu_cmn.h).
constexpr uint32_t irq_smu_context_count = 16;
// IV ring_id ME field values gfx_v11_0_handle_priv_fault accepts: ME0 (gfx),
// MEC1 and MEC2; Linux BUG()s on ME 3, which is counted here instead.
constexpr uint32_t irq_cp_me_count = 3;
// VM faults retained for the drain consumer: two full drain batches (ring 0 and
// ring 1), so a consumer called after every drain never misses one.
constexpr uint32_t irq_vm_fault_log_entries = 2 * irq_drain_batch_entries;

enum class IrqStatus {
    detached,
    attached,
    enabled,
    unavailable,
    unsupported_ip,
    invalid_argument,
    mmio_failure,
    busy,
    quarantined,
    // Detached from disabled IH rings across a system suspend; counters kept.
    suspended
};

// Outcome of one dispatched vector; each has a sequence number in IrqReport.
enum class IrqSource : uint32_t {
    self_interrupt,  // SOC21_IH_CLIENTID_IH source 0 (ih_v6_0_self_irq)
    gfx_eop,         // GRBM_CP CP_EOP_INTERRUPT, ME0
    compute_eop,     // GRBM_CP CP_EOP_INTERRUPT, MEC1/MEC2
    mes_eop,         // CP_EOP_INTERRUPT carrying AMDGPU_FENCE_MES_QUEUE_FLAG
    sdma_trap,       // GFX SDMA_TRAP, kernel queue 0
    sdma_mes_trap,   // SDMA_TRAP carrying AMDGPU_FENCE_MES_QUEUE_FLAG
    cp_priv_reg,     // GRBM_CP CP_PRIV_REG_FAULT
    cp_priv_instr,   // GRBM_CP CP_PRIV_INSTR_FAULT
    cp_bad_opcode,   // GRBM_CP CP_BAD_OPCODE_ERROR
    gfxhub_fault,    // GFX UTCL2_FAULT (redirected to ring 1 by ih_v6_0)
    mmhub_fault,     // VMC VM_FAULT
    thermal_high,    // THM DIG_THERM_L2H: Linux schedules swctf_delayed_work
    thermal_low,     // THM DIG_THERM_H2L
    thermal_ctf,     // ROM_SMUIO GPIO19: hardware CTF, Linux orderly_poweroff
    smu_to_driver,   // MP1 SMU_IH_INTERRUPT_ID_TO_DRIVER (acknowledged)
    ras,             // DF ECC, BIF ATHUB ErrEvent, RLC GC FED (RAS-gated in Linux)
    unhandled,       // registered source whose IV fields match no kernel ring
    unknown,         // unregistered client/source (Linux forwards to amdkfd)
    invalid,         // client_id >= SOC15_IH_CLIENTID_MAX
    count
};

constexpr uint32_t irq_source_count = static_cast<uint32_t>(IrqSource::count);

// Only validate_irq_discovery creates a valid token: GMC (MMHUB 3.0.0 and GC
// 11.0.0) and MP1 13.0.0 tokens whose highest register used here (GC
// CP_ME1_PIPE3_INT_CNTL 0x1e28, MMVM_CONTEXT15_CNTL 0x74f, MP1_SMN_IH_SW_INT_CTRL
// 0x2c3) fits BAR5.
class IrqRegisters final {
public:
    IrqRegisters() noexcept = default;
    bool valid() const noexcept { return gmc_.valid() && smu_.valid() && aperture_bytes_ != 0; }
    uint32_t gc_base() const noexcept { return gmc_.base(VmHub::gfxhub); }
    uint32_t mmhub_base() const noexcept { return gmc_.base(VmHub::mmhub); }
    uint32_t mp1_base() const noexcept { return smu_.segment0_dword_base(); }
    uint32_t aperture_bytes() const noexcept { return aperture_bytes_; }

private:
    friend DiscoveryStatus validate_irq_discovery(ByteView, uint32_t, IrqRegisters&) noexcept;
    GmcRegisters gmc_{};
    SmuRegisters smu_{};
    uint32_t aperture_bytes_ = 0;
};

DiscoveryStatus validate_irq_discovery(ByteView binary, uint32_t register_aperture_bytes,
                                       IrqRegisters& output) noexcept;

// The kernel rings whose fence interrupts amdgpu_fence_driver_hw_init enables
// (amdgpu_irq_get on ring->fence_drv.irq_src/irq_type). Bit n of gfx_pipes is
// ME0 pipe n, of compute_pipes MEC1 pipe n, of sdma_instances SDMA instance n.
// The caller derives them from the rings that were actually started.
struct IrqSourcePlan final {
    uint32_t gfx_pipes;
    uint32_t compute_pipes;
    uint32_t sdma_instances;
};

struct IrqCpFault final {
    IrqSource source = IrqSource::count;
    uint32_t me = 0;
    uint32_t pipe = 0;
    uint32_t queue = 0;
    uint32_t vmid = 0;
    uint32_t pasid = 0;
    uint32_t sequence = 0;
};

// gmc_v11_0_process_interrupt. address is src_data[0] << 12 | (src_data[1] & 0xf)
// << 44. status is VM_L2_PROTECTION_FAULT_STATUS of the hub selected by the
// client (VMC: MMHUB, otherwise GFXHUB), read before CLEAR_PROTECTION_FAULT_STATUS_ADDR
// is written; cache_hub is the hub amdgpu_vm_update_fault_cache records (Linux
// selects it from vmid_src, not from the client).
struct IrqVmFault final {
    uint64_t address = 0;
    uint32_t status = 0;
    uint32_t client = 0;
    uint32_t source = 0;
    uint32_t ring_id = 0;
    uint32_t vmid = 0;
    uint32_t pasid = 0;
    uint32_t sequence = 0;
    VmHub register_hub = VmHub::gfxhub;
    VmHub cache_hub = VmHub::gfxhub;
    bool status_read = false;
    bool status_cleared = false;
    // Decoded from IH ring 1 (the page-fault ring) rather than ring 0.
    bool page_fault_ring = false;
};

// Counters; the only callback is the optional IrqConsumer of drain. Sequence
// numbers wrap modulo 2^32: a consumer compares against the value it last observed.
struct IrqReport final {
    IrqStatus status = IrqStatus::detached;
    bool armed = false;
    bool sources_enabled = false;
    // ISR claim statistics.
    uint64_t isr_calls = 0;
    uint64_t isr_claimed = 0;
    // Ring 0 bytes acknowledged by the ISR but not yet dispatched.
    uint32_t pending_bytes = 0;
    uint64_t dispatched[ih6_ring_count]{};
    // IH overflows observed through Ih6::read_wptr and vectors discarded by an
    // overflow, by the ring 0 backlog bound or by a ring 1 overflow. Any loss
    // latches events_lost: consumers must re-poll every fence and fault owner.
    uint32_t overflows = 0;
    uint64_t entries_lost = 0;
    bool events_lost = false;
    uint32_t sequence[irq_source_count]{};
    // Fence progress hints (Linux amdgpu_fence_process targets), per ring.
    uint32_t gfx_eop[irq_gfx_pipes]{};
    uint32_t compute_eop[irq_mec_count][irq_mec_pipes][irq_mec_queues]{};
    uint32_t sdma_trap[irq_sdma_instances]{};
    // src_data[0] & AMDGPU_FENCE_MES_QUEUE_ID_MASK of the last MES-queue fence.
    uint32_t last_mes_queue = 0;
    // CP faults (Linux drm_sched_fault): the queue must be quarantined.
    IrqCpFault last_cp_fault{};
    bool queue_fault = false;
    // Per IV ring_id (ME, pipe, queue): CP faults seen and the source of the
    // last one, so a consumer can attribute every fault to its kernel ring
    // (gfx_v11_0_handle_priv_fault). ME 3 is counted in cp_fault_invalid_me.
    uint32_t cp_fault_count[irq_cp_me_count][irq_mec_pipes][irq_mec_queues]{};
    IrqSource cp_fault_source[irq_cp_me_count][irq_mec_pipes][irq_mec_queues]{};
    uint32_t cp_fault_invalid_me = 0;
    IrqVmFault last_vm_fault{};
    bool vm_fault = false;
    // ih_v6_0_self_irq: last ring 1 WPTR delivered through ring 0.
    uint32_t ring1_wptr = 0;
    // THM L2H (swctf work), hardware CTF and MP1 context ids awaiting a
    // PASSIVE_LEVEL owner (Linux schedule_work / smu message / THM update).
    bool thermal_work = false;
    bool hardware_ctf = false;
    uint32_t smu_last_context = 0;
    uint32_t smu_context[irq_smu_context_count]{};
    uint32_t smu_ack_failures = 0;
    // The IrqSource::ras vectors by origin: DF client source 0 (the GMC
    // ecc_irq, amdgpu_umc_process_ecc_irq), BIF ERREVENT_ATHUB (the fatal
    // error interrupt amdgpu_ras_global_ras_isr handles) and GFX RLC_GC_FED
    // (gfx.ras is NULL on GC 11.0.0, so Linux only acknowledges it).
    uint32_t ras_umc_ecc = 0;
    uint32_t ras_athub = 0;
    uint32_t ras_gc_fed = 0;
    uint32_t unknown_by_client[irq_client_count]{};
    // Values written by enable_sources (Linux amdgpu_irq_get state).
    uint32_t cp_int_cntl_ring[irq_gfx_pipes]{};
    uint32_t cp_me1_pipe_int_cntl[irq_mec_pipes]{};
    uint32_t sdma_cntl[irq_sdma_instances]{};
    uint32_t vm_context_cntl[2][irq_vm_contexts]{}; // [0] MMHUB, [1] GFXHUB
};

// Every VM fault in dispatch order: entry total % irq_vm_fault_log_entries holds
// fault number total (1-based sequence of IrqVmFault::sequence order across both
// hubs). A consumer that last saw total t reads faults t + 1 .. total; more than
// irq_vm_fault_log_entries new faults means the oldest were overwritten.
struct IrqVmFaultLog final {
    IrqVmFault entries[irq_vm_fault_log_entries]{};
    uint32_t total = 0;
};

// What a drain consumer sees. report and vm_faults stay valid only during the
// call. Only fields written by dispatch are stable there (sequence, the EOP/trap
// counters, CP fault counters, VM fault state and log, thermal/SMU state); the
// ISR may concurrently update isr_calls, isr_claimed, pending_bytes, overflows,
// entries_lost and events_lost, which a consumer must not read. events_lost is
// true when vectors were discarded since the previous drain that ran: every
// fence owner must then re-read its writeback (vectors may have been EOPs).
struct IrqDispatchView final {
    const IrqReport* report;
    const IrqVmFaultLog* vm_faults;
    bool events_lost;
};

// Called by drain after dispatch, outside the interrupt lock but while this
// drain is still the only one (no concurrent dispatch), at the drain's IRQL
// (DISPATCH_LEVEL from the DPC). It must not call drain.
struct IrqConsumer final {
    void* context;
    void (*consume)(void* context, const IrqDispatchView& view) noexcept;
};

// Runs routine(argument) with the ISR excluded (VideoPortSynchronizeExecution at
// VpMediumPriority, i.e. KeSynchronizeExecution on the connected interrupt) and
// returns its result. Called by drain at DISPATCH_LEVEL or below.
struct IrqSynchronize final {
    void* context;
    bool (*run)(void* context, bool (*routine)(void*) noexcept, void* argument) noexcept;
};

// status is attached when the drain ran (quarantined if an Ih6 acknowledgement
// failed during it), busy for a concurrent drain, otherwise why it could not run.
struct IrqDrain final {
    IrqStatus status = IrqStatus::unavailable;
    uint32_t dispatched[ih6_ring_count]{};
    // More acknowledged ring 0 vectors, an undrained ring 1 or a ring 1 WPTR
    // update remain: the caller queues another DPC.
    bool more_pending = false;
};

// Device-owned, constructed once in nonpaged storage; trivially destructible.
//
// Ring 0 is acknowledged ahead of decoding: the level-triggered (INTx) IH line
// only deasserts once RPTR reaches WPTR, so the ISR advances RPTR to WPTR and
// records the acknowledged span; the DPC then decodes it from the ring. IH may
// reuse acknowledged space, so the unread backlog is bounded to half the ring
// (older vectors are discarded and counted) and a batch decoded while the ISR
// discarded entries is dropped. Ring 1 has no interrupt of its own and follows
// amdgpu_ih_process exactly in the DPC: decode, then acknowledge.
//
// IRQL and serialization:
// - attach, enable_sources: PASSIVE_LEVEL, exclusive device owner, after the
//   Ih6 owner reports enabled; enable_sources after arm_locked so the enabled
//   sources are serviced.
// - arm_locked, disarm_locked, isr: under the interrupt lock (isr is the
//   VideoPort HwInterrupt at DIRQL). isr touches only Ih6 ring 0 WPTR/RPTR state
//   (writeback, IH_RB_WPTR/IH_RB_CNTL on overflow, the RPTR doorbell).
// - drain: DISPATCH_LEVEL or below, not reentrant (a concurrent call returns
//   busy); every Ih6 call runs inside sync.run, ring reads and dispatch outside.
//   Dispatch reads/clears VM fault status and acknowledges MP1 through MMIO.
// - report: any IRQL; consistent only when taken inside sync.run.
class IrqService final {
public:
    IrqService() noexcept = default;
    IrqService(const IrqService&) = delete;
    IrqService& operator=(const IrqService&) = delete;

    // Binds to an enabled Ih6 whose rings stay mapped for this object's
    // lifetime. io addresses BAR5 at any IRQL; ih and io.context outlive this.
    IrqStatus attach(Ih6& ih, RegisterIo io, IrqRegisters registers) noexcept;

    // amdgpu_fence_driver_hw_init then the late_init amdgpu_irq_get calls not
    // programmed by other modules: CP EOP (TIME_STAMP and GENERIC0) on the plan's
    // pipes, SDMA TRAP_ENABLE, MMHUB/GFXHUB VM fault interrupt masks, CP
    // PRIV_REG/PRIV_INSTR/OPCODE_ERROR. Read-modify-write only; once.
    IrqStatus enable_sources(IrqSourcePlan plan) noexcept;

    void arm_locked() noexcept;
    void disarm_locked() noexcept;

    // System suspend (the amdgpu_irq handler goes quiet before ih_v6_0_suspend
    // disables the rings): after disarm_locked under the interrupt lock and a
    // final drain, stop using the IH rings. busy while a drain runs, while armed
    // or while acknowledged vectors are still undispatched. Every counter and
    // sequence number continues, so FenceManager consumers see no regression.
    IrqStatus suspend() noexcept;
    // After Ih6::resume re-enabled the (zeroed, RPTR 0) rings: rebind them as
    // attach does, keeping every counter. The resumed device lost its interrupt
    // source enables, so enable_sources is allowed (and needed) again; arm_locked
    // follows under the interrupt lock.
    IrqStatus resume(Ih6& ih) noexcept;

    // amdgpu_irq_handler claim: true only when ring 0 had unacknowledged vectors
    // (or an overflow) and RPTR was advanced; false leaves a shared line to others.
    bool isr() noexcept;

    IrqDrain drain(IrqSynchronize sync) noexcept;
    // drain, then consumer.consume with the dispatched state (skipped when the
    // drain did not run or consume is null).
    IrqDrain drain(IrqSynchronize sync, IrqConsumer consumer) noexcept;

    IrqReport report() const noexcept { return report_; }
    // Same validity rules as IrqDispatchView::vm_faults outside a consumer:
    // consistent only while no drain runs.
    const IrqVmFaultLog& vm_fault_log() const noexcept { return vm_faults_; }

private:
    enum class Phase { detached, attached, faulted, suspended };

    struct LockedCall final {
        IrqService* self;
        IrqStatus status;
        bool more_pending;
        bool events_lost;
    };

    struct Batch final {
        sdma::InterruptVector vectors[irq_drain_batch_entries];
        uint32_t count;
        uint32_t start;
        uint32_t generation;
    };

    static bool collect_trampoline(void* call) noexcept;
    static bool commit_trampoline(void* call) noexcept;
    static bool finish_trampoline(void* call) noexcept;
    bool collect_locked(LockedCall& call) noexcept;
    bool commit_locked(LockedCall& call) noexcept;
    bool finish_locked(LockedCall& call) noexcept;
    void discard_oldest_locked(uint32_t bytes) noexcept;
    bool read_batch(ByteView ring, Batch& batch) const noexcept;
    void dispatch(const sdma::InterruptVector& vector, bool page_fault_ring) noexcept;
    void dispatch_eop(const sdma::InterruptVector& vector) noexcept;
    void dispatch_cp_fault(const sdma::InterruptVector& vector, IrqSource source) noexcept;
    void dispatch_sdma_trap(const sdma::InterruptVector& vector) noexcept;
    void dispatch_vm_fault(const sdma::InterruptVector& vector, bool page_fault_ring) noexcept;
    void dispatch_smu(const sdma::InterruptVector& vector) noexcept;
    void count(IrqSource source) noexcept;
    bool modify(uint32_t base, uint32_t reg, uint32_t set, uint32_t& written) noexcept;
    IrqStatus fail(IrqStatus status) noexcept;
    bool bind_rings(Ih6& ih) noexcept;

    Ih6* ih_ = nullptr;
    RegisterIo io_{};
    IrqRegisters registers_{};
    IrqReport report_{};
    IrqVmFaultLog vm_faults_{};
    Phase phase_ = Phase::detached;
    uint32_t ring_mask_ = 0;
    uint32_t backlog_limit_ = 0;
    // Ring 0 acknowledged-but-undispatched span; ISR and collect/commit only.
    uint32_t pending_start_ = 0;
    uint32_t generation_ = 0;
    bool armed_ = false;
    // Vectors lost since the last drain commit (ISR and collect/commit only).
    bool lost_unconsumed_ = false;
    // Drain state; guarded by drain_active_, set and cleared under the lock.
    bool drain_active_ = false;
    bool hardware_access_ = false;
    bool ring1_update_ = false;
    uint32_t ring1_update_wptr_ = 0;
    bool ring1_more_ = false;
    // Written only by the active drain's dispatch; published by finish_locked.
    bool dispatch_ring1_update_ = false;
    uint32_t dispatch_ring1_wptr_ = 0;
    ByteView ring_views_[ih6_ring_count]{ByteView(nullptr, 0), ByteView(nullptr, 0)};
    Batch batches_[ih6_ring_count]{};
};

} // namespace rx7900::hardware

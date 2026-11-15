/*
Module: rx7900.hardware.asic_init
Purpose: Order Navi31 discovery, NBIO/HDP, PSP13 firmware load, SMU13 setup and S3/S4.
Created: 2026-10-21
Architecture: Device-owned sequencer composing the shared protocol modules;
native code supplies MMIO, a local-VRAM Manager and immutable firmware bytes.
*/
#pragma once

#include "psp_ring.hpp"
#include "smu13.hpp"
#include "ucode.hpp"
#include "vram_discovery.hpp"
#include "atom_firmware.hpp"
#include "psp_training.hpp"
#include "gmc11.hpp"
#include "ih6.hpp"
#include "gfx11_init.hpp"
#include "sdma6_init.hpp"
#include "mes11.hpp"
#include "gfx11_queues.hpp"
#include "clock_gating.hpp"
#include "mes_selftest_vm.hpp"
#include "psp_ta.hpp"
#include "fence.hpp"
#include "gpuvm.hpp"
#include "user_queue.hpp"
#include "atom_interpreter.hpp"

namespace rx7900::hardware {

enum class AsicStep {
    inputs,
    discovery,
    nbio,
    vram_location,
    runtime_database,
    gmc,
    memory_training,
    interrupts,
    psp_bootloader,
    psp_ring,
    psp_toc,
    tmr_allocation,
    smu_firmware,
    psp_tmr,
    ip_firmware,
    rlc_autoload,
    asd,
    register_list,
    smu_start,
    smu_tables,
    smu_pptable,
    smu_btc,
    smu_allowed_features,
    smu_features,
    smu_dpm_tables,
    smu_pcie,
    smu_thermal,
    gfx,
    mes_kiq,
    kernel_queues,
    sdma,
    mes,
    late_init,
    smu_late_init,
    mes_self_test,
    clock_gating,
    ib_tests,
    complete,
    // amdgpu_device_ip_fini_early stages, reported by shutdown.
    fini_ungate,
    fini_fences,
    fini_smu,
    fini_mes,
    fini_sdma,
    fini_gfx,
    fini_psp,
    fini_ih,
    fini_gmc,
    fini_common,
    stopped,
    // explicit mode1_reset stages.
    reset_prepare,
    reset_smu,
    reset_restore,
    reset_bootloader,
    reset_memsize,
    reset_done,
    // AsicInit::suspend stages (amdgpu_device_suspend, then amdgpu_pmops_suspend_noirq).
    suspend_ungate,
    suspend_df_cstate,
    suspend_fences,
    suspend_mes,
    suspend_sdma,
    suspend_gfx,
    suspend_smu,
    suspend_psp,
    suspend_ih,
    suspend_gmc,
    suspend_common,
    suspend_reset,
    suspended,
    // AsicInit::resume stages before the shared hw_init/late_init steps above.
    resume_post,
    resume_common,
    resume_gmc,
    resume_ih,
    resume_training,
    resume_mes,
    resume_submission,
    // AsicInit::repost stages (amdgpu_device_asic_init for GC 11.0.0).
    repost_bootloader,
    repost_atom,
    repost_confirm
};

enum class AsicStatus { completed, invalid_input, unsupported, failed, quarantined, busy, released };


struct AsicInitPolicy final {
    PspWaitPolicy boot;
    PspRingPolicy ring;
    SmuPolicy smu;
    // Linux adev->usec_timeout polls and the 2.1 s MES API completion timeout.
    Mes11Policy mes;
    // Linux loads DMCUB through PSP whenever DC is enabled; DMUB start is display work.
    bool load_dmcub;
};

struct AsicRecovery final {
    void* context;
    bool (*confirm_reset_quiescence)(void*) noexcept;
};

// Linux installs its IRQ handler (amdgpu_irq_init in sw_init) before IH hw_init,
// so vectors raised during the rest of bring-up are serviced. ih_ready is called
// right after IH6 rings are enabled, at PASSIVE_LEVEL; the owner attaches and arms
// its interrupt service there. Null skips it; false fails bring-up.
struct InterruptReady final {
    void* context;
    bool (*ih_ready)(void*, Ih6&) noexcept;
};

// amdgpu_device_suspend services from the native power owner (PASSIVE_LEVEL).
// reset: the PCI callbacks of the mode-1 reset amdgpu_pmops_suspend_noirq runs
// for a dGPU entering S3 or S4 (see mode1_reset). ih_stopping (optional) is
// called right before ih_v6_0_suspend disables the IH rings: the interrupt owner
// disarms its ISR, drains and suspends its IrqService there (the Linux handler
// goes quiet at suspend_noirq; an ISR must never read a disabled ring). false
// fails the IH stage and leaves the rings enabled.
struct SuspendPlatform final {
    ResetPlatform reset;
    void* context;
    bool (*ih_stopping)(void*) noexcept;
};

struct AsicInputs final {
    RegisterIo io;
    ByteView discovery;
    // Validated psp_13_0_0_sos.bin; its TOC (4) and RL (6) parts are used here.
    ByteView sos;
    const UcodeCatalog* catalog;
    // Optional psp_13_0_0_ta.bin; only the ASD descriptor is loaded.
    ByteView ta;
    memory::Manager* vram_manager;
    // Parsed firmwareinfo of the validated ATOM image, or null when no ROM is
    // available. Linux trains only when ATOM_FIRMWARE_CAP_ENABLE_2STAGE_BIST_TRAINING
    // is set; without firmwareinfo that decision is unknown and run refuses.
    const AtomFirmwareInfo* atom;
    // Required only when training actually runs.
    PspTrainingIo training;
    // find_smu13_board_table result; required when SCPM is not active.
    ByteView smu_board_table;
    // Negotiated link in SMU encoding (smu_smc_hw_setup pcie_gen/pcie_width).
    SmuPcieLink pcie;
    // Pinned zeroed 4 KiB system page DMA bus address (amdgpu_gart_dummy_page_init),
    // owned by the native caller for the device lifetime.
    uint64_t dummy_page_bus;
    // IH platform services (pci_set_master; optional doorbells) and MSI state.
    Ih6Platform ih_platform;
    bool msi_enabled;
    // Doorbell BAR bus base (adev->doorbell.base) for the soc21 late-init
    // self-ring aperture; zero leaves the self-ring aperture disabled.
    uint64_t doorbell_bus_base;
    // Doorbell BAR writes for ring write pointers; both callbacks null selects
    // Linux's register WPTR path for the SDMA kernel rings.
    DoorbellIo doorbells;
    // System memory for the GTT buffers Linux binds through the GART: IH rings,
    // kernel rings, EOP, writeback and (GTT fallback of the VRAM|GTT domain) MQDs.
    GttPlatform gtt;
    // adev->pm.ac_power (power_supply_is_system_supplied > 0); true on a desktop.
    bool ac_power;
    InterruptReady interrupt_ready;
    // The validated ATOM image (umc_info MEM ECC for the RAS TA decision); empty
    // when unavailable, which Linux would read as "MEM ECC is not presented".
    ByteView atom_rom{nullptr, 0};
};

struct AsicReport final {
    AsicStatus status = AsicStatus::invalid_input;
    AsicStep step = AsicStep::inputs;
    DiscoveryStatus discovery = DiscoveryStatus::invalid_argument;
    NbioStatus nbio = NbioStatus::unavailable;
    GmcStatus gmc = GmcStatus::unavailable;
    Ih6Status ih = Ih6Status::unavailable;
    Gfx11Status gfx = Gfx11Status::unavailable;
    Sdma6Status sdma = Sdma6Status::unavailable;
    Mes11Status mes = Mes11Status::unavailable;
    Gfx11QueuesStatus queues = Gfx11QueuesStatus::unavailable;
    ClockGatingStatus gating = ClockGatingStatus::unavailable;
    // amdgpu_mes_self_test: a diagnostic whose result Linux ignores.
    MesSelfTestOutcome mes_self_test{MesSelfTestVmStatus::unavailable, Mes11Status::unavailable,
                                     MesSelfTestVmStatus::unavailable};
    // amdgpu_ib_ring_tests (delayed_init_work): gfx, compute and SDMA IB tests.
    Gfx11QueuesStatus gfx_ib = Gfx11QueuesStatus::unavailable;
    Sdma6Status sdma_ib = Sdma6Status::unavailable;
    PspBootStatus boot = PspBootStatus::unavailable;
    PspRingStatus ring = PspRingStatus::unavailable;
    SmuStatus smu = SmuStatus::unavailable;
    memory::Status memory = memory::Status::unavailable;
    VramLocation vram{};
    PspRuntimeDatabase runtime{};
    // Linux psp_sw_init: training is enabled unless a valid runtime boot-config
    // entry clears TWO_STAGE_DRAM_TRAINING; it then runs only with ATOM support.
    bool training_requested = false;
    PspTrainingReport training{};
    // SCPM active means PSP, not the driver, transfers the PPTable to PMFW.
    bool scpm_enabled = false;
    uint32_t tmr_bytes = 0;
    uint32_t loaded_ucode = 0;
    uint8_t failed_ucode_id = 0;
    uint32_t asd_session = 0;
    bool sos_was_running = false;
    bool rlc_autoload_started = false;
    // shutdown: first stage whose stop could not be confirmed (stopped = none).
    AsicStep fini_failure = AsicStep::stopped;
    bool shut_down = false;
    // suspend: first stage that failed (suspended = none); suspended is true only
    // after every stage and the mode-1 reset completed, until resume or release.
    AsicStep suspend_failure = AsicStep::suspended;
    bool suspended = false;
    uint32_t suspends = 0;
    uint32_t resumes = 0;
    // amdgpu_dpm_set_df_cstate(DF_CSTATE_DISALLOW) result (Linux only warns).
    SmuStatus df_cstate = SmuStatus::unavailable;
    // amdgpu_device_need_post on resume: BIOS_SCRATCH_7 ASIC_INIT_COMPLETE.
    bool posted = false;
    // amdgpu_gtt_mgr_recover on resume: GTT allocations and pages rebound.
    uint32_t gart_rebound_allocations = 0;
    uint32_t gart_rebound_pages = 0;
};

// AsicInit::repost evidence. It is kept apart from AsicReport, which run clears,
// so a repost before a reset-path restart stays visible afterwards.
struct AsicRepostReport final {
    AsicStatus status = AsicStatus::invalid_input;
    // The last repost stage reached (repost_confirm with status completed when
    // BIOS_SCRATCH_7 shows ASIC_INIT_COMPLETE after ASIC_Init).
    AsicStep step = AsicStep::inputs;
    // psp_v13_0_wait_for_bootloader observation (C2PMSG_35 == 0x80000000).
    bool bootloader_ready = false;
    bool posted = false;
    uint32_t attempts = 0;
    uint32_t completed = 0;
    // Interpreter FB scratch in bytes (atom_allocate_fb_scratch, clamped).
    uint32_t scratch_bytes = 0;
    AtomExecReport atom{};
};

// Construct once in nonpaged device storage. run is serialized at PASSIVE_LEVEL by
// the exclusive device owner with no ISR/DPC using MMIO, and is not repeatable
// without release_after_reset. Inputs (firmware bytes, catalog files, Manager,
// register mapping) outlive this object. The Manager allocates only local VRAM
// whose GPU addresses are MC addresses inside the MMHUB FB window; boot staging,
// PSP control and firmware buffers must be CPU-mapped uncached.
//
// Two-stage DRAM training runs (PSP_BL__DRAM_LONG_TRAIN with bottom-of-VRAM save
// and restore) exactly where Linux would; without firmwareinfo, or without
// training callbacks when training is needed, run stops with unsupported.
//
// Completion means each command's documented completion was observed, including
// the SDMA kernel ring tests in the simulated model. It is not hardware evidence;
// DCN/DMUB and VCN start-up are separate display and media work.
class AsicInit final {
public:
    AsicInit() noexcept = default;
    AsicInit(const AsicInit&) = delete;
    AsicInit& operator=(const AsicInit&) = delete;

    AsicStatus run(const AsicInputs& inputs, AsicInitPolicy policy,
                   AsicRecovery recovery) noexcept;
    // Requires confirmed reset quiescence; releases every protocol lease and every
    // Manager allocation this sequencer created. Failure keeps everything retained.
    // Optional native owner callback runs only after the reset proof, with
    // GTT destruction in reset-safe mode, before any MES/GART backing is freed.
    // False retains the complete ASIC state for a later safe retry.
    AsicStatus release_after_reset(void* external_context = nullptr,
        bool (*retire_external)(void*) noexcept = nullptr) noexcept;
    // amdgpu_device_ip_fini_early on bare metal after a completed run: every IP's
    // hw_fini in reverse block order (MES, SDMA, GFX with KGQ/KCQ unmap and MES
    // KIQ fini, PSP, IH, GMC, common doorbells). Like Linux each stage runs even
    // after an earlier one failed. Never resets: every lease stays retained and
    // release_after_reset still needs a confirmed reset. Returns completed only
    // if every stage confirmed its stop, else quarantined.
    AsicStatus shutdown() noexcept;
    // amdgpu_device_mode1_reset, only on an explicit owner request (never from
    // run, shutdown or a failure path): BIOS-scratch engine-hung flag, PCI state
    // cached and bus mastering cleared, SMU Mode1Reset (PMFW >= 78.41; Linux's
    // psp_gpu_reset fallback is a no-op on PSP 13.0, which has no mode1_reset hook,
    // so older PMFW is refused here instead), PCI state restored, PSP bootloader ready
    // (C2PMSG_35 bit 31, 10 x usec_timeout polls), RCC_CONFIG_MEMSIZE readable,
    // engine-hung flag cleared. Completion is the reset proof release_after_reset
    // accepts; any failure leaves the ASIC unproven and every lease retained.
    AsicStatus mode1_reset(ResetPlatform platform) noexcept;
    // A completed mode-1 reset (explicit, or the one ending a suspend) whose
    // proof release_after_reset has not consumed yet.
    bool reset_proven() const noexcept { return reset_done_; }
    // Completed restart_after_reset calls (kept outside AsicReport, which run
    // clears, and which would grow run's kernel-stack temporary).
    uint32_t restarts() const noexcept { return restarts_; }
    // Destructive-reset recovery (Linux amdgpu_do_asic_reset after a MODE1
    // reset): after release_after_reset completed (status released), run the
    // whole bring-up again on the same resident inputs, with two differences
    // from a first run. GTT allocations other owners retained across the
    // confirmed reset (2D staging and evicted surfaces, process buffers) keep
    // their GART addresses and are bound into the new GART table
    // (amdgpu_ttm_recover_gart / amdgpu_gtt_mgr_recover) instead of
    // re-initializing the GTT Manager; and PSP DRAM training uses the resume
    // path (psp_resume -> PSP_MEM_TRAIN_RESUME) instead of the cold-boot long
    // training. inputs must name the same VRAM Manager and GTT platform as the
    // first run. Every protocol owner starts from its released state, so the
    // result and its failure semantics are those of run. Before the bring-up
    // the card is re-POSTed (repost, amdgpu_device_asic_init); a failed repost
    // returns its status with the sequencer still released (retryable).
    AsicStatus restart_after_reset(const AsicInputs& inputs, AsicInitPolicy policy,
                                   AsicRecovery recovery) noexcept;
    // Bare-metal system suspend (amdgpu_pmops_suspend/freeze on a dGPU: no runtime
    // PM, no s0ix) after a completed run with no MES process queue registered
    // (their owner evicts them first, as amdgpu_amdkfd_suspend). In Linux order:
    // amdgpu_device_evict_resources (nothing: every sequencer VRAM buffer is pinned
    // kernel memory re-created by resume), ip_suspend_phase1 (PG/CG ungate, DF
    // C-state disallow), amdgpu_fence_driver_hw_fini (wait for every emitted fence,
    // detach the kernel rings), ip_suspend_phase2 in reverse block order (MES
    // amdgpu_mes_suspend, SDMA hw_fini, GFX hw_fini with KGQ/KCQ unmap and KIQ
    // fini, SMU thermal alert off, PSP TA/ASD/TMR teardown and ring stop, IH
    // disable, GMC MMHUB disable, soc21 doorbell apertures off), then the
    // suspend_noirq mode-1 reset. A fence that does not signal aborts before any
    // block suspends (busy, or quarantined for a hung ring) and gating is restored.
    // Later stages run even after a failure, as Linux does, but only a fully
    // confirmed suspend (completed) can be resumed. Every allocation, lease holder
    // and handle stays alive; GTT allocations must not change until resume.
    AsicStatus suspend(SuspendPlatform platform) noexcept;
    // amdgpu_device_resume after a completed suspend, the platform's PCI restore
    // and D0 entry. inputs must name the same Manager, GTT platform, discovery,
    // firmware and catalog as run (the native owner keeps them resident). When
    // amdgpu_device_need_post finds BIOS_SCRATCH_7 without ASIC_INIT_COMPLETE,
    // resume re-POSTs through repost (amdgpu_device_asic_init); without an ATOM
    // image and firmwareinfo it refuses with unsupported before any write. A
    // failed repost returns its status (step repost_*) and leaves the ASIC
    // suspended with its reset proof, so resume may be retried. Otherwise:
    // soc21 common resume, GMC (GART table refilled, every live GTT allocation
    // rebound, MMHUB on), IH resume (InterruptReady again), psp_resume with
    // PSP_MEM_TRAIN_RESUME training, bootloader, KM ring, TOC/TMR (same TMR),
    // every firmware, ASD, RL and TAs, SMU hw setup, GFX hw_init with MES KIQ, KCQ/KGQ
    // restored from their MQD backups, SDMA resume, MES hw_init and
    // amdgpu_mes_resume, late init (self-ring, SMU late init, CG/PG gate), the IB
    // tests, amdgpu_mes_self_test, and kernel submission on the same FenceManager
    // with continuing fence sequences. No allocation is created or released.
    // Completion returns step complete; any later failure leaves the ASIC unproven
    // (only mode1_reset/release_after_reset remain).
    AsicStatus resume(const AsicInputs& inputs) noexcept;
    // amdgpu_device_asic_init for GC >= 11.0.0 (PASSIVE_LEVEL, exclusive owner,
    // no ISR/DPC using MMIO): amdgpu_psp_wait_for_bootloader, then
    // amdgpu_atomfirmware_asic_init(adev, true), i.e. the VBIOS ASIC_Init command
    // table with asic_init_ps_allocation_v2_1 from the firmwareinfo boot sclk/mclk
    // and memflag 0, executed by the bounded AtomInterpreter through inputs.io,
    // then the BIOS_SCRATCH_7 ASIC_INIT_COMPLETE check amdgpu_device_need_post
    // uses. MP0 13.0.0's steady-state bootloader hook returns without polling and
    // Linux ignores its result; this waits as long as mode1_reset does for
    // C2PMSG_35 bit 31, records the observation and runs ASIC_Init either way.
    // Uses inputs.io, inputs.discovery (MP0 base), inputs.atom (boot clocks and
    // bios_scratch_reg), inputs.atom_rom (rebound each call, borrowed until the
    // next repost) and its vram_usagebyfirmware scratch size; allocates nothing and
    // never changes AsicReport status or the suspend/reset state. Allowed only
    // while no run owns a live ASIC: before run, after release_after_reset, or
    // with a mode-1 reset proof (inside resume, or after mode1_reset); else busy.
    // completed: posted confirmed. unsupported: no ROM/firmwareinfo/scratch
    // register or no usable ASIC_Init table (nothing written beyond the
    // bootloader poll). quarantined: the ROM program aborted structurally and the
    // interpreter refuses it from now on. failed: register I/O, budgets or a
    // missing ASIC_INIT_COMPLETE; repost may be called again. Writes made by an
    // aborted ASIC_Init are not undone (its effect is the re-POST itself).
    AsicStatus repost(const AsicInputs& inputs) noexcept;
    AsicRepostReport repost_report() const noexcept { return repost_report_; }
    // After a completed run (and the owner's interrupt source enables): attach
    // the gfx, compute and SDMA kernel rings to fences for amdgpu_ib_schedule-style
    // submission. fences outlives this object; shutdown waits for and detaches it.
    AsicStatus enable_kernel_submission(FenceManager& fences, RingPointerStore store) noexcept;
    Gfx11Queues& kernel_queues() noexcept { return queues_; }
    Sdma6& sdma_rings() noexcept { return sdma_; }
    // GTT Manager for IBs and other kernel-ring buffers (valid after GMC start).
    memory::Manager& gtt_manager() noexcept { return gtt_; }
    // gmc_v11_0_flush_gpu_tlb_pasid for process VMs (MES path after bring-up).
    GpuVmInvalidator& vm_invalidator() noexcept { return vm_invalidator_; }
    // Trusted serialized kernel owner only. Caller supplies its PASSIVE_LEVEL
    // monotonic clock before initialize; no addresses cross the process ABI.
    bool user_queue_config(PasidAllocator& pasids, UserQueueConfig& output) noexcept;
    bool describe_gtt_bus(memory::Handle handle, uint64_t& bus) const noexcept {
        bus = 0;
        return gtt_bus(handle, bus);
    }

    AsicReport report() const noexcept { return report_; }
    // The MC placement of VRAM the last run programmed (report().vram), without
    // copying the whole report (kernel IOCTL paths).
    VramLocation vram_location() const noexcept { return report_.vram; }
    const Nbio& nbio() const noexcept { return nbio_; }
    PspRingReport ring_report() const noexcept { return ring_.report(); }
    SmuReport smu_report() const noexcept { return smu_.report(); }
    GmcReport gmc_report() const noexcept { return gmc_.report(); }
    Ih6Report ih_report() const noexcept { return ih_.report(); }
    Gfx11Report gfx_report() const noexcept { return gfx_.report(); }
    Sdma6Report sdma_report() const noexcept { return sdma_.report(); }
    Mes11Report mes_report() const noexcept { return mes_.report(); }
    Gfx11QueuesReport queues_report() const noexcept { return queues_.report(); }
    ClockGatingReport gating_report() const noexcept { return gating_.report(); }
    Mes11SelfTestReport mes_self_test_report() const noexcept { return mes_.self_test_report(); }
    PspTaReport trusted_apps_report() const noexcept { return apps_.report(); }
    // Interrupt consumers read ring views and acknowledge through this owner.
    Ih6& interrupts() noexcept { return ih_; }
    // GFX bring-up enables GFXHUB through the same GMC owner.
    Gmc11& gmc() noexcept { return gmc_; }
    // The FenceManager enable_kernel_submission attached the kernel rings to
    // (null before it and after a reset released them).
    FenceManager* kernel_fences() const noexcept { return fences_; }
    // The SMU13 owner for PASSIVE_LEVEL interrupt actions (thermal and power
    // notifications), serialized by the device owner like every SMU message.
    Smu13& smu() noexcept { return smu_; }
    // Interrupt actions: the NBIO token (ERREVENT_ATHUB status) and the RAS
    // support Linux derives at TA load (ras_hw_enabled, UMC block).
    NbioRegisters nbio_registers() const noexcept { return nbio_registers_; }
    const PspRasSupport& ras_support() const noexcept { return apps_.ras_support(); }

private:
    enum class Owned {
        gart_table, scratch, ih_ring0, ih_ring1, ih_writeback, boot_staging, psp_control,
        psp_firmware, tmr, smu_driver, smu_tool, clear_state, sdma_ring0, sdma_ring1,
        sdma_writeback, kiq_ring, mes_ring, kiq_mqd, mes_mqd, kiq_eop, mes_eop, mes_writeback,
        ta_ras, ta_hdcp, ta_dtm, selftest_table0, selftest_table1, selftest_table2, selftest_table3, selftest_metadata,
        selftest_process, selftest_gang0, selftest_gang1, selftest_gang2, selftest_mqd0,
        selftest_mqd1, selftest_mqd2, ib_gfx, ib_sdma, gfx_ring, gfx_mqd, compute_ring0,
        compute_mqd0 = compute_ring0 + gfx11_max_compute_rings,
        compute_eop = compute_mqd0 + gfx11_max_compute_rings, queue_writeback, count
    };

    static bool read32(void* context, uint32_t offset, uint32_t& value) noexcept;
    static bool write32(void* context, uint32_t offset, uint32_t value) noexcept;
    static bool publish(void* context, PspMcAddress address, size_t bytes) noexcept;
    static void delay(void* context, uint32_t microseconds) noexcept;
    static bool confirm(void* context) noexcept;
    static memory::CreateResult gtt_create(void* context, sdma::GpuAllocation gpu, memory::Access access,
                                           bool cpu_visible, memory::Backing& backing) noexcept;
    static bool gtt_destroy(void* context, const memory::Backing& backing,
                            sdma::GpuAllocation gpu) noexcept;
    static bool in_gtt(Owned slot) noexcept;
    // IH platform trampolines: bus master and doorbells from the caller, reset
    // proof from this owner (caller confirmation or a completed mode1_reset).
    static bool ih_bus_master(void* context) noexcept;
    static bool mes_misc(void* context, ByteView frame, uint32_t api_status_offset) noexcept;
    static bool ih_doorbell(void* context, uint32_t index, uint32_t value) noexcept;
    memory::Manager& manager_for(Owned slot) noexcept { return in_gtt(slot) ? gtt_ : *manager_; }
    // IH, SDMA, MES and the kernel queues use their VramLocation argument only to
    // bound where their buffers may live; their buffers are GTT, so they get the
    // GART aperture as that VMID0 placement window.
    VramLocation gtt_window() const noexcept {
        const GartLayout gart = gmc_.gart();
        return {gart.start, gart.end, gart.end - gart.start + 1, 0};
    }

    AsicStatus fail(AsicStep step, AsicStatus status) noexcept;
    bool allocate(Owned slot, uint64_t bytes, uint64_t alignment, bool cpu_visible) noexcept;
    memory::Handle handle(Owned slot) const noexcept {
        return handles_[static_cast<size_t>(slot)];
    }
    AsicStatus boot_psp(const AsicInputs& inputs, PspRegisters registers,
                        Ih6Registers ih_registers, NbioRegisters nbio) noexcept;
    AsicStatus train_if_needed(const AsicInputs& inputs, PspRegisters registers) noexcept;
    AsicStatus load_firmware(const AsicInputs& inputs, const FirmwareImage& sos) noexcept;
    AsicStatus start_smu(const AsicInputs& inputs, SmuRegisters registers,
                         ThmRegisters thm) noexcept;
    AsicStatus start_gmc(const AsicInputs& inputs, GmcRegisters registers) noexcept;
    AsicStatus start_ih(const AsicInputs& inputs, Ih6Registers registers, NbioRegisters nbio) noexcept;
    AsicStatus start_gfx(const AsicInputs& inputs, memory::GcRegisterBase registers) noexcept;
    void run_self_test(NbioRegisters nbio) noexcept;
    bool gtt_bus(memory::Handle handle, uint64_t& bus) const noexcept;
    AsicStatus late_gating_and_ib_tests(const AsicInputs& inputs, memory::GcRegisterBase gc,
                                        NbioRegisters nbio) noexcept;
    AsicStatus start_mes_kiq(const AsicInputs& inputs, Mes11Registers registers) noexcept;
    AsicStatus start_queues(const AsicInputs& inputs, memory::GcRegisterBase registers) noexcept;
    AsicStatus start_sdma(const AsicInputs& inputs, Sdma6Registers registers,
                          NbioRegisters nbio) noexcept;

    // Discovered register tokens shared by run and resume.
    struct Discovered final {
        PspRegisters psp;
        NbioRegisters nbio;
        SmuRegisters smu;
        ThmRegisters thm;
        GmcRegisters gmc;
        Ih6Registers ih;
        Sdma6Registers sdma;
        Mes11Registers mes;
        memory::GcRegisterBase gc;
    };

    AsicStatus validate_inputs(const AsicInputs& inputs, Discovered& tokens,
                               FirmwareImage& sos) noexcept;
    AsicStatus load_sos(const AsicInputs& inputs, memory::Handle staging) noexcept;
    bool bind_gtt(uint64_t gart_address, uint64_t bus, uint64_t bytes, memory::Access access) noexcept;
    AsicStatus reset_asic(ResetPlatform platform) noexcept;
    AsicStatus resume_sequence(const AsicInputs& inputs) noexcept;
    AsicStatus repost_sequence(const AsicInputs& inputs) noexcept;
    AsicStatus resume_gmc() noexcept;
    AsicStatus rebind_gtt_pages(AsicStep step) noexcept;
    AsicStatus resume_psp(const AsicInputs& inputs, const Discovered& tokens,
                          const FirmwareImage& sos) noexcept;
    void forget_reset_hardware() noexcept;
    AsicStatus early_late_init(const AsicInputs& inputs) noexcept;

    RegisterIo io_{};
    AsicRecovery recovery_{};
    AsicInitPolicy policy_{};
    AsicReport report_{};
    PspTrainingData training_data_{};
    Nbio nbio_{};
    Gmc11 gmc_{};
    Ih6 ih_{};
    Gfx11Init gfx_{};
    Sdma6 sdma_{};
    Mes11 mes_{};
    PspTrustedApps apps_{};
    Gfx11Queues queues_{};
    ClockGating gating_{};
    MesSelfTestVm selftest_vm_{};
    PspBoot boot_{};
    PspRing ring_{};
    Smu13 smu_{};
    memory::Manager* manager_ = nullptr;
    // GART-aperture allocations over GttPlatform pages, initialized once MMHUB runs.
    memory::Manager gtt_{};
    GttPlatform gtt_platform_{};
    bool gtt_ready_ = false;
    // Set inside release_after_reset: the reset left no live GART to unbind.
    bool reset_confirmed_ = false;
    // Set by a completed mode1_reset: the reset proof for every protocol owner.
    bool reset_done_ = false;
    // Inside restart_after_reset: GTT rebind and resume-style PSP training.
    bool restarting_ = false;
    uint32_t restarts_ = 0;
    // The GART aperture the GTT Manager was initialized over (first run).
    sdma::GpuAllocation gtt_arena_{0, 0};
    PspRegisters psp_registers_{};
    NbioRegisters nbio_registers_{};
    Ih6Registers ih_registers_{};
    GpuVmInvalidator vm_invalidator_{};
    FenceManager* fences_ = nullptr;
    Ih6Platform ih_platform_{};
    // DMA bus base, size and access of each live GTT allocation, keyed by its
    // GART address (amdgpu_gtt_mgr's node list for amdgpu_gtt_mgr_recover).
    struct GttPages final {
        uint64_t gart = 0;
        uint64_t bus = 0;
        uint64_t bytes = 0;
        memory::Access access = memory::Access::read_write;
    };
    GttPages gtt_pages_[memory::allocation_capacity]{};
    uint32_t bios_scratch_reg_ = 0;
    memory::Handle handles_[static_cast<size_t>(Owned::count)]{};
    bool owned_[static_cast<size_t>(Owned::count)]{};
    bool ran_ = false;
    // A completed suspend (all stages and its mode-1 reset) awaits resume.
    bool suspended_ = false;
    // Inside resume: allocate returns the slot's existing allocation.
    bool resuming_ = false;
    // The kernel-submission WPTR store, kept to re-enable submission on resume.
    RingPointerStore store_{};
    DoorbellIo process_doorbells_{};
    // VBIOS ATOM interpreter for repost (bound to the resident ROM once).
    AtomInterpreter atom_{};
    AsicRepostReport repost_report_{};
};

} // namespace rx7900::hardware

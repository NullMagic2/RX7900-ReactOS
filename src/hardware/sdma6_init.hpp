/*
Module: rx7900.hardware.sdma6_init
Purpose: Start, ring-test, IB-test and halt the two SDMA 6.0.0 kernel rings on Navi31.
Created: 2026-10-21
Architecture: Discovery-validated GC/SDMA token, Nbio for HDP publication and the
NBIO4.3 SDMA doorbell range, Manager leases on local-VRAM rings and writeback;
packet bytes come from src/acceleration/sdma; later queue tracking stays in sdma::Ring.
Copyright 2020 Advanced Micro Devices, Inc.; Copyright 2021 Advanced Micro Devices, Inc.;
Copyright 2019 Advanced Micro Devices, Inc.; Copyright 2018 Advanced Micro Devices, Inc.
Adapted Linux v6.12 sdma_v6_0.c, sdma_v6_0_0_pkt_open.h, sdma_common.h,
amdgpu_ib.c, amdgpu_fence.c, amdgpu_sdma.c, amdgpu_device.c, nbio_v4_3.c,
amdgpu_doorbell.h, soc21.c, soc15_hw_ip.h and gc_11_0_0/nbio_4_3_0 register headers. Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#pragma once

#include "fence.hpp"
#include "nbio.hpp"

namespace rx7900::hardware {

// soc15_hw_ip.h SDMA0_HWID and SDMA1_HWID; Linux counts one SDMA instance per
// record. discovery.hpp does not own these identifiers.
constexpr uint16_t discovery_sdma0_hardware_id = 42;
constexpr uint16_t discovery_sdma1_hardware_id = 43;
constexpr uint32_t sdma6_instance_count = 2;

// sdma_v6_0_sw_init: amdgpu_ring_init(max_dw 1024) with the default
// amdgpu_sched_hw_submission 2 gives roundup_pow_of_two(1024 * 4 * 2) bytes.
constexpr uint32_t sdma6_default_ring_bytes = 8192;
// Linux ring BOs are PAGE_SIZE aligned; one page is the smallest ring accepted.
constexpr uint32_t sdma6_minimum_ring_bytes = 4096;
constexpr uint32_t sdma6_ring_alignment = 4096;

// Writeback layout mirrors amdgpu_device_wb_get: every slot is 256 bits (eight
// dwords). Per instance: slot 0 RPTR writeback (64-bit byte RPTR written by
// SDMA), slot 1 WPTR poll shadow (64-bit byte WPTR written by the CPU), slot 2
// the ring-test destination dword.
constexpr uint32_t sdma6_writeback_slot_bytes = 32;
constexpr uint32_t sdma6_writeback_slots_per_instance = 3;
constexpr uint32_t sdma6_writeback_bytes =
    sdma6_instance_count * sdma6_writeback_slots_per_instance * sdma6_writeback_slot_bytes;
constexpr uint32_t sdma6_writeback_alignment = sdma6_writeback_slot_bytes;

// Submission writeback (enable_submission), after the slots above in the same
// allocation: per instance i, slot sdma6_writeback_fence_slot_base + 2i is the
// ring fence dword (Linux fence_drv.cpu_addr, an adev->wb slot) and the next
// slot the cond_exec dword (cond_exe_gpu_addr, value 1). The writeback
// allocation must then be at least sdma6_submission_writeback_bytes.
constexpr uint32_t sdma6_writeback_fence_slot_base =
    sdma6_instance_count * sdma6_writeback_slots_per_instance;
constexpr uint32_t sdma6_submission_writeback_bytes =
    (sdma6_writeback_fence_slot_base + 2 * sdma6_instance_count) * sdma6_writeback_slot_bytes;
static_assert(sdma6_instance_count == fence_sdma_rings, "SDMA instances are FenceManager rings");

// amdgpu_ib_schedule -> amdgpu_ring_alloc(emit_frame_size + emit_ib_size) for
// sdma_v6_0_ring_funcs: 5 + 6 + 6 + 6 * 3 + 3 * 6 + 30 + 18 = 101, rounded up to
// align_mask 0xf: 112 dwords.
constexpr uint32_t sdma6_submit_alloc_dwords = 112;
// SDMA_PKT_INDIRECT_HEADER IB_SIZE is 20 bits; sdma_v6_0_ring_emit_ib needs a
// 32-byte aligned base; sdma_v6_0_ring_pad_ib pads every IB to 8 dwords.
constexpr uint32_t sdma6_max_ib_dwords = 0x000fffffu;
constexpr uint32_t sdma6_ib_alignment = 32;
constexpr uint32_t sdma6_ib_dword_multiple = 8;

// soc21_init_doorbell_index: AMDGPU_NAVI10_DOORBELL_sDMA_ENGINE0/1 (0x100,
// 0x10A) in 64-bit units, shifted to dword indices exactly as sdma_v6_0_sw_init.
// Each WPTR doorbell is 64-bit and occupies this dword and the next one.
constexpr uint32_t sdma6_ring0_doorbell_index = 0x100u << 1;
constexpr uint32_t sdma6_ring1_doorbell_index = 0x10Au << 1;
// soc21 sdma_doorbell_range (20) times num_instances, as gfx_resume passes it.
constexpr uint32_t sdma6_doorbell_range_dwords = 20 * sdma6_instance_count;

// AMDGPU_MAX_USEC_TIMEOUT on bare metal (adev->usec_timeout): the ring-test poll
// limit, one udelay(1) per attempt.
constexpr uint32_t sdma6_usec_timeout = 100000;
// sdma_v6_0_ring_test_ring seed and the value SDMA must write over it.
constexpr uint32_t sdma6_ring_test_seed = 0xCAFEDEADu;
constexpr uint32_t sdma6_ring_test_value = 0xDEADBEEFu;
// amdgpu_ring_alloc(ring, 5) rounded to align_mask 0xf: one 16-dword frame.
constexpr uint32_t sdma6_ring_test_bytes = 16 * sizeof(uint32_t);

// amdgpu_ib.c AMDGPU_IB_TEST_TIMEOUT (1000 ms) is the bare-metal tmo_gfx that
// amdgpu_ib_ring_tests also passes to SDMA rings; the XGMI-hive value (2000 ms)
// is the largest fence poll budget accepted.
constexpr uint32_t sdma6_ib_test_timeout_us = 1000000;
constexpr uint32_t sdma6_ib_test_xgmi_timeout_us = 2000000;
// sdma_v6_0_ring_test_ib: amdgpu_ib_get(256 bytes) from the 256-byte aligned
// IB pool; the IB is WRITE_LINEAR (5 dwords) plus three NOPs, length_dw 8.
// Caller-provided IB buffer layout (one buffer for both engines):
//   instance i IB at byte i * sdma6_ib_test_slot_bytes;
//   at sdma6_ib_test_slots_offset + i * 96: the fence slot (Linux
//   fence_drv.gpu_addr), the cond_exec slot (cond_exe_gpu_addr, value 1) and
//   the WRITE_LINEAR destination (a fresh adev->wb slot in Linux), 32 bytes each.
// Linux keeps the three slots in adev->wb; this module's writeback allocation
// (sdma6_writeback_bytes) has no room for them, so they live in the IB buffer.
constexpr uint32_t sdma6_ib_test_slot_bytes = 256;
constexpr uint32_t sdma6_ib_test_dwords = 8;
constexpr uint32_t sdma6_ib_test_slots_offset = sdma6_instance_count * sdma6_ib_test_slot_bytes;
constexpr uint32_t sdma6_ib_test_slot_stride = 32;
constexpr uint32_t sdma6_ib_test_slots_per_instance = 3;
constexpr uint32_t sdma6_ib_test_buffer_bytes = 1024;
constexpr uint32_t sdma6_ib_test_alignment = 256;
static_assert(sdma6_ib_test_slots_offset + sdma6_instance_count *
                  sdma6_ib_test_slots_per_instance * sdma6_ib_test_slot_stride <=
              sdma6_ib_test_buffer_bytes, "IB test slots fit the IB buffer");
// amdgpu_sdma_init_inst_ctx: burst_nop when the SDMA firmware header's
// ucode_feature_version is at least 20.
constexpr uint32_t sdma6_burst_nop_feature_version = 20;

enum class Sdma6Status {
    ready,
    started,
    stopped,
    released,
    unavailable,
    unsupported_ip,
    invalid_argument,
    invalid_memory,
    mmio_failure,
    doorbell_failure,
    ring_test_timeout,
    busy,
    quarantined,
    // test_ib: an engine's IB test timed out or wrote a wrong value; that ring
    // is disabled and quarantined, the other stays usable.
    ib_test_failed,
    // submit: four fences outstanding or not enough ring space; retry later.
    ring_full,
    // submit/disable_submission: the FenceManager quarantined the ring.
    ring_hung
};

// Linux amdgpu_ring_test_ib result: timeout is -ETIMEDOUT (fence not signalled),
// failed is -EINVAL (fence signalled, destination lacks 0xDEADBEEF).
enum class Sdma6IbTest { not_run, passed, timeout, failed };

// firmware_feature_version: the SDMA firmware header ucode_feature_version
// (sdma_firmware_header_v3_0), which selects burst NOP padding as Linux does.
// timeout_microseconds: fence polls of 1 us, 1..sdma6_ib_test_xgmi_timeout_us;
// Linux bare metal uses sdma6_ib_test_timeout_us.
struct Sdma6IbTestConfig final {
    uint32_t firmware_feature_version;
    uint32_t timeout_microseconds;
};

// Only validate_sdma6_discovery creates a valid token: unique GC instance0
// 11.0.0 with two local segments (sdma_v6_0_get_reg_offset uses GC segment 0
// for queue registers and segment 1 for the HYP_DEC range holding F32_CNTL),
// plus unique SDMA0 (42) and SDMA1 (43) instance0 6.0.0 records proving both
// engines exist. The highest SDMA1 register in each segment must fit BAR5.
class Sdma6Registers final {
public:
    Sdma6Registers() noexcept = default;
    bool valid() const noexcept { return gc_.valid() && aperture_bytes_ != 0; }
    uint32_t gc_base(size_t segment) const noexcept { return gc_.segment(segment); }
    uint32_t aperture_bytes() const noexcept { return aperture_bytes_; }

private:
    friend DiscoveryStatus validate_sdma6_discovery(ByteView, uint32_t, Sdma6Registers&) noexcept;
    DiscoveredIp gc_{};
    uint32_t aperture_bytes_ = 0;
};

DiscoveryStatus validate_sdma6_discovery(ByteView binary, uint32_t register_aperture_bytes,
                                         Sdma6Registers& output) noexcept;

struct Sdma6Config final {
    // Power of two >= sdma6_minimum_ring_bytes for both kernel rings; Linux
    // uses sdma6_default_ring_bytes.
    uint32_t ring_bytes;
};

// confirm_reset_quiescence is true only after a reset proves neither SDMA engine
// can still read either ring or write the writeback memory. Never inferred from
// a failed access or a timeout. Required; context outlives this object.
struct Sdma6Recovery final {
    void* context;
    bool (*confirm_reset_quiescence)(void*) noexcept;
};

struct Sdma6Report final {
    Sdma6Status status = Sdma6Status::unavailable;
    uint64_t ring_mc[sdma6_instance_count]{};
    uint64_t rptr_mc[sdma6_instance_count]{};
    uint64_t wptr_mc[sdma6_instance_count]{};
    uint64_t test_mc[sdma6_instance_count]{};
    uint64_t wptr_bytes[sdma6_instance_count]{};
    uint32_t ring_bytes = 0;
    uint32_t rb_cntl[sdma6_instance_count]{};
    uint32_t ib_cntl[sdma6_instance_count]{};
    uint32_t doorbell[sdma6_instance_count]{};
    uint32_t doorbell_offset[sdma6_instance_count]{};
    uint32_t f32_cntl[sdma6_instance_count]{};
    uint32_t doorbell_range = 0;
    // Poll iterations the last ring test used, per instance.
    uint32_t ring_test_polls[sdma6_instance_count]{};
    // Instance whose programming or ring test failed; UINT32_MAX when none.
    uint32_t failed_instance = UINT32_MAX;
    bool ring_ready[sdma6_instance_count]{};
    bool doorbell_mode = false;
    bool started = false;
    bool memory_retained = false;
    // test_ib state per instance: result, Linux sched.ready = false after a
    // failure (the ring is quarantined), last emitted fence sequence
    // (fence_drv.sync_seq) and value read back, slot/IB addresses and polls.
    Sdma6IbTest ib_test[sdma6_instance_count]{};
    bool ib_disabled[sdma6_instance_count]{};
    uint32_t fence_seq[sdma6_instance_count]{};
    uint32_t fence_value[sdma6_instance_count]{};
    uint64_t fence_mc[sdma6_instance_count]{};
    uint64_t cond_exec_mc[sdma6_instance_count]{};
    uint64_t ib_mc[sdma6_instance_count]{};
    uint64_t ib_test_mc[sdma6_instance_count]{};
    uint32_t ib_polls[sdma6_instance_count]{};
    // First instance whose IB test failed, or UINT32_MAX; IB buffer lease state.
    uint32_t ib_failed_instance = UINT32_MAX;
    bool burst_nop = false;
    bool ib_buffer_retained = false;
    uint64_t ib_buffer_mc = 0;
    // Kernel-ring submission state per instance: FenceManager ring attached,
    // fence/cond_exec writeback addresses, IB leases for unsignalled fences,
    // submissions, consumed ring position (dwords) and free dwords at the last
    // submit.
    bool submission_enabled = false;
    bool submit_enabled[sdma6_instance_count]{};
    uint64_t submit_fence_mc[sdma6_instance_count]{};
    uint64_t submit_cond_exec_mc[sdma6_instance_count]{};
    uint32_t ib_leases[sdma6_instance_count]{};
    uint64_t submissions[sdma6_instance_count]{};
    uint64_t consumed_position[sdma6_instance_count]{};
    uint32_t free_dwords[sdma6_instance_count]{};
};

// enable_submission input: the atomic WPTR shadow store (doorbell mode; the
// register WPTR path writes RB_WPTR/RB_WPTR_HI as Linux does).
struct Sdma6SubmitConfig final {
    RingPointerStore wptr_store;
};

// One amdgpu_ib_schedule without a job (VMID 0) on SDMA instance: ib is an
// allocation of the hw_init Manager owned by the caller; ib_dwords (a nonzero
// multiple of 8, the caller's amdgpu_ring_pad_ib) from byte ib_offset, 32-byte
// aligned GPU address, outside both rings and the writeback.
struct Sdma6Submission final {
    uint32_t instance;
    memory::Handle ib;
    uint64_t ib_offset;
    uint32_t ib_dwords;
};

// Serialized PASSIVE_LEVEL use by the exclusive device owner for every method:
// they use Nbio's shared SMN pair, the Manager and a ring-test busy wait of up to
// sdma6_usec_timeout microseconds per instance. No ISR/DPC may call this class.
// PSP must already have loaded the SDMA firmware (Linux AMDGPU_FW_LOAD_PSP), the
// GFXHUB must map VMID0 and Nbio.program_common must have succeeded.
//
// Linux places both rings in GTT and the RPTR/WPTR/test slots in the adev->wb
// system-memory BO. Here they come from the caller's Manager inside the vram
// window it passes (AsicInit: GART-mapped system pages and the GART window) and
// are programmed by VMID0 address. Every CPU write to the ring or writeback
// memory is published with Nbio.flush_hdp before the engine is told about it
// (needed for VRAM backing, harmless for GTT). Any uncertain register or
// doorbell store, or a ring-test timeout, quarantines the protocol and keeps
// every lease until release_after_reset confirms both engines are quiescent.
//
// After start the kernel ring hardware WPTR equals the ring-test frame
// (sdma6_ring_test_bytes), not zero, as in Linux. sdma::Ring requires a queue
// whose pointers are zero, so handing a started ring to it is not supported;
// kernel-ring work goes through enable_submission/submit instead.
class Sdma6 final {
public:
    Sdma6() noexcept = default;
    Sdma6(const Sdma6&) = delete;
    Sdma6& operator=(const Sdma6&) = delete;

    // nbio_registers is the token nbio was initialized with (NBIF segment 3 for
    // S2A_DOORBELL_ENTRY_2_CTRL). doorbells with both callbacks null selects the
    // Linux non-doorbell register path (ring->use_doorbell false); otherwise both
    // callbacks are required and both 64-bit doorbells must fit the aperture.
    Sdma6Status initialize(Sdma6Registers registers, NbioRegisters nbio_registers, RegisterIo io,
                           Nbio& nbio, VramLocation vram, Sdma6Config config,
                           DoorbellIo doorbells, Sdma6Recovery recovery) noexcept;

    // sdma_v6_0_hw_init (sdma_v6_0_start, PSP-loaded firmware). From ready it
    // retains one lease on each ring (CPU-visible >= ring_bytes, 4 KiB aligned)
    // and on the writeback allocation (CPU-visible >= sdma6_writeback_bytes,
    // 32-byte aligned), fills the rings with NOPs and zeroes the writeback as
    // amdgpu_ring_init and amdgpu_device_wb_init do, flushes HDP, then programs
    // and ring-tests instance 0 and instance 1. Leases are unchanged when
    // validation fails before any write.
    Sdma6Status hw_init(memory::Manager& manager, memory::Handle ring0, memory::Handle ring1,
                        memory::Handle writeback) noexcept;
    // sdma_v6_0_resume after a successful hw_fini: sdma_v6_0_start again on the
    // retained memory (Linux does not clear rings or writeback here).
    Sdma6Status resume() noexcept;

    // sdma_v6_0_hw_fini: CTXEMPTY_INT_ENABLE off, then sdma_v6_0_enable(false)
    // (RB/IB disabled per instance, then both F32 halted). Linux does not wait
    // for idle. From a quarantined state the same stores are attempted to stop
    // the engines, but the protocol stays quarantined.
    Sdma6Status hw_fini() noexcept;

    // amdgpu_ib_ring_tests for sdma0 then sdma1 (sdma_v6_0_ring_test_ib), as
    // the bare-metal delayed init work runs it after amdgpu_device_ip_late_init.
    // Linux tests these after the gfx and compute rings and not at all when the
    // gfx ring IB test failed; the integrator enforces that order. Requires
    // started. A failure of sdma0 does not stop sdma1 (Linux continues).
    //
    // ib_buffer: one allocation from the hw_init manager, CPU visible, GPU
    // read/write, sdma6_ib_test_alignment aligned, at least
    // sdma6_ib_test_buffer_bytes, not overlapping either ring or the writeback.
    // Its GPU address may be VRAM or GART-mapped system memory (VMID0; no VRAM
    // aperture requirement). One lease is held for the call, returned when both
    // engines passed and kept (ib_buffer_retained) after any failure until
    // release_after_reset.
    //
    // Per instance: fence slot = fence_seq, cond_exec slot = 1, destination =
    // 0xCAFEDEAD, IB = WRITE_LINEAR(0xDEADBEEF) + 3 NOPs; then the
    // amdgpu_ib_schedule frame: GCR_REQ (shared encode_cache_sync), COND_EXE,
    // POLL_REGMEM HDP flush, NOP pad to the 8-dword INDIRECT end, INDIRECT vmid 0,
    // FENCE of ++fence_seq + TRAP (shared encode_fence), patched COND_EXE count,
    // NOP pad to 16 dwords; CPU HDP flush and WPTR publication as the ring test;
    // then fence polls once per microsecond and the destination check.
    //
    // Deviation: Linux waits for the dma_fence that amdgpu_fence_process
    // signals from the SDMA TRAP interrupt (or its fallback timer). The IB
    // tests run before IrqService::enable_sources sets SDMA_CNTL TRAP_ENABLE,
    // so the fence slot is polled; the TRAP packet is still emitted. Interrupt-
    // driven completion is the submit path through FenceManager.
    //
    // Returns started when both passed, ib_test_failed when either failed.
    // invalid_argument/invalid_memory refuse before any access; a failed CPU,
    // HDP, MMIO or doorbell store quarantines the whole protocol.
    Sdma6Status test_ib(memory::Manager& manager, memory::Handle ib_buffer,
                        const Sdma6IbTestConfig& config) noexcept;

    // Attaches each instance that passed its ring and IB test to fences as
    // FenceManager ring fence_ring_sdma0 + instance: the fence slot is set to
    // the IB test's sync_seq (amdgpu_fence_driver_start_ring) and cond_exec to
    // 1. Requires started, test_ib completed with the IB buffer returned, a
    // writeback allocation of at least sdma6_submission_writeback_bytes and,
    // in doorbell mode, wptr_store. test_ib is refused afterwards.
    Sdma6Status enable_submission(FenceManager& fences, const Sdma6SubmitConfig& config) noexcept;
    // amdgpu_ib_schedule on one SDMA kernel ring, PASSIVE_LEVEL, serialized by
    // the device owner: reclaims signalled IB leases, validates the IB, reserves
    // the fence, checks ring space against the end of the newest signalled
    // frame or the 64-bit RPTR writeback, retains one IB lease, writes the frame
    // of test_ib with this IB and the writeback fence/cond_exec slots (FENCE of
    // the reserved sequence + TRAP), publishes the fence, then the WPTR (shadow
    // + HDP flush + doorbell, or RB_WPTR registers). Statuses as Gfx11Queues::submit.
    Sdma6Status submit(const Sdma6Submission& submission, uint32_t& sequence) noexcept;
    // Returns IB leases whose fences signalled.
    Sdma6Status retire() noexcept;
    // Detaches both rings once idle and returns every IB lease; busy while
    // fences are outstanding, ring_hung when a ring is quarantined.
    Sdma6Status disable_submission() noexcept;

    // amdgpu_ring_fini after a successful hw_fini: unretain all leases. Refused
    // (quarantined) while a failed IB test keeps a ring or the IB buffer
    // quarantined, busy while submission is enabled.
    Sdma6Status release() noexcept;
    // After confirmed reset quiescence from any state: detaches the submission
    // rings, returns IB leases, unretains all leases.
    Sdma6Status release_after_reset() noexcept;

    Sdma6Report report() const noexcept { return report_; }

private:
    enum class Phase { uninitialized, ready, started, stopped, faulted };

    bool locate(uint32_t instance, uint32_t internal, uint32_t& base, uint32_t& reg) const noexcept;
    bool read(uint32_t instance, uint32_t internal, uint32_t& value) noexcept;
    bool write(uint32_t instance, uint32_t internal, uint32_t value) noexcept;
    bool modify(uint32_t instance, uint32_t internal, uint32_t clear, uint32_t set,
                uint32_t& value) noexcept;
    bool enable(bool on) noexcept;
    bool gfx_stop() noexcept;
    bool ctxempty_int_enable(bool on) noexcept;
    bool program_ring(uint32_t instance) noexcept;
    bool sdma_doorbell_range() noexcept;
    Sdma6Status ring_test(uint32_t instance) noexcept;
    Sdma6Status set_wptr(uint32_t instance) noexcept;
    Sdma6Status start() noexcept;
    Sdma6Status quarantine(Sdma6Status reason, uint32_t instance) noexcept;
    bool retain_memory(memory::Manager& manager, memory::Handle ring0, memory::Handle ring1,
                       memory::Handle writeback) noexcept;
    bool clear_memory() noexcept;
    void release_leases() noexcept;
    uint32_t doorbell_index(uint32_t instance) const noexcept;
    bool retain_ib(memory::Manager& manager, memory::Handle ib_buffer) noexcept;
    void release_ib() noexcept;
    bool ring_put(uint32_t instance, uint64_t& dwords, uint32_t value) noexcept;
    bool insert_nop(uint32_t instance, uint64_t& dwords, uint32_t count) noexcept;
    // One amdgpu_ib_schedule frame from dwords (advanced past the padding).
    bool emit_ib_frame(uint32_t instance, uint64_t& dwords, uint64_t ib, uint32_t ib_dwords,
                       uint64_t fence, uint64_t cond_exec, uint32_t sequence) noexcept;
    Sdma6Status ib_test(uint32_t instance, uint32_t timeout) noexcept;
    bool submit_ib_address(const Sdma6Submission& submission, uint64_t& address) noexcept;
    uint64_t consumed_position(uint32_t instance, uint64_t retired) const noexcept;
    void reclaim(uint32_t instance, bool all) noexcept;
    Sdma6Status publish_wptr(uint32_t instance) noexcept;

    struct IbLease final {
        bool used;
        uint32_t sequence;
        memory::Handle ib;
    };

    Sdma6Registers registers_{};
    NbioRegisters nbio_registers_{};
    RegisterIo io_{};
    Nbio* nbio_ = nullptr;
    VramLocation vram_{};
    Sdma6Config config_{};
    DoorbellIo doorbells_{};
    Sdma6Recovery recovery_{};
    Sdma6Report report_{};
    Phase phase_ = Phase::uninitialized;
    memory::Manager* manager_ = nullptr;
    memory::Handle handles_[sdma6_instance_count + 1]{};
    uint8_t* ring_cpu_[sdma6_instance_count]{};
    uint8_t* writeback_cpu_ = nullptr;
    memory::Handle ib_handle_{};
    uint8_t* ib_cpu_ = nullptr;
    FenceManager* fences_ = nullptr;
    RingPointerStore wptr_store_{};
    IbLease leases_[sdma6_instance_count][fence_slots]{};
};

} // namespace rx7900::hardware

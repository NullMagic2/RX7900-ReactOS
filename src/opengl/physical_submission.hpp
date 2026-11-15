/*
Module: rx7900.opengl.physical_submission
Purpose: Retain trusted graphics-job resources through an actual kernel-ring fence.
Created: 2026-10-23
Architecture: Physical GFX11 submission ownership shared by future Mesa/RADV
integration and trusted built-in graphics producers; no user raw-PM4 ABI. The
production producer is ProcessGraphics' kernel gfx-ring route (2026-10-24): it
submits kernel-built draw IBs as process-VM jobs (Gfx11VmJob) through
KernelGraphicsBridge, one owner per job slot.
*/
#pragma once

#include "../hardware/gfx11_queues.hpp"
#include "../hardware/fence.hpp"

namespace rx7900::opengl {

constexpr uint32_t graphics_job_resource_capacity = 8;

struct GraphicsResource final {
    memory::Slice slice{};
    uint64_t byte_count = 0;
    memory::Access access = memory::Access::read;
};

struct TrustedGraphicsJob final {
    // The IB is created exclusively by a trusted in-driver PM4 producer. This
    // structure is never copied from a user buffer or accepted by an IOCTL.
    // Every resource slice names its exact referenced range and access.
    memory::Handle ib{};
    uint64_t ib_offset = 0;
    uint32_t ib_dwords = 0;
    GraphicsResource resources[graphics_job_resource_capacity]{};
    uint32_t resource_count = 0;
    // in_vm: the IB runs in a process GPUVM (gfx ring VMID 1..7, vm.ib_va maps
    // ib/ib_offset read-only); otherwise it is VMID 0 work at its MC/GART address.
    bool in_vm = false;
    hardware::Gfx11VmJob vm{};
};

struct GraphicsSubmitter final {
    void* context = nullptr;
    // submit reuses Gfx11Queues::submit on ring 0, which itself leases the IB.
    hardware::Gfx11QueuesStatus (*submit)(void*, const hardware::Gfx11Submission&,
                                         uint32_t&) noexcept = nullptr;
    hardware::FenceStatus (*query)(void*, uint32_t) noexcept = nullptr;
    hardware::Gfx11QueuesStatus (*retire)(void*) noexcept = nullptr;
    // True only after the device owner completed Gfx11Queues::release or
    // release_after_reset and detached the corresponding FenceManager ring, or
    // while its confirmed destructive-reset window is open (the reset proof
    // covers every outstanding fence and IB before the rings are released).
    bool (*retired_after_reset)(void*) noexcept = nullptr;
};

enum class GraphicsJobStatus {
    ready, submitted, signalled, busy, invalid_argument, invalid_memory,
    unavailable, quarantined
};

// PASSIVE_LEVEL, externally serialized with Manager, Gfx11Queues and fences.
// No submission from untrusted PM4 is authorized by this class. Every resource
// stays leased until a real fence signals, or the device owner proves queue/VM
// retirement after reset. The caller owns and keeps the IB until the underlying
// Gfx11Queues lease retires; this class never adds a second IB lease.
// submit outcomes: submitted; busy (ring_full: nothing written, retry);
// unavailable (the queue refused before anything reached the CP: submission
// disabled or stopped, ring already hung, an invalid frame or a fence that was
// never published) with every resource returned; quarantined (a doorbell, MMIO
// or ring-store failure, or an ambiguous invalid_memory, after which the CP may
// still run the IB) with every resource kept until retire_after_reset.
class PhysicalGraphicsSubmission final {
public:
    PhysicalGraphicsSubmission() noexcept = default;
    PhysicalGraphicsSubmission(const PhysicalGraphicsSubmission&) = delete;
    PhysicalGraphicsSubmission& operator=(const PhysicalGraphicsSubmission&) = delete;

    GraphicsJobStatus initialize(memory::Manager& manager,
                                 GraphicsSubmitter submitter) noexcept;
    GraphicsJobStatus submit(const TrustedGraphicsJob& job,
                             uint32_t& sequence) noexcept;
    GraphicsJobStatus poll() noexcept;
    GraphicsJobStatus query(uint32_t sequence) noexcept;
    GraphicsJobStatus close_idle() noexcept;
    // Only after the queue/fence owner proved destructive reset quiescence.
    GraphicsJobStatus retire_after_reset() noexcept;
    bool pending() const noexcept { return pending_ || quarantined_; }
    bool initialized() const noexcept { return manager_ != nullptr; }

private:
    GraphicsJobStatus release_resources() noexcept;
    memory::Manager* manager_ = nullptr;
    GraphicsSubmitter submitter_{};
    GraphicsResource retained_[graphics_job_resource_capacity]{};
    uint32_t retained_count_ = 0;
    uint32_t sequence_ = 0;
    bool pending_ = false;
    bool quarantined_ = false;
};

// Production adapter. Its storage and the queue/fence objects outlive every
// PhysicalGraphicsSubmission using its callback view.
struct KernelGraphicsBridge final {
    hardware::Gfx11Queues* queues = nullptr;
    hardware::FenceManager* fences = nullptr;
    // Optional: true inside the device owner's confirmed-reset window
    // (AsicInit::reset_proven while release_after_reset retires its owners).
    void* reset_context = nullptr;
    bool (*reset_confirmed)(void*) noexcept = nullptr;
    GraphicsSubmitter submitter() noexcept;
};

} // namespace rx7900::opengl

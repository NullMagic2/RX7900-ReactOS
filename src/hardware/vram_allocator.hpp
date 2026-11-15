/*
Module: rx7900.hardware.vram_allocator
Purpose: Allocate whole local VRAM with Linux amdgpu_vram_mgr and drm_buddy semantics.
Created: 2026-10-22
Architecture: Pure bounded allocator over VRAM byte offsets; owns no mapping, no
MMIO and no OS resource. vram_backend adapts it to memory::Manager and the
native vram_window module supplies BAR0 CPU access.

Adapted from Linux v6.12 drivers/gpu/drm/drm_buddy.c and include/drm/drm_buddy.h
(Copyright 2021 Intel Corporation; file SPDX-License-Identifier MIT, module
MODULE_LICENSE "Dual MIT/GPL"; this port is used under the MIT terms) and
drivers/gpu/drm/amd/amdgpu/amdgpu_vram_mgr.c, amdgpu_object.c and amdgpu_ttm.c
(MIT-style permission notice, Copyright 2016 and later Advanced Micro Devices,
Inc.). The full MIT permission notice is reproduced in vram_allocator.cpp.
Pinned sources and SHA-256 sums: work/upstream-gpuinit/SHA256SUMS.
*/
#pragma once

#include "manager.hpp"

namespace rx7900::hardware {

// drm_buddy chunk_size and amdgpu_vram_mgr default_page_size: PAGE_SIZE.
constexpr uint64_t vram_chunk_bytes = memory::page_bytes;
// amdgpu_vram_mgr_new pages_per_block for non-contiguous BOs (HPAGE_PMD_NR on
// x86-64, else the explicit 2 MiB default): 2 MiB blocks when the size allows.
constexpr uint64_t vram_default_block_bytes = 2ULL << 20;
// amdgpu_vm.h AMDGPU_VM_RESERVED_VRAM: headroom non-kernel BOs may not consume.
constexpr uint64_t vram_vm_reserved_bytes = 8ULL << 20;

// Fixed metadata bounds (no heap). Every buddy block, free or allocated, and
// every split interior node is one 16-byte node. A trimmed contiguous request
// costs at most about 3 * order nodes (a split path to each edge plus the
// trimmed pieces); with order 22 for 16 GiB roots this is under 70 nodes, so
// 8192 nodes cover the 256 allocation slots at more than 30 nodes each in the
// worst fragmentation and far more in typical use (kernel BOs split one path).
// The node pool costs 128 KiB of device-owned storage. Exhaustion fails the
// allocation with no_metadata and changes nothing; release never needs nodes.
constexpr size_t vram_node_capacity = 8192;
// 64 memory::Manager slots per pool, two pools, the reservation carve-outs and
// direct GEM-like users (scanout, cursor, staging) fit with headroom.
constexpr size_t vram_allocation_capacity = 256;
// VBIOS vram_usagebyfirmware, driver usage, discovery TMR, training C2P,
// stolen VGA, extended and reserved memory, plus RAS-style retired ranges.
constexpr size_t vram_reservation_capacity = 32;
// Page numbers are 32-bit: at most 2^32 pages (16 TiB) of VRAM, order <= 32.
constexpr uint32_t vram_max_order = 32;

enum class VramStatus {
    success,
    invalid_argument,
    uninitialized,
    busy,
    // drm_buddy -ENOSPC: no suitable free VRAM for the placement.
    no_space,
    // Linux -ENOMEM equivalent: the fixed node, slot or DFS storage is exhausted.
    no_metadata,
    stale_handle,
    // amdgpu_vram_mgr_reserve_range accepted a range still in use; it is claimed
    // as soon as the overlapping allocations are released.
    pending,
    not_reserved
};

// Half-open VRAM byte offsets [start, end). Offsets are not MC addresses: MC =
// VramLocation.mc_start + offset. Reservation inputs need not be page aligned;
// they are widened outward to whole pages (fail-closed).
struct VramRange final {
    uint64_t start;
    uint64_t end;
};

// real_vram_size and visible_vram_size (gmc_v11_0_mc_init: BAR0 aperture size
// clamped to VRAM). Both must be page multiples; visible may equal vram.
struct VramLayout final {
    uint64_t vram_bytes;
    uint64_t visible_bytes;
};

// The amdgpu_bo creation flags and BO type that decide VRAM placement.
struct VramBoFlags final {
    bool cpu_access_required;   // AMDGPU_GEM_CREATE_CPU_ACCESS_REQUIRED
    bool no_cpu_access;         // AMDGPU_GEM_CREATE_NO_CPU_ACCESS
    bool vram_contiguous;       // AMDGPU_GEM_CREATE_VRAM_CONTIGUOUS
    bool kernel;                // ttm_bo_type_kernel
    bool pinned;                // placed through amdgpu_bo_pin
};

// One ttm_place for TTM_PL_VRAM, in bytes. last_byte 0 means no limit (lpfn 0).
struct VramPlacement final {
    uint64_t first_byte;
    uint64_t last_byte;
    bool topdown;      // TTM_PL_FLAG_TOPDOWN
    bool contiguous;   // TTM_PL_FLAG_CONTIGUOUS: no non-contiguous fallback
};

// amdgpu_bo_create_reserved/amdgpu_bo_create_kernel: a pinned contiguous kernel
// BO, CPU_ACCESS_REQUIRED with a CPU pointer, otherwise NO_CPU_ACCESS.
VramBoFlags vram_kernel_bo_flags(bool cpu_access) noexcept;

// amdgpu_bo_pin flag forcing followed by amdgpu_bo_placement_from_domain for
// AMDGPU_GEM_DOMAIN_VRAM without memory partitions: CPU access limits lpfn to
// the visible size (bottom-up range search); otherwise the place is TOPDOWN
// over all VRAM. A kernel or pinned contiguous BO gets TTM_PL_FLAG_CONTIGUOUS.
VramPlacement vram_placement(const VramBoFlags& flags, uint64_t visible_bytes) noexcept;

// bytes: nonzero page multiple. alignment: 0 or a power of two (the BO
// byte_align; values below a page mean one page, as amdgpu_bo_create rounds).
struct VramRequest final {
    uint64_t bytes;
    uint64_t alignment;
    VramBoFlags flags;
};

struct VramAllocationHandle final {
    uint32_t slot = UINT32_MAX;
    uint32_t generation = 0;
};

struct VramAllocationInfo final {
    uint64_t requested_bytes;
    // Sum of buddy block sizes; can exceed the request when trimming failed for
    // lack of metadata (Linux ignores drm_buddy_block_trim errors the same way).
    uint64_t allocated_bytes;
    // amdgpu_vram_mgr_vis_size summed over blocks.
    uint64_t visible_bytes;
    uint64_t first_byte;
    uint32_t block_count;
    // amdgpu_is_vram_mgr_blocks_contiguous over the block list order.
    bool contiguous;
    bool cpu_access_required;
    // AMDGPU_GEM_CREATE_NO_CPU_ACCESS: amdgpu_bo_kmap refuses such a BO.
    bool no_cpu_access;
};

struct VramBlock final {
    uint64_t start;
    uint64_t bytes;
};

// Iteration state for one allocation's block list; valid only while that
// allocation is live. Blocks appear in drm_buddy list order (ascending for
// contiguous and trimmed allocations, allocation order otherwise).
struct VramBlockCursor final {
    VramAllocationHandle allocation{};
    uint16_t node = UINT16_MAX;
};

// Device-owned, about 140 KiB, trivially destructible and value-initialized.
// Calls are externally serialized (the Linux mgr->lock); no callback is made,
// so any IRQL at which the storage is resident is acceptable. shutdown is the
// only teardown and requires that every allocation was released.
class VramAllocator final {
public:
    VramAllocator() noexcept = default;
    VramAllocator(const VramAllocator&) = delete;
    VramAllocator& operator=(const VramAllocator&) = delete;

    // drm_buddy_init over all of VRAM, then the caller's reserved ranges are
    // claimed exactly (amdgpu_bo_create_kernel_at for firmware, TMR, training
    // and stolen memory). reserved may be null only when reserved_count is 0.
    // Overlapping or adjacent ranges are merged; a range past VRAM, more than
    // vram_reservation_capacity merged ranges, or metadata exhaustion fails the
    // whole initialization with nothing retained.
    VramStatus initialize(VramLayout layout, const VramRange* reserved,
                          size_t reserved_count) noexcept;
    // drm_buddy_fini: busy while any allocation remains; reservations are freed.
    VramStatus shutdown() noexcept;

    // amdgpu_vram_mgr_new with the placement derived by vram_placement.
    // no_space leaves all state unchanged; no_metadata likewise.
    VramStatus allocate(const VramRequest& request, VramAllocationHandle& handle) noexcept;
    // amdgpu_bo_create_kernel_at: exactly [offset, offset + bytes), both page
    // aligned. A CPU-access request must lie wholly inside visible VRAM.
    VramStatus allocate_at(uint64_t offset, uint64_t bytes, const VramBoFlags& flags,
                           VramAllocationHandle& handle) noexcept;
    // amdgpu_vram_mgr_del: frees and coalesces every block, then retries pending
    // reservations. Never needs metadata, so it cannot fail for a live handle.
    VramStatus release(VramAllocationHandle handle) noexcept;

    VramStatus describe(VramAllocationHandle handle, VramAllocationInfo& info) const noexcept;
    VramStatus first_block(VramAllocationHandle handle, VramBlockCursor& cursor,
                           VramBlock& block) const noexcept;
    // no_space marks the end of the list.
    VramStatus next_block(VramBlockCursor& cursor, VramBlock& block) const noexcept;

    // amdgpu_vram_mgr_reserve_range: page aligned [start, start + bytes) inside
    // VRAM. Free parts are claimed at once (never handed out again); success
    // when the whole range is claimed, pending while allocations still overlap.
    VramStatus reserve_range(uint64_t start, uint64_t bytes) noexcept;
    // amdgpu_vram_mgr_query_page_status: busy (pending), success (reserved) or
    // not_reserved for the page holding offset.
    VramStatus query_page_status(uint64_t offset) const noexcept;

    bool initialized() const noexcept { return initialized_; }
    uint64_t vram_bytes() const noexcept { return layout_.vram_bytes; }
    uint64_t visible_bytes() const noexcept { return layout_.visible_bytes; }
    // ttm_resource_manager_usage: requested BO sizes plus claimed reservations.
    uint64_t usage_bytes() const noexcept { return usage_bytes_; }
    // amdgpu_vram_mgr_vis_usage.
    uint64_t visible_usage_bytes() const noexcept { return visible_usage_bytes_; }
    // drm_buddy avail: bytes in free blocks.
    uint64_t available_bytes() const noexcept { return available_pages_ * vram_chunk_bytes; }
    // Largest free block (one buddy block) in bytes, for diagnostics and tests.
    uint64_t largest_free_block_bytes() const noexcept;
    size_t spare_nodes() const noexcept { return spare_count_; }

private:
    static constexpr uint16_t no_node = UINT16_MAX;
    static constexpr size_t stack_capacity = 160;
    static constexpr size_t root_capacity = vram_max_order + 1;

    enum class NodeState : uint8_t { unused, free, split, allocated };

    struct Node final {
        uint32_t page = 0;
        uint8_t order = 0;
        NodeState state = NodeState::unused;
        uint16_t parent = no_node;
        uint16_t left = no_node;
        uint16_t right = no_node;
        // Free nodes: sorted free_list links. Allocated nodes: owner list links.
        uint16_t prev = no_node;
        uint16_t next = no_node;
    };

    struct BlockList final {
        uint16_t head = no_node;
        uint16_t tail = no_node;
        uint32_t count = 0;
    };

    struct BuddyFlags final {
        bool range;
        bool topdown;
        bool contiguous;
    };

    struct Allocation final {
        BlockList blocks{};
        uint64_t requested_pages = 0;
        uint64_t visible_bytes = 0;
        uint32_t generation = 0;
        bool used = false;
        bool cpu_access_required = false;
        bool no_cpu_access = false;
    };

    struct Reservation final {
        BlockList blocks{};
        uint64_t start_page = 0;
        uint64_t pages = 0;
        uint64_t claimed_pages = 0;
        uint64_t visible_bytes = 0;
        bool used = false;
    };

    // Node pool and free lists (drm_block_alloc, list_insert_sorted, mark_*).
    uint16_t take_node(uint16_t parent, uint32_t order, uint64_t page) noexcept;
    void return_node(uint16_t index) noexcept;
    uint64_t block_pages(uint16_t index) const noexcept;
    void insert_free(uint16_t index) noexcept;
    void remove_free(uint16_t index) noexcept;
    void mark_free(uint16_t index) noexcept;
    void mark_allocated(uint16_t index) noexcept;
    void mark_split(uint16_t index) noexcept;
    uint16_t buddy_of(uint16_t index) const noexcept;
    bool split_block(uint16_t index) noexcept;
    void free_block(uint16_t index) noexcept;
    void free_list(BlockList& list) noexcept;
    void append(BlockList& list, uint16_t index) noexcept;
    void splice_tail(BlockList& list, BlockList& from) noexcept;
    void splice_head(BlockList& list, BlockList& from) noexcept;
    void collapse_free_subtree(uint16_t index) noexcept;
    void normalize() noexcept;

    // DFS stack in device storage (Linux tmp_link lists), never the kernel stack.
    bool push(size_t& depth, uint16_t index) noexcept;
    bool push_roots(size_t& depth) noexcept;

    // drm_buddy algorithm ports, page units.
    VramStatus alloc_range(size_t depth, uint64_t start, uint64_t pages, BlockList& blocks,
                           uint64_t* allocated_on_no_space) noexcept;
    VramStatus alloc_range_bias(uint64_t start, uint64_t end, uint32_t order,
                                uint16_t& block) noexcept;
    VramStatus alloc_from_free_list(uint32_t order, bool topdown, uint16_t& block) noexcept;
    VramStatus alloc_contig_try_harder(uint64_t pages, uint64_t min_block_pages,
                                       BlockList& blocks) noexcept;
    VramStatus trim(BlockList& list, uint64_t new_pages) noexcept;
    VramStatus alloc_blocks(uint64_t start, uint64_t end, uint64_t pages,
                            uint64_t min_block_pages, BuddyFlags flags,
                            BlockList& blocks) noexcept;

    VramStatus allocate_placed(uint64_t bytes, uint64_t alignment, const VramBoFlags& flags,
                               const VramPlacement& place, VramAllocationHandle& handle) noexcept;
    VramStatus claim_range(Reservation& reservation) noexcept;
    void retry_pending() noexcept;
    uint64_t visible_bytes_of(const BlockList& list) const noexcept;
    const Allocation* find(VramAllocationHandle handle) const noexcept;
    void reset() noexcept;

    Node nodes_[vram_node_capacity]{};
    uint16_t spare_head_ = no_node;
    size_t spare_count_ = 0;
    uint16_t free_head_[vram_max_order + 1]{};
    uint16_t free_tail_[vram_max_order + 1]{};
    uint16_t roots_[root_capacity]{};
    uint32_t root_count_ = 0;
    uint32_t max_order_ = 0;
    uint64_t total_pages_ = 0;
    uint64_t available_pages_ = 0;
    uint16_t stack_[stack_capacity]{};
    Allocation allocations_[vram_allocation_capacity]{};
    Reservation reservations_[vram_reservation_capacity]{};
    VramLayout layout_{};
    uint64_t usage_bytes_ = 0;
    uint64_t visible_usage_bytes_ = 0;
    uint32_t generation_ = 0;
    bool initialized_ = false;
};

} // namespace rx7900::hardware

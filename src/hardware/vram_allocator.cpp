/*
Module: rx7900.hardware.vram_allocator
Purpose: Port drm_buddy and amdgpu_vram_mgr placement to fixed device storage.
Created: 2026-10-22
Architecture: Pure C++ over page numbers; no OS, MMIO or mapping dependency.

Adapted from Linux v6.12 drivers/gpu/drm/drm_buddy.c (SPDX-License-Identifier:
MIT, MODULE_LICENSE "Dual MIT/GPL", used here under MIT; Copyright 2021 Intel
Corporation) and drivers/gpu/drm/amd/amdgpu/
amdgpu_vram_mgr.c, amdgpu_object.c and amdgpu_ttm.c (Copyright 2016 and later
Advanced Micro Devices, Inc.; author of amdgpu_vram_mgr.c: Christian Koenig).

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions: The above copyright notice and this
permission notice shall be included in all copies or substantial portions of
the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO
EVENT SHALL THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
OR OTHER DEALINGS IN THE SOFTWARE.

Deliberate divergences from Linux, all fail-closed:
- No clear-state tracking (DRM_BUDDY_CLEAR_ALLOCATION/CLEARED): every block is
  dirty, so force_merge and the incompatible-block fallback never apply.
- Free buddy pairs left split by a failed or exploratory search are merged at
  the end of every public operation (Linux leaves them until a neighbour frees),
  keeping metadata use minimal; free-list order choice can then differ slightly.
- A contiguous request keeps its alignment: the power-of-two block is at least
  the alignment (Linux replaces min_block_size by the rounded size), and the
  try-harder search skips candidates whose start would be misaligned.
- Pending reservations claim their free parts at once and the rest as soon as
  an overlapping allocation is released (Linux retries all-or-nothing, before
  freeing, so a free part of a pending range can still be handed out).
- Reservation input is widened outward to whole pages; overlaps are merged.
*/
#include "vram_allocator.hpp"

namespace rx7900::hardware {
namespace {

constexpr uint64_t chunk_pages(uint64_t bytes) noexcept {
    return bytes / vram_chunk_bytes;
}

bool power_of_two(uint64_t value) noexcept {
    return value != 0 && (value & (value - 1)) == 0;
}

// fls(value) - 1 for nonzero value.
uint32_t floor_log2(uint64_t value) noexcept {
    uint32_t result = 0;

    while (value > 1) {
        value >>= 1;
        ++result;
    }

    return result;
}

uint64_t round_up_power_of_two(uint64_t value) noexcept {
    uint64_t result = 1;

    while (result < value) {
        result <<= 1;
    }

    return result;
}

bool overlaps(uint64_t s1, uint64_t e1, uint64_t s2, uint64_t e2) noexcept {
    return s1 <= e2 && e1 >= s2;
}

bool contains(uint64_t s1, uint64_t e1, uint64_t s2, uint64_t e2) noexcept {
    return s1 <= s2 && e1 >= e2;
}

// amdgpu_bo_pin: anything not marked NO_CPU_ACCESS is forced into visible VRAM.
VramBoFlags effective_flags(const VramBoFlags& flags) noexcept {
    VramBoFlags result = flags;

    if (result.pinned && !result.no_cpu_access) {
        result.cpu_access_required = true;
    }

    return result;
}

uint64_t visible_overlap(uint64_t start, uint64_t bytes, uint64_t visible) noexcept {
    if (start >= visible) {
        return 0;
    }

    const uint64_t end = start + bytes;
    return (end > visible ? visible : end) - start;
}

} // namespace

VramBoFlags vram_kernel_bo_flags(bool cpu_access) noexcept {
    return {cpu_access, !cpu_access, true, true, true};
}

VramPlacement vram_placement(const VramBoFlags& flags, uint64_t visible_bytes) noexcept {
    const VramBoFlags effective = effective_flags(flags);
    VramPlacement place{0, 0, false, false};

    if (effective.cpu_access_required) {
        place.last_byte = visible_bytes;
    } else {
        place.topdown = true;
    }

    // placement_from_domain marks kernel contiguous BOs; amdgpu_bo_pin marks
    // every pinned contiguous BO.
    place.contiguous = effective.vram_contiguous && (effective.kernel || effective.pinned);
    return place;
}

// ---------------------------------------------------------------- node pool

void VramAllocator::reset() noexcept {
    for (size_t index = 0; index < vram_node_capacity; ++index) {
        nodes_[index] = Node{};
        nodes_[index].next = index + 1 < vram_node_capacity ?
            static_cast<uint16_t>(index + 1) : no_node;
    }

    spare_head_ = 0;
    spare_count_ = vram_node_capacity;

    for (uint32_t order = 0; order <= vram_max_order; ++order) {
        free_head_[order] = no_node;
        free_tail_[order] = no_node;
    }

    for (auto& root : roots_) {
        root = no_node;
    }

    for (auto& allocation : allocations_) {
        allocation = Allocation{};
    }

    for (auto& reservation : reservations_) {
        reservation = Reservation{};
    }

    root_count_ = 0;
    max_order_ = 0;
    total_pages_ = 0;
    available_pages_ = 0;
    layout_ = {};
    usage_bytes_ = 0;
    visible_usage_bytes_ = 0;
    initialized_ = false;
}

uint16_t VramAllocator::take_node(uint16_t parent, uint32_t order, uint64_t page) noexcept {
    const uint16_t index = spare_head_;

    if (index == no_node) {
        return no_node;
    }

    spare_head_ = nodes_[index].next;
    --spare_count_;
    nodes_[index] = Node{};
    nodes_[index].page = static_cast<uint32_t>(page);
    nodes_[index].order = static_cast<uint8_t>(order);
    nodes_[index].parent = parent;
    return index;
}

void VramAllocator::return_node(uint16_t index) noexcept {
    nodes_[index] = Node{};
    nodes_[index].next = spare_head_;
    spare_head_ = index;
    ++spare_count_;
}

uint64_t VramAllocator::block_pages(uint16_t index) const noexcept {
    return 1ULL << nodes_[index].order;
}

// list_insert_sorted: each order's free list ascends by offset.
void VramAllocator::insert_free(uint16_t index) noexcept {
    const uint32_t order = nodes_[index].order;
    uint16_t after = no_node;
    uint16_t before = free_head_[order];

    while (before != no_node && nodes_[before].page < nodes_[index].page) {
        after = before;
        before = nodes_[before].next;
    }

    nodes_[index].prev = after;
    nodes_[index].next = before;

    if (after == no_node) {
        free_head_[order] = index;
    } else {
        nodes_[after].next = index;
    }

    if (before == no_node) {
        free_tail_[order] = index;
    } else {
        nodes_[before].prev = index;
    }
}

void VramAllocator::remove_free(uint16_t index) noexcept {
    const uint32_t order = nodes_[index].order;
    const uint16_t prev = nodes_[index].prev;
    const uint16_t next = nodes_[index].next;

    if (prev == no_node) {
        free_head_[order] = next;
    } else {
        nodes_[prev].next = next;
    }

    if (next == no_node) {
        free_tail_[order] = prev;
    } else {
        nodes_[next].prev = prev;
    }

    nodes_[index].prev = no_node;
    nodes_[index].next = no_node;
}

void VramAllocator::mark_free(uint16_t index) noexcept {
    nodes_[index].state = NodeState::free;
    insert_free(index);
}

void VramAllocator::mark_allocated(uint16_t index) noexcept {
    remove_free(index);
    nodes_[index].state = NodeState::allocated;
}

void VramAllocator::mark_split(uint16_t index) noexcept {
    remove_free(index);
    nodes_[index].state = NodeState::split;
}

uint16_t VramAllocator::buddy_of(uint16_t index) const noexcept {
    const uint16_t parent = nodes_[index].parent;

    if (parent == no_node) {
        return no_node;
    }

    return nodes_[parent].left == index ? nodes_[parent].right : nodes_[parent].left;
}

// split_block: both children or neither (two spare nodes are checked first).
bool VramAllocator::split_block(uint16_t index) noexcept {
    const uint32_t order = nodes_[index].order;

    if (order == 0 || nodes_[index].state != NodeState::free || spare_count_ < 2) {
        return false;
    }

    const uint64_t page = nodes_[index].page;
    const uint16_t left = take_node(index, order - 1, page);
    const uint16_t right = take_node(index, order - 1, page + (1ULL << (order - 1)));
    nodes_[index].left = left;
    nodes_[index].right = right;
    mark_split(index);
    mark_free(left);
    mark_free(right);
    return true;
}

// drm_buddy_free_block and __drm_buddy_free: coalesce upward while the buddy
// is free. The caller has already unlinked an allocated block from its list.
void VramAllocator::free_block(uint16_t index) noexcept {
    if (nodes_[index].state == NodeState::allocated) {
        available_pages_ += block_pages(index);
    }

    while (nodes_[index].parent != no_node) {
        const uint16_t buddy = buddy_of(index);

        if (buddy == no_node || nodes_[buddy].state != NodeState::free) {
            break;
        }

        const uint16_t parent = nodes_[index].parent;
        remove_free(buddy);
        return_node(index);
        return_node(buddy);
        nodes_[parent].left = no_node;
        nodes_[parent].right = no_node;
        index = parent;
    }

    mark_free(index);
}

void VramAllocator::free_list(BlockList& list) noexcept {
    uint16_t index = list.head;

    while (index != no_node) {
        const uint16_t next = nodes_[index].next;
        nodes_[index].prev = no_node;
        nodes_[index].next = no_node;
        free_block(index);
        index = next;
    }

    list = {};
}

void VramAllocator::append(BlockList& list, uint16_t index) noexcept {
    nodes_[index].prev = list.tail;
    nodes_[index].next = no_node;

    if (list.tail == no_node) {
        list.head = index;
    } else {
        nodes_[list.tail].next = index;
    }

    list.tail = index;
    ++list.count;
}

void VramAllocator::splice_tail(BlockList& list, BlockList& from) noexcept {
    if (from.head == no_node) {
        return;
    }

    if (list.head == no_node) {
        list = from;
    } else {
        nodes_[list.tail].next = from.head;
        nodes_[from.head].prev = list.tail;
        list.tail = from.tail;
        list.count += from.count;
    }

    from = {};
}

void VramAllocator::splice_head(BlockList& list, BlockList& from) noexcept {
    BlockList combined = from;
    from = {};
    splice_tail(combined, list);
    list = combined;
}

// Used only after a failed trim: every descendant is free; restore one block.
void VramAllocator::collapse_free_subtree(uint16_t index) noexcept {
    if (nodes_[index].state == NodeState::free) {
        remove_free(index);
        return;
    }

    if (nodes_[index].state != NodeState::split) {
        return;
    }

    size_t depth = 0;
    push(depth, nodes_[index].left);
    push(depth, nodes_[index].right);

    while (depth != 0) {
        const uint16_t node = stack_[--depth];

        if (nodes_[node].state == NodeState::split) {
            push(depth, nodes_[node].left);
            push(depth, nodes_[node].right);
        } else if (nodes_[node].state == NodeState::free) {
            remove_free(node);
        }

        return_node(node);
    }

    nodes_[index].left = no_node;
    nodes_[index].right = no_node;
}

// Merge every free buddy pair. A free right buddy directly follows its left
// buddy in the ascending list of their order; merged parents land in the next
// order's list, which the ascending loop visits afterwards.
void VramAllocator::normalize() noexcept {
    for (uint32_t order = 0; order < max_order_; ++order) {
        uint16_t index = free_head_[order];

        while (index != no_node) {
            uint16_t next = nodes_[index].next;
            const uint16_t parent = nodes_[index].parent;

            if (parent != no_node && nodes_[parent].left == index && next != no_node &&
                nodes_[parent].right == next) {
                const uint16_t after = nodes_[next].next;
                remove_free(index);
                remove_free(next);
                return_node(index);
                return_node(next);
                nodes_[parent].left = no_node;
                nodes_[parent].right = no_node;
                mark_free(parent);
                next = after;
            }

            index = next;
        }
    }
}

bool VramAllocator::push(size_t& depth, uint16_t index) noexcept {
    if (depth >= stack_capacity || index == no_node) {
        return false;
    }

    stack_[depth++] = index;
    return true;
}

// Roots in reverse so the first root is visited first (list_add_tail order).
bool VramAllocator::push_roots(size_t& depth) noexcept {
    for (uint32_t index = root_count_; index != 0; --index) {
        if (!push(depth, roots_[index - 1])) {
            return false;
        }
    }

    return true;
}

// ------------------------------------------------------------ buddy algorithms

// __alloc_range: claim exactly [start, start + pages) from the nodes already on
// the DFS stack. Children are pushed right then left so lower offsets come first.
VramStatus VramAllocator::alloc_range(size_t depth, uint64_t start, uint64_t pages,
                                      BlockList& blocks, uint64_t* allocated_on_no_space) noexcept {
    const uint64_t end = start + pages - 1;
    uint64_t total = 0;
    BlockList allocated{};
    VramStatus status = VramStatus::success;

    while (depth != 0) {
        const uint16_t block = stack_[--depth];
        const uint64_t block_start = nodes_[block].page;
        const uint64_t block_end = block_start + block_pages(block) - 1;

        if (!overlaps(start, end, block_start, block_end)) {
            continue;
        }

        if (nodes_[block].state == NodeState::allocated) {
            status = VramStatus::no_space;
            break;
        }

        if (contains(start, end, block_start, block_end)) {
            if (nodes_[block].state == NodeState::free) {
                mark_allocated(block);
                total += block_pages(block);
                available_pages_ -= block_pages(block);
                append(allocated, block);
                continue;
            }

            // A split block inside the range holds an allocated descendant
            // (free buddy pairs are always merged): Linux !clear_avail -ENOSPC.
            status = VramStatus::no_space;
            break;
        }

        if (nodes_[block].state == NodeState::free && !split_block(block)) {
            status = VramStatus::no_metadata;
            break;
        }

        if (!push(depth, nodes_[block].right) || !push(depth, nodes_[block].left)) {
            status = VramStatus::no_metadata;
            break;
        }
    }

    if (status == VramStatus::success && total < pages) {
        status = VramStatus::no_space;
    }

    if (status == VramStatus::success) {
        splice_tail(blocks, allocated);
        return status;
    }

    if (status == VramStatus::no_space && allocated_on_no_space != nullptr) {
        splice_tail(blocks, allocated);
        *allocated_on_no_space = total;
    } else {
        free_list(allocated);
    }

    return status;
}

// __alloc_range_bias: first block of the order fitting [start, end) in
// ascending DFS order. end is exclusive here.
VramStatus VramAllocator::alloc_range_bias(uint64_t start, uint64_t end, uint32_t order,
                                           uint16_t& block) noexcept {
    const uint64_t request = 1ULL << order;
    const uint64_t last = end - 1;
    size_t depth = 0;
    block = no_node;

    if (!push_roots(depth)) {
        return VramStatus::no_metadata;
    }

    while (depth != 0) {
        const uint16_t node = stack_[--depth];

        if (nodes_[node].order < order) {
            continue;
        }

        const uint64_t node_start = nodes_[node].page;
        const uint64_t node_end = node_start + block_pages(node) - 1;

        if (!overlaps(start, last, node_start, node_end) ||
            nodes_[node].state == NodeState::allocated) {
            continue;
        }

        if (node_start < start || node_end > last) {
            const uint64_t adjusted_start = node_start > start ? node_start : start;
            const uint64_t adjusted_end = node_end < last ? node_end : last;
            const uint64_t aligned_end = (adjusted_end + 1) & ~(request - 1);
            const uint64_t aligned_start = (adjusted_start + request - 1) & ~(request - 1);

            if (aligned_end <= aligned_start) {
                continue;
            }
        }

        if (contains(start, last, node_start, node_end) && nodes_[node].order == order) {
            if (nodes_[node].state == NodeState::free) {
                block = node;
                return VramStatus::success;
            }

            continue;
        }

        if (nodes_[node].state == NodeState::free && !split_block(node)) {
            return VramStatus::no_metadata;
        }

        if (!push(depth, nodes_[node].right) || !push(depth, nodes_[node].left)) {
            return VramStatus::no_metadata;
        }
    }

    return VramStatus::no_space;
}

// alloc_from_freelist: TOPDOWN takes the highest-offset block of any order at
// least order (get_maxblock); otherwise the highest-offset block of the
// smallest sufficient order. The right half is kept on each split.
VramStatus VramAllocator::alloc_from_free_list(uint32_t order, bool topdown,
                                               uint16_t& block) noexcept {
    uint16_t found = no_node;
    uint32_t found_order = order;
    block = no_node;

    for (uint32_t current = order; current <= max_order_; ++current) {
        const uint16_t candidate = free_tail_[current];

        if (candidate == no_node) {
            continue;
        }

        if (!topdown) {
            found = candidate;
            found_order = current;
            break;
        }

        if (found == no_node || nodes_[candidate].page > nodes_[found].page) {
            found = candidate;
            found_order = current;
        }
    }

    if (found == no_node) {
        return VramStatus::no_space;
    }

    while (found_order != order) {
        if (!split_block(found)) {
            return VramStatus::no_metadata;
        }

        found = nodes_[found].right;
        --found_order;
    }

    block = found;
    return VramStatus::success;
}

// __alloc_contig_try_harder: around each free block of the largest power of two
// not above the size, from high to low offsets, try the right-hand extension and
// complete it on the left.
VramStatus VramAllocator::alloc_contig_try_harder(uint64_t pages, uint64_t min_block_pages,
                                                  BlockList& blocks) noexcept {
    const uint32_t order = floor_log2(pages);

    if (order == 0 || order > max_order_) {
        return VramStatus::no_space;
    }

    uint64_t limit = UINT64_MAX;

    for (;;) {
        uint16_t candidate = free_tail_[order];

        while (candidate != no_node && nodes_[candidate].page >= limit) {
            candidate = nodes_[candidate].prev;
        }

        if (candidate == no_node) {
            return VramStatus::no_space;
        }

        const uint64_t offset = nodes_[candidate].page;
        limit = offset;

        if ((offset & (min_block_pages - 1)) != 0) {
            continue;
        }

        BlockList right{};
        uint64_t filled = 0;
        size_t depth = 0;

        if (!push_roots(depth)) {
            return VramStatus::no_metadata;
        }

        VramStatus status = alloc_range(depth, offset, pages, right, &filled);

        if (status != VramStatus::no_space) {
            splice_tail(blocks, right);
            return status;
        }

        uint64_t left_pages = pages - filled > min_block_pages ? pages - filled : min_block_pages;
        left_pages = (left_pages + min_block_pages - 1) & ~(min_block_pages - 1);

        if (offset < left_pages || ((offset - left_pages) & (min_block_pages - 1)) != 0) {
            free_list(right);
            continue;
        }

        BlockList left{};
        depth = 0;

        if (!push_roots(depth)) {
            free_list(right);
            return VramStatus::no_metadata;
        }

        status = alloc_range(depth, offset - left_pages, left_pages, left, nullptr);

        if (status == VramStatus::success) {
            splice_head(right, left);
            splice_tail(blocks, right);
            return status;
        }

        free_list(right);

        if (status != VramStatus::no_space) {
            return status;
        }
    }
}

// drm_buddy_block_trim with start NULL: keep the first new_pages of the single
// block in list; on failure the original block is restored unchanged.
VramStatus VramAllocator::trim(BlockList& list, uint64_t new_pages) noexcept {
    if (list.count != 1) {
        return VramStatus::invalid_argument;
    }

    const uint16_t block = list.head;
    const uint64_t pages = block_pages(block);

    if (nodes_[block].state != NodeState::allocated || new_pages == 0 || new_pages > pages) {
        return VramStatus::invalid_argument;
    }

    if (new_pages == pages) {
        return VramStatus::success;
    }

    list = {};
    nodes_[block].prev = no_node;
    nodes_[block].next = no_node;
    mark_free(block);
    available_pages_ += pages;
    // Prevent the failure path from coalescing above this block.
    const uint16_t parent = nodes_[block].parent;
    nodes_[block].parent = no_node;
    size_t depth = 0;
    push(depth, block);
    const VramStatus status = alloc_range(depth, nodes_[block].page, new_pages, list, nullptr);

    if (status != VramStatus::success) {
        collapse_free_subtree(block);
        nodes_[block].state = NodeState::allocated;
        available_pages_ -= pages;
        list = {};
        append(list, block);
    }

    nodes_[block].parent = parent;
    return status;
}

// drm_buddy_alloc_blocks over page numbers; end is exclusive.
VramStatus VramAllocator::alloc_blocks(uint64_t start, uint64_t end, uint64_t pages,
                                       uint64_t min_block_pages, BuddyFlags flags,
                                       BlockList& blocks) noexcept {
    if (pages == 0 || !power_of_two(min_block_pages) || end > total_pages_ ||
        start >= total_pages_ || pages > total_pages_ - start) {
        return VramStatus::invalid_argument;
    }

    if (start + pages == end) {
        if (((start | end) & (min_block_pages - 1)) != 0) {
            return VramStatus::invalid_argument;
        }

        size_t depth = 0;

        if (!push_roots(depth)) {
            return VramStatus::no_metadata;
        }

        return alloc_range(depth, start, pages, blocks, nullptr);
    }

    const uint64_t original_pages = pages;
    const uint64_t original_min = min_block_pages;

    if (flags.contiguous) {
        pages = round_up_power_of_two(pages);

        if (pages < min_block_pages) {
            pages = min_block_pages;
        }

        min_block_pages = pages;
    } else if ((pages & (min_block_pages - 1)) != 0) {
        pages = (pages + min_block_pages - 1) & ~(min_block_pages - 1);
    }

    uint32_t order = floor_log2(pages);
    const uint32_t min_order = floor_log2(min_block_pages);

    if (min_order > max_order_) {
        return flags.contiguous && !flags.range ?
            alloc_contig_try_harder(original_pages, original_min, blocks) : VramStatus::no_space;
    }

    BlockList allocated{};
    uint64_t remaining = pages;
    bool metadata_failed = false;

    for (;;) {
        const uint32_t fit = floor_log2(remaining);
        order = order < fit ? order : fit;

        if (order > max_order_) {
            order = max_order_;
        }

        uint16_t block = no_node;

        for (;;) {
            const VramStatus status = flags.range ?
                alloc_range_bias(start, end, order, block) :
                alloc_from_free_list(order, flags.topdown, block);

            if (status == VramStatus::success) {
                break;
            }

            metadata_failed = metadata_failed || status == VramStatus::no_metadata;

            if (order == min_order) {
                if (flags.contiguous && !flags.range) {
                    free_list(allocated);
                    normalize();
                    return alloc_contig_try_harder(original_pages, original_min, blocks);
                }

                free_list(allocated);
                return metadata_failed ? VramStatus::no_metadata : VramStatus::no_space;
            }

            --order;
        }

        mark_allocated(block);
        available_pages_ -= block_pages(block);
        append(allocated, block);
        remaining -= 1ULL << order;

        if (remaining == 0) {
            break;
        }
    }

    // Trim the rounded allocation back to the request; a failed trim keeps the
    // larger block, exactly as Linux ignores the trim result.
    if (original_pages != pages) {
        if (allocated.count == 1) {
            trim(allocated, original_pages);
        } else {
            const uint16_t last = allocated.tail;
            const uint16_t before = nodes_[last].prev;
            nodes_[before].next = no_node;
            allocated.tail = before;
            --allocated.count;
            BlockList tail{};
            append(tail, last);
            trim(tail, block_pages(last) - (pages - original_pages));
            splice_tail(allocated, tail);
        }
    }

    splice_tail(blocks, allocated);
    return VramStatus::success;
}

// -------------------------------------------------------------- reservations

// Claim every free page of the reservation range; allocated blocks (other
// owners, or this reservation's earlier claims) are skipped.
VramStatus VramAllocator::claim_range(Reservation& reservation) noexcept {
    const uint64_t start = reservation.start_page;
    const uint64_t last = start + reservation.pages - 1;
    size_t depth = 0;
    VramStatus status = VramStatus::success;

    if (!push_roots(depth)) {
        return VramStatus::no_metadata;
    }

    while (depth != 0) {
        const uint16_t block = stack_[--depth];
        const uint64_t block_start = nodes_[block].page;
        const uint64_t block_end = block_start + block_pages(block) - 1;

        if (!overlaps(start, last, block_start, block_end) ||
            nodes_[block].state == NodeState::allocated) {
            continue;
        }

        if (nodes_[block].state == NodeState::free && contains(start, last, block_start, block_end)) {
            mark_allocated(block);
            available_pages_ -= block_pages(block);
            append(reservation.blocks, block);
            reservation.claimed_pages += block_pages(block);
            const uint64_t visible = visible_overlap(block_start * vram_chunk_bytes,
                                                     block_pages(block) * vram_chunk_bytes,
                                                     layout_.visible_bytes);
            reservation.visible_bytes += visible;
            visible_usage_bytes_ += visible;
            usage_bytes_ += block_pages(block) * vram_chunk_bytes;
            continue;
        }

        if (nodes_[block].state == NodeState::free && !split_block(block)) {
            status = VramStatus::no_metadata;
            break;
        }

        if (!push(depth, nodes_[block].right) || !push(depth, nodes_[block].left)) {
            status = VramStatus::no_metadata;
            break;
        }
    }

    return status;
}

void VramAllocator::retry_pending() noexcept {
    for (auto& reservation : reservations_) {
        if (reservation.used && reservation.claimed_pages != reservation.pages) {
            claim_range(reservation);
        }
    }
}

VramStatus VramAllocator::initialize(VramLayout layout, const VramRange* reserved,
                                     size_t reserved_count) noexcept {
    if (initialized_) {
        return VramStatus::busy;
    }

    const uint64_t pages = chunk_pages(layout.vram_bytes);

    if (layout.vram_bytes < vram_chunk_bytes || (layout.vram_bytes % vram_chunk_bytes) != 0 ||
        layout.visible_bytes == 0 || layout.visible_bytes > layout.vram_bytes ||
        (layout.visible_bytes % vram_chunk_bytes) != 0 || pages > (1ULL << vram_max_order) ||
        (reserved == nullptr && reserved_count != 0)) {
        return VramStatus::invalid_argument;
    }

    reset();
    size_t count = 0;

    // Sorted, merged, page-widened reservation list in reservations_[0, count).
    for (size_t input = 0; input < reserved_count; ++input) {
        const VramRange range = reserved[input];

        if (range.end < range.start || range.end > layout.vram_bytes) {
            reset();
            return VramStatus::invalid_argument;
        }

        if (range.end == range.start) {
            continue;
        }

        uint64_t first = range.start / vram_chunk_bytes;
        uint64_t end = (range.end + vram_chunk_bytes - 1) / vram_chunk_bytes;
        size_t write = 0;

        // Absorb every existing range that overlaps or touches [first, end).
        for (size_t read = 0; read < count; ++read) {
            const uint64_t other_first = reservations_[read].start_page;
            const uint64_t other_end = other_first + reservations_[read].pages;

            if (other_first <= end && other_end >= first) {
                first = other_first < first ? other_first : first;
                end = other_end > end ? other_end : end;
                continue;
            }

            reservations_[write++] = reservations_[read];
        }

        count = write;

        if (count == vram_reservation_capacity) {
            reset();
            return VramStatus::invalid_argument;
        }

        size_t position = count;

        while (position != 0 && reservations_[position - 1].start_page > first) {
            reservations_[position] = reservations_[position - 1];
            --position;
        }

        reservations_[position] = Reservation{};
        reservations_[position].used = true;
        reservations_[position].start_page = first;
        reservations_[position].pages = end - first;
        ++count;
    }

    for (size_t index = count; index < vram_reservation_capacity; ++index) {
        reservations_[index] = Reservation{};
    }

    // drm_buddy_init: power-of-two roots from offset zero, largest first.
    uint64_t remaining = pages;
    uint64_t offset = 0;

    while (remaining != 0) {
        const uint32_t order = floor_log2(remaining);
        const uint16_t root = take_node(no_node, order, offset);
        mark_free(root);
        roots_[root_count_++] = root;
        offset += 1ULL << order;
        remaining -= 1ULL << order;
    }

    layout_ = layout;
    total_pages_ = pages;
    available_pages_ = pages;
    max_order_ = floor_log2(pages);
    initialized_ = true;

    for (size_t index = 0; index < count; ++index) {
        auto& reservation = reservations_[index];

        if (claim_range(reservation) != VramStatus::success ||
            reservation.claimed_pages != reservation.pages) {
            reset();
            return VramStatus::no_metadata;
        }
    }

    normalize();
    return VramStatus::success;
}

VramStatus VramAllocator::shutdown() noexcept {
    if (!initialized_) {
        return VramStatus::uninitialized;
    }

    for (const auto& allocation : allocations_) {
        if (allocation.used) {
            return VramStatus::busy;
        }
    }

    for (auto& reservation : reservations_) {
        if (reservation.used) {
            free_list(reservation.blocks);
        }
    }

    reset();
    return VramStatus::success;
}

VramStatus VramAllocator::reserve_range(uint64_t start, uint64_t bytes) noexcept {
    if (!initialized_) {
        return VramStatus::uninitialized;
    }

    if (bytes == 0 || (start % vram_chunk_bytes) != 0 || (bytes % vram_chunk_bytes) != 0 ||
        start >= layout_.vram_bytes || bytes > layout_.vram_bytes - start) {
        return VramStatus::invalid_argument;
    }

    const uint64_t first = start / vram_chunk_bytes;
    const uint64_t end = first + bytes / vram_chunk_bytes;
    Reservation* slot = nullptr;

    for (auto& reservation : reservations_) {
        if (!reservation.used) {
            slot = slot == nullptr ? &reservation : slot;
            continue;
        }

        const uint64_t other_end = reservation.start_page + reservation.pages;

        if (first >= reservation.start_page && end <= other_end) {
            return reservation.claimed_pages == reservation.pages ?
                VramStatus::success : VramStatus::pending;
        }

        // A partial overlap could never complete: the other reservation owns it.
        if (first < other_end && end > reservation.start_page) {
            return VramStatus::invalid_argument;
        }
    }

    if (slot == nullptr) {
        return VramStatus::no_metadata;
    }

    *slot = Reservation{};
    slot->used = true;
    slot->start_page = first;
    slot->pages = end - first;
    const VramStatus status = claim_range(*slot);
    normalize();

    if (status != VramStatus::success) {
        return status;
    }

    return slot->claimed_pages == slot->pages ? VramStatus::success : VramStatus::pending;
}

VramStatus VramAllocator::query_page_status(uint64_t offset) const noexcept {
    if (!initialized_) {
        return VramStatus::uninitialized;
    }

    const uint64_t page = offset / vram_chunk_bytes;

    for (const auto& reservation : reservations_) {
        if (reservation.used && page >= reservation.start_page &&
            page - reservation.start_page < reservation.pages) {
            return reservation.claimed_pages == reservation.pages ?
                VramStatus::success : VramStatus::busy;
        }
    }

    return VramStatus::not_reserved;
}

// --------------------------------------------------------------- allocations

uint64_t VramAllocator::visible_bytes_of(const BlockList& list) const noexcept {
    uint64_t visible = 0;

    for (uint16_t index = list.head; index != no_node; index = nodes_[index].next) {
        visible += visible_overlap(nodes_[index].page * vram_chunk_bytes,
                                   block_pages(index) * vram_chunk_bytes, layout_.visible_bytes);
    }

    return visible;
}

// amdgpu_vram_mgr_new.
VramStatus VramAllocator::allocate_placed(uint64_t bytes, uint64_t alignment,
                                          const VramBoFlags& requested,
                                          const VramPlacement& place,
                                          VramAllocationHandle& handle) noexcept {
    handle = {};

    if (!initialized_) {
        return VramStatus::uninitialized;
    }

    const VramBoFlags flags = effective_flags(requested);
    const uint64_t vram = layout_.vram_bytes;
    const uint64_t last = place.last_byte != 0 ? place.last_byte : vram;

    if (bytes == 0 || (bytes % vram_chunk_bytes) != 0 || bytes > vram ||
        (alignment != 0 && !power_of_two(alignment)) ||
        (flags.cpu_access_required && flags.no_cpu_access) ||
        (place.first_byte % vram_chunk_bytes) != 0 || (last % vram_chunk_bytes) != 0 ||
        place.first_byte >= last || last > vram) {
        return VramStatus::invalid_argument;
    }

    const uint64_t align = alignment > vram_chunk_bytes ? alignment : vram_chunk_bytes;

    if (align > vram) {
        return VramStatus::no_space;
    }

    // ttm_resource_init counts the BO before the quick bail-out check.
    const uint64_t max_bytes = flags.kernel ? vram :
        (vram > vram_vm_reserved_bytes ? vram - vram_vm_reserved_bytes : 0);

    if (usage_bytes_ > max_bytes || bytes > max_bytes - usage_bytes_) {
        return VramStatus::no_space;
    }

    Allocation* slot = nullptr;
    size_t slot_index = 0;

    for (size_t index = 0; index < vram_allocation_capacity; ++index) {
        if (!allocations_[index].used) {
            slot = &allocations_[index];
            slot_index = index;
            break;
        }
    }

    if (slot == nullptr || generation_ == UINT32_MAX) {
        return VramStatus::no_metadata;
    }

    bool contiguous = flags.vram_contiguous;
    uint64_t block_bytes = contiguous ? 0 :
        (vram_default_block_bytes > align ? vram_default_block_bytes : align);
    const BuddyFlags base{place.first_byte != 0 || last != vram, place.topdown, contiguous};
    BuddyFlags buddy = base;
    BlockList blocks{};
    VramStatus status = VramStatus::success;

    for (;;) {
        uint64_t min_block = align;

        if (block_bytes != 0 && bytes >= block_bytes && (bytes % block_bytes) == 0) {
            min_block = block_bytes;
        }

        status = alloc_blocks(place.first_byte / vram_chunk_bytes, last / vram_chunk_bytes,
                              bytes / vram_chunk_bytes, min_block / vram_chunk_bytes, buddy, blocks);

        // VRAM_CONTIGUOUS without TTM_PL_FLAG_CONTIGUOUS falls back to 2 MiB blocks.
        if (status == VramStatus::no_space && contiguous && !place.contiguous) {
            contiguous = false;
            buddy.contiguous = false;
            block_bytes = vram_default_block_bytes > align ? vram_default_block_bytes : align;
            normalize();
            continue;
        }

        break;
    }

    if (status != VramStatus::success) {
        free_list(blocks);
        normalize();
        return status;
    }

    normalize();
    *slot = Allocation{};
    slot->blocks = blocks;
    slot->requested_pages = bytes / vram_chunk_bytes;
    slot->visible_bytes = visible_bytes_of(blocks);
    slot->generation = ++generation_;
    slot->used = true;
    slot->cpu_access_required = flags.cpu_access_required;
    slot->no_cpu_access = flags.no_cpu_access;
    usage_bytes_ += bytes;
    visible_usage_bytes_ += slot->visible_bytes;
    handle = {static_cast<uint32_t>(slot_index), slot->generation};
    return VramStatus::success;
}

VramStatus VramAllocator::allocate(const VramRequest& request, VramAllocationHandle& handle) noexcept {
    return allocate_placed(request.bytes, request.alignment, request.flags,
                           vram_placement(request.flags, layout_.visible_bytes), handle);
}

VramStatus VramAllocator::allocate_at(uint64_t offset, uint64_t bytes, const VramBoFlags& flags,
                                      VramAllocationHandle& handle) noexcept {
    handle = {};

    if (!initialized_) {
        return VramStatus::uninitialized;
    }

    if (bytes == 0 || (offset % vram_chunk_bytes) != 0 || (bytes % vram_chunk_bytes) != 0 ||
        offset >= layout_.vram_bytes || bytes > layout_.vram_bytes - offset ||
        (effective_flags(flags).cpu_access_required && offset + bytes > layout_.visible_bytes)) {
        return VramStatus::invalid_argument;
    }

    VramPlacement place = vram_placement(flags, layout_.visible_bytes);
    place.first_byte = offset;
    place.last_byte = offset + bytes;
    return allocate_placed(bytes, vram_chunk_bytes, flags, place, handle);
}

const VramAllocator::Allocation* VramAllocator::find(VramAllocationHandle handle) const noexcept {
    if (!initialized_ || handle.slot >= vram_allocation_capacity || handle.generation == 0) {
        return nullptr;
    }

    const auto& allocation = allocations_[handle.slot];
    return allocation.used && allocation.generation == handle.generation ? &allocation : nullptr;
}

VramStatus VramAllocator::release(VramAllocationHandle handle) noexcept {
    if (find(handle) == nullptr) {
        return VramStatus::stale_handle;
    }

    auto& allocation = allocations_[handle.slot];
    free_list(allocation.blocks);
    usage_bytes_ -= allocation.requested_pages * vram_chunk_bytes;
    visible_usage_bytes_ -= allocation.visible_bytes;
    const uint32_t generation = allocation.generation;
    allocation = Allocation{};
    allocation.generation = generation;
    retry_pending();
    normalize();
    return VramStatus::success;
}

VramStatus VramAllocator::describe(VramAllocationHandle handle, VramAllocationInfo& info) const noexcept {
    info = {};
    const Allocation* allocation = find(handle);

    if (allocation == nullptr) {
        return VramStatus::stale_handle;
    }

    uint64_t allocated = 0;
    bool contiguous = true;
    uint64_t expected = nodes_[allocation->blocks.head].page;

    for (uint16_t index = allocation->blocks.head; index != no_node; index = nodes_[index].next) {
        contiguous = contiguous && nodes_[index].page == expected;
        expected = nodes_[index].page + block_pages(index);
        allocated += block_pages(index);
    }

    info.requested_bytes = allocation->requested_pages * vram_chunk_bytes;
    info.allocated_bytes = allocated * vram_chunk_bytes;
    info.visible_bytes = allocation->visible_bytes;
    info.first_byte = static_cast<uint64_t>(nodes_[allocation->blocks.head].page) * vram_chunk_bytes;
    info.block_count = allocation->blocks.count;
    info.contiguous = contiguous;
    info.cpu_access_required = allocation->cpu_access_required;
    info.no_cpu_access = allocation->no_cpu_access;
    return VramStatus::success;
}

VramStatus VramAllocator::first_block(VramAllocationHandle handle, VramBlockCursor& cursor,
                                      VramBlock& block) const noexcept {
    cursor = {};
    block = {};
    const Allocation* allocation = find(handle);

    if (allocation == nullptr) {
        return VramStatus::stale_handle;
    }

    const uint16_t index = allocation->blocks.head;
    cursor = {handle, index};
    block = {static_cast<uint64_t>(nodes_[index].page) * vram_chunk_bytes,
             block_pages(index) * vram_chunk_bytes};
    return VramStatus::success;
}

VramStatus VramAllocator::next_block(VramBlockCursor& cursor, VramBlock& block) const noexcept {
    block = {};

    if (find(cursor.allocation) == nullptr || cursor.node >= vram_node_capacity ||
        nodes_[cursor.node].state != NodeState::allocated) {
        return VramStatus::stale_handle;
    }

    const uint16_t index = nodes_[cursor.node].next;

    if (index == no_node) {
        return VramStatus::no_space;
    }

    cursor.node = index;
    block = {static_cast<uint64_t>(nodes_[index].page) * vram_chunk_bytes,
             block_pages(index) * vram_chunk_bytes};
    return VramStatus::success;
}

uint64_t VramAllocator::largest_free_block_bytes() const noexcept {
    if (!initialized_) {
        return 0;
    }

    for (uint32_t order = max_order_ + 1; order != 0; --order) {
        if (free_head_[order - 1] != no_node) {
            return (1ULL << (order - 1)) * vram_chunk_bytes;
        }
    }

    return 0;
}

} // namespace rx7900::hardware

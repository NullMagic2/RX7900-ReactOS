/*
Module: rx7900.tests.vram_allocator
Purpose: Verify Linux amdgpu_vram_mgr/drm_buddy placement, reservations and the Manager bridge.
Created: 2026-10-22
Architecture: Host-only; synthetic VRAM layouts and a host-buffer CPU mapper, no hardware.
*/
#include <algorithm>
#include <cstdio>
#include <memory>
#include <vector>

#include "vram_allocator.hpp"
#include "vram_backend.hpp"

namespace {

using namespace rx7900::hardware;
namespace memory = rx7900::memory;

unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* contract) {
    ++checks;

    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", contract);
    }
}

constexpr uint64_t KiB = 1024;
constexpr uint64_t MiB = 1024 * KiB;
constexpr uint64_t GiB = 1024 * MiB;

std::unique_ptr<VramAllocator> make_allocator() {
    return std::make_unique<VramAllocator>();
}

VramBoFlags user_flags(bool cpu, bool contiguous) {
    return {cpu, false, contiguous, false, false};
}

std::vector<VramBlock> blocks_of(const VramAllocator& allocator, VramAllocationHandle handle) {
    std::vector<VramBlock> result;
    VramBlockCursor cursor{};
    VramBlock block{};

    if (allocator.first_block(handle, cursor, block) != VramStatus::success) {
        return result;
    }

    do {
        result.push_back(block);
    } while (allocator.next_block(cursor, block) == VramStatus::success);

    return result;
}

bool overlaps_any(const std::vector<VramBlock>& blocks, const std::vector<VramRange>& ranges) {
    for (const auto& block : blocks) {
        for (const auto& range : ranges) {
            if (block.start < range.end && range.start < block.start + block.bytes) {
                return true;
            }
        }
    }

    return false;
}

// Live allocations never overlap each other or reservations; every byte is
// accounted as free, allocated or reserved; CPU BOs stay in visible VRAM.
bool consistent(const VramAllocator& allocator, const std::vector<VramAllocationHandle>& live,
                const std::vector<VramRange>& reserved) {
    std::vector<VramRange> spans;
    uint64_t allocated = 0;
    uint64_t visible = 0;

    for (const auto handle : live) {
        VramAllocationInfo info{};

        if (allocator.describe(handle, info) != VramStatus::success) {
            return false;
        }

        allocated += info.allocated_bytes;
        visible += info.visible_bytes;
        const auto blocks = blocks_of(allocator, handle);

        if (overlaps_any(blocks, reserved)) {
            return false;
        }

        for (const auto& block : blocks) {
            if (info.cpu_access_required && block.start + block.bytes > allocator.visible_bytes()) {
                return false;
            }

            spans.push_back({block.start, block.start + block.bytes});
        }
    }

    std::sort(spans.begin(), spans.end(),
              [](const VramRange& a, const VramRange& b) { return a.start < b.start; });

    for (size_t index = 1; index < spans.size(); ++index) {
        if (spans[index].start < spans[index - 1].end) {
            return false;
        }
    }

    uint64_t reserved_bytes = 0;
    uint64_t reserved_visible = 0;

    for (const auto& range : reserved) {
        reserved_bytes += range.end - range.start;
        const uint64_t end = std::min(range.end, allocator.visible_bytes());
        reserved_visible += range.start < end ? end - range.start : 0;
    }

    return allocator.available_bytes() + allocated + reserved_bytes == allocator.vram_bytes() &&
           allocator.visible_usage_bytes() == visible + reserved_visible;
}

void check_placement_rules() {
    const auto kernel_cpu = vram_placement(vram_kernel_bo_flags(true), 256 * MiB);
    expect(kernel_cpu.first_byte == 0 && kernel_cpu.last_byte == 256 * MiB && !kernel_cpu.topdown &&
           kernel_cpu.contiguous, "CPU kernel BO: lpfn = visible, bottom-up, mandatory contiguous");
    const auto kernel_gpu = vram_placement(vram_kernel_bo_flags(false), 256 * MiB);
    expect(kernel_gpu.last_byte == 0 && kernel_gpu.topdown && kernel_gpu.contiguous,
           "NO_CPU_ACCESS kernel BO: TOPDOWN over all VRAM, mandatory contiguous");
    const auto user = vram_placement(user_flags(false, true), 256 * MiB);
    expect(user.topdown && !user.contiguous, "unpinned user contiguous BO may fall back");
    const auto pinned = vram_placement({false, false, false, false, true}, 256 * MiB);
    expect(pinned.last_byte == 256 * MiB && !pinned.topdown,
           "amdgpu_bo_pin forces CPU access unless NO_CPU_ACCESS");
    const auto flags = vram_kernel_bo_flags(false);
    expect(!flags.cpu_access_required && flags.no_cpu_access && flags.vram_contiguous &&
           flags.kernel && flags.pinned, "amdgpu_bo_create_reserved flags");
}

void check_initialization() {
    auto allocator = make_allocator();
    VramAllocationHandle handle{};
    expect(allocator->allocate({4 * KiB, 0, user_flags(false, false)}, handle) ==
           VramStatus::uninitialized, "allocation before initialization");
    expect(allocator->initialize({0, 0}, nullptr, 0) == VramStatus::invalid_argument, "empty VRAM");
    expect(allocator->initialize({64 * MiB + 1, 16 * MiB}, nullptr, 0) == VramStatus::invalid_argument,
           "unaligned VRAM size");
    expect(allocator->initialize({64 * MiB, 128 * MiB}, nullptr, 0) == VramStatus::invalid_argument,
           "visible larger than VRAM");
    expect(allocator->initialize({64 * MiB, 16 * MiB}, nullptr, 2) == VramStatus::invalid_argument,
           "null reservation list with count");
    const VramRange beyond[]{{60 * MiB, 64 * MiB + 4 * KiB}};
    expect(allocator->initialize({64 * MiB, 16 * MiB}, beyond, 1) == VramStatus::invalid_argument,
           "reservation past VRAM fails closed");
    std::vector<VramRange> many;

    for (uint64_t index = 0; index <= vram_reservation_capacity; ++index) {
        many.push_back({index * MiB, index * MiB + 4 * KiB});
    }

    expect(allocator->initialize({64 * MiB, 16 * MiB}, many.data(), many.size()) ==
           VramStatus::invalid_argument, "too many disjoint reservations fails closed");
    expect(!allocator->initialized(), "failed initialization retains nothing");

    // Navi31: 24 GiB is two drm_buddy roots (16 GiB + 8 GiB).
    expect(allocator->initialize({24 * GiB, 256 * MiB}, nullptr, 0) == VramStatus::success,
           "24 GiB whole-VRAM initialization");
    expect(allocator->available_bytes() == 24 * GiB && allocator->largest_free_block_bytes() == 16 * GiB,
           "24 GiB splits into 16 GiB and 8 GiB roots");
    expect(allocator->initialize({24 * GiB, 256 * MiB}, nullptr, 0) == VramStatus::busy,
           "double initialization");
    expect(allocator->shutdown() == VramStatus::success && !allocator->initialized(), "shutdown");
}

void check_reservations_never_handed_out() {
    auto allocator = make_allocator();
    // Unaligned, overlapping and adjacent inputs (stolen VGA + extended, a
    // firmware range, discovery TMR at the top).
    const VramRange input[]{{0, 1 * MiB + 100}, {1 * MiB, 3 * MiB}, {3 * MiB, 3 * MiB + 4 * KiB},
                            {40 * MiB + 10, 41 * MiB}, {60 * MiB, 64 * MiB}};
    const std::vector<VramRange> merged{{0, 3 * MiB + 4 * KiB}, {40 * MiB, 41 * MiB}, {60 * MiB, 64 * MiB}};
    expect(allocator->initialize({64 * MiB, 16 * MiB}, input, 5) == VramStatus::success,
           "reservation list accepted");
    expect(allocator->available_bytes() == 64 * MiB - (3 * MiB + 4 * KiB) - MiB - 4 * MiB,
           "reservations widened to pages and merged");
    expect(allocator->query_page_status(0) == VramStatus::success &&
           allocator->query_page_status(3 * MiB) == VramStatus::success &&
           allocator->query_page_status(3 * MiB + 4 * KiB) == VramStatus::not_reserved &&
           allocator->query_page_status(40 * MiB) == VramStatus::success,
           "reserved page status");
    expect(allocator->usage_bytes() == 8 * MiB + 4 * KiB, "reservations count toward usage");
    expect(allocator->visible_usage_bytes() == 3 * MiB + 4 * KiB, "visible reservation usage");

    std::vector<VramAllocationHandle> live;
    VramAllocationHandle handle{};

    // Fill all of VRAM with 4 KiB kernel BOs of both classes; none may touch a
    // reservation, and exhaustion is reported as no_space.
    VramStatus status = VramStatus::success;

    while (live.size() < vram_allocation_capacity) {
        // Prefer a CPU-visible BO; once visible VRAM is full, use invisible VRAM.
        status = allocator->allocate({1 * MiB, 0, vram_kernel_bo_flags(true)}, handle);

        if (status == VramStatus::no_space) {
            status = allocator->allocate({1 * MiB, 0, vram_kernel_bo_flags(false)}, handle);
        }

        if (status != VramStatus::success) {
            break;
        }

        live.push_back(handle);
    }

    expect(status == VramStatus::no_space, "whole VRAM exhausted with no_space");
    expect(consistent(*allocator, live, merged), "no allocation overlaps a reservation");
    expect(allocator->available_bytes() < 1 * MiB &&
           allocator->allocate({1 * MiB, 0, vram_kernel_bo_flags(false)}, handle) == VramStatus::no_space,
           "no further 1 MiB BO after exhaustion");

    for (const auto live_handle : live) {
        expect(allocator->release(live_handle) == VramStatus::success, "release");
    }

    expect(allocator->available_bytes() == 64 * MiB - (8 * MiB + 4 * KiB), "full coalescing");
    expect(allocator->allocate_at(40 * MiB, 4 * KiB, vram_kernel_bo_flags(false), handle) ==
           VramStatus::no_space, "allocate_at inside a reservation is refused");
    expect(allocator->shutdown() == VramStatus::success, "shutdown frees reservations");
}

void check_visible_and_topdown() {
    auto allocator = make_allocator();
    const std::vector<VramRange> reserved{{0, 2 * MiB}, {63 * MiB, 64 * MiB}};
    expect(allocator->initialize({64 * MiB, 16 * MiB}, reserved.data(), reserved.size()) ==
           VramStatus::success, "layout");
    VramAllocationHandle cpu{};
    VramAllocationHandle gpu{};
    VramAllocationInfo info{};
    expect(allocator->allocate({64 * KiB, 0, vram_kernel_bo_flags(true)}, cpu) == VramStatus::success &&
           allocator->describe(cpu, info) == VramStatus::success,
           "CPU-visible kernel BO");
    expect(info.first_byte == 2 * MiB && info.visible_bytes == 64 * KiB && info.contiguous &&
           info.cpu_access_required, "bottom-up range search: lowest free visible offset");
    expect(allocator->allocate({64 * KiB, 0, vram_kernel_bo_flags(false)}, gpu) == VramStatus::success &&
           allocator->describe(gpu, info) == VramStatus::success, "GPU-only kernel BO");
    expect(info.first_byte == 63 * MiB - 64 * KiB && info.visible_bytes == 0 && info.no_cpu_access,
           "TOPDOWN: directly below the top reservation");
    expect(allocator->visible_usage_bytes() == 2 * MiB + 64 * KiB, "visible usage accounting");

    // Visible exhaustion: CPU BOs fail although invisible VRAM is free.
    std::vector<VramAllocationHandle> live{cpu, gpu};
    VramAllocationHandle handle{};

    while (allocator->allocate({1 * MiB, 0, vram_kernel_bo_flags(true)}, handle) == VramStatus::success) {
        live.push_back(handle);
    }

    expect(allocator->visible_usage_bytes() <= 16 * MiB && allocator->available_bytes() >= 40 * MiB,
           "visible window exhausted while invisible VRAM remains");
    expect(allocator->allocate({1 * MiB, 0, user_flags(false, false)}, handle) == VramStatus::success,
           "invisible allocation still succeeds");
    live.push_back(handle);
    expect(consistent(*allocator, live, reserved), "visible-range restriction holds");

    expect(allocator->allocate_at(15 * MiB, 2 * MiB, vram_kernel_bo_flags(true), handle) ==
           VramStatus::invalid_argument, "allocate_at CPU range crossing visible end");

    for (const auto live_handle : live) {
        allocator->release(live_handle);
    }

    expect(allocator->release(cpu) == VramStatus::stale_handle, "double release is stale");
    expect(allocator->visible_usage_bytes() == 2 * MiB, "visible usage returns to reservations");
    allocator->shutdown();
}

void check_contiguous_and_scattered() {
    auto allocator = make_allocator();
    expect(allocator->initialize({64 * MiB, 64 * MiB}, nullptr, 0) == VramStatus::success, "layout");
    const size_t baseline_nodes = allocator->spare_nodes();
    VramAllocationHandle handle{};
    VramAllocationInfo info{};

    // 3 MiB contiguous: one 4 MiB block trimmed to 2 MiB + 1 MiB.
    expect(allocator->allocate({3 * MiB, 0, vram_kernel_bo_flags(false)}, handle) == VramStatus::success &&
           allocator->describe(handle, info) == VramStatus::success, "contiguous 3 MiB");
    const auto trimmed = blocks_of(*allocator, handle);
    expect(info.allocated_bytes == 3 * MiB && info.contiguous && trimmed.size() == 2 &&
           trimmed[0].bytes == 2 * MiB && trimmed[1].bytes == 1 * MiB &&
           info.first_byte % (4 * MiB) == 0, "trimmed power-of-two block, ascending");
    expect(allocator->available_bytes() == 61 * MiB, "trim returns the unused tail");
    allocator->release(handle);
    expect(allocator->spare_nodes() == baseline_nodes && allocator->largest_free_block_bytes() == 64 * MiB,
           "release coalesces back to the root");

    // Alignment larger than the size is honoured (fail-closed divergence).
    expect(allocator->allocate({4 * KiB, 1 * MiB, vram_kernel_bo_flags(false)}, handle) ==
           VramStatus::success && allocator->describe(handle, info) == VramStatus::success &&
           info.first_byte % MiB == 0 && info.allocated_bytes == 4 * KiB,
           "contiguous alignment above size");
    allocator->release(handle);
    expect(allocator->allocate({12 * KiB, 0, user_flags(false, false)}, handle) == VramStatus::success &&
           allocator->describe(handle, info) == VramStatus::success && info.block_count == 2,
           "non-contiguous 12 KiB as 8 KiB + 4 KiB blocks");
    allocator->release(handle);

    // Fragment: 32 x 2 MiB, then free every other one.
    std::vector<VramAllocationHandle> pieces;

    for (int index = 0; index < 32; ++index) {
        expect(allocator->allocate({2 * MiB, 0, {false, true, false, true, false}}, handle) ==
               VramStatus::success, "2 MiB kernel piece");
        pieces.push_back(handle);
    }

    expect(allocator->available_bytes() == 0, "VRAM full of pieces");
    std::vector<VramAllocationHandle> kept;

    for (size_t index = 0; index < pieces.size(); ++index) {
        if (index % 2 == 0) {
            allocator->release(pieces[index]);
        } else {
            kept.push_back(pieces[index]);
        }
    }

    expect(allocator->largest_free_block_bytes() == 2 * MiB, "only isolated 2 MiB holes remain");
    expect(allocator->allocate({8 * MiB, 0, vram_kernel_bo_flags(false)}, handle) == VramStatus::no_space,
           "pinned contiguous 8 MiB cannot be placed in fragmented VRAM");
    expect(allocator->allocate({8 * MiB, 0, user_flags(false, false)}, handle) == VramStatus::success &&
           allocator->describe(handle, info) == VramStatus::success, "scattered 8 MiB");
    expect(info.block_count == 4 && !info.contiguous && info.allocated_bytes == 8 * MiB,
           "scattered allocation is a block list of 2 MiB blocks");
    kept.push_back(handle);
    VramAllocationHandle fallback{};
    expect(allocator->allocate({4 * MiB, 0, user_flags(false, true)}, fallback) == VramStatus::success &&
           allocator->describe(fallback, info) == VramStatus::success && !info.contiguous,
           "unpinned VRAM_CONTIGUOUS falls back to 2 MiB blocks (TTM_PL_FLAG_CONTIGUOUS absent)");
    kept.push_back(fallback);
    expect(consistent(*allocator, kept, {}), "fragmented state consistent");

    for (const auto live : kept) {
        allocator->release(live);
    }

    expect(allocator->available_bytes() == 64 * MiB && allocator->largest_free_block_bytes() == 64 * MiB &&
           allocator->spare_nodes() == baseline_nodes, "fragmentation-safe free coalesces completely");
    allocator->shutdown();
}

void check_try_harder() {
    auto allocator = make_allocator();
    expect(allocator->initialize({16 * MiB, 16 * MiB}, nullptr, 0) == VramStatus::success, "layout");
    VramAllocationHandle low{};
    VramAllocationHandle middle{};
    VramAllocationHandle handle{};
    VramAllocationInfo info{};
    expect(allocator->allocate_at(0, 2 * MiB, user_flags(false, false), low) == VramStatus::success &&
           allocator->allocate_at(10 * MiB, 2 * MiB, user_flags(false, false), middle) ==
               VramStatus::success, "exact placements");
    expect(allocator->largest_free_block_bytes() == 4 * MiB, "no 8 MiB buddy block is free");
    // 6 MiB contiguous: [2,10) MiB crosses a buddy boundary; found by
    // __alloc_contig_try_harder from the [4,8) MiB block plus the right-hand side.
    expect(allocator->allocate({6 * MiB, 0, vram_kernel_bo_flags(false)}, handle) == VramStatus::success &&
           allocator->describe(handle, info) == VramStatus::success, "try-harder contiguous");
    expect(info.contiguous && info.first_byte == 4 * MiB && info.allocated_bytes == 6 * MiB,
           "try-harder range [4, 10) MiB");
    expect(consistent(*allocator, {low, middle, handle}, {}), "try-harder consistent");
    VramAllocationHandle refused{};
    expect(allocator->allocate_at(9 * MiB, 4 * KiB, user_flags(false, false), refused) == VramStatus::no_space,
           "allocate_at over a live allocation");
    allocator->release(handle);
    allocator->release(middle);
    expect(allocator->release(low) == VramStatus::success && allocator->available_bytes() == 16 * MiB,
           "released");
    allocator->shutdown();
}

void check_pending_reservation() {
    auto allocator = make_allocator();
    expect(allocator->initialize({64 * MiB, 64 * MiB}, nullptr, 0) == VramStatus::success, "layout");
    VramAllocationHandle busy{};
    VramAllocationHandle handle{};
    expect(allocator->allocate_at(8 * MiB, 2 * MiB, user_flags(false, false), busy) == VramStatus::success,
           "busy range");
    expect(allocator->reserve_range(8 * MiB, 4 * MiB) == VramStatus::pending,
           "amdgpu_vram_mgr_reserve_range over a live BO stays pending");
    expect(allocator->query_page_status(9 * MiB) == VramStatus::busy &&
           allocator->query_page_status(11 * MiB) == VramStatus::busy, "pending page status");
    expect(allocator->allocate_at(10 * MiB, 4 * KiB, user_flags(false, false), handle) == VramStatus::no_space,
           "free part of a pending reservation is never handed out");
    expect(allocator->reserve_range(9 * MiB, 1 * MiB) == VramStatus::pending,
           "contained duplicate reports the pending state");
    expect(allocator->reserve_range(11 * MiB, 2 * MiB) == VramStatus::invalid_argument,
           "partially overlapping reservation refused");
    expect(allocator->reserve_range(8 * MiB + 1, 4 * KiB) == VramStatus::invalid_argument,
           "unaligned reservation refused");
    expect(allocator->release(busy) == VramStatus::success, "release busy range");
    expect(allocator->query_page_status(8 * MiB) == VramStatus::success,
           "pending reservation claimed on release");
    expect(allocator->allocate_at(8 * MiB, 4 * KiB, user_flags(false, false), handle) == VramStatus::no_space,
           "claimed reservation is never handed out");
    expect(allocator->available_bytes() == 60 * MiB && allocator->usage_bytes() == 4 * MiB,
           "reservation accounting");
    expect(allocator->shutdown() == VramStatus::success, "shutdown");
}

void check_metadata_exhaustion() {
    auto allocator = make_allocator();
    // Isolated pages far apart each cost a split path of about 28 nodes;
    // 32 reserved plus 256 allocated pages exceed the 8192-node pool.
    std::vector<VramRange> reserved;

    for (uint64_t index = 0; index < vram_reservation_capacity; ++index) {
        reserved.push_back({index * 512 * MiB + 4 * KiB, index * 512 * MiB + 8 * KiB});
    }

    expect(allocator->initialize({16 * GiB, 256 * MiB}, reserved.data(), reserved.size()) ==
           VramStatus::success, "sparse reservations");
    std::vector<VramAllocationHandle> live;
    VramStatus status = VramStatus::success;
    VramAllocationHandle handle{};

    for (uint64_t index = 0; index < vram_allocation_capacity; ++index) {
        const uint64_t offset = index * 64 * MiB + 32 * MiB + 12 * KiB;
        const uint64_t available = allocator->available_bytes();
        const size_t nodes = allocator->spare_nodes();
        status = allocator->allocate_at(offset, 4 * KiB, user_flags(false, false), handle);

        if (status != VramStatus::success) {
            expect(allocator->available_bytes() == available && allocator->spare_nodes() == nodes,
                   "metadata failure changes nothing");
            break;
        }

        live.push_back(handle);
    }

    expect(status == VramStatus::no_metadata, "node pool exhaustion reported as no_metadata");
    expect(consistent(*allocator, live, reserved), "state consistent at metadata exhaustion");
    expect(allocator->release(live.back()) == VramStatus::success, "release never needs metadata");
    live.pop_back();

    for (const auto live_handle : live) {
        allocator->release(live_handle);
    }

    expect(allocator->available_bytes() == 16 * GiB - 32 * 4 * KiB, "all pages coalesce after exhaustion");
    allocator->shutdown();

    // A failed trim keeps the whole power-of-two block (Linux ignores the trim
    // result). 16 TiB (one order-32 root) makes each isolated page cost about
    // 48 nodes, so metadata runs out well before the allocation slots.
    constexpr uint64_t TiB = 1024 * GiB;
    expect(allocator->initialize({16 * TiB, 256 * MiB}, nullptr, 0) == VramStatus::success,
           "16 TiB maximum layout");
    live.clear();
    uint64_t isolated = 0;

    while (allocator->allocate_at(isolated * 80 * GiB, 4 * KiB, user_flags(false, false), handle) ==
           VramStatus::success) {
        live.push_back(handle);
        ++isolated;
    }

    // Each page in the free 8 KiB half next to an isolated page costs two nodes.
    for (uint64_t index = 1; index < isolated && allocator->spare_nodes() >= 2; ++index) {
        if (allocator->allocate_at(index * 80 * GiB + 8 * KiB, 4 * KiB, user_flags(false, false), handle) ==
            VramStatus::success) {
            live.push_back(handle);
        }
    }

    VramAllocationInfo info{};
    expect(allocator->spare_nodes() < 2 && live.size() < vram_allocation_capacity,
           "node pool drained below one split");
    // [16 KiB, 32 KiB) is a free order-2 block found without splitting; the
    // 3-page trim then needs a split that cannot be satisfied.
    expect(allocator->allocate({12 * KiB, 0, vram_kernel_bo_flags(true)}, handle) == VramStatus::success &&
           allocator->describe(handle, info) == VramStatus::success, "contiguous BO despite trim failure");
    expect(info.first_byte == 16 * KiB && info.allocated_bytes == 16 * KiB && info.requested_bytes == 12 * KiB &&
           info.block_count == 1 && info.contiguous, "untrimmed block retained intact");
    live.push_back(handle);
    expect(consistent(*allocator, live, {}), "state consistent after trim failure");

    for (const auto live_handle : live) {
        allocator->release(live_handle);
    }

    expect(allocator->available_bytes() == 16 * TiB && allocator->largest_free_block_bytes() == 16 * TiB,
           "16 TiB tree coalesces to its root");
    allocator->shutdown();

    // Slot exhaustion is also bounded.
    expect(allocator->initialize({64 * MiB, 64 * MiB}, nullptr, 0) == VramStatus::success, "layout");
    live.clear();

    while (allocator->allocate({4 * KiB, 0, user_flags(false, false)}, handle) == VramStatus::success) {
        live.push_back(handle);
    }

    expect(live.size() == vram_allocation_capacity &&
           allocator->allocate({4 * KiB, 0, user_flags(false, false)}, handle) == VramStatus::no_metadata,
           "allocation table exhaustion");

    for (const auto live_handle : live) {
        allocator->release(live_handle);
    }

    allocator->shutdown();
}

void check_usage_limit() {
    auto allocator = make_allocator();
    expect(allocator->initialize({64 * MiB, 64 * MiB}, nullptr, 0) == VramStatus::success, "layout");
    VramAllocationHandle handle{};
    // Non-kernel BOs may not use the last AMDGPU_VM_RESERVED_VRAM; kernel BOs may.
    expect(allocator->allocate({60 * MiB, 0, user_flags(false, false)}, handle) == VramStatus::no_space,
           "user BO bounded by VM reserved headroom");
    expect(allocator->allocate({60 * MiB, 0, {false, true, false, true, false}}, handle) ==
           VramStatus::success, "kernel BO may use all VRAM");
    allocator->release(handle);
    expect(allocator->allocate({4 * KiB, 3 * KiB, user_flags(false, false)}, handle) ==
           VramStatus::invalid_argument, "non-power-of-two alignment");
    expect(allocator->allocate({5 * KiB, 0, user_flags(false, false)}, handle) ==
           VramStatus::invalid_argument, "non-page size");
    expect(allocator->allocate({4 * KiB, 0, {true, true, false, false, false}}, handle) ==
           VramStatus::invalid_argument, "contradictory CPU access flags");
    allocator->shutdown();
}

void check_stress() {
    auto allocator = make_allocator();
    const std::vector<VramRange> reserved{{0, 8 * MiB}, {100 * MiB, 101 * MiB}, {248 * MiB, 256 * MiB}};
    expect(allocator->initialize({256 * MiB, 128 * MiB}, reserved.data(), reserved.size()) ==
           VramStatus::success, "stress layout");
    const size_t baseline = allocator->spare_nodes();
    std::vector<VramAllocationHandle> live;
    uint32_t seed = 0x12345678;
    auto next = [&seed]() {
        seed = seed * 1664525u + 1013904223u;
        return seed >> 8;
    };
    bool ok = true;
    unsigned placed_count = 0;
    unsigned refused_count = 0;

    for (int step = 0; step < 4000; ++step) {
        if (!live.empty() && (next() % 3 == 0 || live.size() > 200)) {
            const size_t index = next() % live.size();
            ok = ok && allocator->release(live[index]) == VramStatus::success;
            live.erase(live.begin() + static_cast<std::ptrdiff_t>(index));
        } else {
            const uint64_t pages = 1 + next() % 1024;
            const uint64_t alignment = 4 * KiB << (next() % 6);
            const uint32_t kind = next() % 4;
            const VramBoFlags flags = kind == 0 ? vram_kernel_bo_flags(true) :
                kind == 1 ? vram_kernel_bo_flags(false) : user_flags(kind == 2, next() % 2 == 0);
            VramAllocationHandle handle{};
            const VramStatus status = allocator->allocate({pages * 4 * KiB, alignment, flags}, handle);

            if (status == VramStatus::success) {
                VramAllocationInfo info{};
                allocator->describe(handle, info);
                const auto blocks = blocks_of(*allocator, handle);
                // drm_buddy aligns the first block; trimmed tail pieces of a
                // rounded allocation may be smaller than the alignment.
                const bool needs_visible = flags.cpu_access_required ||
                    (flags.pinned && !flags.no_cpu_access);
                bool placed = info.first_byte % alignment == 0 && !blocks.empty() &&
                    (!flags.vram_contiguous || !flags.pinned || info.contiguous);

                for (const auto& block : blocks) {
                    placed = placed && (!needs_visible ||
                                        block.start + block.bytes <= allocator->visible_bytes());
                }

                if (ok && !placed) {
                    std::fprintf(stderr, "misplaced at step %d: first %llx align %llx\n", step,
                                 static_cast<unsigned long long>(info.first_byte),
                                 static_cast<unsigned long long>(alignment));
                }

                ok = ok && placed;
                ++placed_count;
                live.push_back(handle);
            } else {
                ok = ok && (status == VramStatus::no_space || status == VramStatus::no_metadata);
                ++refused_count;
            }
        }

        if (step % 10 == 0 && ok && !consistent(*allocator, live, reserved)) {
            std::fprintf(stderr, "inconsistent at step %d\n", step);
            ok = false;
        }
    }

    std::printf("stress: %u placed, %u refused\n", placed_count, refused_count);
    expect(ok, "4000 random operations keep alignment, placement and accounting");
    expect(placed_count > 1000 && refused_count > 0, "stress reaches both success and exhaustion");
    expect(consistent(*allocator, live, reserved), "final stress state consistent");

    for (const auto handle : live) {
        allocator->release(handle);
    }

    expect(allocator->available_bytes() == 256 * MiB - 17 * MiB && allocator->spare_nodes() == baseline,
           "stress releases coalesce to the initial tree");
    allocator->shutdown();
}

// Synthetic visible VRAM: the "BAR" is a host buffer at VRAM offset zero.
struct SyntheticVram final {
    std::vector<uint8_t> bar;
    int maps = 0;
    int unmaps = 0;
    bool fail_map = false;
    bool fail_unmap = false;
    uint64_t tokens[memory::allocation_capacity * 2]{};

    static bool map(void* context, uint64_t offset, uint64_t bytes, uint8_t*& cpu, void*& mapping) noexcept {
        auto& self = *static_cast<SyntheticVram*>(context);

        if (self.fail_map || offset > self.bar.size() || bytes > self.bar.size() - offset) {
            return false;
        }

        for (auto& token : self.tokens) {
            if (token == 0) {
                token = offset + 1;
                cpu = self.bar.data() + offset;
                mapping = &token;
                ++self.maps;
                return true;
            }
        }

        return false;
    }

    static bool unmap(void* context, void* mapping) noexcept {
        auto& self = *static_cast<SyntheticVram*>(context);

        if (self.fail_unmap) {
            return false;
        }

        for (auto& token : self.tokens) {
            if (&token == mapping && token != 0) {
                token = 0;
                ++self.unmaps;
                return true;
            }
        }

        return false;
    }

    VramCpuMapper mapper() { return {this, map, unmap}; }
};

void check_manager_integration() {
    constexpr uint64_t mc_base = 0x8000000000ULL;
    auto allocator = make_allocator();
    const std::vector<VramRange> reserved{{0, 4 * MiB}, {63 * MiB, 64 * MiB}};
    expect(allocator->initialize({64 * MiB, 32 * MiB}, reserved.data(), reserved.size()) ==
           VramStatus::success, "integration layout");
    SyntheticVram vram{};
    vram.bar.resize(32 * MiB);
    auto visible = std::make_unique<VramBackend>();
    auto hidden = std::make_unique<VramBackend>();
    expect(visible->initialize(*allocator, mc_base, VramPool::cpu_visible, 8 * MiB, 0, {}) ==
           VramStatus::invalid_argument, "visible pool requires a mapper");
    expect(visible->initialize(*allocator, mc_base, VramPool::cpu_visible, 8 * MiB, 0, vram.mapper()) ==
           VramStatus::success, "visible pool carved");
    expect(hidden->initialize(*allocator, mc_base, VramPool::gpu_only, 8 * MiB, 0, {}) ==
           VramStatus::success, "GPU-only pool carved");
    expect(visible->arena().virtual_address == mc_base + 8 * MiB && visible->arena().byte_length == 8 * MiB,
           "visible arena: first free 8 MiB buddy block above the reservation, inside visible VRAM");
    expect(hidden->arena().virtual_address == mc_base + 48 * MiB,
           "GPU-only arena: TOPDOWN, highest free 8 MiB block below the top reservation");
    expect(visible->initialize(*allocator, mc_base, VramPool::cpu_visible, 8 * MiB, 0, vram.mapper()) ==
           VramStatus::busy, "pool initialized once");

    memory::Manager visible_manager;
    memory::Manager hidden_manager;
    expect(visible_manager.initialize(visible->arena(), visible->backend()) == memory::Status::success &&
           hidden_manager.initialize(hidden->arena(), hidden->backend()) == memory::Status::success,
           "Managers over the carved arenas");

    memory::Handle ring{};
    memory::Handle tmr{};
    memory::Handle refused{};
    memory::AllocationInfo info{};
    expect(visible_manager.allocate(64 * KiB, 64 * KiB, memory::Access::read_write, true, ring) ==
           memory::Status::success && visible_manager.describe(ring, info) == memory::Status::success,
           "CPU-visible ring");
    const uint64_t ring_offset = info.gpu.virtual_address - mc_base;
    expect(info.backing.cpu_bytes == vram.bar.data() + ring_offset &&
           info.backing.cpu_byte_count == 64 * KiB && vram.maps == 1,
           "CPU view maps exactly the allocation through the BAR");
    info.backing.cpu_bytes[0] = 0x5a;
    expect(vram.bar[ring_offset] == 0x5a, "CPU store lands at the MC-derived VRAM offset");
    expect(hidden_manager.allocate(1 * MiB, 1 * MiB, memory::Access::read_write, false, tmr) ==
           memory::Status::success && hidden_manager.describe(tmr, info) == memory::Status::success &&
           info.backing.cpu_bytes == nullptr && info.gpu.virtual_address >= mc_base + 48 * MiB,
           "GPU-only BO has no CPU view");
    expect(hidden_manager.allocate(4 * KiB, 4 * KiB, memory::Access::read_write, true, refused) ==
           memory::Status::backend_failure, "CPU mapping of invisible VRAM is refused");

    vram.fail_map = true;
    expect(visible_manager.allocate(4 * KiB, 4 * KiB, memory::Access::read_write, true, refused) ==
           memory::Status::backend_failure, "failed map rolls back");
    vram.fail_map = false;

    // Uncertain unmap: Manager quarantines, the record survives for a retry.
    vram.fail_unmap = true;
    expect(visible_manager.release(ring) == memory::Status::quarantined, "unmap failure quarantines");
    expect(visible->shutdown() == VramStatus::busy, "pool busy while a backing is live");
    vram.fail_unmap = false;
    expect(visible_manager.release(ring) == memory::Status::success && vram.unmaps == 1,
           "retried destroy unmaps");

    expect(hidden_manager.release(tmr) == memory::Status::success, "GPU-only release");
    expect(visible_manager.shutdown() == memory::Status::success &&
           hidden_manager.shutdown() == memory::Status::success, "Manager shutdown");
    expect(allocator->shutdown() == VramStatus::busy, "allocator busy while pools hold carve-outs");
    expect(visible->shutdown() == VramStatus::success && hidden->shutdown() == VramStatus::success,
           "pools release their carve-outs");
    expect(allocator->available_bytes() == 59 * MiB, "carve-outs returned");
    expect(allocator->shutdown() == VramStatus::success, "allocator shutdown");

    // A pool larger than free visible VRAM fails cleanly.
    expect(allocator->initialize({64 * MiB, 32 * MiB}, reserved.data(), reserved.size()) ==
           VramStatus::success, "relayout");
    expect(visible->initialize(*allocator, mc_base, VramPool::cpu_visible, 32 * MiB, 0, vram.mapper()) ==
           VramStatus::no_space && allocator->available_bytes() == 59 * MiB,
           "oversized visible pool retains nothing");
    allocator->shutdown();
}

} // namespace

int main() {
    check_placement_rules();
    check_initialization();
    check_reservations_never_handed_out();
    check_visible_and_topdown();
    check_contiguous_and_scattered();
    check_try_harder();
    check_pending_reservation();
    check_metadata_exhaustion();
    check_usage_limit();
    check_stress();
    check_manager_integration();
    std::printf("vram allocator: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

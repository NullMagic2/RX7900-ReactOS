/*
Module: rx7900.memory.page_table
Purpose: Prepare checked GFX11 system-memory leaf PTEs for native GPUVM mapping.
Created: 2026-10-21
Architecture: CPU-side preparation only; hierarchy install/TLB ownership is native.
*/
#pragma once

#include "manager.hpp"
#include "byte_view.hpp"

namespace rx7900 {
namespace memory {

enum class PteStatus {
    prepared,
    invalid_address,
    invalid_access,
    invalid_page_list,
    output_too_small
};

struct PageTableResult final {
    size_t bytes_written;
    size_t bytes_required;
    uint64_t virtual_byte_count;
};

// A DMA bus address is supplied by the NT DMA/IOMMU contract for a pinned 4 KiB
// system page. It is not an MDL virtual pointer or GPU virtual address. Encodes
// valid/system/snooped, UC and explicit data read/write access; never executable.
PteStatus prepare_system_pte(uint64_t dma_bus_address, Access access,
                             uint64_t& pte) noexcept;

// Input is an immutable, caller-owned little-endian array of 64-bit DMA page
// addresses; every page must be pinned until native unmap/TLB completion. Output
// must not alias input/result and represents consecutive leaf entries starting
// at virtual_address. Prevalidates the complete list before any write. Entries
// may cross multiple leaf pages; the native caller must place them in the proper
// hierarchy. No existing table may be overwritten while GPU accesses are live.
// prepared does not mean mapped: native publication/HDP synchronization and a
// confirmed GFXHUB VMID0 TLB invalidation are mandatory before exposing handles.
PteStatus encode_system_ptes(uint64_t virtual_address, ByteView dma_page_addresses,
                             Access access, sdma::CommandBuffer output,
                             PageTableResult& result) noexcept;

} // namespace memory
} // namespace rx7900

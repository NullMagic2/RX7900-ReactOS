/*
Module: rx7900.memory.page_table
Purpose: Build permission-bounded GFX11 leaf PTE bytes without pointer overlays.
Created: 2026-10-21
Architecture: Reuses checked ByteView/CommandBuffer and memory access contracts.
Copyright 2016, 2019, 2021 Advanced Micro Devices, Inc.
Adapted GFX11 PTE layout/constants from Linux v6.12 gmc_v11_0.c, amdgpu_vm.h,
and navi10_enum.h. Permission/provenance: licenses/AMDGPU-SDMA.txt.
*/
#include "page_table.hpp"

namespace rx7900 {
namespace memory {
namespace {

uint64_t page_address(ByteView pages, size_t offset) noexcept {
    uint32_t low = 0;
    uint32_t high = 0;
    pages.read_u32(offset, low);
    pages.read_u32(offset + sizeof(uint32_t), high);
    return low | (static_cast<uint64_t>(high) << 32);
}

} // namespace

PteStatus prepare_system_pte(uint64_t dma_bus_address, Access access,
                             uint64_t& pte) noexcept {
    pte = 0;
    const uint32_t permissions = static_cast<uint32_t>(access);

    if (permissions == 0 || (permissions & ~3u) != 0) {
        return PteStatus::invalid_access;
    }

    if ((dma_bus_address & (page_bytes - 1)) != 0 ||
        dma_bus_address > sdma::gpu_virtual_address_max - (page_bytes - 1)) {
        return PteStatus::invalid_address;
    }

    constexpr uint64_t valid_system_snooped_uc = 1 | 2 | 4 | (3ULL << 48);
    const uint64_t readable = (permissions & 1) != 0 ? (1ULL << 5) : 0;
    const uint64_t writable = (permissions & 2) != 0 ? (1ULL << 6) : 0;
    pte = dma_bus_address | valid_system_snooped_uc | readable | writable;
    return PteStatus::prepared;
}

PteStatus encode_system_ptes(uint64_t virtual_address, ByteView dma_page_addresses,
                             Access access, sdma::CommandBuffer output,
                             PageTableResult& result) noexcept {
    result = {};
    const size_t bytes = dma_page_addresses.size();

    if (bytes == 0 || (bytes & 7) != 0) {
        return PteStatus::invalid_page_list;
    }

    const size_t count = bytes / sizeof(uint64_t);

    if (count > (sdma::gpu_virtual_address_max + 1) / page_bytes ||
        (virtual_address & (page_bytes - 1)) != 0 ||
        virtual_address > sdma::gpu_virtual_address_max ||
        static_cast<uint64_t>(count) * page_bytes - 1 >
            sdma::gpu_virtual_address_max - virtual_address) {
        return PteStatus::invalid_address;
    }

    for (size_t offset = 0; offset < bytes; offset += sizeof(uint64_t)) {
        uint64_t pte = 0;
        const auto status = prepare_system_pte(page_address(dma_page_addresses, offset), access, pte);

        if (status != PteStatus::prepared) {
            return status;
        }
    }

    result.bytes_required = bytes;
    result.virtual_byte_count = static_cast<uint64_t>(count) * page_bytes;

    if (output.size() < bytes) {
        return PteStatus::output_too_small;
    }

    for (size_t offset = 0; offset < bytes; offset += sizeof(uint64_t)) {
        uint64_t pte = 0;
        prepare_system_pte(page_address(dma_page_addresses, offset), access, pte);
        output.write_u32(offset, static_cast<uint32_t>(pte));
        output.write_u32(offset + sizeof(uint32_t), static_cast<uint32_t>(pte >> 32));
    }

    result.bytes_written = bytes;
    return PteStatus::prepared;
}

} // namespace memory
} // namespace rx7900

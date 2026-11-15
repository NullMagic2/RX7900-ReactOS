/*
Module: rx7900.memory.gfxhub
Purpose: Validate discovery bases and execute checked VMID0/PTE/TLB MMIO sequences.
Created: 2026-10-21
Architecture: Linux-derived narrow GFXHUB3.0 protocol, unverified on physical hardware.
Copyright 2018, 2021 Advanced Micro Devices, Inc.
Copyright 2009 Jerome Glisse.
Adapted Linux v6.12 gfxhub_v3_0.c, gmc_v11_0.c, amdgpu_gmc.c,
amdgpu_discovery.c/discovery.h and GC11 register definitions.
Permission/provenance: licenses/AMDGPU-SDMA.txt and THIRD_PARTY.txt.
*/
#include "gfxhub.hpp"
#include "../hardware/discovery.hpp"
#include "../hardware/vmhub_layout.hpp"

namespace rx7900 {
namespace memory {
namespace {

constexpr const auto& hub = hardware::gfxhub_3_0_layout;
constexpr uint32_t context0_control = hub.context0_control;
constexpr uint32_t context0_base_low = hub.context0_base_low;
constexpr uint32_t context0_base_high = hub.context0_base_high;
constexpr uint32_t context0_start_low = hub.context0_start_low;
constexpr uint32_t context0_start_high = hub.context0_start_high;
constexpr uint32_t context0_end_low = hub.context0_end_low;
constexpr uint32_t context0_end_high = hub.context0_end_high;
constexpr uint32_t invalidate17_request = hub.invalidate0_request + 17;
constexpr uint32_t invalidate17_ack = hub.invalidate0_ack + 17;
constexpr uint32_t invalidate17_range_low = hub.invalidate0_range_low + 17 * 2;
constexpr uint32_t invalidate17_range_high = hub.invalidate0_range_high + 17 * 2;
constexpr uint32_t vmid0_invalidate_request = 0x00f80001;
constexpr size_t maximum_table_bytes = 8 * 1024 * 1024;

uint32_t u32(ByteView view, size_t offset) noexcept {
    uint32_t value = 0;
    view.read_u32(offset, value);
    return value;
}

bool register_fits(uint32_t base, uint32_t reg, uint32_t aperture) noexcept {
    uint32_t offset = 0;
    return hardware::discovery_register_offset(base, reg, aperture, offset);
}

bool page_span(uint64_t address, uint64_t bytes) noexcept {
    return bytes != 0 && ((address | bytes) & (page_bytes - 1)) == 0 &&
           address <= sdma::gpu_virtual_address_max &&
           bytes - 1 <= sdma::gpu_virtual_address_max - address;
}

} // namespace

GfxhubStatus validate_gc_discovery(ByteView binary, uint32_t aperture,
                                  GcRegisterBase& output) noexcept {
    output = {};

    if (aperture == 0 || (aperture & 3) != 0) {
        return GfxhubStatus::invalid_discovery;
    }

    hardware::DiscoveredIp ip;
    const auto status = hardware::discover_ip(binary,
        {hardware::discovery_gc_hardware_id, 0, 11, 0, 0, 2}, ip);

    if (status == hardware::DiscoveryStatus::unsupported) {
        return GfxhubStatus::unsupported;
    }

    if (status != hardware::DiscoveryStatus::validated ||
        !register_fits(ip.segment(0), context0_end_high, aperture) ||
        !register_fits(ip.segment(1), 0x58ba, aperture)) {
        return GfxhubStatus::invalid_discovery;
    }

    output.bases_[0] = ip.segment(0);
    output.bases_[1] = ip.segment(1);
    output.aperture_bytes_ = aperture;
    output.valid_ = true;
    return GfxhubStatus::discovery_validated;
}

bool Gfxhub::read(size_t segment, uint32_t reg, uint32_t& value) const noexcept {
    if (!registers_.valid() || segment > 1 ||
        !register_fits(registers_.segment(segment), reg, registers_.aperture_bytes())) {
        return false;
    }

    const uint64_t offset = (static_cast<uint64_t>(registers_.segment(segment)) + reg) * 4;
    return io_.read32(io_.context, static_cast<uint32_t>(offset), value);
}

bool Gfxhub::write(uint32_t reg, uint32_t value) const noexcept {
    if (!registers_.valid() ||
        !register_fits(registers_.segment(0), reg, registers_.aperture_bytes())) {
        return false;
    }

    const uint64_t offset = (static_cast<uint64_t>(registers_.segment(0)) + reg) * 4;
    return io_.write32(io_.context, static_cast<uint32_t>(offset), value);
}

bool Gfxhub::write_checked(uint32_t reg, uint32_t value) const noexcept {
    uint32_t observed = 0;
    return write(reg, value) && read(0, reg, observed) && observed == value;
}

GfxhubStatus Gfxhub::idle() const noexcept {
    uint32_t status = 0;

    if (!read(0, 0x0da4, status)) {
        return GfxhubStatus::mmio_failure;
    }

    if ((status & 0x80000000u) != 0) {
        return GfxhubStatus::busy;
    }

    for (uint32_t instance = 0; instance < 2; ++instance) {
        uint32_t halt = 0;

        if (!read(0, 0x25 + instance * 0x600, status) ||
            !read(1, 0x589a + instance * 0x20, halt)) {
            return GfxhubStatus::mmio_failure;
        }

        if ((status & 1) == 0 || (halt & 1) == 0) {
            return GfxhubStatus::busy;
        }
    }

    // User VMIDs must stay disabled: this module owns a system-only bring-up VM.
    for (uint32_t vmid = 1; vmid < 16; ++vmid) {
        if (!read(0, context0_control + vmid, status)) {
            return GfxhubStatus::mmio_failure;
        }

        if ((status & 1) != 0) {
            return GfxhubStatus::busy;
        }
    }

    return GfxhubStatus::configured;
}

GfxhubStatus Gfxhub::dependencies() const noexcept {
    const auto idle_status = idle();

    if (idle_status != GfxhubStatus::configured) {
        return idle_status;
    }

    struct Expected final { uint32_t reg; uint32_t mask; uint32_t value; };
    const Expected expected[]{
        {hub.fb_location_base, 0x00ffffff,
            static_cast<uint32_t>(configuration_.vram_mc_start >> 24)},
        {hub.fb_location_top, 0x00ffffff, static_cast<uint32_t>((configuration_.vram_mc_start +
            configuration_.vram_byte_count - 1) >> 24)},
        {hub.l1_tlb_control, 0x00003879, 0x00001859},
        {hub.l2_control, 0x0c00003f, 1},
        {hub.l2_control3, 0x000f803f, 9 | (6 << 15)},
        {hub.l2_control4, 0x000000c0, 0},
        {hub.l2_control5, 0x0000001f, 0}
    };

    for (const auto& item : expected) {
        uint32_t value = 0;

        if (!read(0, item.reg, value)) {
            return GfxhubStatus::mmio_failure;
        }

        if ((value & item.mask) != item.value) {
            return GfxhubStatus::dependencies_unmet;
        }
    }

    uint32_t system_low = 0;
    uint32_t system_high = 0;
    uint32_t agp_low = 0;
    uint32_t agp_high = 0;
    uint32_t agp_base = 0;

    if (!read(0, hub.system_aperture_low, system_low) ||
        !read(0, hub.system_aperture_high, system_high) ||
        !read(0, hub.agp_bot, agp_low) || !read(0, hub.agp_top, agp_high) ||
        !read(0, hub.agp_base, agp_base)) {
        return GfxhubStatus::mmio_failure;
    }

    const uint64_t vram_end = configuration_.vram_mc_start + configuration_.vram_byte_count - 1;
    const auto aperture = configuration_.virtual_aperture;
    const uint64_t aperture_end = aperture.virtual_address + aperture.byte_length - 1;

    if (system_low > 0x3fffffff || system_high > 0x3fffffff || system_low > system_high ||
        system_low > (configuration_.vram_mc_start >> 18) || system_high < (vram_end >> 18) ||
        ((aperture.virtual_address >> 18) <= system_high && system_low <= (aperture_end >> 18)) ||
        agp_base != 0 || agp_low > 0xffffff || agp_high > 0xffffff ||
        (agp_low <= agp_high && (aperture.virtual_address >> 24) <= agp_high &&
         agp_low <= (aperture_end >> 24))) {
        return GfxhubStatus::dependencies_unmet;
    }

    return GfxhubStatus::configured;
}

bool Gfxhub::set_context_enabled(bool enabled) const noexcept {
    uint32_t control = 0;

    if (!read(0, context0_control, control)) {
        return false;
    }

    // Linux system domain is depth0 with permission/invalid-page retry disabled.
    control &= ~(1u | 6u | 0x80u);
    control |= enabled ? 1u : 0u;
    return write_checked(context0_control, control);
}

GfxhubStatus Gfxhub::invalidate() const noexcept {
    if (!write_checked(invalidate17_range_low, 0xffffffff) ||
        !write_checked(invalidate17_range_high, 0x1f) ||
        !write(invalidate17_request, vmid0_invalidate_request)) {
        return GfxhubStatus::mmio_failure;
    }

    // Readback drains the posted request before ACK is interpreted. REQ may
    // self-clear; Linux's direct path polls the VMID bit in engine17 ACK.
    uint32_t value = 0;

    if (!read(0, invalidate17_request, value) || value == UINT32_MAX) {
        return GfxhubStatus::mmio_failure;
    }

    for (uint32_t iteration = 0; iteration < configuration_.poll_iterations; ++iteration) {
        if (!read(0, invalidate17_ack, value) || value == UINT32_MAX) {
            return GfxhubStatus::mmio_failure;
        }

        if ((value & 1) != 0) {
            return GfxhubStatus::configured;
        }

        io_.stall_one_microsecond(io_.context);
    }

    return GfxhubStatus::timeout;
}

GfxhubStatus Gfxhub::resume_translation() noexcept {
    if (!io_.publish_ptes(io_.context) || !set_context_enabled(true)) {
        quarantined_ = true;
        return GfxhubStatus::quarantined;
    }

    if (invalidate() != GfxhubStatus::configured) {
        set_context_enabled(false);
        quarantined_ = true;
        return GfxhubStatus::quarantined;
    }

    return GfxhubStatus::configured;
}

GfxhubStatus Gfxhub::initialize(GcRegisterBase registers, FlatGart configuration,
                                GfxhubIo io) noexcept {
    if (bound_) {
        return GfxhubStatus::busy;
    }

    if (!registers.valid() || io.read32 == nullptr || io.write32 == nullptr ||
        io.publish_ptes == nullptr || io.stall_one_microsecond == nullptr) {
        return GfxhubStatus::unavailable;
    }

    const auto aperture = configuration.virtual_aperture;
    const uint64_t pages = aperture.byte_length / page_bytes;

    if (!page_span(aperture.virtual_address, aperture.byte_length) ||
        !page_span(configuration.table_dma_bus_address, configuration.table_cpu_byte_count) ||
        configuration.table_cpu_bytes == nullptr ||
        configuration.table_cpu_byte_count > maximum_table_bytes ||
        pages > configuration.table_cpu_byte_count / sizeof(uint64_t) ||
        !page_span(configuration.vram_mc_start, configuration.vram_byte_count) ||
        ((configuration.vram_mc_start | configuration.vram_byte_count) & 0xffffff) != 0 ||
        configuration.poll_iterations == 0 || configuration.poll_iterations > 100000) {
        return GfxhubStatus::invalid_argument;
    }

    const bool overlaps_vram = aperture.virtual_address <= configuration.vram_mc_start
        ? configuration.vram_mc_start - aperture.virtual_address < aperture.byte_length
        : aperture.virtual_address - configuration.vram_mc_start < configuration.vram_byte_count;

    if (overlaps_vram) {
        return GfxhubStatus::invalid_argument;
    }

    registers_ = registers;
    configuration_ = configuration;
    io_ = io;
    const auto prerequisite = dependencies();
    uint32_t control = 0;

    if (prerequisite != GfxhubStatus::configured) {
        return prerequisite;
    }

    if (!read(0, context0_control, control)) {
        return GfxhubStatus::mmio_failure;
    }

    if ((control & 1) != 0) {
        return GfxhubStatus::busy;
    }

    used_table_bytes_ = static_cast<size_t>(pages) * sizeof(uint64_t);
    bound_ = true;
    quarantined_ = false;
    sdma::CommandBuffer table(configuration.table_cpu_bytes, used_table_bytes_);

    for (size_t offset = 0; offset < used_table_bytes_; offset += sizeof(uint32_t)) {
        table.write_u32(offset, 0);
    }

    const uint64_t root = configuration.table_dma_bus_address | 7;
    const uint64_t end = aperture.virtual_address + aperture.byte_length - 1;
    const struct { uint32_t reg; uint32_t value; } settings[]{
        {context0_base_low, static_cast<uint32_t>(root)},
        {context0_base_high, static_cast<uint32_t>(root >> 32)},
        {context0_start_low, static_cast<uint32_t>(aperture.virtual_address >> 12)},
        {context0_start_high, static_cast<uint32_t>(aperture.virtual_address >> 44)},
        {context0_end_low, static_cast<uint32_t>(end >> 12)},
        {context0_end_high, static_cast<uint32_t>(end >> 44)}
    };

    for (const auto& setting : settings) {
        if (!write_checked(setting.reg, setting.value)) {
            quarantined_ = true;
            return GfxhubStatus::quarantined;
        }
    }

    return resume_translation();
}

bool Gfxhub::window(uint64_t address, uint64_t bytes, size_t& offset,
                    size_t& output_bytes) const noexcept {
    const auto arena = configuration_.virtual_aperture;

    if (!page_span(address, bytes) || address < arena.virtual_address ||
        address - arena.virtual_address > arena.byte_length ||
        bytes > arena.byte_length - (address - arena.virtual_address)) {
        return false;
    }

    offset = static_cast<size_t>((address - arena.virtual_address) / page_bytes) * sizeof(uint64_t);
    output_bytes = static_cast<size_t>(bytes / page_bytes) * sizeof(uint64_t);
    return offset <= used_table_bytes_ && output_bytes <= used_table_bytes_ - offset;
}

GfxhubStatus Gfxhub::map(uint64_t address, ByteView pages, Access access) noexcept {
    if (!bound_ || quarantined_) {
        return quarantined_ ? GfxhubStatus::quarantined : GfxhubStatus::unavailable;
    }

    if (pages.size() == 0 || (pages.size() & 7) != 0 || pages.size() > used_table_bytes_) {
        return GfxhubStatus::invalid_argument;
    }

    size_t offset = 0;
    size_t bytes = 0;
    const uint64_t span_bytes = static_cast<uint64_t>(pages.size() / 8) * page_bytes;

    if (!window(address, span_bytes, offset, bytes)) {
        return GfxhubStatus::invalid_argument;
    }

    PageTableResult result{};

    if (encode_system_ptes(address, pages, access, {nullptr, 0}, result) != PteStatus::output_too_small) {
        return GfxhubStatus::invalid_argument;
    }

    const ByteView table(configuration_.table_cpu_bytes, used_table_bytes_);

    for (size_t entry = 0; entry < bytes; entry += 8) {
        if (u32(table, offset + entry) != 0 || u32(table, offset + entry + 4) != 0) {
            return GfxhubStatus::busy;
        }

        const uint64_t dma = u32(pages, entry) | (static_cast<uint64_t>(u32(pages, entry + 4)) << 32);

        if (dma >= configuration_.table_dma_bus_address &&
            dma - configuration_.table_dma_bus_address < configuration_.table_cpu_byte_count) {
            return GfxhubStatus::invalid_argument;
        }
    }

    const auto idle_status = idle();

    if (idle_status != GfxhubStatus::configured) {
        return idle_status;
    }

    if (!set_context_enabled(false)) {
        quarantined_ = true;
        return GfxhubStatus::quarantined;
    }

    sdma::CommandBuffer writable(configuration_.table_cpu_bytes, used_table_bytes_);

    if (encode_system_ptes(address, pages, access, writable.subbuffer(offset, bytes), result) !=
        PteStatus::prepared) {
        quarantined_ = true;
        return GfxhubStatus::quarantined;
    }

    return resume_translation() == GfxhubStatus::configured
        ? GfxhubStatus::mapped : GfxhubStatus::quarantined;
}

GfxhubStatus Gfxhub::unmap(uint64_t address, uint64_t byte_count) noexcept {
    if (!bound_ || quarantined_) {
        return quarantined_ ? GfxhubStatus::quarantined : GfxhubStatus::unavailable;
    }

    size_t offset = 0;
    size_t bytes = 0;

    if (!window(address, byte_count, offset, bytes)) {
        return GfxhubStatus::invalid_argument;
    }

    const ByteView table(configuration_.table_cpu_bytes, used_table_bytes_);

    for (size_t entry = 0; entry < bytes; entry += 8) {
        if ((u32(table, offset + entry) & 1) == 0) {
            return GfxhubStatus::invalid_argument;
        }
    }

    const auto idle_status = idle();

    if (idle_status != GfxhubStatus::configured) {
        return idle_status;
    }

    if (!set_context_enabled(false)) {
        quarantined_ = true;
        return GfxhubStatus::quarantined;
    }

    sdma::CommandBuffer writable(configuration_.table_cpu_bytes, used_table_bytes_);

    for (size_t entry = 0; entry < bytes; entry += 4) {
        writable.write_u32(offset + entry, 0);
    }

    return resume_translation() == GfxhubStatus::configured
        ? GfxhubStatus::unmapped : GfxhubStatus::quarantined;
}

GfxhubStatus Gfxhub::disable() noexcept {
    if (!bound_) {
        return GfxhubStatus::unavailable;
    }

    const auto idle_status = idle();

    if (idle_status != GfxhubStatus::configured) {
        return idle_status;
    }

    if (!set_context_enabled(false) || !io_.publish_ptes(io_.context) ||
        invalidate() != GfxhubStatus::configured) {
        quarantined_ = true;
        return GfxhubStatus::quarantined;
    }

    bound_ = false;
    quarantined_ = false;
    registers_ = {};
    configuration_ = {};
    io_ = {};
    used_table_bytes_ = 0;
    return GfxhubStatus::disabled;
}

} // namespace memory
} // namespace rx7900

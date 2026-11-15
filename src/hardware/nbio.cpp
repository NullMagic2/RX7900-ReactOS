/*
Module: rx7900.hardware.nbio
Purpose: Program HDP remap/flush, indirect SMN access and read VRAM placement.
Created: 2026-10-21
Architecture: Bounded register protocol; native owner performs the MMIO.
Copyright 2021 Advanced Micro Devices, Inc.; Copyright 2020 Advanced Micro Devices, Inc.;
Copyright 2008 Advanced Micro Devices, Inc., Copyright 2008 Red Hat Inc.,
Copyright 2009 Jerome Glisse. Adapted Linux v6.12 nbio_v4_3.c, hdp_v6_0.c,
mmhub_v3_0.c, gmc_v11_0.c, soc21.c, amdgpu_device.c, nbio_4_3_0 and mmhub_3_0_0
register headers, kfd_ioctl.h. Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#include "nbio.hpp"
#include "vmhub_layout.hpp"

namespace rx7900::hardware {
namespace {

constexpr uint8_t nbif_segments = 6;
constexpr uint32_t rsmu_index = 0x0000;         // segment 1
constexpr uint32_t rsmu_data = 0x0001;          // segment 1
constexpr uint32_t rcc_config_memsize = 0x00c3; // segment 2
constexpr uint32_t remap_hdp_mem_flush = 0x012d; // segment 2
constexpr uint32_t remap_hdp_reg_flush = 0x012e; // segment 2
constexpr uint32_t epf2_strap2 = 0xd102;        // segment 5
constexpr uint32_t doorbell_aperture_enable = 0x00c0; // segment 2
constexpr uint32_t selfring_base_high = 0x00f3; // segment 2
constexpr uint32_t selfring_base_low = 0x00f4;  // segment 2
constexpr uint32_t selfring_control = 0x00f5;   // segment 2
constexpr uint32_t selfring_enable_mode = 0x1u | (1u << 1); // EN=1, MODE=1, SIZE=0
constexpr uint32_t s2a_doorbell_entry0 = 0x4f7640; // segment 3
constexpr uint32_t s2a_doorbell_entry3 = 0x4f7643; // segment 3
constexpr uint32_t strap_no_soft_reset_f2 = 0x00000080u;
constexpr uint32_t kfd_remap_hdp_mem_flush = 0;
constexpr uint32_t kfd_remap_hdp_reg_flush = 4;

bool fits(uint32_t base, uint32_t reg, uint32_t aperture) noexcept {
    uint32_t offset = 0;
    return discovery_register_offset(base, reg, aperture, offset);
}

} // namespace

bool retain_local_vram(memory::Manager& manager, memory::Handle handle,
                       const VramLocation& vram, uint64_t minimum_bytes, uint64_t alignment,
                       bool cpu_visible, memory::AllocationInfo& output) noexcept {
    memory::AllocationInfo allocation{};
    sdma::GpuRange range{};

    if (minimum_bytes == 0 || alignment == 0 || (alignment & (alignment - 1)) != 0 ||
        vram.bytes == 0 || manager.describe(handle, allocation) != memory::Status::success ||
        manager.resolve({handle, 0}, minimum_bytes, memory::Access::read_write, range) !=
            memory::Status::success) {
        return false;
    }

    const uint64_t start = allocation.gpu.virtual_address;
    const uint64_t length = allocation.gpu.byte_length;

    if ((start & (alignment - 1)) != 0 || length < minimum_bytes || start < vram.mc_start ||
        start > vram.mc_end || length - 1 > vram.mc_end - start ||
        (cpu_visible && (allocation.backing.cpu_bytes == nullptr ||
                         allocation.backing.cpu_byte_count < minimum_bytes))) {
        return false;
    }

    if (manager.retain(handle, memory::Access::read_write) != memory::Status::success) {
        return false;
    }

    output = allocation;
    return true;
}

DiscoveryStatus validate_nbio_discovery(ByteView binary, uint32_t aperture,
                                        NbioRegisters& output) noexcept {
    output = {};

    if (aperture == 0 || (aperture & 3u) != 0 ||
        hdp_remap_byte_offset + kfd_remap_hdp_reg_flush + sizeof(uint32_t) > aperture) {
        return DiscoveryStatus::invalid_argument;
    }

    DiscoveredIp nbif;
    DiscoveredIp mmhub;
    auto status = discover_ip(binary, {discovery_nbif_hardware_id, 0, 4, 3, 0, nbif_segments}, nbif);

    if (status != DiscoveryStatus::validated) {
        return status;
    }

    status = discover_ip(binary, {discovery_mmhub_hardware_id, 0, 3, 0, 0, 1}, mmhub);

    if (status != DiscoveryStatus::validated) {
        return status;
    }

    if (!fits(nbif.segment(1), rsmu_data, aperture) ||
        !fits(nbif.segment(2), remap_hdp_reg_flush, aperture) ||
        !fits(nbif.segment(2), rcc_config_memsize, aperture) ||
        (static_cast<uint64_t>(nbif.segment(5)) + epf2_strap2) * 4 > UINT32_MAX - 3 ||
        !fits(mmhub.segment(0), mmhub_3_0_0_layout.fb_location_top, aperture) ||
        !fits(mmhub.segment(0), mmhub_3_0_0_layout.fb_offset, aperture)) {
        return DiscoveryStatus::invalid_argument;
    }

    output.nbif_ = nbif;
    output.mmhub_ = mmhub;
    output.aperture_bytes_ = aperture;
    return DiscoveryStatus::validated;
}

NbioStatus Nbio::initialize(NbioRegisters registers, RegisterIo io) noexcept {
    initialized_ = false;
    remapped_ = false;

    if (!registers.valid()) {
        return NbioStatus::unsupported_ip;
    }

    if (!register_io_valid(io) || io.aperture_bytes != registers.aperture_bytes()) {
        return NbioStatus::unavailable;
    }

    registers_ = registers;
    io_ = io;
    initialized_ = true;
    return NbioStatus::ready;
}

NbioStatus Nbio::program_common() noexcept {
    if (!initialized_) {
        return NbioStatus::unavailable;
    }

    remapped_ = false;
    uint32_t strap = 0;

    if (!read_register(registers_.nbif_base(5), epf2_strap2, strap) || strap == UINT32_MAX) {
        return NbioStatus::mmio_failure;
    }

    if ((strap & strap_no_soft_reset_f2) != 0 &&
        !write_register(registers_.nbif_base(5), epf2_strap2, strap & ~strap_no_soft_reset_f2)) {
        return NbioStatus::mmio_failure;
    }

    const uint32_t mem_flush = hdp_remap_byte_offset + kfd_remap_hdp_mem_flush;
    const uint32_t reg_flush = hdp_remap_byte_offset + kfd_remap_hdp_reg_flush;
    uint32_t observed_mem = 0;
    uint32_t observed_reg = 0;

    if (!write_ip_register(io_, registers_.nbif_base(2), remap_hdp_mem_flush, mem_flush) ||
        !write_ip_register(io_, registers_.nbif_base(2), remap_hdp_reg_flush, reg_flush) ||
        !read_ip_register(io_, registers_.nbif_base(2), remap_hdp_mem_flush, observed_mem) ||
        !read_ip_register(io_, registers_.nbif_base(2), remap_hdp_reg_flush, observed_reg)) {
        return NbioStatus::mmio_failure;
    }

    if (observed_mem != mem_flush || observed_reg != reg_flush) {
        return NbioStatus::readback_mismatch;
    }

    remapped_ = true;
    return enable_doorbell_aperture(true);
}

NbioStatus Nbio::enable_doorbell_aperture(bool enable) noexcept {
    uint32_t value = 0;

    if (!initialized_) {
        return NbioStatus::unavailable;
    }

    if (!read_register(registers_.nbif_base(2), doorbell_aperture_enable, value) ||
        value == UINT32_MAX ||
        !write_register(registers_.nbif_base(2), doorbell_aperture_enable,
                        (value & ~1u) | (enable ? 1u : 0u))) {
        return NbioStatus::mmio_failure;
    }

    return NbioStatus::ready;
}

NbioStatus Nbio::enable_doorbell_selfring(uint64_t doorbell_bus_base) noexcept {
    if (!initialized_) {
        return NbioStatus::unavailable;
    }

    if (!write_register(registers_.nbif_base(2), selfring_base_low,
                        static_cast<uint32_t>(doorbell_bus_base)) ||
        !write_register(registers_.nbif_base(2), selfring_base_high,
                        static_cast<uint32_t>(doorbell_bus_base >> 32)) ||
        !write_register(registers_.nbif_base(2), selfring_control, selfring_enable_mode)) {
        return NbioStatus::mmio_failure;
    }

    return NbioStatus::ready;
}

NbioStatus Nbio::disable_doorbell_selfring() noexcept {
    if (!initialized_) {
        return NbioStatus::unavailable;
    }

    return write_register(registers_.nbif_base(2), selfring_control, 0) ? NbioStatus::ready :
                                                                          NbioStatus::mmio_failure;
}

bool Nbio::read_memsize(uint32_t& value) noexcept {
    return initialized_ && read_ip_register(io_, registers_.nbif_base(2), rcc_config_memsize, value);
}

NbioStatus Nbio::gc_doorbell_init() noexcept {
    if (!initialized_) {
        return NbioStatus::unavailable;
    }

    if (!write_register(registers_.nbif_base(3), s2a_doorbell_entry0, 0x30000007u) ||
        !write_register(registers_.nbif_base(3), s2a_doorbell_entry3, 0x3000000du)) {
        return NbioStatus::mmio_failure;
    }

    return NbioStatus::ready;
}

bool Nbio::flush_hdp() noexcept {
    if (!initialized_ || !remapped_) {
        return false;
    }

    // Byte offset in the remap hole; base 0 means an absolute dword index.
    return write_ip_register(io_, 0, (hdp_remap_byte_offset + kfd_remap_hdp_mem_flush) / 4, 0);
}

NbioStatus Nbio::vram_location(VramLocation& output) noexcept {
    output = {};

    if (!initialized_) {
        return NbioStatus::unavailable;
    }

    uint32_t memsize = 0;
    uint32_t base = 0;
    uint32_t top = 0;
    uint32_t offset = 0;

    if (!read_ip_register(io_, registers_.nbif_base(2), rcc_config_memsize, memsize) ||
        !read_ip_register(io_, registers_.mmhub_base(), mmhub_3_0_0_layout.fb_location_base, base) ||
        !read_ip_register(io_, registers_.mmhub_base(), mmhub_3_0_0_layout.fb_location_top, top) ||
        !read_ip_register(io_, registers_.mmhub_base(), mmhub_3_0_0_layout.fb_offset, offset) ||
        memsize == UINT32_MAX || base == UINT32_MAX || top == UINT32_MAX || offset == UINT32_MAX) {
        return NbioStatus::mmio_failure;
    }

    const uint64_t bytes = static_cast<uint64_t>(memsize) << 20;
    const uint64_t start = static_cast<uint64_t>(base & vmhub_fb_field_mask) << 24;
    const uint64_t window_end = (static_cast<uint64_t>(top & vmhub_fb_field_mask) << 24) | 0xffffffu;

    // MC space is 48-bit; start+bytes-1 cannot overflow 64-bit arithmetic here.
    if (bytes == 0 || start > window_end || bytes - 1 > window_end - start) {
        return NbioStatus::invalid_location;
    }

    output.mc_start = start;
    output.mc_end = start + bytes - 1;
    output.bytes = bytes;
    output.physical_offset = static_cast<uint64_t>(offset & vmhub_fb_field_mask) << 24;
    return NbioStatus::ready;
}

bool Nbio::read_smn(uint32_t address, uint32_t& value) noexcept {
    uint32_t posted = 0;

    return initialized_ &&
           write_ip_register(io_, registers_.nbif_base(1), rsmu_index, address) &&
           read_ip_register(io_, registers_.nbif_base(1), rsmu_index, posted) &&
           read_ip_register(io_, registers_.nbif_base(1), rsmu_data, value);
}

bool Nbio::write_smn(uint32_t address, uint32_t value) noexcept {
    uint32_t posted = 0;

    return initialized_ &&
           write_ip_register(io_, registers_.nbif_base(1), rsmu_index, address) &&
           read_ip_register(io_, registers_.nbif_base(1), rsmu_index, posted) &&
           write_ip_register(io_, registers_.nbif_base(1), rsmu_data, value) &&
           read_ip_register(io_, registers_.nbif_base(1), rsmu_data, posted);
}

bool Nbio::read_register(uint32_t base, uint32_t reg, uint32_t& value) noexcept {
    const uint64_t byte = (static_cast<uint64_t>(base) + reg) * 4;

    if (!initialized_ || byte > UINT32_MAX - 3) {
        return false;
    }

    if (byte + 4 <= io_.aperture_bytes) {
        return read_ip_register(io_, base, reg, value);
    }

    return read_smn(static_cast<uint32_t>(byte), value);
}

bool Nbio::write_register(uint32_t base, uint32_t reg, uint32_t value) noexcept {
    const uint64_t byte = (static_cast<uint64_t>(base) + reg) * 4;

    if (!initialized_ || byte > UINT32_MAX - 3) {
        return false;
    }

    if (byte + 4 <= io_.aperture_bytes) {
        return write_ip_register(io_, base, reg, value);
    }

    return write_smn(static_cast<uint32_t>(byte), value);
}

} // namespace rx7900::hardware

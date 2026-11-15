/*
Module: rx7900.common.pci_caps
Purpose: Checked PCI capability decoding and Linux v6.12 PCIe/ReBAR/restore policy.
Created: 2026-10-22
Architecture: Allocation-free; every read is a bounded ByteView access.
Adapted from Linux v6.12 (GPL-2.0 drivers/pci and MIT amdgpu sources; pins and
notices in THIRD_PARTY.txt): __pci_find_next_cap_ttl, pci_find_next_ext_capability,
pci_cfg_space_size_ext, pcie_capability_reg_implemented, pcie_get_speed_cap,
pcie_get_width_cap, pcie_bandwidth_available, pci_rebar_*, pci_save_state,
pci_restore_state, amdgpu_device_partner_bandwidth, amdgpu_device_get_pcie_info,
amdgpu_device_resize_fb_bar and the smu_smc_hw_setup PCIe encoding.
*/
#include "pci_caps.hpp"

namespace rx7900 {
namespace {

constexpr uint16_t pci_status_cap_list = 0x0010;
constexpr size_t pci_capability_list_offset = 0x34;
constexpr size_t pci_cardbus_capability_list_offset = 0x14;
constexpr uint8_t pci_header_type_cardbus = 2;
constexpr uint8_t pci_capability_minimum = 0x40;
constexpr uint8_t pci_capability_end_id = 0xff;
constexpr uint16_t pci_ext_first = 0x100;
constexpr uint32_t pci_ext_ttl = (pci_extended_config_bytes - pci_config_bytes) / 8;
constexpr uint16_t pci_invalid_vendor = 0xffff;

constexpr uint16_t pcie_flags_version = 0x000f;
constexpr uint16_t pcie_flags_slot = 0x0100;
constexpr uint32_t pcie_lnkcap_sls = 0x0000000f;
constexpr uint32_t pcie_width_mask = 0x3f;
constexpr uint32_t pcie_width_shift = 4;
constexpr uint32_t pcie_lnkcap2_2_5 = 0x02;
constexpr uint32_t pcie_lnkcap2_5_0 = 0x04;
constexpr uint32_t pcie_lnkcap2_8_0 = 0x08;
constexpr uint32_t pcie_lnkcap2_16_0 = 0x10;
constexpr uint32_t pcie_lnkcap2_32_0 = 0x20;
constexpr uint32_t pcie_lnkcap2_64_0 = 0x40;

constexpr uint16_t msi_enable = 0x0001;
constexpr uint16_t msi_64bit = 0x0080;
constexpr uint16_t msi_mask_capable = 0x0100;
constexpr uint8_t msi_reserved_vectors = 5;
constexpr uint16_t msix_table_size = 0x07ff;
constexpr uint16_t msix_mask_all = 0x4000;
constexpr uint16_t msix_enable = 0x8000;
constexpr uint32_t msix_bir = 0x7;
constexpr uint8_t pci_last_bar = 5;

constexpr uint32_t rebar_cap_sizes = 0x00fffff0;
constexpr uint32_t rebar_ctrl_index = 0x7;
constexpr uint32_t rebar_ctrl_nbar_shift = 5;
constexpr uint32_t rebar_ctrl_size_shift = 8;
constexpr uint32_t rebar_ctrl_size_mask = 0x1f;
constexpr uint8_t rebar_largest_shift = 43;
constexpr uint8_t rebar_invalid_size = 0xff;

constexpr uint16_t amd_rx5600_pulse_device = 0x731f;
constexpr uint32_t amd_rx5600_bad_sizes = 0x700;
constexpr uint32_t amd_rx5600_sizes = 0x3f00;

constexpr uint8_t bar_restore_retries = 10;
constexpr uint32_t pci_bridge_busses_offset = 0x18;
constexpr uint32_t pci_header_dword_offset = 0x0c;
constexpr uint8_t pci_multifunction_bit = 0x80;
constexpr uint8_t pci_device_count = 32;
constexpr uint8_t pci_function_count = 8;

bool header_type(ByteView config, uint8_t& type) noexcept {
    uint8_t raw = 0;

    if (!config.read_u8(pci_header_type_offset, raw)) {
        return false;
    }

    type = static_cast<uint8_t>(raw & pci_header_type_mask);
    return true;
}

// __pci_bus_find_cap_start: zero when Status.CapList is clear or the header
// type has no capability pointer.
bool capability_pointer(ByteView config, size_t& pointer) noexcept {
    uint16_t status = 0;
    uint8_t type = 0;

    if (!config.read_u16(pci_status_offset, status) || !header_type(config, type)) {
        return false;
    }

    pointer = 0;

    if ((status & pci_status_cap_list) == 0) {
        return true;
    }

    if (type == pci_header_type_normal || type == pci_header_type_bridge) {
        pointer = pci_capability_list_offset;
    } else if (type == pci_header_type_cardbus) {
        pointer = pci_cardbus_capability_list_offset;
    }

    return true;
}

bool contains_extent(ByteView config, size_t offset, size_t bytes) noexcept {
    return config.contains(offset, bytes);
}

// Standard capabilities live entirely inside the legacy 256-byte space, even
// when the caller supplies an extended image; a register crossing 0xFF is
// malformed. Output is empty when the image is shorter than 256 bytes.
ByteView legacy_view(ByteView config) noexcept {
    ByteView legacy(nullptr, 0);
    config.subview(0, pci_config_bytes, legacy);
    return legacy;
}

bool is_downstream_port(uint8_t type) noexcept {
    return type == pcie_type_root_port || type == pcie_type_downstream ||
           type == pcie_type_pcie_bridge;
}

bool has_link_registers(uint8_t type) noexcept {
    return type == pcie_type_endpoint || type == pcie_type_legacy_endpoint ||
           type == pcie_type_root_port || type == pcie_type_upstream ||
           type == pcie_type_downstream || type == pcie_type_pci_bridge ||
           type == pcie_type_pcie_bridge;
}

uint32_t speed_to_encoded_mbps(PcieSpeed speed) noexcept {
    switch (speed) {
    case PcieSpeed::gt64_0: return 64000;
    case PcieSpeed::gt32_0: return 32000 * 128 / 130;
    case PcieSpeed::gt16_0: return 16000 * 128 / 130;
    case PcieSpeed::gt8_0: return 8000 * 128 / 130;
    case PcieSpeed::gt5_0: return 5000 * 8 / 10;
    case PcieSpeed::gt2_5: return 2500 * 8 / 10;
    default: return 0;
    }
}

uint32_t asic_gen_bits(PcieSpeed speed) noexcept {
    constexpr uint32_t gen12 = cail_asic_pcie_link_speed_gen1 | cail_asic_pcie_link_speed_gen2;
    constexpr uint32_t gen123 = gen12 | cail_asic_pcie_link_speed_gen3;
    constexpr uint32_t gen1234 = gen123 | cail_asic_pcie_link_speed_gen4;

    switch (speed) {
    case PcieSpeed::unknown: return gen123;
    case PcieSpeed::gt32_0: return gen1234 | cail_asic_pcie_link_speed_gen5;
    case PcieSpeed::gt16_0: return gen1234;
    case PcieSpeed::gt8_0: return gen123;
    case PcieSpeed::gt5_0: return gen12;
    // amdgpu has no 64 GT/s case: it falls into the Gen1-only else branch.
    default: return cail_asic_pcie_link_speed_gen1;
    }
}

uint32_t platform_gen_bits(PcieSpeed speed) noexcept {
    constexpr uint32_t gen12 = cail_pcie_link_speed_gen1 | cail_pcie_link_speed_gen2;
    constexpr uint32_t gen123 = gen12 | cail_pcie_link_speed_gen3;
    constexpr uint32_t gen1234 = gen123 | cail_pcie_link_speed_gen4;

    switch (speed) {
    case PcieSpeed::unknown: return gen12;
    case PcieSpeed::gt32_0: return gen1234 | cail_pcie_link_speed_gen5;
    case PcieSpeed::gt16_0: return gen1234;
    case PcieSpeed::gt8_0: return gen123;
    case PcieSpeed::gt5_0: return gen12;
    default: return cail_pcie_link_speed_gen1;
    }
}

uint32_t platform_width_bits(uint8_t width) noexcept {
    constexpr uint32_t x1 = cail_pcie_link_width_x1;
    constexpr uint32_t x2 = x1 | cail_pcie_link_width_x2;
    constexpr uint32_t x4 = x2 | cail_pcie_link_width_x4;
    constexpr uint32_t x8 = x4 | cail_pcie_link_width_x8;
    constexpr uint32_t x12 = x8 | cail_pcie_link_width_x12;
    constexpr uint32_t x16 = x12 | cail_pcie_link_width_x16;

    switch (width) {
    case pcie_width_unknown: return amdgpu_default_pcie_mlw_mask;
    case 32: return x16 | cail_pcie_link_width_x32;
    case 16: return x16;
    case 12: return x12;
    case 8: return x8;
    case 4: return x4;
    case 2: return x2;
    case 1: return x1;
    // Reserved/odd widths hit the switch default: the mask stays zero.
    default: return 0;
    }
}

// pci_upstream_bridge returns bus->self: always a type-1 bridge header.
CapStatus validate_function(ByteView config, uint8_t expected_type) noexcept {
    uint16_t vendor = 0;
    uint8_t type = 0;

    if (config.size() < pci_config_bytes) {
        return CapStatus::truncated_input;
    }

    if (!config.read_u16(pci_vendor_offset, vendor) || !header_type(config, type)) {
        return CapStatus::truncated_input;
    }

    if (vendor == pci_invalid_vendor || vendor == 0 || type != expected_type) {
        return CapStatus::malformed;
    }

    return CapStatus::success;
}

// Absent capabilities read as zero (pcie_capability_read_* on a non-PCIe function).
CapStatus decode_pcie_or_zero(ByteView config, PcieCapability& output) noexcept {
    const CapStatus status = decode_pcie(config, output);
    return status == CapStatus::absent ? CapStatus::success : status;
}

uint8_t highest_bit(uint32_t value) noexcept {
    uint8_t bit = 0;

    while (value > 1) {
        value >>= 1;
        ++bit;
    }

    return bit;
}

class PlanWriter final {
public:
    explicit PlanWriter(RestorePlan& plan) noexcept : plan_(plan) {}

    void add(size_t offset, uint8_t width, uint32_t value, bool force,
             uint8_t retries = 0) noexcept {
        if (plan_.count >= pci_restore_capacity || offset > UINT16_MAX) {
            overflow_ = true;
            return;
        }

        plan_.writes[plan_.count] = {static_cast<uint16_t>(offset), width, retries, force, value};
        ++plan_.count;
    }

    bool overflowed() const noexcept { return overflow_; }

private:
    RestorePlan& plan_;
    bool overflow_ = false;
};

void add_header_range(PlanWriter& writer, const PciSavedState& state, int first, int last,
                      uint8_t retries, bool force) noexcept {
    for (int index = last; index >= first; --index) {
        writer.add(static_cast<size_t>(index) * sizeof(uint32_t), sizeof(uint32_t),
                   state.header[index], force, retries);
    }
}

} // namespace

CapStatus walk_capabilities(ByteView config, CapabilityList& output) noexcept {
    output = {};

    if (config.size() < pci_config_bytes) {
        return CapStatus::truncated_input;
    }

    size_t pointer = 0;

    if (!capability_pointer(config, pointer)) {
        return CapStatus::truncated_input;
    }

    if (pointer == 0) {
        return CapStatus::success;
    }

    CapabilityList list{};
    uint8_t position = 0;
    config.read_u8(pointer, position);

    for (;;) {
        if (position < pci_capability_minimum) {
            break;
        }

        position = static_cast<uint8_t>(position & ~3u);
        uint16_t entry = 0;

        if (!config.read_u16(position, entry)) {
            return CapStatus::truncated_input;
        }

        const uint8_t id = static_cast<uint8_t>(entry & 0xff);

        if (id == pci_capability_end_id) {
            break;
        }

        if (list.count == pci_capability_capacity) {
            // PCI_FIND_CAP_TTL exhausted: a cycle or an unterminated list.
            return CapStatus::malformed;
        }

        list.entries[list.count] = {id, position};
        ++list.count;
        position = static_cast<uint8_t>(entry >> 8);
    }

    output = list;
    return CapStatus::success;
}

CapStatus find_capability(ByteView config, uint8_t id, uint8_t& offset) noexcept {
    CapabilityList list{};
    const CapStatus status = walk_capabilities(config, list);

    if (status != CapStatus::success) {
        return status;
    }

    for (uint32_t index = 0; index < list.count; ++index) {
        if (list.entries[index].id == id) {
            offset = list.entries[index].offset;
            return CapStatus::success;
        }
    }

    return CapStatus::absent;
}

bool extended_config_reachable(const ExtendedConfigProbe& probe) noexcept {
    if (probe.block_heads[0] == UINT32_MAX) {
        return false;
    }

    for (const uint32_t head : probe.block_heads) {
        if (head != probe.identity) {
            return true;
        }
    }

    return false;
}

CapStatus walk_ext_capabilities(ByteView config, ExtCapabilityList& output) noexcept {
    output = {};

    if (config.size() < pci_extended_config_bytes) {
        return CapStatus::extended_unavailable;
    }

    uint32_t header = 0;
    uint16_t position = pci_ext_first;
    config.read_u32(position, header);

    if (header == 0) {
        return CapStatus::success;
    }

    ExtCapabilityList list{};

    for (uint32_t ttl = pci_ext_ttl; ttl > 0; --ttl) {
        if (list.count == pci_ext_capability_capacity) {
            return CapStatus::capacity_exceeded;
        }

        list.entries[list.count] = {static_cast<uint16_t>(header & 0xffff),
                                    static_cast<uint8_t>((header >> 16) & 0xf), position};
        ++list.count;
        position = static_cast<uint16_t>((header >> 20) & 0xffc);

        if (position < pci_ext_first) {
            output = list;
            return CapStatus::success;
        }

        // A revisited header is a cycle, not merely a long list.
        for (uint32_t index = 0; index < list.count; ++index) {
            if (list.entries[index].offset == position) {
                return CapStatus::malformed;
            }
        }

        config.read_u32(position, header);
    }

    return CapStatus::malformed;
}

CapStatus find_ext_capability(ByteView config, uint16_t id, uint16_t& offset) noexcept {
    if (config.size() < pci_extended_config_bytes) {
        return CapStatus::extended_unavailable;
    }

    uint32_t header = 0;
    uint16_t position = pci_ext_first;
    config.read_u32(position, header);

    if (header == 0) {
        return CapStatus::absent;
    }

    for (uint32_t ttl = pci_ext_ttl; ttl > 0; --ttl) {
        if ((header & 0xffff) == id) {
            offset = position;
            return CapStatus::success;
        }

        position = static_cast<uint16_t>((header >> 20) & 0xffc);

        if (position < pci_ext_first) {
            return CapStatus::absent;
        }

        config.read_u32(position, header);
    }

    return CapStatus::malformed;
}

CapStatus decode_pcie(ByteView image, PcieCapability& output) noexcept {
    output = {};
    const ByteView config = legacy_view(image);
    uint8_t offset = 0;
    const CapStatus found = find_capability(config, pci_cap_id_express, offset);

    if (found != CapStatus::success) {
        return found;
    }

    PcieCapability express{};
    express.present = true;
    express.offset = offset;

    if (!config.read_u16(offset + pcie_flags_register, express.flags)) {
        return CapStatus::malformed;
    }

    express.version = static_cast<uint8_t>(express.flags & pcie_flags_version);
    express.port_type = static_cast<uint8_t>((express.flags >> 4) & 0xf);
    express.link_implemented = has_link_registers(express.port_type);
    express.slot_implemented = is_downstream_port(express.port_type) &&
                               (express.flags & pcie_flags_slot) != 0;
    express.root_implemented = express.port_type == pcie_type_root_port ||
                               express.port_type == pcie_type_rc_event_collector;
    express.version2 = express.version > 1;
    express.link2_implemented = express.link_implemented && express.version2;

    // Highest register byte the implemented set needs (PCIe r6 7.5.3 layout).
    size_t extent = 0x0c;
    extent = express.link_implemented ? 0x14 : extent;
    extent = express.slot_implemented ? 0x1c : extent;
    extent = express.root_implemented ? 0x24 : extent;
    extent = express.version2 ? 0x2c : extent;
    extent = express.link2_implemented ? 0x34 : extent;

    if (!contains_extent(config, offset, extent)) {
        return CapStatus::malformed;
    }

    config.read_u32(offset + 0x04, express.device_capabilities);
    config.read_u16(offset + pcie_devctl_register, express.device_control);
    config.read_u16(offset + 0x0a, express.device_status);

    if (express.link_implemented) {
        config.read_u32(offset + 0x0c, express.link_capabilities);
        config.read_u16(offset + pcie_lnkctl_register, express.link_control);
        config.read_u16(offset + 0x12, express.link_status);
    }

    if (express.slot_implemented) {
        config.read_u32(offset + 0x14, express.slot_capabilities);
        config.read_u16(offset + pcie_sltctl_register, express.slot_control);
        config.read_u16(offset + 0x1a, express.slot_status);
    }

    if (express.root_implemented) {
        config.read_u16(offset + pcie_rtctl_register, express.root_control);
        config.read_u16(offset + 0x1e, express.root_capabilities);
        config.read_u32(offset + 0x20, express.root_status);
    }

    if (express.version2) {
        config.read_u32(offset + 0x24, express.device_capabilities2);
        config.read_u16(offset + pcie_devctl2_register, express.device_control2);
    }

    if (express.link2_implemented) {
        config.read_u32(offset + 0x2c, express.link_capabilities2);
        config.read_u16(offset + pcie_lnkctl2_register, express.link_control2);
        config.read_u16(offset + 0x32, express.link_status2);
    }

    output = express;
    return CapStatus::success;
}

PcieSpeed pcie_speed_from_code(uint32_t code) noexcept {
    switch (code & 0xf) {
    case 1: return PcieSpeed::gt2_5;
    case 2: return PcieSpeed::gt5_0;
    case 3: return PcieSpeed::gt8_0;
    case 4: return PcieSpeed::gt16_0;
    case 5: return PcieSpeed::gt32_0;
    case 6: return PcieSpeed::gt64_0;
    default: return PcieSpeed::unknown;
    }
}

PcieSpeed pcie_speed_cap(const PcieCapability& express) noexcept {
    const uint32_t vector = express.link_capabilities2;

    if (vector != 0) {
        if ((vector & pcie_lnkcap2_64_0) != 0) return PcieSpeed::gt64_0;
        if ((vector & pcie_lnkcap2_32_0) != 0) return PcieSpeed::gt32_0;
        if ((vector & pcie_lnkcap2_16_0) != 0) return PcieSpeed::gt16_0;
        if ((vector & pcie_lnkcap2_8_0) != 0) return PcieSpeed::gt8_0;
        if ((vector & pcie_lnkcap2_5_0) != 0) return PcieSpeed::gt5_0;
        if ((vector & pcie_lnkcap2_2_5) != 0) return PcieSpeed::gt2_5;
        return PcieSpeed::unknown;
    }

    const uint32_t supported = express.link_capabilities & pcie_lnkcap_sls;

    if (supported == 2) {
        return PcieSpeed::gt5_0;
    }

    return supported == 1 ? PcieSpeed::gt2_5 : PcieSpeed::unknown;
}

uint8_t pcie_width_cap(const PcieCapability& express) noexcept {
    if (express.link_capabilities == 0) {
        return pcie_width_unknown;
    }

    return static_cast<uint8_t>((express.link_capabilities >> pcie_width_shift) & pcie_width_mask);
}

PcieSpeed pcie_link_speed(const PcieCapability& express) noexcept {
    return pcie_speed_from_code(express.link_status);
}

uint8_t pcie_link_width(const PcieCapability& express) noexcept {
    return static_cast<uint8_t>((express.link_status >> pcie_width_shift) & pcie_width_mask);
}

uint32_t pcie_bandwidth_mbps(PcieSpeed speed, uint8_t width) noexcept {
    return static_cast<uint32_t>(width) * speed_to_encoded_mbps(speed);
}

CapStatus decode_power_management(ByteView image, PowerManagementCapability& output) noexcept {
    output = {};
    const ByteView config = legacy_view(image);
    uint8_t offset = 0;
    const CapStatus found = find_capability(config, pci_cap_id_power_management, offset);

    if (found != CapStatus::success) {
        return found;
    }

    PowerManagementCapability power{};
    power.present = true;
    power.offset = offset;

    if (!config.read_u16(offset + 2u, power.capabilities) ||
        !config.read_u16(offset + 4u, power.control_status)) {
        return CapStatus::malformed;
    }

    power.power_state = static_cast<uint8_t>(power.control_status & 3u);
    output = power;
    return CapStatus::success;
}

CapStatus decode_msi(ByteView image, MsiCapability& output) noexcept {
    output = {};
    const ByteView config = legacy_view(image);
    uint8_t offset = 0;
    const CapStatus found = find_capability(config, pci_cap_id_msi, offset);

    if (found != CapStatus::success) {
        return found;
    }

    MsiCapability msi{};
    msi.present = true;
    msi.offset = offset;

    if (!config.read_u16(offset + 2u, msi.control)) {
        return CapStatus::malformed;
    }

    const uint8_t capable = static_cast<uint8_t>((msi.control >> 1) & 7u);
    const uint8_t enabled = static_cast<uint8_t>((msi.control >> 4) & 7u);

    if (capable > msi_reserved_vectors || enabled > msi_reserved_vectors) {
        return CapStatus::malformed;
    }

    msi.enabled = (msi.control & msi_enable) != 0;
    msi.address64 = (msi.control & msi_64bit) != 0;
    msi.per_vector_mask = (msi.control & msi_mask_capable) != 0;
    msi.vectors_capable = static_cast<uint8_t>(1u << capable);
    msi.vectors_enabled = static_cast<uint8_t>(1u << enabled);
    const size_t data = offset + (msi.address64 ? 0x0cu : 0x08u);
    const size_t mask = data + 4u;
    const size_t extent = msi.per_vector_mask ? mask + 8u - offset : data + 2u - offset;

    if (!contains_extent(config, offset, extent)) {
        return CapStatus::malformed;
    }

    config.read_u32(offset + 4u, msi.address_low);

    if (msi.address64) {
        config.read_u32(offset + 8u, msi.address_high);
    }

    config.read_u16(data, msi.data);
    msi.data_offset = static_cast<uint8_t>(data);

    if (msi.per_vector_mask) {
        config.read_u32(mask, msi.mask);
        config.read_u32(mask + 4u, msi.pending);
        msi.mask_offset = static_cast<uint8_t>(mask);
    }

    output = msi;
    return CapStatus::success;
}

CapStatus decode_msix(ByteView image, MsixCapability& output) noexcept {
    output = {};
    const ByteView config = legacy_view(image);
    uint8_t offset = 0;
    const CapStatus found = find_capability(config, pci_cap_id_msix, offset);

    if (found != CapStatus::success) {
        return found;
    }

    MsixCapability msix{};
    uint32_t table = 0;
    uint32_t pba = 0;

    if (!contains_extent(config, offset, 12) || !config.read_u16(offset + 2u, msix.control) ||
        !config.read_u32(offset + 4u, table) || !config.read_u32(offset + 8u, pba)) {
        return CapStatus::malformed;
    }

    msix.present = true;
    msix.offset = offset;
    msix.enabled = (msix.control & msix_enable) != 0;
    msix.function_masked = (msix.control & msix_mask_all) != 0;
    msix.table_entries = static_cast<uint16_t>((msix.control & msix_table_size) + 1u);
    msix.table_bar = static_cast<uint8_t>(table & msix_bir);
    msix.table_offset = table & ~msix_bir;
    msix.pba_bar = static_cast<uint8_t>(pba & msix_bir);
    msix.pba_offset = pba & ~msix_bir;

    if (msix.table_bar > pci_last_bar || msix.pba_bar > pci_last_bar) {
        return CapStatus::malformed;
    }

    output = msix;
    return CapStatus::success;
}

CapStatus decode_ltr(ByteView config, LtrCapability& output) noexcept {
    output = {};
    uint16_t offset = 0;
    const CapStatus found = find_ext_capability(config, pci_ext_cap_id_ltr, offset);

    if (found != CapStatus::success) {
        return found;
    }

    LtrCapability ltr{true, offset, 0};

    if (!config.read_u32(offset + 4u, ltr.max_latencies)) {
        return CapStatus::malformed;
    }

    output = ltr;
    return CapStatus::success;
}

CapStatus decode_resizable_bar(ByteView config, ResizableBarCapability& output) noexcept {
    output = {};
    uint16_t offset = 0;
    const CapStatus found = find_ext_capability(config, pci_ext_cap_id_rebar, offset);

    if (found != CapStatus::success) {
        return found;
    }

    uint32_t first_control = 0;

    if (!config.read_u32(offset + 8u, first_control)) {
        return CapStatus::malformed;
    }

    const uint32_t count = (first_control >> rebar_ctrl_nbar_shift) & 7u;

    if (count == 0 || count > pci_rebar_capacity) {
        return CapStatus::malformed;
    }

    uint16_t vendor = 0;
    uint16_t device = 0;
    config.read_u16(pci_vendor_offset, vendor);
    config.read_u16(pci_device_offset, device);
    ResizableBarCapability rebar{};
    rebar.present = true;
    rebar.offset = offset;
    uint32_t seen = 0;

    for (uint32_t index = 0; index < count; ++index) {
        const size_t entry = offset + index * 8u;
        uint32_t capability = 0;
        uint32_t control = 0;

        if (!config.read_u32(entry + 4u, capability) || !config.read_u32(entry + 8u, control)) {
            return CapStatus::malformed;
        }

        const uint8_t bar = static_cast<uint8_t>(control & rebar_ctrl_index);

        if (bar > pci_last_bar || (seen & (1u << bar)) != 0) {
            return CapStatus::malformed;
        }

        seen |= 1u << bar;
        uint32_t sizes = (capability & rebar_cap_sizes) >> 4;

        if (vendor == amd_vendor_id && device == amd_rx5600_pulse_device && bar == 0 &&
            sizes == amd_rx5600_bad_sizes) {
            sizes = amd_rx5600_sizes;
        }

        rebar.bars[index] = {bar, sizes,
                             static_cast<uint8_t>((control >> rebar_ctrl_size_shift) & rebar_ctrl_size_mask),
                             static_cast<uint16_t>(entry + 8u), control};
    }

    rebar.count = static_cast<uint8_t>(count);
    output = rebar;
    return CapStatus::success;
}

CapStatus find_resizable_bar(const ResizableBarCapability& rebar, uint8_t bar_index,
                             ResizableBar& output) noexcept {
    if (!rebar.present) {
        return CapStatus::absent;
    }

    for (uint32_t index = 0; index < rebar.count && index < pci_rebar_capacity; ++index) {
        if (rebar.bars[index].bar_index == bar_index) {
            output = rebar.bars[index];
            return CapStatus::success;
        }
    }

    return CapStatus::absent;
}

uint64_t rebar_size_to_bytes(uint8_t size) noexcept {
    return size > rebar_largest_shift ? 0 : uint64_t{1} << (size + 20u);
}

uint8_t rebar_bytes_to_size(uint64_t bytes) noexcept {
    constexpr uint64_t largest = uint64_t{1} << 63;

    if (bytes == 0 || bytes > largest) {
        return rebar_invalid_size;
    }

    uint8_t log2 = 0;

    while ((uint64_t{1} << log2) < bytes) {
        ++log2;
    }

    return static_cast<uint8_t>(log2 < 20 ? 0 : log2 - 20);
}

CapStatus plan_fb_bar_resize(const FbBarInputs& inputs, FbBarPlan& output) noexcept {
    output = {FbBarAction::keep_32bit_physical, 0};
    const uint8_t wanted = rebar_bytes_to_size(inputs.real_vram_bytes);

    if (wanted == rebar_invalid_size) {
        return CapStatus::malformed;
    }

    FbBarPlan plan{FbBarAction::resize, 0};

    if (!inputs.physical_address_64bit) {
        plan.action = FbBarAction::keep_32bit_physical;
    } else if (inputs.sriov_vf) {
        plan.action = FbBarAction::keep_sriov_vf;
    } else if (inputs.bar0_bytes >= inputs.real_vram_bytes) {
        plan.action = FbBarAction::keep_large_enough;
    } else if (!inputs.root_window_above_4g) {
        plan.action = FbBarAction::keep_no_root_window;
    } else if (inputs.firmware_config_preserved) {
        plan.action = FbBarAction::keep_config_preserved;
    } else if (inputs.bar0_supported_sizes == 0) {
        plan.action = FbBarAction::keep_not_resizable;
    } else {
        // min(fls(possible) - 1, rbar_size)
        const uint8_t largest = highest_bit(inputs.bar0_supported_sizes);
        plan.target_size = largest < wanted ? largest : wanted;

        if (plan.target_size >= 32 || (inputs.bar0_supported_sizes & (1u << plan.target_size)) == 0) {
            plan.action = FbBarAction::keep_size_unsupported;
        }
    }

    output = plan;
    return CapStatus::success;
}

CapStatus derive_pcie_info(ByteView device, const UpstreamChain& upstream,
                           PcieInfoPolicy policy, PcieInfoReport& output) noexcept {
    output = {};

    if (upstream.count > pci_upstream_capacity) {
        return CapStatus::capacity_exceeded;
    }

    CapStatus status = validate_function(device, pci_header_type_normal);

    if (status != CapStatus::success) {
        return status;
    }

    for (uint32_t index = 0; index < upstream.count; ++index) {
        const ByteView bridge(upstream.bridges[index].bytes, pci_config_bytes);
        status = validate_function(bridge, pci_header_type_bridge);

        if (status != CapStatus::success) {
            return status;
        }
    }

    PcieInfoReport report{};
    report.masks = {policy.gen_cap_override, policy.lane_cap_override};
    report.platform_speed = PcieSpeed::unknown;
    report.platform_width = pcie_width_unknown;
    report.partner_index = -1;
    PcieCapability own{};
    status = decode_pcie_or_zero(device, own);

    if (status != CapStatus::success) {
        return status;
    }

    report.device_speed_cap = pcie_speed_cap(own);

    // pci_is_root_bus(adev->pdev->bus) && !amdgpu_passthrough(adev)
    if (upstream.count == 0 && !policy.passthrough) {
        report.root_bus = true;
        report.masks.gen_mask = report.masks.gen_mask != 0 ? report.masks.gen_mask :
                                                             amdgpu_default_pcie_gen_mask;
        report.masks.mlw_mask = report.masks.mlw_mask != 0 ? report.masks.mlw_mask :
                                                             amdgpu_default_pcie_mlw_mask;
        output = report;
        return CapStatus::success;
    }

    if (report.masks.gen_mask != 0 && report.masks.mlw_mask != 0) {
        output = report;
        return CapStatus::success;
    }

    if (policy.dynamic_switching) {
        // First upstream bridge that is not an ATI/AMD-graphics (0x1002) switch port.
        for (uint32_t index = 0; index < upstream.count; ++index) {
            const ByteView bridge(upstream.bridges[index].bytes, pci_config_bytes);
            uint16_t vendor = 0;
            bridge.read_u16(pci_vendor_offset, vendor);

            if (vendor == amd_vendor_id) {
                continue;
            }

            PcieCapability express{};
            status = decode_pcie_or_zero(bridge, express);

            if (status != CapStatus::success) {
                return status;
            }

            report.platform_speed = pcie_speed_cap(express);
            report.platform_width = pcie_width_cap(express);
            report.partner_index = static_cast<int32_t>(index);
            break;
        }
    } else {
        // pcie_bandwidth_available(adev->pdev, NULL, &speed, &width)
        uint32_t bandwidth = 0;

        for (uint32_t hop = 0; hop <= upstream.count; ++hop) {
            PcieCapability express{};

            if (hop == 0) {
                express = own;
            } else {
                const ByteView bridge(upstream.bridges[hop - 1].bytes, pci_config_bytes);
                status = decode_pcie_or_zero(bridge, express);

                if (status != CapStatus::success) {
                    return status;
                }
            }

            const PcieSpeed speed = pcie_link_speed(express);
            const uint8_t width = pcie_link_width(express);
            const uint32_t next = pcie_bandwidth_mbps(speed, width);

            if (bandwidth == 0 || next <= bandwidth) {
                bandwidth = next;
                report.platform_speed = speed;
                report.platform_width = width;
                report.partner_index = static_cast<int32_t>(hop) - 1;
            }
        }
    }

    if (report.masks.gen_mask == 0) {
        report.masks.gen_mask = asic_gen_bits(report.device_speed_cap) |
                                platform_gen_bits(report.platform_speed);
    }

    if (report.masks.mlw_mask == 0) {
        report.masks.mlw_mask = platform_width_bits(report.platform_width);
    }

    output = report;
    return CapStatus::success;
}

PcieSmuLink encode_smu_pcie(PcieMasks masks) noexcept {
    PcieSmuLink link{0, 0};
    const uint32_t gen = masks.gen_mask;
    const uint32_t width = masks.mlw_mask;

    if ((gen & cail_pcie_link_speed_gen4) != 0) {
        link.gen = 3;
    } else if ((gen & cail_pcie_link_speed_gen3) != 0) {
        link.gen = 2;
    } else if ((gen & cail_pcie_link_speed_gen2) != 0) {
        link.gen = 1;
    }

    if ((width & cail_pcie_link_width_x16) != 0) {
        link.lane = 6;
    } else if ((width & cail_pcie_link_width_x12) != 0) {
        link.lane = 5;
    } else if ((width & cail_pcie_link_width_x8) != 0) {
        link.lane = 4;
    } else if ((width & cail_pcie_link_width_x4) != 0) {
        link.lane = 3;
    } else if ((width & cail_pcie_link_width_x2) != 0) {
        link.lane = 2;
    } else if ((width & cail_pcie_link_width_x1) != 0) {
        link.lane = 1;
    }

    return link;
}

CapStatus pcie_smu_link(ByteView device, const UpstreamChain& upstream, PcieInfoPolicy policy,
                        PcieSmuLink& output) noexcept {
    output = {0, 0};
    PcieInfoReport report{};
    const CapStatus status = derive_pcie_info(device, upstream, policy, report);

    if (status == CapStatus::success) {
        output = encode_smu_pcie(report.masks);
    }

    return status;
}

CapStatus resolve_upstream_path(ConfigDwordReader reader, PciLocation device,
                                UpstreamPath& output) noexcept {
    output = {};

    if (reader.read == nullptr || device.slot >= pci_device_count ||
        device.function >= pci_function_count) {
        return CapStatus::malformed;
    }

    UpstreamPath top_down{};
    uint8_t bus = 0;

    while (bus != device.bus) {
        if (top_down.count == pci_upstream_capacity) {
            return CapStatus::capacity_exceeded;
        }

        bool found = false;
        PciLocation forwarding{};
        uint8_t forwarding_secondary = 0;

        for (uint8_t slot = 0; slot < pci_device_count; ++slot) {
            uint8_t functions = 1;

            for (uint8_t function = 0; function < functions; ++function) {
                const PciLocation location{0, bus, slot, function};
                uint32_t identity = 0;

                if (!reader.read(reader.context, location, pci_vendor_offset, identity) ||
                    (identity & 0xffff) == pci_invalid_vendor || (identity & 0xffff) == 0) {
                    if (function == 0) {
                        break;
                    }

                    continue;
                }

                uint32_t header = 0;

                if (!reader.read(reader.context, location, pci_header_dword_offset, header)) {
                    return CapStatus::malformed;
                }

                const uint8_t type = static_cast<uint8_t>(header >> 16);

                if (function == 0 && (type & pci_multifunction_bit) != 0) {
                    functions = pci_function_count;
                }

                if ((type & pci_header_type_mask) != pci_header_type_bridge) {
                    continue;
                }

                uint32_t buses = 0;

                if (!reader.read(reader.context, location, pci_bridge_busses_offset, buses)) {
                    return CapStatus::malformed;
                }

                const uint8_t secondary = static_cast<uint8_t>(buses >> 8);
                const uint8_t subordinate = static_cast<uint8_t>(buses >> 16);

                if (secondary <= device.bus && device.bus <= subordinate) {
                    if (found) {
                        return CapStatus::malformed;
                    }

                    found = true;
                    forwarding = location;
                    forwarding_secondary = secondary;
                }
            }
        }

        if (!found) {
            return top_down.count == 0 ? CapStatus::absent : CapStatus::malformed;
        }

        if (forwarding_secondary <= bus) {
            return CapStatus::malformed;
        }

        top_down.bridges[top_down.count] = forwarding;
        ++top_down.count;
        bus = forwarding_secondary;
    }

    UpstreamPath path{};

    for (uint32_t index = 0; index < top_down.count; ++index) {
        path.bridges[index] = top_down.bridges[top_down.count - 1 - index];
    }

    path.count = top_down.count;
    output = path;
    return CapStatus::success;
}

CapStatus save_pci_state(ByteView config, PciSavedState& output) noexcept {
    output = {};

    if (config.size() < pci_config_bytes) {
        return CapStatus::truncated_input;
    }

    PciSavedState state{};

    if (!header_type(config, state.header_type)) {
        return CapStatus::truncated_input;
    }

    if (state.header_type != pci_header_type_normal &&
        state.header_type != pci_header_type_bridge) {
        return CapStatus::malformed;
    }

    for (uint32_t index = 0; index < 16; ++index) {
        config.read_u32(index * sizeof(uint32_t), state.header[index]);
    }

    CapStatus status = decode_pcie_or_zero(config, state.express);

    if (status == CapStatus::success) {
        status = decode_power_management(config, state.power);
        status = status == CapStatus::absent ? CapStatus::success : status;
    }

    if (status == CapStatus::success) {
        status = decode_msi(config, state.msi);
        status = status == CapStatus::absent ? CapStatus::success : status;
    }

    if (status == CapStatus::success) {
        status = decode_msix(config, state.msix);
        status = status == CapStatus::absent ? CapStatus::success : status;
    }

    state.extended = config.size() >= pci_extended_config_bytes;

    if (status == CapStatus::success && state.extended && state.express.present) {
        status = decode_ltr(config, state.ltr);
        status = status == CapStatus::absent ? CapStatus::success : status;
    }

    if (status == CapStatus::success && state.extended) {
        status = decode_resizable_bar(config, state.rebar);
        status = status == CapStatus::absent ? CapStatus::success : status;
    }

    if (status != CapStatus::success) {
        return status;
    }

    state.valid = true;
    output = state;
    return CapStatus::success;
}

CapStatus plan_pci_restore(const PciSavedState& state, RestorePlan& output) noexcept {
    output = {};

    if (!state.valid) {
        return CapStatus::malformed;
    }

    RestorePlan plan{};
    PlanWriter writer(plan);
    const PcieCapability& express = state.express;

    // pci_restore_pcie_state: LTR first, then the unconditional control writes.
    if (state.extended && state.ltr.present && express.present) {
        writer.add(state.ltr.offset + 4u, 4, state.ltr.max_latencies, true);
    }

    if (express.present) {
        writer.add(express.offset + pcie_devctl_register, 2, express.device_control, true);

        if (express.link_implemented) {
            writer.add(express.offset + pcie_lnkctl_register, 2, express.link_control, true);
        }

        if (express.slot_implemented) {
            writer.add(express.offset + pcie_sltctl_register, 2, express.slot_control, true);
        }

        if (express.root_implemented) {
            writer.add(express.offset + pcie_rtctl_register, 2, express.root_control, true);
        }

        if (express.version2) {
            writer.add(express.offset + pcie_devctl2_register, 2, express.device_control2, true);
        }

        if (express.link2_implemented) {
            writer.add(express.offset + pcie_lnkctl2_register, 2, express.link_control2, true);
        }
    }

    // pci_restore_rebar_state
    if (state.extended && state.rebar.present) {
        for (uint32_t index = 0; index < state.rebar.count && index < pci_rebar_capacity; ++index) {
            const ResizableBar& bar = state.rebar.bars[index];
            writer.add(bar.control_offset, 4, bar.control, true);
        }
    }

    // pci_restore_config_space
    if (state.header_type == pci_header_type_normal) {
        add_header_range(writer, state, 10, 15, 0, false);
        add_header_range(writer, state, 4, 9, bar_restore_retries, false);
        add_header_range(writer, state, 0, 3, 0, false);
    } else {
        add_header_range(writer, state, 12, 15, 0, false);
        add_header_range(writer, state, 9, 11, 0, true);
        add_header_range(writer, state, 0, 8, 0, false);
    }

    // __pci_restore_msi_state with the saved message instead of msi_desc.
    const MsiCapability& msi = state.msi;

    if (msi.present && msi.enabled) {
        writer.add(msi.offset + 2u, 2, static_cast<uint16_t>(msi.control & ~msi_enable), true);
        writer.add(msi.offset + 4u, 4, msi.address_low, true);

        if (msi.address64) {
            writer.add(msi.offset + 8u, 4, msi.address_high, true);
        }

        writer.add(msi.data_offset, 2, msi.data, true);

        if (msi.per_vector_mask) {
            writer.add(msi.mask_offset, 4, msi.mask, true);
        }

        writer.add(msi.offset + 2u, 2, msi.control, true);
    }

    // __pci_restore_msix_state (vector-control words live in BAR memory).
    const MsixCapability& msix = state.msix;

    if (msix.present && msix.enabled) {
        writer.add(msix.offset + 2u, 2, static_cast<uint16_t>(msix.control | msix_enable | msix_mask_all),
                   true);
        writer.add(msix.offset + 2u, 2, msix.control, true);
    }

    if (writer.overflowed()) {
        return CapStatus::capacity_exceeded;
    }

    output = plan;
    return CapStatus::success;
}

} // namespace rx7900

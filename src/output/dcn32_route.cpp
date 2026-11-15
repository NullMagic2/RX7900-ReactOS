/*
Module: rx7900.output.dcn32_route
Purpose: Checked ATOM-to-DCN3.2 route decisions and DDC pad mode ownership.
Created: 2026-10-24
Architecture: Exact v6.12 dcn_3_2_0_offset.h/_sh_mask.h DC_GPIO values; route
rules from link_factory.c, bios_parser2.c and hw_translate_dcn32.c; pad sequence
from hw_gpio.c and hw_ddc.c with the ddc_regs.h DCN2 register/field lists.
Copyright 2012-2023 Advanced Micro Devices, Inc.
Linux notice and provenance: licenses/AMDGPU-OUTPUT.txt.
*/
#include "dcn32_route.hpp"

namespace rx7900::output {
namespace {

// ObjectID.h encoder and connector object ids (graphics object byte 7:0).
constexpr uint8_t encoder_uniphy = 0x1e;
constexpr uint8_t encoder_uniphy1 = 0x20;
constexpr uint8_t encoder_uniphy2 = 0x21;
constexpr uint8_t connector_dvi_single = 0x03;
constexpr uint8_t connector_dvi_dual = 0x04;
constexpr uint8_t connector_hdmi_a = 0x0c;
constexpr uint8_t connector_display_port = 0x13;
constexpr uint8_t connector_usb_c = 0x17;

// dcn_3_2_0_offset.h, BASE_IDX 2. Per-line registers advance by four dwords.
constexpr uint32_t ddc1_mask = 0x28d0u;
constexpr uint32_t ddc1_a = 0x28d1u;
constexpr uint32_t ddc1_en = 0x28d2u;
constexpr uint32_t ddc_line_stride = 4u;
constexpr uint32_t hpd_a = 0x28f5u;
constexpr uint32_t aux_ctrl_5 = 0x291du;
constexpr uint32_t phy_aux_cntl = 0x28ffu;

// dcn_3_2_0_sh_mask.h DC_GPIO_HPD_A__DC_GPIO_HPDn_A_MASK for HPD1..5 (n-1).
constexpr uint32_t hpd_masks[dcn32_link_count]{
    0x00000001u, 0x00000100u, 0x00010000u, 0x01000000u, 0x04000000u};

// DC_GPIO_DDCn_MASK/_A/_EN fields, identical for lines 1..5.
constexpr uint32_t clock_field = 0x00000001u;
constexpr uint32_t data_field = 0x00000100u;
constexpr uint32_t clock_pull_down = 0x00000010u;
constexpr uint32_t data_pull_down = 0x00001000u;
constexpr uint32_t aux_pad_mode = 0x00010000u;
// DC_GPIO_AUX_CTRL_5__DDC_PADn_I2CMODE (shift 12 + line) and
// PHY_AUX_CNTL__AUXn_PAD_RXSEL (two bits at 10 + 2 * line).
constexpr uint32_t i2c_mode_shift = 12u;
constexpr uint32_t rxsel_shift = 10u;
constexpr uint32_t rxsel_field_mask = 3u;
constexpr uint32_t aux_discharge_us = 2000u;

uint8_t object_id(uint16_t object) noexcept { return static_cast<uint8_t>(object & 0xffu); }
uint8_t object_enum(uint16_t object) noexcept { return static_cast<uint8_t>((object >> 8) & 7u); }

bool space_valid(const display::Dcn32RegisterSpace& space) noexcept {
    return space.discovery_verified && space.dmu_major == 3 && space.dmu_minor == 2 &&
           space.dmu_revision == 0 && space.aperture_bytes != 0;
}

// link_factory.c translate_encoder_to_transmitter.
uint8_t transmitter_of(uint16_t encoder) noexcept {
    const uint8_t enumeration = object_enum(encoder);
    uint8_t first = dcn32_route_none;

    switch (object_id(encoder)) {
    case encoder_uniphy: first = 0; break;
    case encoder_uniphy1: first = 2; break;
    case encoder_uniphy2: first = 4; break;
    default: return dcn32_route_none;
    }

    if (enumeration != 1 && enumeration != 2) {
        return dcn32_route_none;
    }

    return static_cast<uint8_t>(first + enumeration - 1u);
}

RouteFault signal_of(const Connector& connector, LinkSignal& signal) noexcept {
    switch (object_id(connector.connector_object_id)) {
    case connector_display_port: signal = LinkSignal::display_port; return RouteFault::none;
    case connector_hdmi_a: signal = LinkSignal::hdmi; return RouteFault::none;
    case connector_dvi_single: signal = LinkSignal::dvi_single; return RouteFault::none;
    case connector_dvi_dual: signal = LinkSignal::dvi_dual; return RouteFault::none;
    case connector_usb_c: return RouteFault::usb_c_alt_mode_unverified;
    default: return RouteFault::unsupported_connector;
    }
}

// bios_parser2 truncates data_a_reg_index to 16 bits; a nonzero high half
// can never name a DCN register, so it is refused instead of truncated.
bool absolute_index(uint32_t segment, uint32_t reg, uint32_t& output) noexcept {
    const uint64_t value = static_cast<uint64_t>(segment) + reg;
    if (value > 0xffffu) {
        return false;
    }
    output = static_cast<uint32_t>(value);
    return true;
}

RouteFault ddc_line_of(const Connector& connector, uint32_t segment, uint8_t& line) noexcept {
    if (!connector.has_i2c_record) {
        return RouteFault::missing_ddc;
    }
    if (!connector.ddc_gpio.found) {
        return RouteFault::ddc_gpio_unresolved;
    }
    for (uint8_t candidate = 0; candidate < dcn32_link_count; ++candidate) {
        uint32_t expected = 0;
        if (absolute_index(segment, ddc1_a + candidate * ddc_line_stride, expected) &&
            connector.ddc_gpio.register_index == expected) {
            line = candidate;
            return RouteFault::none;
        }
    }
    // DDCVGA and every other register are not DCN 3.2 connector DDC lines.
    return RouteFault::ddc_register_mismatch;
}

RouteFault hpd_source_of(const Connector& connector, uint32_t segment, uint8_t& source) noexcept {
    if (!connector.has_hpd_record) {
        return RouteFault::missing_hpd;
    }
    if (!connector.hpd_gpio.found) {
        return RouteFault::hpd_gpio_unresolved;
    }
    uint32_t expected = 0;
    if (!absolute_index(segment, hpd_a, expected) ||
        connector.hpd_gpio.register_index != expected) {
        return RouteFault::hpd_register_mismatch;
    }
    const uint32_t mask = 1u << connector.hpd_gpio.bit_shift;
    for (uint8_t candidate = 0; candidate < dcn32_link_count; ++candidate) {
        if (hpd_masks[candidate] == mask) {
            source = candidate;
            return RouteFault::none;
        }
    }
    return RouteFault::hpd_mask_unknown;
}

Dcn32ConnectorRoute route_of(const Connector& connector, uint32_t segment) noexcept {
    Dcn32ConnectorRoute route{};
    route.firmware_path_index = connector.firmware_path_index;
    route.connector_object_id = connector.connector_object_id;
    route.encoder_object_id = connector.encoder_object_id;
    route.connector_id = object_id(connector.connector_object_id);
    route.hpd_plugged_level = connector.hpd_plugged_level;
    route.i2c_slave_address = connector.i2c_slave_address;

    RouteFault fault = signal_of(connector, route.signal);
    if (fault == RouteFault::none && connector.external_encoder_object_id != 0) {
        fault = RouteFault::external_encoder;
    }
    if (fault == RouteFault::none) {
        route.transmitter = transmitter_of(connector.encoder_object_id);
        if (route.transmitter == dcn32_route_none) {
            fault = RouteFault::unknown_encoder;
        } else if (route.transmitter >= dcn32_link_count) {
            fault = RouteFault::transmitter_range;
        } else {
            route.dig_engine = route.transmitter;
        }
    }
    if (fault == RouteFault::none) {
        fault = ddc_line_of(connector, segment, route.ddc_line);
        if (fault == RouteFault::none) {
            route.aux_engine = route.ddc_line;
        }
    }
    if (fault == RouteFault::none) {
        fault = hpd_source_of(connector, segment, route.hpd_source);
    }
    route.fault = fault;
    route.validated = fault == RouteFault::none;
    return route;
}

void quarantine_duplicates(Dcn32RouteTable& table) noexcept {
    for (uint32_t first = 0; first < table.count; ++first) {
        for (uint32_t second = first + 1; second < table.count; ++second) {
            Dcn32ConnectorRoute& a = table.routes[first];
            Dcn32ConnectorRoute& b = table.routes[second];
            RouteFault fault = RouteFault::none;
            if (a.transmitter != dcn32_route_none && a.transmitter == b.transmitter) {
                fault = RouteFault::duplicate_transmitter;
            } else if (a.ddc_line != dcn32_route_none && a.ddc_line == b.ddc_line) {
                fault = RouteFault::duplicate_ddc;
            } else if (a.hpd_source != dcn32_route_none && a.hpd_source == b.hpd_source) {
                fault = RouteFault::duplicate_hpd;
            }
            if (fault == RouteFault::none) {
                continue;
            }
            // Both claimants lose: neither can prove it owns the shared block.
            if (a.validated) {
                a.validated = false;
                a.fault = fault;
            }
            if (b.validated) {
                b.validated = false;
                b.fault = fault;
            }
        }
    }
}

struct PadRegisters final {
    uint32_t mask = 0;
    uint32_t a = 0;
    uint32_t en = 0;
};

PadRegisters pad_registers(uint8_t line) noexcept {
    const uint32_t step = static_cast<uint32_t>(line) * ddc_line_stride;
    return {ddc1_mask + step, ddc1_a + step, ddc1_en + step};
}

bool read_reg(const hardware::RegisterIo& io, const display::Dcn32RegisterSpace& space,
              uint32_t reg, uint32_t& value) noexcept {
    return hardware::read_ip_register(io, space.segment2_dword_base, reg, value) &&
           value != UINT32_MAX;
}

bool write_reg(const hardware::RegisterIo& io, const display::Dcn32RegisterSpace& space,
               uint32_t reg, uint32_t value) noexcept {
    return hardware::write_ip_register(io, space.segment2_dword_base, reg, value);
}

// REG_UPDATE: read, replace field, write. read_ok separates a failed read
// (nothing written) from a failed write (state unknown).
DdcPadResult update(const hardware::RegisterIo& io, const display::Dcn32RegisterSpace& space,
                    uint32_t reg, uint32_t field, uint32_t value) noexcept {
    uint32_t current = 0;
    if (!read_reg(io, space, reg, current)) {
        return DdcPadResult::read_failure;
    }
    const uint32_t next = (current & ~field) | (value & field);
    return write_reg(io, space, reg, next) ? DdcPadResult::success : DdcPadResult::state_unknown;
}

bool field_set(uint32_t value, uint32_t field) noexcept { return (value & field) != 0; }

DdcPadResult store_pin(const hardware::RegisterIo& io, const display::Dcn32RegisterSpace& space,
                       const PadRegisters& regs, uint32_t field, bool& mask, bool& a,
                       bool& en) noexcept {
    uint32_t mask_value = 0;
    uint32_t a_value = 0;
    uint32_t en_value = 0;
    if (!read_reg(io, space, regs.mask, mask_value) || !read_reg(io, space, regs.a, a_value) ||
        !read_reg(io, space, regs.en, en_value)) {
        return DdcPadResult::read_failure;
    }
    mask = field_set(mask_value, field);
    a = field_set(a_value, field);
    en = field_set(en_value, field);
    return DdcPadResult::success;
}

DdcPadResult restore_pin(const hardware::RegisterIo& io, const display::Dcn32RegisterSpace& space,
                         const PadRegisters& regs, uint32_t field, bool mask, bool a,
                         bool en) noexcept {
    DdcPadResult result = update(io, space, regs.mask, field, mask ? field : 0);
    if (result == DdcPadResult::success) {
        result = update(io, space, regs.a, field, a ? field : 0);
    }
    if (result == DdcPadResult::success) {
        result = update(io, space, regs.en, field, en ? field : 0);
    }
    return result;
}

// After the first write any failure leaves the pad in an unknown state.
DdcPadResult after_write(DdcPadResult result) noexcept {
    return result == DdcPadResult::success ? result : DdcPadResult::state_unknown;
}

DdcPadResult set_config(const hardware::RegisterIo& io, const display::Dcn32RegisterSpace& space,
                        const PadRegisters& regs, uint8_t line, DdcMode mode) noexcept {
    uint32_t cached = 0;
    if (!read_reg(io, space, regs.mask, cached)) {
        return DdcPadResult::state_unknown;
    }
    const uint32_t i2c_mode = 1u << (i2c_mode_shift + line);

    if (mode == DdcMode::aux) {
        if (!field_set(cached, aux_pad_mode) &&
            !write_reg(io, space, regs.mask, cached | aux_pad_mode)) {
            return DdcPadResult::state_unknown;
        }
        return after_write(update(io, space, aux_ctrl_5, i2c_mode, 0));
    }

    if (!field_set(cached, data_pull_down) || !field_set(cached, clock_pull_down)) {
        // REG_SET_2 writes the cached value with both pull-downs set.
        cached |= data_pull_down | clock_pull_down;
        if (!write_reg(io, space, regs.mask, cached)) {
            return DdcPadResult::state_unknown;
        }
    }
    if (field_set(cached, aux_pad_mode)) {
        // Conservative: always allow the AUX pad discharge before I2C use.
        io.delay_microseconds(io.context, aux_discharge_us);
        const DdcPadResult cleared = update(io, space, regs.mask, aux_pad_mode, 0);
        if (cleared != DdcPadResult::success) {
            return DdcPadResult::state_unknown;
        }
    }
    const DdcPadResult moded = update(io, space, aux_ctrl_5, i2c_mode, i2c_mode);
    if (moded != DdcPadResult::success) {
        return DdcPadResult::state_unknown;
    }
    const uint32_t rxsel = rxsel_field_mask << (rxsel_shift + 2u * line);
    return after_write(update(io, space, phy_aux_cntl, rxsel, 1u << (rxsel_shift + 2u * line)));
}

} // namespace

RouteStatus bind_dcn32_routes(const ConnectorCatalog& catalog,
                              const display::Dcn32RegisterSpace& space,
                              Dcn32RouteTable& output) noexcept {
    if (!space_valid(space)) {
        return RouteStatus::discovery_mismatch;
    }
    if (catalog.count > connector_capacity) {
        return RouteStatus::invalid_catalog;
    }

    // Built in place: every failure is decided above, and a route table on
    // the kernel stack would exceed the 512-byte frame budget.
    output.clear();
    for (uint32_t index = 0; index < catalog.count; ++index) {
        output.routes[index] = route_of(catalog.connectors[index], space.segment2_dword_base);
    }
    output.count = catalog.count;
    quarantine_duplicates(output);
    for (uint32_t index = 0; index < output.count; ++index) {
        output.validated_count += output.routes[index].validated ? 1u : 0u;
    }
    return RouteStatus::success;
}

DdcPadResult open_ddc_pad(const hardware::RegisterIo& io,
                          const display::Dcn32RegisterSpace& space, uint8_t line,
                          DdcMode mode, DdcPadSave& save) noexcept {
    if (!space_valid(space) || !hardware::register_io_valid(io) || line >= dcn32_link_count ||
        save.valid || (mode != DdcMode::aux && mode != DdcMode::i2c)) {
        return DdcPadResult::invalid_argument;
    }

    DdcPadSave candidate{};
    candidate.line = line;
    const PadRegisters regs = pad_registers(line);

    // Data pin: store, then GPIO_MODE_HARDWARE clears its MASK field.
    DdcPadResult result = store_pin(io, space, regs, data_field, candidate.data_mask,
                                    candidate.data_a, candidate.data_en);
    if (result != DdcPadResult::success) {
        return result;
    }
    result = update(io, space, regs.mask, data_field, 0);
    if (result != DdcPadResult::success) {
        return result;
    }
    // From here the saved state must survive any failure so close can restore.
    candidate.valid = true;
    save = candidate;

    result = store_pin(io, space, regs, clock_field, save.clock_mask, save.clock_a, save.clock_en);
    if (result != DdcPadResult::success) {
        // The clock pin was never touched: undo only the data pin.
        if (restore_pin(io, space, regs, data_field, save.data_mask, save.data_a,
                        save.data_en) != DdcPadResult::success) {
            return DdcPadResult::state_unknown;
        }
        save = {};
        return DdcPadResult::read_failure;
    }
    result = after_write(update(io, space, regs.mask, clock_field, 0));
    if (result == DdcPadResult::success) {
        result = set_config(io, space, regs, line, mode);
    }
    return result;
}

DdcPadResult close_ddc_pad(const hardware::RegisterIo& io,
                           const display::Dcn32RegisterSpace& space,
                           DdcPadSave& save) noexcept {
    if (!space_valid(space) || !hardware::register_io_valid(io) || !save.valid ||
        save.line >= dcn32_link_count) {
        return DdcPadResult::invalid_argument;
    }
    const PadRegisters regs = pad_registers(save.line);
    DdcPadResult result = restore_pin(io, space, regs, clock_field, save.clock_mask,
                                      save.clock_a, save.clock_en);
    if (result == DdcPadResult::success) {
        result = restore_pin(io, space, regs, data_field, save.data_mask, save.data_a,
                             save.data_en);
    }
    if (result != DdcPadResult::success) {
        return DdcPadResult::state_unknown;
    }
    save = {};
    return DdcPadResult::success;
}

bool ddc_pad_in_aux_mode(const hardware::RegisterIo& io,
                         const display::Dcn32RegisterSpace& space, uint8_t line) noexcept {
    if (!space_valid(space) || !hardware::register_io_valid(io) || line >= dcn32_link_count) {
        return false;
    }
    uint32_t mask = 0;
    uint32_t control = 0;
    return read_reg(io, space, pad_registers(line).mask, mask) &&
           read_reg(io, space, aux_ctrl_5, control) &&
           (mask & (data_field | clock_field | aux_pad_mode)) == aux_pad_mode &&
           (control & (1u << (i2c_mode_shift + line))) == 0;
}

} // namespace rx7900::output

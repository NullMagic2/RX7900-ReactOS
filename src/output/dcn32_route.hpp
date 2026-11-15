/*
Module: rx7900.output.dcn32_route
Purpose: Bind each ATOM connector to its Navi31 DIG/PHY, AUX/DDC line and HPD pin.
Created: 2026-10-24
Architecture: Pure decision over the checked ATOM catalog (atom_connectors) and
the discovery-validated DMU 3.2.0 segment2 base; the DDC pad helpers are the only
register users and run through the shared hardware::RegisterIo contract. This is
the route proof that NativeDcn32Hpd/NativeDcn32Aux require, consumed by
display_link. Rules follow Linux v6.12 link_factory.c (translate_encoder_to_
transmitter, get_ddc_line, get_hpd_line), bios_parser2.c (get_gpio_i2c_info,
get_gpio_pin_info), gpio/dcn32/hw_translate_dcn32.c (offset_to_id),
gpio/hw_gpio.c, gpio/hw_ddc.c, gpio/ddc_regs.h and dcn32_resource.c res_cap.
Linux AMD register provenance and notice: licenses/AMDGPU-OUTPUT.txt.
*/
#pragma once

#include "atom_connectors.hpp"
#include "dcn32_otg.hpp"
#include "register_io.hpp"

namespace rx7900::output {

// dcn32_resource.c res_cap_dcn32: five DIO link encoders (UNIPHY A..E), five
// DDC/AUX channels and five HPD sources. UNIPHY F/G do not exist on DCN 3.2.
constexpr uint8_t dcn32_link_count = 5;
constexpr uint8_t dcn32_route_none = 0xff;

enum class LinkSignal : uint8_t {
    none,
    display_port,
    hdmi,
    dvi_single,
    dvi_dual
};

// Why a connector route is not driven. Every value except none quarantines
// the route permanently for this catalog; nothing is guessed or repaired.
enum class RouteFault : uint8_t {
    none,
    unsupported_connector,
    // USB-C needs the DMUB DP-alt-mode query (dcn32_link_encoder_is_in_alt_mode)
    // and a TCPC owner; not proven here.
    usb_c_alt_mode_unverified,
    external_encoder,
    unknown_encoder,
    transmitter_range,
    missing_ddc,
    ddc_gpio_unresolved,
    // The ATOM GPIO register index is not DC_GPIO_DDCn_A at the discovered
    // segment2 base (offset_to_id would fail or select a different line).
    ddc_register_mismatch,
    missing_hpd,
    hpd_gpio_unresolved,
    hpd_register_mismatch,
    hpd_mask_unknown,
    duplicate_transmitter,
    duplicate_ddc,
    duplicate_hpd
};

struct Dcn32ConnectorRoute final {
    uint8_t firmware_path_index = 0;
    uint16_t connector_object_id = 0;
    uint16_t encoder_object_id = 0;
    LinkSignal signal = LinkSignal::none;
    // TRANSMITTER_UNIPHY_A..E == PHY id 0..4 == link_enc_regs[] index.
    uint8_t transmitter = dcn32_route_none;
    // dcn32_link_encoder_construct preferred_engine: ENGINE_ID_DIGA + transmitter.
    uint8_t dig_engine = dcn32_route_none;
    // GPIO_DDC_LINE_DDC1..5 as 0..4; CHANNEL_ID_DDCn - 1 selects the AUX engine.
    uint8_t ddc_line = dcn32_route_none;
    uint8_t aux_engine = dcn32_route_none;
    // GPIO_HPD_1..5 as HPD_SOURCEID 0..4 (DCN HPD and AUX_HPD_SEL instance).
    uint8_t hpd_source = dcn32_route_none;
    uint8_t hpd_plugged_level = 0;
    uint8_t i2c_slave_address = 0;
    // Connector object id byte (connobj_id of DIGxTransmitterControl).
    uint8_t connector_id = 0;
    bool validated = false;
    RouteFault fault = RouteFault::none;
};

struct Dcn32RouteTable final {
    Dcn32ConnectorRoute routes[connector_capacity]{};
    uint32_t count = 0;
    uint32_t validated_count = 0;

    // In-place reset (a temporary table would exceed a 512-byte kernel frame).
    void clear() noexcept {
        for (Dcn32ConnectorRoute& route : routes) {
            route = Dcn32ConnectorRoute{};
        }
        count = 0;
        validated_count = 0;
    }
};

enum class RouteStatus : uint8_t {
    success,
    // The register space is not a discovery-verified DMU 3.2.0 token.
    discovery_mismatch,
    invalid_catalog
};

// Any IRQL, pure. Every catalog connector produces one route; output is
// complete on success and unchanged when the arguments are refused. A route is validated only when
// signal, internal UNIPHY encoder, DDC line (absolute DC_GPIO_DDCn_A index) and
// HPD pin (DC_GPIO_HPD_A plus a known HPDn mask) all agree with the discovered
// segment2 base and no other connector claims the same PHY, DDC line or HPD.
// HDMI/DVI routes report their DDC line (I2C over the DC_I2C engine) and keep
// aux_engine for diagnostics only.
RouteStatus bind_dcn32_routes(const ConnectorCatalog& catalog,
                              const display::Dcn32RegisterSpace& space,
                              Dcn32RouteTable& output) noexcept;

// hw_gpio.c store_registers values of one DDC line (data and clock pins share
// DC_GPIO_DDCn_MASK/_A/_EN; only their fields differ). Restored on close.
struct DdcPadSave final {
    uint8_t line = dcn32_route_none;
    bool valid = false;
    bool data_mask = false;
    bool data_a = false;
    bool data_en = false;
    bool clock_mask = false;
    bool clock_a = false;
    bool clock_en = false;
};

enum class DdcMode : uint8_t { aux, i2c };

enum class DdcPadResult : uint8_t {
    success,
    invalid_argument,
    read_failure,
    // A write was attempted and not confirmed: the pad is in an unknown mode.
    state_unknown
};

// PASSIVE_LEVEL, exclusive register owner, line 0..4 of a validated route.
// dal_ddc_open(GPIO_MODE_HARDWARE, mode): opens the data then the clock pin
// (store MASK/A/EN, clear the GPIO MASK field so hardware owns the pin), then
// hw_ddc set_config on the data pin. AUX: AUX_PAD1_MODE=1 when clear (REG_SET of
// the cached value), DC_GPIO_AUX_CTRL_5 DDC_PADn_I2CMODE=0. I2C: pull-downs on
// (REG_SET_2) when either is off, AUX_PAD1_MODE=0 when set, DDC_PADn_I2CMODE=1,
// PHY_AUX_CNTL AUXn_PAD_RXSEL=1. The 2 ms AUX pad discharge before the I2C
// clear is always taken (Linux takes it only with EN bits declared). A failure
// after the first write returns state_unknown with save still valid, so the
// owner can retry close_ddc_pad or quarantine the route.
DdcPadResult open_ddc_pad(const hardware::RegisterIo& io,
                          const display::Dcn32RegisterSpace& space, uint8_t line,
                          DdcMode mode, DdcPadSave& save) noexcept;

// dal_ddc_close: restores the clock then the data pin MASK/A/EN fields.
DdcPadResult close_ddc_pad(const hardware::RegisterIo& io,
                           const display::Dcn32RegisterSpace& space,
                           DdcPadSave& save) noexcept;

// Read-only proof that a line is currently in hardware AUX mode: both GPIO
// MASK fields clear, AUX_PAD1_MODE set and DDC_PADn_I2CMODE clear. This is the
// exact_ddc_route_in_aux_mode fact NativeDcn32Aux requires.
bool ddc_pad_in_aux_mode(const hardware::RegisterIo& io,
                         const display::Dcn32RegisterSpace& space,
                         uint8_t line) noexcept;

} // namespace rx7900::output

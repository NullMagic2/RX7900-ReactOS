/*
Module: rx7900.tests.dcn32_route
Purpose: Check ATOM-to-DCN3.2 route binding, quarantine and DDC pad ownership.
Created: 2026-10-24
Architecture: Synthetic catalogs and the shared DCN register model only; no
physical GPU access. Expected values are the Linux v6.12 link_factory /
bios_parser2 / hw_translate_dcn32 / hw_gpio / hw_ddc rules.
*/
#include "dcn32_display_model.hpp"
#include "dcn32_route.hpp"

namespace {
using namespace rx7900::output;
using rx7900_test::Checks;
namespace dcn = rx7900_test::dcn;

constexpr uint32_t ddc1_a = 0x28d1u;
constexpr uint32_t hpd_a = 0x28f5u;

Connector make(uint16_t connector, uint16_t encoder, uint8_t ddc_line, uint8_t hpd_shift,
               uint8_t path) {
    Connector value{};
    value.connector_object_id = connector;
    value.encoder_object_id = encoder;
    value.firmware_path_index = path;
    value.has_i2c_record = true;
    value.i2c_id = static_cast<uint8_t>(0x90u + ddc_line);
    value.ddc_gpio = {dcn::segment2 + ddc1_a + 4u * ddc_line, 0, 0, true};
    value.has_hpd_record = true;
    value.hpd_pin_id = static_cast<uint8_t>(path + 1u);
    value.hpd_plugged_level = 1;
    value.hpd_gpio = {dcn::segment2 + hpd_a, hpd_shift, hpd_shift, true};
    return value;
}

ConnectorCatalog navi31_board() {
    ConnectorCatalog catalog{};
    // DP on UNIPHY A (0x211e), DP on UNIPHY B (0x221e), HDMI on UNIPHY1 enum 1
    // (C, 0x2120), DP on UNIPHY1 enum 2 (D, 0x2220).
    catalog.connectors[0] = make(0x3113, 0x211e, 0, 0, 0);
    catalog.connectors[1] = make(0x3213, 0x221e, 1, 8, 1);
    catalog.connectors[2] = make(0x310c, 0x2120, 2, 16, 2);
    catalog.connectors[3] = make(0x3313, 0x2220, 3, 24, 3);
    catalog.count = 4;
    return catalog;
}

void check_board(Checks& checks) {
    Dcn32RouteTable table{};
    const ConnectorCatalog catalog = navi31_board();
    checks.expect(bind_dcn32_routes(catalog, dcn::space(), table) == RouteStatus::success,
                  "board routes bind");
    checks.expect(table.count == 4 && table.validated_count == 4, "four validated routes");
    const uint8_t transmitters[4]{0, 1, 2, 3};
    for (uint32_t index = 0; index < 4; ++index) {
        const Dcn32ConnectorRoute& route = table.routes[index];
        checks.expect(route.validated && route.fault == RouteFault::none, "route validated");
        checks.expect(route.transmitter == transmitters[index] &&
                          route.dig_engine == transmitters[index],
                      "translate_encoder_to_transmitter and preferred DIG engine");
        checks.expect(route.ddc_line == index && route.aux_engine == index,
                      "DDC line selects AUX engine channel-1");
        checks.expect(route.hpd_source == index, "HPD mask selects HPD source");
        checks.expect(route.firmware_path_index == index, "firmware path index kept");
    }
    checks.expect(table.routes[0].signal == LinkSignal::display_port &&
                      table.routes[2].signal == LinkSignal::hdmi,
                  "connector object selects signal");
    checks.expect(table.routes[2].connector_id == 0x0c && table.routes[0].connector_id == 0x13,
                  "connector id byte for transmitter control");

    // UNIPHY2 enum 1 is UNIPHY E (transmitter 4) on HPD5 (mask 0x04000000).
    ConnectorCatalog e{};
    e.connectors[0] = make(0x3113, 0x2121, 4, 26, 0);
    e.count = 1;
    checks.expect(bind_dcn32_routes(e, dcn::space(), table) == RouteStatus::success &&
                      table.routes[0].validated && table.routes[0].transmitter == 4 &&
                      table.routes[0].hpd_source == 4,
                  "UNIPHY E on HPD5");
}

void check_faults(Checks& checks) {
    struct Case final {
        Connector connector;
        RouteFault fault;
        const char* contract;
    };
    Connector external = make(0x3113, 0x211e, 0, 0, 0);
    external.external_encoder_object_id = 0x2119;
    Connector unknown = make(0x3113, 0x2125, 0, 0, 0);       // UNIPHY3: G, not DCN 3.2
    Connector enum3 = make(0x3113, 0x231e, 0, 0, 0);         // UNIPHY enum 3 does not exist
    Connector no_ddc = make(0x3113, 0x211e, 0, 0, 0);
    no_ddc.has_i2c_record = false;
    Connector unresolved = make(0x3113, 0x211e, 0, 0, 0);
    unresolved.ddc_gpio.found = false;
    Connector wrong_ddc = make(0x3113, 0x211e, 0, 0, 0);
    wrong_ddc.ddc_gpio.register_index += 2;                  // DC_GPIO_DDC1_Y, not _A
    Connector vga = make(0x3113, 0x211e, 0, 0, 0);
    vga.ddc_gpio.register_index = dcn::segment2 + 0x28e9u;   // DDCVGA
    Connector high = make(0x3113, 0x211e, 0, 0, 0);
    high.ddc_gpio.register_index |= 0x10000u;                // Linux would truncate
    Connector no_hpd = make(0x3113, 0x211e, 0, 0, 0);
    no_hpd.has_hpd_record = false;
    Connector wrong_hpd = make(0x3113, 0x211e, 0, 0, 0);
    wrong_hpd.hpd_gpio.register_index += 1;
    Connector hpd6 = make(0x3113, 0x211e, 0, 28, 0);         // HPD6 mask: not DCN 3.2
    Connector odd = make(0x3113, 0x211e, 0, 4, 0);
    Connector usbc = make(0x3117, 0x211e, 0, 0, 0);
    Connector edp = make(0x3114, 0x211e, 0, 0, 0);
    const Case cases[]{
        {external, RouteFault::external_encoder, "external encoder quarantined"},
        {unknown, RouteFault::unknown_encoder, "UNIPHY3 quarantined"},
        {enum3, RouteFault::unknown_encoder, "UNIPHY enum 3 quarantined"},
        {no_ddc, RouteFault::missing_ddc, "missing I2C record quarantined"},
        {unresolved, RouteFault::ddc_gpio_unresolved, "unresolved DDC GPIO quarantined"},
        {wrong_ddc, RouteFault::ddc_register_mismatch, "non-_A DDC register quarantined"},
        {vga, RouteFault::ddc_register_mismatch, "DDCVGA quarantined"},
        {high, RouteFault::ddc_register_mismatch, "high register bits quarantined"},
        {no_hpd, RouteFault::missing_hpd, "missing HPD record quarantined"},
        {wrong_hpd, RouteFault::hpd_register_mismatch, "wrong HPD register quarantined"},
        {hpd6, RouteFault::hpd_mask_unknown, "HPD6 quarantined"},
        {odd, RouteFault::hpd_mask_unknown, "unknown HPD mask quarantined"},
        {usbc, RouteFault::usb_c_alt_mode_unverified, "USB-C quarantined"},
        {edp, RouteFault::unsupported_connector, "eDP quarantined"},
    };
    for (const Case& entry : cases) {
        ConnectorCatalog catalog{};
        catalog.connectors[0] = entry.connector;
        catalog.count = 1;
        Dcn32RouteTable table{};
        checks.expect(bind_dcn32_routes(catalog, dcn::space(), table) == RouteStatus::success &&
                          !table.routes[0].validated && table.routes[0].fault == entry.fault &&
                          table.validated_count == 0,
                      entry.contract);
    }

    // A different segment2 base makes every board GPIO index mismatch.
    rx7900::display::Dcn32RegisterSpace moved = dcn::space();
    moved.segment2_dword_base += 0x100u;
    Dcn32RouteTable table{};
    const ConnectorCatalog board = navi31_board();
    checks.expect(bind_dcn32_routes(board, moved, table) == RouteStatus::success &&
                      table.validated_count == 0 &&
                      table.routes[0].fault == RouteFault::ddc_register_mismatch,
                  "discovery base mismatch quarantines every route");

    // A segment2 base that pushes DC_GPIO indices past 16 bits can never match
    // the 16-bit bios_parser2 index, even when the ATOM record holds the full value.
    rx7900::display::Dcn32RegisterSpace high_base = dcn::space();
    high_base.segment2_dword_base = 0xf000u;
    ConnectorCatalog wide{};
    wide.connectors[0] = make(0x3113, 0x211e, 0, 0, 0);
    wide.connectors[0].ddc_gpio.register_index = 0xf000u + ddc1_a;
    wide.connectors[0].hpd_gpio.register_index = 0xf000u + hpd_a;
    wide.count = 1;
    checks.expect(bind_dcn32_routes(wide, high_base, table) == RouteStatus::success &&
                      table.routes[0].fault == RouteFault::ddc_register_mismatch,
                  "register index above 16 bits refused");

    rx7900::display::Dcn32RegisterSpace unverified = dcn::space();
    unverified.discovery_verified = false;
    table.count = 77;
    checks.expect(bind_dcn32_routes(board, unverified, table) == RouteStatus::discovery_mismatch &&
                      table.count == 77,
                  "unverified discovery refused without output change");
    rx7900::display::Dcn32RegisterSpace wrong = dcn::space();
    wrong.dmu_minor = 1;
    checks.expect(bind_dcn32_routes(board, wrong, table) == RouteStatus::discovery_mismatch,
                  "DMU 3.1 refused");
}

void check_duplicates(Checks& checks) {
    struct Case final {
        Connector second;
        RouteFault fault;
        const char* contract;
    };
    const Case cases[]{
        {make(0x3213, 0x211e, 1, 8, 1), RouteFault::duplicate_transmitter, "duplicate PHY"},
        {make(0x3213, 0x221e, 0, 8, 1), RouteFault::duplicate_ddc, "duplicate DDC line"},
        {make(0x3213, 0x221e, 1, 0, 1), RouteFault::duplicate_hpd, "duplicate HPD"},
    };
    for (const Case& entry : cases) {
        ConnectorCatalog catalog{};
        catalog.connectors[0] = make(0x3113, 0x211e, 0, 0, 0);
        catalog.connectors[1] = entry.second;
        catalog.connectors[2] = make(0x310c, 0x2120, 2, 16, 2);
        catalog.count = 3;
        Dcn32RouteTable table{};
        checks.expect(bind_dcn32_routes(catalog, dcn::space(), table) == RouteStatus::success &&
                          !table.routes[0].validated && !table.routes[1].validated &&
                          table.routes[0].fault == entry.fault &&
                          table.routes[1].fault == entry.fault && table.routes[2].validated &&
                          table.validated_count == 1,
                      entry.contract);
    }
}

// DC_GPIO_DDCn_MASK/_A/_EN and AUX_CTRL_5 / PHY_AUX_CNTL dword indices.
uint32_t mask_reg(uint8_t line) { return 0x28d0u + 4u * line; }
uint32_t a_reg(uint8_t line) { return 0x28d1u + 4u * line; }
uint32_t en_reg(uint8_t line) { return 0x28d2u + 4u * line; }
constexpr uint32_t aux_ctrl_5 = 0x291du;
constexpr uint32_t phy_aux_cntl = 0x28ffu;

void check_aux_pad(Checks& checks) {
    dcn::Model model;
    const uint8_t line = 2;
    // Boot state: GPIO masks owned by software, I2C mode, no AUX pad.
    model.set(dcn::segment2, mask_reg(line), 0x00001111u);
    model.set(dcn::segment2, a_reg(line), 0x00000101u);
    model.set(dcn::segment2, en_reg(line), 0x00000100u);
    model.set(dcn::segment2, aux_ctrl_5, 0x0001f000u);
    const auto io = model.io();
    checks.expect(!ddc_pad_in_aux_mode(io, dcn::space(), line), "boot pad not in AUX mode");

    DdcPadSave save{};
    checks.expect(open_ddc_pad(io, dcn::space(), line, DdcMode::aux, save) == DdcPadResult::success &&
                      save.valid,
                  "AUX pad opens");
    const std::vector<uint32_t> mask_writes = model.written(dcn::segment2, mask_reg(line));
    // Data MASK cleared, clock MASK cleared, then REG_SET of the cached value with AUX_PAD1_MODE.
    checks.expect(mask_writes.size() == 3 && mask_writes[0] == 0x00001011u &&
                      mask_writes[1] == 0x00001010u && mask_writes[2] == 0x00011010u,
                  "hw_gpio HARDWARE mode then hw_ddc AUX pad, register exact");
    checks.expect(model.written(dcn::segment2, aux_ctrl_5) ==
                      std::vector<uint32_t>{0x0001b000u},
                  "DDC_PAD3_I2CMODE cleared only");
    checks.expect(model.written(dcn::segment2, a_reg(line)).empty() &&
                      model.written(dcn::segment2, en_reg(line)).empty(),
                  "A/EN only stored on open");
    checks.expect(ddc_pad_in_aux_mode(io, dcn::space(), line), "AUX mode proven by read-back");
    checks.expect(!ddc_pad_in_aux_mode(io, dcn::space(), 1), "other line not proven");
    checks.expect(save.data_mask && save.clock_mask && save.data_a && save.clock_a &&
                      save.data_en && !save.clock_en,
                  "store_registers fields");

    DdcPadSave twice = save;
    checks.expect(open_ddc_pad(io, dcn::space(), line, DdcMode::aux, twice) ==
                      DdcPadResult::invalid_argument,
                  "already-open save refused");

    model.writes.clear();
    checks.expect(close_ddc_pad(io, dcn::space(), save) == DdcPadResult::success && !save.valid,
                  "pad closes");
    checks.expect((model.get(dcn::segment2, mask_reg(line)) & 0x101u) == 0x101u,
                  "GPIO masks restored");
    checks.expect(model.writes.size() == 6, "clock then data pin restore: three fields each");
    checks.expect(!ddc_pad_in_aux_mode(io, dcn::space(), line), "AUX mode no longer proven");
}

void check_i2c_pad(Checks& checks) {
    dcn::Model model;
    const uint8_t line = 0;
    model.set(dcn::segment2, mask_reg(line), 0x00010000u); // AUX pad set, no pull-downs
    model.set(dcn::segment2, aux_ctrl_5, 0u);
    model.set(dcn::segment2, phy_aux_cntl, 0x00000c00u);
    DdcPadSave save{};
    const auto io = model.io();
    checks.expect(open_ddc_pad(io, dcn::space(), line, DdcMode::i2c, save) == DdcPadResult::success,
                  "I2C pad opens");
    const std::vector<uint32_t> mask_writes = model.written(dcn::segment2, mask_reg(line));
    checks.expect(mask_writes.size() == 4 && mask_writes[2] == 0x00011010u &&
                      mask_writes[3] == 0x00001010u,
                  "pull-downs REG_SET_2 then AUX_PAD1_MODE cleared");
    checks.expect(model.delayed_us == 2000u, "AUX pad discharge before I2C");
    checks.expect(model.get(dcn::segment2, aux_ctrl_5) == 0x00001000u, "DDC_PAD1_I2CMODE set");
    checks.expect(model.get(dcn::segment2, phy_aux_cntl) == 0x00000400u, "AUX1_PAD_RXSEL = 1");
    checks.expect(!ddc_pad_in_aux_mode(io, dcn::space(), line), "I2C mode is not AUX proof");
}

void check_pad_failures(Checks& checks) {
    {
        dcn::Model model;
        model.fail_read_offset = dcn::byte_offset(dcn::segment2, mask_reg(1));
        DdcPadSave save{};
        checks.expect(open_ddc_pad(model.io(), dcn::space(), 1, DdcMode::aux, save) ==
                          DdcPadResult::read_failure &&
                          model.writes.empty() && !save.valid,
                      "read failure before any write is a clean refusal");
    }
    for (size_t failing = 0; failing < 4; ++failing) {
        dcn::Model model;
        model.fail_write = failing;
        DdcPadSave save{};
        const DdcPadResult result = open_ddc_pad(model.io(), dcn::space(), 1, DdcMode::aux, save);
        checks.expect(result == DdcPadResult::state_unknown, "failed pad write is state_unknown");
        // The first write (data MASK) has nothing stored to restore yet; every
        // later failure keeps the saved pin state for close.
        checks.expect(save.valid == (failing != 0), "save survives for close after the first write");
        if (save.valid) {
            checks.expect(close_ddc_pad(model.io(), dcn::space(), save) == DdcPadResult::success &&
                              (model.get(dcn::segment2, mask_reg(1)) & 0x101u) == 0u,
                          "close after a failed open restores the stored pins");
        }
    }
    dcn::Model model;
    DdcPadSave save{};
    checks.expect(open_ddc_pad(model.io(), dcn::space(), 5, DdcMode::aux, save) ==
                      DdcPadResult::invalid_argument,
                  "line 5 refused");
    rx7900::display::Dcn32RegisterSpace unverified = dcn::space();
    unverified.discovery_verified = false;
    checks.expect(open_ddc_pad(model.io(), unverified, 0, DdcMode::aux, save) ==
                      DdcPadResult::invalid_argument &&
                      model.writes.empty(),
                  "unverified space touches nothing");
}

} // namespace

int main() {
    Checks checks;
    check_board(checks);
    check_faults(checks);
    check_duplicates(checks);
    check_aux_pad(checks);
    check_i2c_pad(checks);
    check_pad_failures(checks);
    return checks.finish("dcn32_route");
}

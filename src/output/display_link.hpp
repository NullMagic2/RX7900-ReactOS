/*
Module: rx7900.output.display_link
Purpose: The DCN3.2 link-side display owner consumed by the stream/commit side.
Created: 2026-10-24
Architecture: Composes dcn32_route (connector routes and DDC pads),
dcn32_link_encoder (DIO/PHY over the VBIOS tables), hdmi_scdc (DDC I2C and
SCDC), display/dcn32_clocks (DALSMC and DCCG) and display/dcn32_dmub (DMCUB)
behind one exclusive owner with route leases. It reuses dp_training for DP
link training and supplies the route proofs NativeDcn32Hpd/NativeDcn32Aux
require. DIG FE/stream encoder, OTG/OPTC, HUBP/DPP/MPC/OPP, unblank, info-
frames, commit and mode changes belong to the dcn-stream side (agents.txt
"Dcn_link" interface). Order follows Linux v6.12 link/link_dpms.c
link_set_dpms_on/off, link/hwss/link_hwss_dio.c and dcn32_hwseq.c.
Linux AMD notice and provenance: licenses/AMDGPU-OUTPUT.txt.
*/
#pragma once

#include "dcn32_clocks.hpp"
#include "dcn32_dmub.hpp"
#include "dcn32_link_encoder.hpp"
#include "hdmi_scdc.hpp"

namespace rx7900::output {

enum class LinkResult : uint8_t {
    success,
    invalid_argument,
    not_bound,
    // A quarantined or unvalidated route, or a route already leased.
    route_unavailable,
    stale_lease,
    // Clock table, DMUB or a source capability is not proven.
    not_ready,
    clock_failed,
    training_failed,
    sink_failed,
    // A write, command table, ring command or cleanup may have changed
    // hardware without proof. Only reset_proven clears it.
    quarantined
};

struct DisplayLinkConfig final {
    hardware::RegisterIo io{};
    // Discovery-validated DMU 3.2.0: segment2 in dcn, segment1 as dccg_segment.
    display::Dcn32RegisterSpace dcn{};
    uint32_t dccg_segment = 0;
    uint32_t mp1_segment0 = 0;
    uint32_t xtalin_khz = 0;
    uint32_t boot_dprefclk_khz = 0;
    uint32_t pll_request = 0;
    // Borrowed until unbind; owned by the device extension.
    const ConnectorCatalog* catalog = nullptr;
    hardware::AtomInterpreter* atom = nullptr;
    hardware::AtomPolicy atom_policy = hardware::atom_default_policy;
    // Optional DMCUB start (empty firmware view = DMCUB left untouched).
    ByteView dmcub_firmware{nullptr, 0};
    ByteView vbios{nullptr, 0};
    display::DmubMemory dmub_memory{};
    // Source DP capability for every DP route (dcn32 link_enc_feature unless
    // the owner decoded the VBIOS connector speed caps).
    DpSourceCapabilities dp_source{};
    uint64_t epoch = 0;
};

// dcn32_resource.c link_enc_feature: HBR3, TPS3/TPS4, four lanes, RGB8 path.
DpSourceCapabilities dcn32_default_dp_source() noexcept;

struct RouteLease final {
    uint8_t route = dcn32_route_none;   // index into routes()
    uint64_t epoch = 0;
    uint32_t generation = 0;
};

enum class TmdsKind : uint8_t { hdmi, dvi_single, dvi_dual };

struct DpTrainedLink final {
    uint8_t rate_code = 0;
    uint8_t lanes = 0;
    bool enhanced_framing = false;
};

// Device-owned (not stack-local: it is several KiB); PASSIVE_LEVEL; the owner
// serializes every call with reset, removal, power and mode changes.
class DisplayLinkOwner final {
public:
    DisplayLinkOwner() noexcept = default;
    DisplayLinkOwner(const DisplayLinkOwner&) = delete;
    DisplayLinkOwner& operator=(const DisplayLinkOwner&) = delete;

    // Binds routes (always), clocks (DALSMC table) and, with firmware, the
    // DMCUB. No PHY, PLL or DTO is touched here.
    LinkResult bind(const DisplayLinkConfig& config) noexcept;
    // Releases leases, clocks and DMCUB when nothing is uncertain.
    LinkResult unbind() noexcept;
    const Dcn32RouteTable& routes() const noexcept { return routes_; }

    // Exclusive lease of one validated route; runs enc32_hw_init the first
    // time a transmitter is leased in this epoch.
    LinkResult acquire_route(uint8_t firmware_path_index, uint64_t epoch, RouteLease& lease) noexcept;
    LinkResult release_route(RouteLease& lease) noexcept;

    const display::DisplayClockTable* clock_table() const noexcept;
    LinkResult set_display_clocks(const display::DisplayClocks& clocks, bool safe_to_lower,
                                  display::DisplayClocks& applied) noexcept;
    uint32_t dprefclk_khz() const noexcept { return clocks_.dprefclk_khz(); }
    // DRAM channel facts for the bandwidth model (ATOM vram_info, decoded by
    // the caller; this owner does not parse vram_info). Table memory fields
    // stay invalid until set.
    void set_memory_info(uint32_t channels, uint32_t width_bytes, uint32_t vram_type) noexcept {
        clocks_.set_memory_info(channels, width_bytes, vram_type);
    }

    // DP: DCCG DP DTO over DPREFCLK; TMDS: PIPE_DTO_SRC_SEL 0, SetPixelClock
    // on the first free COMBO PHY PLL (dc_resource_find_first_free_pll), deep
    // colour resync; then dccg32_set_pixel_rate_div(k1, k2).
    LinkResult set_pixel_clock(const RouteLease& lease, uint32_t otg, uint32_t pixel_100hz,
                               TmdsKind tmds, TmdsDepth depth, uint32_t k1, uint32_t k2) noexcept;
    LinkResult clear_pixel_clock(const RouteLease& lease, uint32_t otg) noexcept;

    // Opens the DDC pad in AUX mode for this DP route (dal_ddc_open AUX) so
    // the AUX engine may run; closed by disable/release.
    LinkResult prepare_aux(const RouteLease& lease) noexcept;
    // link_set_dpms_on DP order: setup (DIG_MODE), connect DIG FE, then
    // train_dp_sst over the caller's AUX and this encoder's PHY callbacks.
    LinkResult enable_dp_link(const RouteLease& lease, const AuxIo& aux,
                              const DpPortSnapshot& port, const DpTrainingRequest& request,
                              uint8_t dig_fe, DpTrainedLink& trained) noexcept;
    // link_set_dpms_on HDMI/DVI order: setup, connect DIG FE, SCDC write
    // (HDMI), TMDS transmitter enable, SCDC read-back (HDMI).
    LinkResult enable_tmds_link(const RouteLease& lease, uint32_t tmds_khz, uint8_t dig_fe,
                                TmdsKind kind, bool lte_340_scramble, ScdcReport& scdc) noexcept;
    // link_set_dpms_off: PHY off (and DP link_encoder_disable), DIG FE
    // disconnect, DDC pad restored.
    LinkResult disable_link(const RouteLease& lease) noexcept;
    // dcn32_init_hw takeover bookkeeping (dcn-stream, additive): when the
    // firmware enabled this route's DIG (is_dig_enabled: link_active,
    // SYMCLK_ON_TX_ON) feeding DIG FE dig_fe, the route is recorded as enabled
    // with that FE so disable_link turns the firmware PHY off and disconnects
    // the FE exactly like a link this owner enabled. Register reads only.
    // tmds selects HDMI or DVI for a TMDS route (ignored for DP).
    LinkResult adopt_firmware_link(const RouteLease& lease, TmdsKind tmds, uint8_t dig_fe,
                                   bool& adopted) noexcept;

    // dc_dmub_srv_p_state_delegate (dcn32_prepare_bandwidth, no FPO stream):
    // with a DMCUB started by this owner, the FAMS SETUP_FW_CTRL command
    // (fams_enabled = should_manage_pstate) and a 100 ms idle wait; without
    // one, success and nothing sent (Linux has no dmub_srv to call).
    // should_manage_pstate true needs FPO pipe data and is refused.
    LinkResult p_state_delegate(bool should_manage_pstate) noexcept;
    // link_set_dpms_off of an HDMI stream on this TMDS route: SCDC
    // write_scdc_data(165000, false) over the route's DDC line. sink_failed
    // when the sink did not take it (Linux ignores that); quarantined when the
    // I2C engine or pad state is unknown.
    LinkResult hdmi_scdc_legacy(const RouteLease& lease) noexcept;

    bool dmub_ready() const noexcept { return dmub_.ready(); }
    LinkResult dmub_execute(uint8_t (&command)[display::dmub_command_bytes], bool reply,
                            uint32_t timeout_us) noexcept;
    bool dmub_status(display::DmubStatus& status) noexcept { return dmub_.status(status); }

    // Proof callbacks for NativeDcn32Hpd / NativeDcn32Aux: true only for the
    // current epoch, a leased validated route with exactly this HPD pin (and
    // AUX engine), the bound segment2/aperture and, for AUX, a DP route whose
    // encoder is initialized, whose DDC pad is read back in AUX mode now and
    // whose DMCUB is either running under this owner or held in reset.
    bool hpd_proof(uint64_t epoch, uint8_t pin, uint32_t segment2, uint32_t aperture) noexcept;
    bool aux_proof(uint64_t epoch, uint8_t engine, uint8_t pin, uint32_t segment2,
                   uint32_t aperture) noexcept;

    // The owner proved a device reset of old_epoch: every lease, encoder,
    // clock and DMCUB object is dropped; bind must be called again.
    void reset_proven(uint64_t old_epoch) noexcept;
    bool quarantined() const noexcept;
    bool bound() const noexcept { return bound_; }

private:
    struct RouteState final {
        uint32_t generation = 0;
        bool leased = false;
        bool aux_open = false;
        bool link_enabled = false;
        bool pll_held = false;
        uint8_t pll = 0;
        uint8_t dig_fe = dcn32_route_none;
        LinkSignal active = LinkSignal::none;
        DdcPadSave pad{};
        // dcn-dsc-odm: link->fec_state (ready / enabled) and the AUX channel
        // disable_link needs to clear FEC_CONFIGURATION (the caller's AuxIo,
        // valid for the lease, as its other AUX users require).
        bool fec_ready = false;
        bool fec_enabled = false;
        AuxIo fec_aux{};
    };

    bool lease_current(const RouteLease& lease) const noexcept;
    Dcn32LinkEncoder& encoder_of(const Dcn32ConnectorRoute& route) noexcept;
    LinkResult from_encoder(EncoderResult result) noexcept;
    LinkResult close_pad(RouteState& state) noexcept;
    I2cEngineConfig i2c_config() const noexcept;

    DisplayLinkConfig config_{};
    Dcn32RouteTable routes_{};
    RouteState states_[connector_capacity]{};
    Dcn32LinkEncoder encoders_[dcn32_link_count]{};
    bool encoder_initialized_[dcn32_link_count]{};
    bool pll_used_[dcn32_link_count]{};
    display::Dcn32ClockManager clocks_{};
    display::Dcn32Dmub dmub_{};
    bool clocks_ready_ = false;
    bool dmub_started_ = false;
    bool bound_ = false;
    bool quarantined_ = false;
};

} // namespace rx7900::output

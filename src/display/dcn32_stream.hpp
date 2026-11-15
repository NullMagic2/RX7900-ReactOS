/*
Module: rx7900.display.dcn32_stream
Purpose: Program, commit, verify and recover one DCN 3.2 stream with up to two linear XRGB8888 planes.
Created: 2026-10-24
Architecture: Portable display core between the bandwidth model
(dcn32_bandwidth, DML32) and the link owner (output::DisplayLinkOwner, reached
only through Dcn32StreamHooks). The commit is the Linux v6.12 DC sequence of
dc_commit_state_no_check for a context holding one stream with one plane on
pipe p (OTG p = OPP p = HUBP p = DPP p = MPCC p), with the dcn32 hw sequencer
table of dcn32_init.c: dcn32_prepare_bandwidth, disable_dangling_plane,
dce110_apply_ctx_to_hw (dcn20_reset_hw_ctx_wrap, dce110_apply_single_controller_
ctx_to_hw, dcn32_resync_fifo_dccg_dio), dcn20_program_front_end_for_ctx under
dcn32_interdependent_update_lock, dcn20_post_unlock_program_front_end,
dcn10_setup_stereo and dcn20_optimize_bandwidth; link_set_dpms_on/off of
link/link_dpms.c with link_hwss_dio. Block programming follows optc1/optc3/
optc32, opp1/opp2, mpc1/mpc2/mpc3/mpc32, dpp1/dpp3, hubp2/hubp3/hubp21/hubp31/
hubp32 and hubbub32. The configuration is the one amdgpu_dm builds for an
XRGB8888 framebuffer on an RGB 8 bpc SDR stream: no scaling, rotation, DCC,
gamma/degamma/CTM/3D LUT, MPC split, SubVP, FAMS, PSR, audio or writeback.
dcn-dsc-odm: the plan's pipe topology (dcn32_topology, from the multi-pipe
DML32 validation) may hold ODM combine 2:1/4:1 slices, a second plane (MPO,
amdgpu_dm zpos order, plane 0 on top) and DP SST DSC (one DSC engine per OPP
head, link_set_dsc_on_stream, PPS SDP, DP_DSC_ENABLE) with FEC; update_planes
is dc_update_planes_and_stream for a FULL surface update of the running stream. The first commit after the pre-OS (GOP) scanout treats the firmware
pipe as the current stream and runs the Linux disable path for it after the
dcn32_init_hw subset that owns DCHUB state (power-gating enable, watermark and
CRB init). Recovery (bounded retry, previous state, blank) and readback
verification are this owner's additions; Linux has neither.
Linux AMD notice and provenance: licenses/AMDGPU-DISPLAY.txt.
*/
#pragma once

#include "dcn32_bandwidth.hpp"
#include "dcn32_dsc.hpp"
#include "dcn32_stream_encoder.hpp"

namespace rx7900::display {

constexpr uint32_t dcn32_stream_pipes = 4;
constexpr uint32_t dcn32_stream_dscs = 4;
// dcn32 dc->caps.cache_line_size / cache_num_ways (MALL).
constexpr uint32_t dcn32_mall_cache_line_bytes = 64;
constexpr uint32_t dcn32_mall_cache_ways = 16;

enum class StreamSignal : uint8_t {
    display_port,
    hdmi,
    dvi_single
};

// dc_tiling_info.gfx9 of a linear surface as amdgpu_dm fills it from the GPU
// (amdgpu_dm_plane_fill_gfx9_tiling_info_from_device: GB_ADDR_CONFIG fields).
struct Dcn32SurfaceTiling final {
    uint32_t num_pipes = 1;               // 1 << GB_ADDR_CONFIG.NUM_PIPES
    uint32_t pipe_interleave_bytes = 256; // 1 << (8 + PIPE_INTERLEAVE_SIZE)
    uint32_t max_compressed_frags = 1;    // 1 << MAX_COMPRESSED_FRAGS
    uint32_t num_pkrs = 1;                // 1 << NUM_PKRS
};

// One plane of a stream (dcn-dsc-odm): an unscaled linear XRGB/ARGB8888
// surface at geometry.dst in the active area, and its MPCC blending
// (dc_plane_state per_pixel_alpha/pre_multiplied_alpha/global_alpha).
struct Dcn32PlaneScanout final {
    uint64_t surface_address = 0;
    uint32_t pitch_pixels = 0;
    Dcn32PlaneGeometry geometry{};
    bool per_pixel_alpha = false;
    bool pre_multiplied_alpha = false;
    bool global_alpha = false;
    uint32_t global_alpha_value = 0xff;
};

// Everything a commit programs, derived once from a validated request
// (build_dcn32_stream_plan). Device-owned; never kept on a kernel stack.
struct Dcn32StreamPlan final {
    Timing timing{};
    StreamSignal signal = StreamSignal::display_port;
    uint32_t pipe = 0;
    // DIG FE / VPG / AFMT / DME instance (ENGINE_ID_DIGA + route transmitter).
    uint32_t engine = 0;
    uint64_t surface_address = 0;
    uint32_t pitch_pixels = 0;
    Dcn32SurfaceTiling tiling{};
    Dcn32BandwidthPlan bandwidth{};
    // dcn20 get_pixel_clock_parameters / dcn32_calculate_dccg_k1_k2_values.
    uint32_t requested_pixel_100hz = 0;
    uint32_t pixel_rate_k1 = 0;
    uint32_t pixel_rate_k2 = 0;
    uint32_t dio_pix_per_cycle = 1;
    // dcn32_update_mall_sel.
    uint32_t mall_sel = 0;
    // HUBP pipe_dest vblank_end (dcn20_populate_dml_pipes_from_context).
    uint32_t vblank_end = 0;
    uint32_t dchub_refclk_mhz = 0;
    // DisplayPort SST 8b/10b.
    uint8_t dp_rate_code = 0;
    uint8_t dp_lanes = 0;
    bool dp_use_vsc = false;
    bool dp_ignore_msa = false;
    // TMDS: character rate equals the pixel clock (8 bpc RGB).
    uint32_t tmds_khz = 0;
    bool lte_340_scramble = false;
    // encoder_info_frame: HDMI avi (0) and vendor (1); DP vsc (0 and 1).
    InfoPacket avi{};
    InfoPacket vendor{};
    InfoPacket vsc{};
    // dcn-dsc-odm. The pipes (bandwidth.topology and bandwidth.pipes), the
    // planes in dc order (planes[0] the top layer; a one-plane plan mirrors
    // surface_address and pitch_pixels; with two planes those name the bottom
    // plane), the ODM slices with their widths, DSC and FEC.
    uint32_t plane_count = 1;
    Dcn32PlaneScanout planes[dcn32_max_planes]{};
    uint32_t odm_slices = 1;
    uint32_t odm_slice_width = 0;
    uint32_t odm_last_slice_width = 0;
    // DSC (DP 8b/10b): the stream's dc_dsc_config, each ODM slice's engine
    // registers (dsc_prepare_config with slices / opp_cnt), the OPTC and
    // encoder configuration, and the PPS SDP of the whole picture.
    bool dsc = false;
    DscConfig dsc_config{};
    DscRegisterValues dsc_registers{};
    DscOptcConfig dsc_optc{};
    uint8_t dsc_pps[dsc_pps_bytes]{};
    // FEC ready/enable in DP training (required with DSC here).
    bool fec = false;
};

struct Dcn32StreamRequest final {
    Timing timing{};
    StreamSignal signal = StreamSignal::display_port;
    uint32_t pipe = 0;
    uint32_t engine = 0;
    uint64_t surface_address = 0;
    uint32_t pitch_pixels = 0;
    Dcn32SurfaceTiling tiling{};
    uint32_t dchub_refclk_khz = 0;
    // dc->caps.max_cab_allocation_bytes (dcn32_calc_num_avail_chans_for_mall * 4 MiB).
    uint32_t max_cab_allocation_bytes = 0;
    uint8_t dp_rate_code = 0;
    uint8_t dp_lanes = 0;
    bool dp_use_vsc = false;
    // TMDS stream plan (hdmi_stream: signal, rate, VIC packets).
    TmdsStreamPlan tmds{};
    // dcn-dsc-odm: planes in dc order (0 keeps the one plane above), DSC
    // (dsc_compute_config result; num_slices_h 0: none) and FEC.
    uint32_t plane_count = 0;
    Dcn32PlaneScanout planes[dcn32_max_planes]{};
    DscConfig dsc{};
    bool fec = false;
};

enum class StreamStatus : uint8_t {
    success,
    invalid_argument,
    // The timing cannot be expressed by OPTC fields or the plane by HUBP fields.
    unsupported,
    // Pitch, alignment or address range not usable for linear scanout.
    invalid_surface
};

// Pure. Fills output only on success.
StreamStatus build_dcn32_stream_plan(const Dcn32StreamRequest& request, const Dcn32BandwidthPlan& bandwidth,
                                     Dcn32StreamPlan& output) noexcept;

// dcn32_is_dp_dig_pixel_rate_div_policy (enable_dp_dig_pixel_rate_div_policy = 1
// on dcn32) for 8b/10b DP: is_h_timing_divisible_by_2.
bool dcn32_dp_dig_pixel_rate_div_policy(const Timing& timing) noexcept;
// dcn32_calculate_mall_ways_from_bytes.
uint32_t dcn32_mall_ways_from_bytes(uint32_t total_bytes, uint32_t max_cab_allocation_bytes) noexcept;

enum class LinkCall : uint8_t {
    success,
    // Refused before anything changed (the link owner says so).
    refused,
    // Hardware may have changed without proof: the commit stops and quarantines.
    uncertain
};

// clk_mgr update_clocks input: the context's bw.dcn.clk plus the DPP DTO of the
// plane's pipe (plane_res.bw.dppclk_khz) and the active display count.
struct Dcn32StreamClocks final {
    Dcn32Clocks clocks{};
    uint32_t pipe = 0;
    uint32_t dpp_khz = 0;
    uint32_t display_count = 0;
    // dcn-dsc-odm: plane_res.bw.dppclk_khz of every pipe of the context (the
    // DPP DTOs of dcn20_update_clocks_update_dpp_dto); all zero for one pipe,
    // whose DTO stays in pipe/dpp_khz.
    uint32_t dpp_pipe_khz[dcn32_stream_pipes]{};
};

// The link owner's operations in the order Linux calls them. Callbacks are
// PASSIVE_LEVEL, serialized with the controller, and never retain the plan.
struct Dcn32StreamHooks final {
    void* context = nullptr;
    // dcn32_update_clocks (prepare: safe_to_lower false; optimize: true).
    LinkCall (*update_clocks)(void*, const Dcn32StreamClocks&, bool safe_to_lower) noexcept = nullptr;
    // program_pix_clk (DP DTO / TMDS PHY PLL) + dccg32_set_pixel_rate_div.
    LinkCall (*set_pixel_clock)(void*, const Dcn32StreamPlan&) noexcept = nullptr;
    // cs_power_down of the old pipe's clock source after its back end reset.
    LinkCall (*power_down_pixel_clock)(void*, const Dcn32StreamPlan&) noexcept = nullptr;
    // enable_link: link_enc setup, DIG FE->BE connect, DP training or SCDC +
    // TMDS output. refused leaves the link off (the commit then recovers).
    LinkCall (*enable_link)(void*, const Dcn32StreamPlan&) noexcept = nullptr;
    // disable_link: PHY off, DIG FE->BE disconnect.
    LinkCall (*disable_link)(void*, const Dcn32StreamPlan&) noexcept = nullptr;
    // DPCD byte access for enable_stream_features (DP_DOWNSPREAD_CTRL).
    LinkCall (*dpcd_read)(void*, uint32_t address, uint8_t& value) noexcept = nullptr;
    LinkCall (*dpcd_write)(void*, uint32_t address, uint8_t value) noexcept = nullptr;
    // dc_dmub_srv_p_state_delegate at the end of dcn32_prepare_bandwidth: a
    // context without FPO streams sends the DMCUB FAMS SETUP_FW_CTRL command
    // with should_manage_pstate false. nullptr means no DMUB service, where
    // Linux dc_wake_and_execute_dmub_cmd returns at once. Linux ignores the
    // result; only uncertain stops the sequence (and quarantines).
    LinkCall (*p_state_delegate)(void*, bool should_manage_pstate) noexcept = nullptr;
    // link_set_dpms_off of an HDMI stream: write_scdc_data(165000, false)
    // returns the sink to legacy TMDS (no scrambling, 1/10 clock ratio) before
    // the stream and link go down. Result ignored like Linux except uncertain.
    LinkCall (*hdmi_scdc_legacy)(void*, const Dcn32StreamPlan&) noexcept = nullptr;
};

// The pre-OS pipe the firmware lights (decoded and validated by preos_scanout).
struct Dcn32FirmwarePipe final {
    Timing timing{};
    StreamSignal signal = StreamSignal::display_port;
    uint32_t pipe = 0;
    uint32_t engine = 0;
};

enum class StreamResult : uint8_t {
    // The requested state is on the hardware, read back and healthy.
    committed,
    // Refused before any register access; the previous state is untouched.
    unchanged,
    // The request failed; the previous state was committed again and verified.
    restored,
    // The request and the previous state failed; the stream is disabled.
    blanked,
    // A register access or a link call was not confirmed: quarantined until an
    // owner-proven reset (reset_proven).
    uncertain
};

enum class StreamFailure : uint8_t {
    none,
    invalid_plan,
    quarantined,
    register_fault,
    register_timeout,
    clock_refused,
    link_refused,
    hook_uncertain,
    readback_mismatch,
    underflow,
    timing_stalled
};

struct StreamHealth final {
    bool active = false;
    bool optc_underflow = false;
    bool hubp_underflow = false;
    bool frame_counter_moving = false;
    bool dpg_blanked = false;
    uint32_t frame_count = 0;
};

struct StreamReport final {
    StreamResult result = StreamResult::unchanged;
    StreamFailure failure = StreamFailure::none;
    // Failure of the first attempt when a later attempt or the rollback won.
    StreamFailure first_failure = StreamFailure::none;
    uint32_t attempts = 0;
    uint32_t commits = 0;
    uint32_t recoveries = 0;
    uint32_t writes = 0;
};

// hubbub2 software state Linux keeps across commits.
struct Dcn32HubbubCache final {
    Dcn32Watermarks a{};
    Dcn32Watermarks b{};
    Dcn32Watermarks c{};
    Dcn32Watermarks d{};
    uint32_t det_size[dcn32_stream_pipes]{};
    uint32_t compbuf_size_segments = 0;
};

// Device-owned (several KiB, trivially destructible), PASSIVE_LEVEL, serialized
// by the device lock with reset, power, removal and every mode IOCTL.
class Dcn32StreamController final {
public:
    Dcn32StreamController() noexcept = default;
    Dcn32StreamController(const Dcn32StreamController&) = delete;
    Dcn32StreamController& operator=(const Dcn32StreamController&) = delete;

    // Segments: discovery DMU 3.2.0 segment1 (DCCG), segment2 (DCN), segment3 (MPC).
    bool bind(const hardware::RegisterIo& io, uint32_t segment1, uint32_t segment2, uint32_t segment3,
              const Dcn32StreamHooks& hooks) noexcept;
    void unbind() noexcept;
    bool bound() const noexcept { return io_.bound(); }

    // The firmware pipe as the current stream. The next commit runs the
    // dcn32_init_hw DCHUB subset and then the Linux disable path for it.
    bool adopt_firmware(const Dcn32FirmwarePipe& firmware) noexcept;

    // Linux-ordered disable of the current state and enable of plan, readback
    // verification and a health check; on failure one retry from a disabled
    // pipe, then the previous state, then blank. plan must outlive the call.
    StreamResult commit(const Dcn32StreamPlan& plan) noexcept;
    // dc_commit_state_no_check with no stream (disables the current one).
    StreamResult disable() noexcept;
    // DPMS: link_set_dpms_off / link_set_dpms_on of the current stream.
    StreamResult set_dpms(bool on) noexcept;
    // Underflow, OTG enable and frame-counter movement of the current stream
    // (register reads, plus one frame of delay). Clears sticky underflow bits
    // it saw so the next check measures a fresh frame.
    bool check_health(StreamHealth& health) noexcept;
    // Re-commits the current plan (after a persistent underflow or hang);
    // falls back to blank like commit.
    StreamResult recover() noexcept;
    // dcn-dsc-odm driver-internal plane update (no GDI overlay surface; a
    // later DirectDraw overlay could use it): plan keeps the current stream
    // (timing, signal, engine, link, DSC/FEC, OTG master and ODM slices) and
    // changes only its planes. Linux dc_update_planes_and_stream for a FULL
    // surface update (update_planes_and_stream_v2 -> commit_planes_for_stream
    // under the interdependent pipe locks), then dc_post_update_surfaces_to_stream
    // at once (DEVIATION: Linux defers it to the next flip). A plan that
    // changes the stream or the ODM slices is refused unchanged (commit()
    // takes it); a failed update falls back to commit(plan).
    StreamResult update_planes(const Dcn32StreamPlan& plan) noexcept;

    // The owner proved a reset of the display hardware (mode-1 reset): every
    // hardware assumption is dropped; the next commit starts from nothing.
    void reset_proven() noexcept;

    bool has_stream() const noexcept { return current_stream_; }
    bool firmware_current() const noexcept { return current_firmware_; }
    bool dpms_on() const noexcept { return current_stream_ && !dpms_off_; }
    const Dcn32StreamPlan* current() const noexcept {
        return current_stream_ && !current_firmware_ ? &current_ : nullptr;
    }
    bool quarantined() const noexcept { return quarantined_; }
    const StreamReport& report() const noexcept { return report_; }
    const Dcn32HubbubCache& hubbub_cache() const noexcept { return hubbub_; }

private:
    enum class Step : uint8_t { ok, failed, uncertain };

    // pipe_ctx->update_flags (dcn20_detect_pipe_changes), the bits used here.
    struct PipeFlags final {
        bool enable = false;
        bool disable = false;
        bool odm = false;
        bool global_sync = false;
        bool mpcc = false;
        bool dppclk = false;
        bool hubp_interdependent = false;
        bool hubp_rq_dlg_ttu = false;
        bool unbounded_req = false;
        bool gamut_remap = false;
        bool scaler = false;
        bool viewport = false;
        bool det_size = false;
        bool plane_changed = false;
        bool opp_changed = false;
        bool any() const noexcept {
            return enable || disable || odm || global_sync || mpcc || dppclk || hubp_interdependent ||
                   hubp_rq_dlg_ttu || unbounded_req || gamut_remap || scaler || viewport || det_size ||
                   plane_changed || opp_changed;
        }
    };
    // dpp->scl_data as dpp1_dscl_set_scaler_manual_scale compares it (the
    // fields that differ between this owner's 1:1 pipes); dpp_reset clears it.
    struct ScalerCache final {
        bool valid = false;
        Dcn32Rect recout{};
        Dcn32Rect viewport{};
        uint32_t h_active = 0;
        uint32_t v_active = 0;
        bool alpha_en = false;
    };
    // The update kind: dc_commit_state_no_check sets every stream update flag,
    // a FULL surface update every plane update flag.
    struct UpdateKind final {
        bool stream_all = false;
        bool plane_all = false;
    };

    Step apply(const Dcn32StreamPlan* target) noexcept;
    Step apply_planes(const Dcn32StreamPlan& plan) noexcept;
    Step verify(const Dcn32StreamPlan& plan) noexcept;
    Step settle_and_check(const Dcn32StreamPlan& plan) noexcept;
    Step takeover() noexcept;
    Step disable_current() noexcept;
    Step attempt(const Dcn32StreamPlan& plan) noexcept;
    // After a failed first attempt: retry from a disabled pipe, then the
    // previous plan, then blank (the commit contract).
    StreamResult retry_or_rollback(const Dcn32StreamPlan& plan) noexcept;
    StreamResult finish(StreamResult result, StreamFailure failure) noexcept;
    Step fail(StreamFailure failure) noexcept;
    Step link(LinkCall call, StreamFailure refused) noexcept;
    // A Linux call whose result Linux ignores: only uncertain stops.
    Step ignored_result(LinkCall call) noexcept;
    // dccg31_init (the dcn32 dccg_init) at the start of dcn32_init_hw.
    Step dccg_init() noexcept;

    // Linux block and sequencer functions.
    Step prepare_bandwidth(const Dcn32BandwidthPlan* bandwidth, const Dcn32StreamClocks& clocks) noexcept;
    Step optimize_bandwidth(const Dcn32StreamPlan* target, const Dcn32BandwidthPlan* bandwidth,
                            const Dcn32StreamClocks& clocks) noexcept;
    Step expect(const dcn32_stream_reg::Register& reg, uint32_t instance, dcn32_stream_reg::Field field,
                uint32_t value) noexcept;
    Step update_dsc_pg(const Dcn32StreamPlan* target, bool safe_to_disable) noexcept;
    Step dsc_pg_control(uint32_t dsc, bool power_on) noexcept;
    Step dsc_pg_status(uint32_t dsc, bool& ungated) noexcept;
    Step program_watermarks(const Dcn32BandwidthPlan& plan, bool safe_to_lower) noexcept;
    Step program_compbuf_size(uint32_t compbuf_kb, bool safe_to_increase) noexcept;
    Step program_det_size(uint32_t hubp, uint32_t det_kb) noexcept;
    Step force_pstate_change_control(bool force, bool allow) noexcept;
    Step update_force_pstate(const Dcn32Topology& context) noexcept;
    Step interdependent_lock(const Dcn32Topology& context, bool lock) noexcept;
    Step pipe_lock(uint32_t otg, bool lock, bool& locked) noexcept;
    Step disable_dangling_plane() noexcept;
    Step blank_pixel_data(const Dcn32Topology& context, const Timing& timing, bool blank) noexcept;
    Step dpg_pattern(uint32_t opp, uint32_t width, uint32_t height, uint32_t offset, bool blank) noexcept;
    Step wait_for_blank_complete(uint32_t opp) noexcept;
    Step plane_atomic_disconnect(uint32_t pipe, uint32_t opp) noexcept;
    Step wait_for_mpcc_disconnect(uint32_t opp, uint32_t otg) noexcept;
    Step plane_atomic_disable(uint32_t pipe, uint32_t opp, uint32_t otg) noexcept;
    Step hubp_pg_control(uint32_t hubp, bool power_on) noexcept;
    Step reset_back_end_for_pipe() noexcept;
    Step link_set_dpms_off(const Dcn32StreamPlan& stream) noexcept;
    Step disable_stream(const Dcn32StreamPlan& stream) noexcept;
    Step disable_crtc(uint32_t otg) noexcept;
    Step enable_optc_clock(uint32_t otg, bool enable) noexcept;
    Step set_odm_bypass(uint32_t otg) noexcept;
    Step set_odm_combine(const Dcn32StreamPlan& plan) noexcept;
    Step setup_manual_trigger(uint32_t otg) noexcept;
    Step apply_single_controller(const Dcn32StreamPlan& plan) noexcept;
    Step program_fmt(uint32_t opp) noexcept;
    Step set_dyn_expansion(uint32_t opp, StreamSignal signal) noexcept;
    Step enable_stream_timing(const Dcn32StreamPlan& plan) noexcept;
    Step program_timing(const Dcn32StreamPlan& plan) noexcept;
    Step program_global_sync(uint32_t otg, uint32_t vready_offset, const Dcn32PipeBandwidth& head) noexcept;
    Step set_vtg_params(const Dcn32StreamPlan& plan, bool program_fp2) noexcept;
    Step setup_vupdate_interrupt(const Dcn32StreamPlan& plan) noexcept;
    Step enable_crtc(uint32_t otg) noexcept;
    Step link_set_dpms_on(const Dcn32StreamPlan& plan) noexcept;
    Step update_info_frame(const Dcn32StreamPlan& plan) noexcept;
    Step enable_stream_features(const Dcn32StreamPlan& plan) noexcept;
    Step resync_fifo_dccg_dio() noexcept;
    // DSC (link_dpms.c link_set_dsc_on_stream / link_set_dsc_pps_packet /
    // dp_set_dsc_on_rx, dcn32_update_dsc_on_stream, optc3_set_dsc_config).
    Step set_dsc_on_stream(const Dcn32StreamPlan& plan, bool enable) noexcept;
    Step update_dsc_on_stream(const Dcn32StreamPlan& plan) noexcept;
    Step set_dsc_pps_packet(const Dcn32StreamPlan& plan, bool enable, bool immediate_update) noexcept;
    Step set_dsc_on_rx(const Dcn32StreamPlan& plan, bool enable) noexcept;
    Step optc_set_dsc_config(uint32_t otg, uint32_t mode, uint32_t bytes_per_pixel, uint32_t slice_width) noexcept;
    Step update_odm(const Dcn32StreamPlan& plan) noexcept;
    // Front end (dcn20_program_front_end_for_ctx and its pipe functions).
    void detect_pipe_changes(const Dcn32StreamPlan* next_plan, const Dcn32Topology& next, uint32_t pipe,
                             PipeFlags& flags) const noexcept;
    Step program_front_end(const Dcn32StreamPlan* next_plan, const Dcn32Topology& next, UpdateKind kind) noexcept;
    Step post_unlock_front_end(const Dcn32StreamPlan* next_plan, const Dcn32Topology& next) noexcept;
    Step post_unlock_reset_opp(uint32_t opp_head) noexcept;
    Step program_pipe(const Dcn32StreamPlan& plan, uint32_t pipe, const PipeFlags& flags, UpdateKind kind) noexcept;
    Step enable_plane(uint32_t pipe, uint32_t opp) noexcept;
    Step update_dchubp_dpp(const Dcn32StreamPlan& plan, uint32_t pipe, const PipeFlags& flags,
                           UpdateKind kind) noexcept;
    Step hubp_setup(const Dcn32StreamPlan& plan, uint32_t pipe) noexcept;
    Step hubp_setup_interdependent(const Dcn32StreamPlan& plan, uint32_t pipe) noexcept;
    Step dpp_setup(uint32_t dpp) noexcept;
    Step update_mpcc(const Dcn32StreamPlan& plan, uint32_t pipe, const PipeFlags& flags, UpdateKind kind) noexcept;
    Step update_blending(const Dcn32StreamPlan& plan, uint32_t pipe) noexcept;
    Step remove_mpcc(uint32_t opp, uint32_t mpcc) noexcept;
    Step dpp_set_scaler(const Dcn32StreamPlan& plan, uint32_t pipe) noexcept;
    Step program_viewport(const Dcn32StreamPlan& plan, uint32_t pipe) noexcept;
    Step program_gamut_and_output_csc(const Dcn32StreamPlan& plan, uint32_t pipe) noexcept;
    Step program_surface_config(const Dcn32StreamPlan& plan, uint32_t pipe) noexcept;
    Step program_surface_address(const Dcn32StreamPlan& plan, uint32_t pipe) noexcept;
    Step set_input_transfer_func(uint32_t pipe) noexcept;
    Step set_output_transfer_func(uint32_t pipe) noexcept;
    Step is_flip_pending(uint32_t hubp, bool& pending) noexcept;
    Step wait_for_flip(uint32_t hubp) noexcept;
    Step wait_for_no_pipes_pending(const Dcn32Topology& context) noexcept;
    Step wait_for_odm_update_pending_complete(const Dcn32Topology& context) noexcept;
    Step setup_stereo(uint32_t opp, uint32_t otg) noexcept;
    Step set_avmute(const Dcn32StreamPlan& stream, bool enable) noexcept;
    Step wait_for_state(uint32_t otg, bool vblank) noexcept;
    Step check_frame(const Dcn32StreamPlan& plan, StreamHealth& health) noexcept;
    Dcn32StreamClocks stream_clocks(const Dcn32StreamPlan& plan, uint32_t display_count) const noexcept;

    Dcn32StreamIo io_{};
    Dcn32StreamHooks hooks_{};
    // The state on the hardware (Linux dc->current_state of pipe current_.pipe).
    Dcn32StreamPlan current_{};
    bool current_stream_ = false;
    bool current_plane_ = false;
    bool current_firmware_ = false;
    bool dpms_off_ = false;
    // The state before the running commit (its rollback target) and the copy
    // recover() commits from (both device-owned: a plan is too large for a
    // kernel stack frame).
    Dcn32StreamPlan rollback_{};
    bool rollback_valid_ = false;
    Dcn32StreamPlan scratch_{};
    // DCHUB reference clock of the contexts being programmed (MHz).
    uint32_t dchub_refclk_mhz_ = 0;
    // dc->current_state's pipe relations as the Linux sequences leave them
    // (disable_dangling_plane removes the planes, the back end reset clears
    // the OTG master's stream).
    Dcn32Topology live_{};
    // opp->mpc_tree_params.opp_list head per OPP (MPCC id, 0xf = empty), each
    // MPCC's mpcc_bot and dpp_id (0xf = none), and
    // opp->mpcc_disconnect_pending[mpcc] per OPP.
    uint32_t mpc_tree_top_[dcn32_stream_pipes]{0xf, 0xf, 0xf, 0xf};
    uint32_t mpcc_bot_[dcn32_stream_pipes]{0xf, 0xf, 0xf, 0xf};
    uint32_t mpcc_dpp_[dcn32_stream_pipes]{0xf, 0xf, 0xf, 0xf};
    bool mpcc_disconnect_pending_[dcn32_stream_pipes][dcn32_stream_pipes]{};
    // optc1->opp_count (OTG_H_TIMING_DIV_MODE of optc1_program_timing).
    uint32_t opp_count_[dcn32_stream_pipes]{1, 1, 1, 1};
    // hubp->power_gated, hubp->request_address and dpp->scl_data.
    bool hubp_power_gated_[dcn32_stream_pipes]{};
    uint64_t hubp_request_address_[dcn32_stream_pipes]{};
    ScalerCache scl_[dcn32_stream_pipes]{};
    // The update flags program_front_end computed for post_unlock_front_end.
    PipeFlags flags_[dcn32_stream_pipes]{};
    Dcn32HubbubCache hubbub_{};
    // The dcn32_init_hw DCHUB subset (init_crb reads the DET/compbuf sizes the
    // VBIOS left) runs before the first commit and after every proven reset.
    bool takeover_pending_ = true;
    bool quarantined_ = false;
    StreamFailure failure_ = StreamFailure::none;
    StreamReport report_{};
};

} // namespace rx7900::display

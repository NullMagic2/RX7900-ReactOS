/*
Module: rx7900.display.dcn32_dmub
Purpose: Start the PSP-loaded DCN3.2 DMCUB, verify its boot status and own its inbox1 ring.
Created: 2026-10-24
Architecture: Uses the checked DMCUB container (hardware/firmware), the shared
hardware::RegisterIo contract and an owner-supplied VRAM window for the DMUB
framebuffer regions. Follows Linux v6.12 dmub/src/dmub_srv.c
(dmub_srv_calc_region_info, dmub_get_fw_meta_info, dmub_srv_hw_init,
dmub_srv_wait_for_auto_load, cmd_queue/cmd_execute/wait_for_idle,
cmd_with_reply_data), dmub/src/dmub_dcn32.c (reset, reset_release,
setup_windows, setup_mailbox, setup_out_mailbox, setup_outbox0, boot options,
is_supported, is_hw_init), dmub/inc/dmub_cmd.h (ring, header, boot status,
GPINT) and amdgpu_dm.c dm_dmub_sw_init/dm_dmub_hw_init (PSP header/footer,
VBIOS copy, region clearing). Instruction memory is loaded by PSP
(AsicInit, ucode id 61); this module never backdoor-loads it.
Linux AMD register provenance and notice: licenses/AMDGPU-DISPLAY.txt.
*/
#pragma once

#include "dcn32_otg.hpp"
#include "firmware.hpp"
#include "register_io.hpp"

namespace rx7900::display {

constexpr uint32_t dmub_command_bytes = 64;   // DMUB_RB_CMD_SIZE
constexpr uint32_t dmub_ring_bytes = 8192;    // DMUB_RB_SIZE (128 entries)
constexpr uint32_t dmub_window_count = 9;     // DMUB_WINDOW_TOTAL

enum class DmubWindow : uint8_t {
    inst_const = 0, stack, bss_data, vbios, mailbox, trace_buffer, fw_state, scratch, shared_state
};

struct DmubRegion final {
    uint32_t base = 0;
    uint32_t top = 0;
};

// dmub_srv_calc_region_info output plus the metadata it consumed.
struct DmubLayout final {
    DmubRegion windows[dmub_window_count]{};
    uint32_t framebuffer_bytes = 0;
    uint32_t inst_const_bytes = 0;      // without the PSP header and footer
    uint32_t bss_data_bytes = 0;
    uint32_t bss_data_file_offset = 0;
    uint32_t fw_version = 0;
    bool metadata_found = false;
};

enum class DmubResult : uint8_t {
    success,
    invalid_argument,
    invalid_firmware,
    // CC_DC_PIPE_DIS.DC_DMCUB_ENABLE is clear: no DMCUB on this part.
    unsupported,
    not_ready,
    timeout,
    queue_full,
    // A write or a started command could not be proven; the DMCUB and its
    // ring are quarantined until a reset proof and a new hw_init.
    quarantined
};

// Pure. Uses the DMCUB container (FirmwareKind::dmcub_3_2) and the VBIOS size.
DmubResult dmub_calc_layout(ByteView firmware, uint32_t vbios_bytes, DmubLayout& layout) noexcept;

// CPU access to the owner's DMUB framebuffer allocation (VRAM, MC address
// gpu_address, at least layout.framebuffer_bytes, 4 KiB aligned). Callbacks
// are bounds-checked by the owner; flush orders CPU writes before MMIO.
struct DmubMemory final {
    void* context = nullptr;
    uint64_t gpu_address = 0;
    uint32_t bytes = 0;
    // Copies source.size() bytes to offset (checked views, no raw source pointer).
    bool (*write)(void*, uint32_t offset, ByteView source) noexcept = nullptr;
    bool (*read)(void*, uint32_t offset, uint8_t* data, uint32_t bytes) noexcept = nullptr;
    bool (*fill_zero)(void*, uint32_t offset, uint32_t bytes) noexcept = nullptr;
    bool (*flush)(void*) noexcept = nullptr;
};

struct DmubStatus final {
    uint32_t boot_status = 0;  // DMCUB_SCRATCH0 (dmub_fw_boot_status)
    uint32_t inbox_rptr = 0;
    uint32_t inbox_wptr = 0;
    bool enabled = false;      // DMCUB_CNTL.DMCUB_ENABLE
    bool soft_reset = false;   // DMCUB_CNTL2.DMCUB_SOFT_RESET
};

// Device-owned DMCUB service; PASSIVE_LEVEL, exclusive with every DMUB, AUX
// arbitration and display user. The firmware and ROM views must stay resident
// until start returns; memory stays valid until stop or quarantine.
class Dcn32Dmub final {
public:
    Dcn32Dmub() noexcept = default;
    Dcn32Dmub(const Dcn32Dmub&) = delete;
    Dcn32Dmub& operator=(const Dcn32Dmub&) = delete;

    // dm_dmub_hw_init + dmub_srv_hw_init + wait_for_auto_load (100 ms in
    // 100 us steps): checks hardware support, copies BSS/data and the VBIOS,
    // zeroes mailbox/trace/fw-state/shared regions, resets the DMCUB (GPINT
    // STOP_FW first when running), programs CW3..CW6, REGION5, OUTBOX0,
    // INBOX1, OUTBOX1, boot options, releases reset and waits for dal_fw and
    // mailbox_rdy in DMCUB_SCRATCH0.
    DmubResult start(const hardware::RegisterIo& io, const Dcn32RegisterSpace& space,
                     ByteView firmware, ByteView vbios, const DmubMemory& memory) noexcept;
    // dmub_srv_hw_reset: GPINT stop and forced soft reset; ring state cleared.
    DmubResult stop() noexcept;

    // dmub_srv_cmd_queue + execute + wait_for_idle(timeout); with reply the
    // ring slot is read back into command (dmub_rb_get_return_data). The
    // command is the 64-byte union dmub_rb_cmd image with its header filled.
    DmubResult execute(uint8_t (&command)[dmub_command_bytes], bool reply,
                       uint32_t timeout_us) noexcept;

    // Only after the device owner proved an ASIC reset (DMCUB back in reset).
    void reset_proven() noexcept {
        io_ = {};
        space_ = {};
        memory_ = {};
        layout_ = {};
        wptr_ = 0;
        rptr_ = 0;
        ready_ = false;
        quarantined_ = false;
    }
    bool ready() const noexcept { return ready_ && !quarantined_; }
    bool quarantined() const noexcept { return quarantined_; }
    bool status(DmubStatus& output) noexcept;
    const DmubLayout& layout() const noexcept { return layout_; }

private:
    bool read(uint32_t reg, uint32_t& value) noexcept;
    bool write(uint32_t reg, uint32_t value) noexcept;
    bool update(uint32_t reg, uint32_t field, uint32_t value) noexcept;
    bool reset() noexcept;
    bool program_windows() noexcept;
    bool prepare_memory(ByteView firmware, ByteView vbios) noexcept;
    DmubResult fail() noexcept;

    hardware::RegisterIo io_{};
    Dcn32RegisterSpace space_{};
    DmubMemory memory_{};
    DmubLayout layout_{};
    uint32_t wptr_ = 0;
    uint32_t rptr_ = 0;
    bool ready_ = false;
    bool quarantined_ = false;
};

// Read-only: true and held set when DMCUB_CNTL2.DMCUB_SOFT_RESET reads back,
// false on a failed read. Used by owners that never started the DMCUB to prove
// it cannot arbitrate for AUX.
bool dmub_held_in_reset(const hardware::RegisterIo& io, const Dcn32RegisterSpace& space,
                        bool& held) noexcept;

// dmub_cmd_header (type 7:0, sub_type 15:8, payload_bytes 29:24) for a
// command image; payload_bytes excludes the four header bytes.
void dmub_set_header(uint8_t (&command)[dmub_command_bytes], uint8_t type, uint8_t sub_type,
                     uint8_t payload_bytes) noexcept;

// dmub_cmd.h DMUB_CMD__FW_ASSISTED_MCLK_SWITCH / DMUB_CMD__FAMS_SETUP_FW_CTRL.
constexpr uint8_t dmub_cmd_fw_assisted_mclk_switch = 76;
constexpr uint8_t dmub_cmd_fams_setup_fw_ctrl = 0;
// sizeof(struct dmub_rb_cmd_fw_assisted_mclk_switch) minus its header:
// config_data (fams_enabled, visual_confirm_enabled, vactive_stretch_margin_us)
// and DMUB_MAX_FPO_STREAMS (4) 12-byte pipe_data entries.
constexpr uint8_t dmub_fams_config_payload_bytes = 52;
// dc_dmub_srv_wait_for_idle / dc_dmub_srv_cmd_run_list (DM_DMUB_WAIT_TYPE_WAIT).
constexpr uint32_t dmub_command_wait_us = 100000;

// dc_dmub_srv_p_state_delegate's command image for a context without FPO
// streams: header, fams_enabled = should_manage_pstate, visual confirm 0
// (VISUAL_CONFIRM_FAMS is a debug option), no vactive stretch margin and
// empty pipe_data (Linux fills pipe_data only for streams with fpo_in_use,
// which this driver never plans). Every other byte is zero (union dmub_rb_cmd
// cmd = { 0 }).
void dmub_build_fams_setup(bool should_manage_pstate, uint8_t (&command)[dmub_command_bytes]) noexcept;

} // namespace rx7900::display

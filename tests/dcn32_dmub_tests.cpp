/*
Module: rx7900.tests.dcn32_dmub
Purpose: Check DMCUB region layout, start-up, boot status and the inbox1 ring.
Created: 2026-10-24
Architecture: Synthetic DMCUB container, VRAM window and DMCUB register model
on the shared DCN register model; expected values follow Linux v6.12
dmub_srv.c, dmub_dcn32.c and dmub_cmd.h. No physical GPU access.
*/
#include "dcn32_display_model.hpp"
#include "dcn32_dmub.hpp"

namespace {
using namespace rx7900::display;
using rx7900_test::Checks;
namespace dcn = rx7900_test::dcn;

constexpr uint32_t seg2 = dcn::segment2;
constexpr uint32_t inbox1_wptr = 0x01d6u;
constexpr uint32_t inbox1_rptr = 0x01d7u;
constexpr uint32_t scratch0 = 0x01e3u;
constexpr uint32_t scratch7 = 0x01eau;
constexpr uint32_t dmcub_cntl = 0x01f6u;
constexpr uint32_t gpint = 0x01f8u;
constexpr uint32_t dmcub_cntl2 = 0x0200u;
constexpr uint64_t gpu = 0x0000008000400000ull;

void put32(std::vector<uint8_t>& bytes, size_t offset, uint32_t value) {
    for (uint32_t index = 0; index < 4; ++index) {
        bytes.at(offset + index) = static_cast<uint8_t>(value >> (8 * index));
    }
}

// amdgpu_firmware_header (32 bytes) + dmcub_firmware_header_v1_0 inst/bss sizes.
std::vector<uint8_t> firmware(uint32_t inst_const, uint32_t bss, bool meta) {
    const uint32_t offset = 0x100;
    std::vector<uint8_t> bytes(offset + inst_const + bss, 0);
    put32(bytes, 0, static_cast<uint32_t>(bytes.size()));
    put32(bytes, 4, 40);
    put32(bytes, 8, 1);                 // header 1.0
    put32(bytes, 12, 3u | (2u << 16));  // IP 3.2
    put32(bytes, 16, 0x07002300u);
    put32(bytes, 20, inst_const + bss);
    put32(bytes, 24, offset);
    put32(bytes, 32, inst_const);
    put32(bytes, 36, bss);
    if (meta && bss != 0) {
        const size_t at = offset + inst_const + bss - 0x24 - 64;
        put32(bytes, at, 0x444d5542u);
        put32(bytes, at + 4, 0x8000u);
        put32(bytes, at + 8, 0x4000u);
        put32(bytes, at + 12, 0x07002300u);
    }
    return bytes;
}

struct Vram final {
    std::vector<uint8_t> bytes = std::vector<uint8_t>(0x100000, 0xcc);
    uint32_t flushes = 0;
    bool fail_writes = false;

    static bool write(void* context, uint32_t offset, rx7900::ByteView source) noexcept {
        auto& vram = *static_cast<Vram*>(context);
        if (vram.fail_writes || offset > vram.bytes.size() ||
            source.size() > vram.bytes.size() - offset) {
            return false;
        }
        for (size_t index = 0; index < source.size(); ++index) {
            source.read_u8(index, vram.bytes[offset + index]);
        }
        return true;
    }
    static bool read(void* context, uint32_t offset, uint8_t* data, uint32_t bytes) noexcept {
        auto& vram = *static_cast<Vram*>(context);
        if (offset > vram.bytes.size() || bytes > vram.bytes.size() - offset) {
            return false;
        }
        for (uint32_t index = 0; index < bytes; ++index) {
            data[index] = vram.bytes[offset + index];
        }
        return true;
    }
    static bool fill(void* context, uint32_t offset, uint32_t bytes) noexcept {
        auto& vram = *static_cast<Vram*>(context);
        if (offset > vram.bytes.size() || bytes > vram.bytes.size() - offset) {
            return false;
        }
        for (uint32_t index = 0; index < bytes; ++index) {
            vram.bytes[offset + index] = 0;
        }
        return true;
    }
    static bool flush(void* context) noexcept {
        ++static_cast<Vram*>(context)->flushes;
        return true;
    }
    DmubMemory memory() {
        return {this, gpu, static_cast<uint32_t>(bytes.size()), write, read, fill, flush};
    }
};

struct Firmware final {
    bool boots = true;
    bool consumes = true;
    uint32_t commands = 0;
    uint32_t mailbox = 0;
    Vram* vram = nullptr;
};

void install(dcn::Model& model, Firmware& fw) {
    model.set(seg2, 0x00cau, 0x00010000u);  // CC_DC_PIPE_DIS.DC_DMCUB_ENABLE
    model.set(seg2, dmcub_cntl, 0x00010000u); // running pre-OS firmware
    model.on_write = [&fw](dcn::Model& m, uint32_t offset, uint32_t value) {
        if (offset == dcn::byte_offset(seg2, gpint) && value == 0x10020000u) {
            m.set(seg2, gpint, 0x00020000u);
            m.set(seg2, scratch7, 0xdeaddeadu);
        } else if (offset == dcn::byte_offset(seg2, dmcub_cntl2) && (value & 1u) == 0 && fw.boots) {
            m.set(seg2, scratch0, 0x3u);
        } else if (offset == dcn::byte_offset(seg2, inbox1_wptr) && fw.consumes &&
                   value != m.get(seg2, inbox1_rptr)) {
            ++fw.commands;
            // Mark the consumed command's return status, as firmware does.
            const uint32_t slot = (value + 8192u - 64u) % 8192u;
            fw.vram->bytes.at(fw.mailbox + slot + 2) |= 1u;
            m.set(seg2, inbox1_rptr, value);
        }
    };
}

void check_layout(Checks& checks) {
    DmubLayout layout{};
    const std::vector<uint8_t> image = firmware(0x1200, 0x400, true);
    checks.expect(dmub_calc_layout(rx7900::ByteView(image.data(), image.size()), 0x10000, layout) ==
                      DmubResult::success,
                  "layout computes");
    checks.expect(layout.inst_const_bytes == 0x1000 && layout.bss_data_bytes == 0x400 &&
                      layout.bss_data_file_offset == 0x1300 && layout.metadata_found,
                  "PSP header/footer removed, legacy metadata found");
    const uint32_t expected[9][2]{{0, 0x1000},          {0x1000, 0xa1000},   {0xa1000, 0xa1400},
                                  {0xa1400, 0xb1400},   {0xb1400, 0xb5400},  {0xb5400, 0xb9400},
                                  {0xb9400, 0xc1400},   {0xc1400, 0xc1800},  {0xc1800, 0xc1c00}};
    bool same = true;
    for (uint32_t index = 0; index < 9; ++index) {
        same = same && layout.windows[index].base == expected[index][0] &&
               layout.windows[index].top == expected[index][1];
    }
    checks.expect(same, "dmub_srv_calc_region_info windows (256 base, 64 size alignment)");
    checks.expect(layout.framebuffer_bytes == 0xc2000, "framebuffer rounded to 4 KiB");

    const std::vector<uint8_t> plain = firmware(0x1200, 0, false);
    checks.expect(dmub_calc_layout(rx7900::ByteView(plain.data(), plain.size()), 0x10000, layout) ==
                      DmubResult::success &&
                      !layout.metadata_found &&
                      layout.windows[5].top - layout.windows[5].base == 0x10000 &&
                      layout.windows[6].top - layout.windows[6].base == 0x10000,
                  "default 64 KiB trace and fw-state without metadata");
    checks.expect(dmub_calc_layout(rx7900::ByteView(image.data(), image.size()), 0x10001, layout) ==
                      DmubResult::success &&
                      layout.windows[3].top == 0xa1400u + 0x10040u &&
                      layout.windows[4].base == 0xb1500u,
                  "window sizes rounded to 64 bytes, bases to 256");
    const std::vector<uint8_t> tiny = firmware(0x200, 0, false);
    checks.expect(dmub_calc_layout(rx7900::ByteView(tiny.data(), tiny.size()), 0x10000, layout) ==
                      DmubResult::invalid_firmware,
                  "instruction region smaller than PSP header+footer refused");
    std::vector<uint8_t> wrong = image;
    put32(wrong, 12, 3u | (1u << 16));
    checks.expect(dmub_calc_layout(rx7900::ByteView(wrong.data(), wrong.size()), 0x10000, layout) ==
                      DmubResult::invalid_firmware,
                  "DCN 3.1 DMCUB refused");
    checks.expect(dmub_calc_layout(rx7900::ByteView(image.data(), image.size()), 0, layout) ==
                      DmubResult::invalid_firmware,
                  "empty VBIOS refused");

    uint8_t command[dmub_command_bytes]{};
    dmub_set_header(command, 128, 1, 0x7f);
    checks.expect(command[0] == 128 && command[1] == 1 && command[2] == 0 && command[3] == 0x3f,
                  "dmub_cmd_header layout");
}

void check_start(Checks& checks) {
    dcn::Model model;
    Vram vram;
    Firmware fw;
    fw.vram = &vram;
    install(model, fw);
    const std::vector<uint8_t> image = firmware(0x1200, 0x400, true);
    std::vector<uint8_t> bios(0x10000, 0x5a);
    Dcn32Dmub dmub;
    const DmubResult started = dmub.start(model.io(), dcn::space(),
                                          rx7900::ByteView(image.data(), image.size()),
                                          rx7900::ByteView(bios.data(), bios.size()), vram.memory());
    checks.expect(started == DmubResult::success && dmub.ready(), "DMCUB starts and boots");
    fw.mailbox = dmub.layout().windows[4].base;
    checks.expect(vram.bytes[0xa1400] == 0x5a && vram.bytes[0xb13ff] == 0x5a, "VBIOS copied to CW3");
    checks.expect(vram.bytes[0xb1400] == 0 && vram.bytes[0xc1bff] == 0 && vram.bytes[0xc1400] == 0xcc,
                  "mailbox/trace/fw-state/shared cleared, scratch untouched");
    checks.expect(vram.bytes[0xa1000 + 0x400 - 0x24 - 64] == 0x42, "BSS data copied (metadata magic)");
    checks.expect(model.written(seg2, gpint).front() == 0x10020000u &&
                      model.written(seg2, gpint).back() == 0u,
                  "GPINT STOP_FW then cleared");
    const size_t reset = model.write_index(seg2, dmcub_cntl2);
    const size_t window = model.write_index(seg2, 0x01bbu);
    const size_t release = model.write_index(seg2, dmcub_cntl2, reset + 1);
    checks.expect(reset < window && window < release, "reset, windows, then release");
    checks.expect(model.get(seg2, 0x01bbu) == static_cast<uint32_t>(gpu + 0xa1400) &&
                      model.get(seg2, 0x01bcu) == 0x80u && model.get(seg2, 0x01a8u) == 0x63000000u &&
                      model.get(seg2, 0x01b0u) == (0x03010000u | 0x80000000u),
                  "CW3 offset, base and top+enable");
    checks.expect(model.get(seg2, 0x01bdu) == static_cast<uint32_t>(gpu + 0xb1400) &&
                      model.get(seg2, 0x01b1u) == (0x04004000u | 0x80000000u),
                  "CW4 mailbox window");
    checks.expect(model.get(seg2, 0x0198u) == static_cast<uint32_t>(gpu + 0xb5400) &&
                      model.get(seg2, 0x01a2u) == (0x3fffu | 0x80000000u),
                  "REGION5 trace buffer");
    checks.expect(model.get(seg2, 0x01d8u) == 0xa0000010u && model.get(seg2, 0x01d9u) == 0x3ff0u,
                  "OUTBOX0 after the trace entry offset");
    checks.expect(model.get(seg2, 0x01d4u) == 0x64000000u && model.get(seg2, 0x01d5u) == 0x2000u &&
                      model.get(seg2, 0x01dcu) == 0x64002000u && model.get(seg2, 0x01ddu) == 0x2000u,
                  "INBOX1 and OUTBOX1 split the mailbox");
    checks.expect(model.get(seg2, 0x01c1u) == static_cast<uint32_t>(gpu + 0xb9400) &&
                      model.get(seg2, 0x01b3u) == (0x06008000u | 0x80000000u),
                  "CW6 firmware state (metadata size)");
    checks.expect(model.get(seg2, dmcub_cntl) == (0x00010000u | 0x00080000u) &&
                      (model.get(seg2, dmcub_cntl2) & 1u) == 0 &&
                      (model.get(seg2, 0x0342u) & 0x100u) == 0 && model.get(seg2, 0x01f2u) == 0,
                  "reset_release: enable, traceport, DMUIF out of reset");
    DmubStatus status{};
    checks.expect(dmub.status(status) && status.boot_status == 3 && status.enabled && !status.soft_reset,
                  "status reads boot state");

    uint8_t command[dmub_command_bytes]{};
    dmub_set_header(command, 0, 0, 0);
    command[4] = 0x11;
    checks.expect(dmub.execute(command, true, 1000) == DmubResult::success && fw.commands == 1,
                  "NULL command executes");
    checks.expect(vram.bytes[0xb1400 + 4] == 0x11 && model.get(seg2, inbox1_wptr) == 64u,
                  "command copied to ring slot 0, WPTR advanced by 64");
    checks.expect((command[2] & 1u) == 1u, "reply read back from the ring");
    for (uint32_t index = 0; index < 127; ++index) {
        dmub.execute(command, false, 1000);
    }
    checks.expect(model.get(seg2, inbox1_wptr) == 0u, "ring wraps at 8 KiB");

    // An RPTR one slot ahead of WPTR reads as a full ring (dmub_rb_full).
    fw.consumes = false;
    const uint32_t wptr = model.get(seg2, inbox1_wptr);
    model.set(seg2, inbox1_rptr, (wptr + 64u) % 8192u);
    const size_t writes = model.writes.size();
    checks.expect(dmub.execute(command, false, 5) == DmubResult::queue_full &&
                      model.writes.size() == writes && dmub.ready(),
                  "full ring refused without publishing");
    model.set(seg2, inbox1_rptr, wptr);
    // Fill to capacity - 64 without consumption.
    DmubResult last = DmubResult::success;
    uint32_t accepted = 0;
    for (uint32_t index = 0; index < 130 && last != DmubResult::quarantined; ++index) {
        last = dmub.execute(command, false, 5);
        accepted += last == DmubResult::timeout ? 1u : 0u;
    }
    checks.expect(last == DmubResult::quarantined && dmub.quarantined() && accepted == 1,
                  "unconsumed command times out and quarantines the ring");
    checks.expect(dmub.execute(command, false, 5) == DmubResult::quarantined, "quarantine sticky");
    checks.expect(dmub.stop() == DmubResult::success && !dmub.quarantined() && !dmub.ready() &&
                      (model.get(seg2, dmcub_cntl2) & 1u) == 1u,
                  "stop proves a soft reset");
}

void check_start_failures(Checks& checks) {
    const std::vector<uint8_t> image = firmware(0x1200, 0x400, true);
    std::vector<uint8_t> bios(0x10000, 0x5a);
    {
        dcn::Model model;
        Vram vram;
        Firmware fw;
        fw.vram = &vram;
        install(model, fw);
        model.set(seg2, 0x00cau, 0);
        Dcn32Dmub dmub;
        checks.expect(dmub.start(model.io(), dcn::space(), rx7900::ByteView(image.data(), image.size()),
                                 rx7900::ByteView(bios.data(), bios.size()), vram.memory()) ==
                          DmubResult::unsupported &&
                          model.writes.empty() && vram.flushes == 0,
                      "no DMCUB: nothing touched");
    }
    {
        dcn::Model model;
        Vram vram;
        Firmware fw;
        fw.vram = &vram;
        fw.boots = false;
        install(model, fw);
        Dcn32Dmub dmub;
        checks.expect(dmub.start(model.io(), dcn::space(), rx7900::ByteView(image.data(), image.size()),
                                 rx7900::ByteView(bios.data(), bios.size()), vram.memory()) ==
                          DmubResult::timeout &&
                          dmub.quarantined() && !dmub.ready(),
                      "no boot status: timeout and quarantine");
        checks.expect(model.delayed_us >= 100000u && model.delayed_us <= 100200u,
                      "wait_for_auto_load bounded to 100 ms");
    }
    {
        dcn::Model model;
        Vram vram;
        Firmware fw;
        fw.vram = &vram;
        fw.boots = false;
        install(model, fw);
        model.on_write = [](dcn::Model& m, uint32_t offset, uint32_t value) {
            if (offset == dcn::byte_offset(seg2, dmcub_cntl2) && (value & 1u) == 0) {
                m.set(seg2, scratch0, 0x1u); // dal_fw without mailbox_rdy
            }
        };
        Dcn32Dmub dmub;
        checks.expect(dmub.start(model.io(), dcn::space(), rx7900::ByteView(image.data(), image.size()),
                                 rx7900::ByteView(bios.data(), bios.size()), vram.memory()) ==
                          DmubResult::timeout,
                      "boot needs both dal_fw and mailbox_rdy");
    }
    {
        dcn::Model model;
        Vram vram;
        Firmware fw;
        fw.vram = &vram;
        install(model, fw);
        vram.fail_writes = true;
        Dcn32Dmub dmub;
        checks.expect(dmub.start(model.io(), dcn::space(), rx7900::ByteView(image.data(), image.size()),
                                 rx7900::ByteView(bios.data(), bios.size()), vram.memory()) ==
                          DmubResult::quarantined,
                      "VRAM write failure quarantines");
    }
    {
        dcn::Model model;
        Vram vram;
        Firmware fw;
        fw.vram = &vram;
        install(model, fw);
        DmubMemory small = vram.memory();
        small.bytes = 0x1000;
        Dcn32Dmub dmub;
        checks.expect(dmub.start(model.io(), dcn::space(), rx7900::ByteView(image.data(), image.size()),
                                 rx7900::ByteView(bios.data(), bios.size()), small) ==
                          DmubResult::invalid_argument &&
                          model.writes.empty(),
                      "undersized DMUB framebuffer refused");
        DmubMemory unaligned = vram.memory();
        unaligned.gpu_address += 0x100;
        checks.expect(dmub.start(model.io(), dcn::space(), rx7900::ByteView(image.data(), image.size()),
                                 rx7900::ByteView(bios.data(), bios.size()), unaligned) ==
                          DmubResult::invalid_argument,
                      "unaligned DMUB framebuffer refused");
    }
    {
        dcn::Model model;
        Vram vram;
        Firmware fw;
        fw.vram = &vram;
        install(model, fw);
        model.set(seg2, dmcub_cntl2, 1u); // already held in reset: no GPINT stop
        Dcn32Dmub dmub;
        checks.expect(dmub.start(model.io(), dcn::space(), rx7900::ByteView(image.data(), image.size()),
                                 rx7900::ByteView(bios.data(), bios.size()), vram.memory()) ==
                          DmubResult::success &&
                          model.written(seg2, gpint) == std::vector<uint32_t>{0u},
                      "DMCUB in reset is not sent STOP_FW");
        bool held = true;
        checks.expect(dmub_held_in_reset(model.io(), dcn::space(), held) && !held,
                      "released DMCUB reads as not held");
    }
}

// dcn-restore-modes (2026-10-24, additive): dc_dmub_srv_p_state_delegate's
// FAMS SETUP_FW_CTRL image (struct dmub_rb_cmd_fw_assisted_mclk_switch) and
// its trip through the inbox1 ring with the DM_DMUB_WAIT_TYPE_WAIT budget.
void check_fams(Checks& checks) {
    uint8_t command[dmub_command_bytes];
    for (uint8_t& byte : command) {
        byte = 0xa5;
    }
    dmub_build_fams_setup(false, command);
    bool zero_tail = true;
    for (uint32_t index = 5; index < dmub_command_bytes; ++index) {
        zero_tail = zero_tail && command[index] == 0;
    }
    checks.expect(command[0] == 76 && command[1] == 0 && command[2] == 0 && command[3] == 52 && command[4] == 0 &&
                      zero_tail,
                  "FAMS SETUP_FW_CTRL: type 76, sub type 0, 52 payload bytes, fams_enabled 0, rest zero");
    dmub_build_fams_setup(true, command);
    checks.expect(command[4] == 1, "fams_enabled follows should_manage_pstate");

    dcn::Model model;
    Vram vram;
    Firmware fw;
    fw.vram = &vram;
    install(model, fw);
    const std::vector<uint8_t> image = firmware(0x1200, 0x400, true);
    std::vector<uint8_t> bios(0x10000, 0x5a);
    Dcn32Dmub dmub;
    checks.expect(dmub.start(model.io(), dcn::space(), rx7900::ByteView(image.data(), image.size()),
                             rx7900::ByteView(bios.data(), bios.size()), vram.memory()) == DmubResult::success,
                  "DMCUB for the FAMS command");
    fw.mailbox = dmub.layout().windows[4].base;
    dmub_build_fams_setup(false, command);
    checks.expect(dmub.execute(command, false, dmub_command_wait_us) == DmubResult::success && fw.commands == 1,
                  "the delegate command is consumed within the 100 ms wait");
    checks.expect(vram.bytes[fw.mailbox] == 76 && vram.bytes[fw.mailbox + 3] == 52 && vram.bytes[fw.mailbox + 4] == 0,
                  "the ring slot holds the command image");
}

} // namespace

int main() {
    Checks checks;
    check_layout(checks);
    check_start(checks);
    check_start_failures(checks);
    check_fams(checks);
    return checks.finish("dcn32_dmub");
}

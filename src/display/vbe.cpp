/*
Module: rx7900.display.vbe
Purpose: Checked VBE 2.0/3.0 parsing, mode choice and the INT 10h mode-set sequence.
Created: 2026-10-22
Architecture: Pure decisions plus one ordered sequence over the VbeBios callbacks;
no OS dependency. Offsets: VBE Core 3.0 VbeInfoBlock (section 4.4) and
ModeInfoBlock (section 4.5), VBE/DDC 1.1 function 4F15h.
*/
#include "vbe.hpp"

namespace rx7900::display {
namespace {

constexpr uint16_t vbe_success = 0x004f;
constexpr uint16_t function_controller_info = 0x4f00;
constexpr uint16_t function_mode_info = 0x4f01;
constexpr uint16_t function_set_mode = 0x4f02;
constexpr uint16_t function_current_mode = 0x4f03;
constexpr uint16_t function_ddc = 0x4f15;
// AH=00h legacy set mode; AL bit 7 keeps video memory.
constexpr uint16_t function_legacy_set_mode = 0x0000;
constexpr uint8_t legacy_keep_memory = 0x80;
constexpr uint16_t ddc_read_edid = 0x0001;

// 4F02/4F03 mode word fields.
constexpr uint16_t mode_number_mask = 0x3fff;
constexpr uint16_t mode_linear = 0x4000;
constexpr uint16_t mode_keep_memory = 0x8000;
constexpr uint16_t vesa_mode_first = 0x0100;
constexpr uint16_t vesa_mode_last = 0x01ff;
constexpr uint16_t mode_list_end = 0xffff;

// VbeInfoBlock.
constexpr size_t info_signature = 0;
constexpr size_t info_version = 4;
constexpr size_t info_mode_pointer = 14;
constexpr size_t info_total_memory = 18;
constexpr uint64_t total_memory_unit = 64 * 1024;

// ModeInfoBlock.
constexpr size_t mode_attributes = 0;
constexpr size_t mode_bytes_per_line = 16;
constexpr size_t mode_width = 18;
constexpr size_t mode_height = 20;
constexpr size_t mode_planes = 24;
constexpr size_t mode_bits_per_pixel = 25;
constexpr size_t mode_memory_model = 27;
constexpr size_t mode_red_size = 31;
constexpr size_t mode_physical_base = 40;
constexpr size_t mode_linear_bytes_per_line = 50;
constexpr uint16_t attribute_supported = 0x0001;
constexpr uint16_t attribute_information = 0x0002;
constexpr uint16_t attribute_color = 0x0008;
constexpr uint16_t attribute_graphics = 0x0010;
constexpr uint16_t attribute_linear = 0x0080;
constexpr uint8_t memory_model_direct = 6;
constexpr uint64_t page_bytes = 4096;

constexpr uint32_t edid_extension_count_offset = 126;

uint32_t far_pointer_linear(uint32_t far_pointer) noexcept {
    const uint32_t segment = far_pointer >> 16;
    const uint32_t offset = far_pointer & 0xffffu;
    return segment * 16u + offset;
}

uint16_t low_word(uint32_t value) noexcept { return static_cast<uint16_t>(value & 0xffffu); }

void reset_report(VbeReport& report) noexcept {
    report = {};
    report.status = VbeStatus::invalid_argument;
    report.preference = {vbe_fallback_width, vbe_fallback_height, VbePreferenceSource::fallback};
}

class Sequence final {
public:
    Sequence(const VbeBios& bios, VbeReport& report) noexcept : bios_(bios), report_(report) {}

    // One INT 10h; for VBE functions AX must report 0x004f. Records the first failure.
    VbeStatus call(uint16_t function, VbeRegisters& registers) noexcept {
        registers.eax = function;

        if (!bios_.call(bios_.context, registers)) {
            return fail(function, 0, VbeStatus::bios_unavailable);
        }

        const uint16_t ax = low_word(registers.eax);

        if (function >= function_controller_info && ax != vbe_success) {
            return fail(function, ax, VbeStatus::function_failed);
        }

        return VbeStatus::mode_set;
    }

    VbeRegisters at_scratch(uint32_t offset) const noexcept {
        VbeRegisters registers{};
        const uint32_t linear = bios_.buffer_linear + offset;
        registers.es = static_cast<uint16_t>(linear >> 4);
        registers.edi = linear & 0xfu;
        return registers;
    }

    bool read(uint32_t linear, uint8_t* output, uint32_t bytes) noexcept {
        return bios_.read(bios_.context, linear, output, bytes);
    }

    bool write(uint32_t linear, const uint8_t* input, uint32_t bytes) noexcept {
        return bios_.write(bios_.context, linear, input, bytes);
    }

    uint32_t scratch(uint32_t offset) const noexcept { return bios_.buffer_linear + offset; }

    VbeStatus fail(uint16_t function, uint16_t ax, VbeStatus status) noexcept {
        if (report_.failed_function == 0 && report_.failed_ax == 0) {
            report_.failed_function = function;
            report_.failed_ax = ax;
        }

        return status;
    }

private:
    const VbeBios& bios_;
    VbeReport& report_;
};

bool valid_bios(const VbeBios& bios) noexcept {
    return bios.call != nullptr && bios.read != nullptr && bios.write != nullptr &&
           bios.buffer_bytes >= vbe_scratch_bytes && bios.buffer_linear < vbe_real_mode_limit &&
           bios.buffer_bytes <= vbe_real_mode_limit - bios.buffer_linear;
}

// VBE/DDC: block 0, then the extensions it declares. False leaves the sink unused.
bool read_edid(Sequence& sequence, VbeWorkspace& workspace) noexcept {
    uint32_t blocks = 1;

    for (uint32_t block = 0; block < blocks; ++block) {
        VbeRegisters registers = sequence.at_scratch(vbe_edid_offset);
        registers.ebx = ddc_read_edid;
        registers.ecx = 0;
        registers.edx = block;

        if (sequence.call(function_ddc, registers) != VbeStatus::mode_set ||
            !sequence.read(sequence.scratch(vbe_edid_offset), &workspace.edid[block * output::edid_block_bytes],
                           output::edid_block_bytes)) {
            return false;
        }

        if (block == 0) {
            blocks = 1u + workspace.edid[edid_extension_count_offset];

            if (blocks > output::maximum_edid_blocks) {
                return false;
            }
        }
    }

    return output::parse_edid(ByteView(workspace.edid, blocks * output::edid_block_bytes), workspace.sink) ==
           output::Status::success;
}

// Requests the mode found by 4F03 again. True when the BIOS reported success.
bool restore_previous(Sequence& sequence, uint16_t previous) noexcept {
    VbeRegisters registers{};

    if (previous < vesa_mode_first) {
        return sequence.call(static_cast<uint16_t>(function_legacy_set_mode |
                                                   ((previous & 0x7fu) | legacy_keep_memory)),
                             registers) == VbeStatus::mode_set;
    }

    registers.ebx = static_cast<uint16_t>((previous & mode_number_mask) | mode_keep_memory);
    return sequence.call(function_set_mode, registers) == VbeStatus::mode_set;
}

} // namespace

VbeStatus parse_vbe_controller_info(ByteView block, VbeControllerInfo& output) noexcept {
    uint8_t signature[4]{};
    uint16_t version = 0;
    uint32_t mode_pointer = 0;
    uint16_t total_memory = 0;

    if (!block.contains(0, vbe_controller_info_bytes) || !block.read_u8(info_signature, signature[0]) ||
        !block.read_u8(info_signature + 1, signature[1]) || !block.read_u8(info_signature + 2, signature[2]) ||
        !block.read_u8(info_signature + 3, signature[3]) || !block.read_u16(info_version, version) ||
        !block.read_u32(info_mode_pointer, mode_pointer) || !block.read_u16(info_total_memory, total_memory)) {
        return VbeStatus::malformed_controller_info;
    }

    if (signature[0] != 'V' || signature[1] != 'E' || signature[2] != 'S' || signature[3] != 'A') {
        return VbeStatus::malformed_controller_info;
    }

    if (version < vbe_minimum_version || version > vbe_maximum_version) {
        return VbeStatus::unsupported_version;
    }

    const uint32_t list = far_pointer_linear(mode_pointer);

    // The list needs at least its terminator inside real-mode memory.
    if (mode_pointer == 0 || list > vbe_real_mode_limit - 2 || total_memory == 0) {
        return VbeStatus::malformed_controller_info;
    }

    output = {version, list, total_memory * total_memory_unit};
    return VbeStatus::mode_set;
}

VbeStatus parse_vbe_mode_list(ByteView list, uint16_t* modes, uint32_t capacity, uint32_t& count) noexcept {
    count = 0;

    if (modes == nullptr || capacity == 0) {
        return VbeStatus::invalid_argument;
    }

    uint32_t stored = 0;

    for (uint32_t entry = 0; entry < capacity; ++entry) {
        uint16_t mode = 0;

        if (!list.read_u16(static_cast<size_t>(entry) * 2u, mode)) {
            break;
        }

        if (mode == mode_list_end) {
            count = stored;
            return VbeStatus::mode_set;
        }

        if (mode >= vesa_mode_first && mode <= vesa_mode_last) {
            modes[stored] = mode;
            ++stored;
        }
    }

    return VbeStatus::malformed_mode_list;
}

VbeModeStatus parse_vbe_mode_info(ByteView block, uint16_t version, uint16_t mode,
                                  const VbeAperture& aperture, uint64_t total_memory_bytes,
                                  VbeModeInfo& output) noexcept {
    uint16_t attributes = 0;
    uint16_t bytes_per_line = 0;
    uint16_t linear_bytes_per_line = 0;
    uint16_t width = 0;
    uint16_t height = 0;
    uint8_t planes = 0;
    uint8_t bits = 0;
    uint8_t model = 0;
    uint8_t fields[8]{};
    uint32_t base = 0;

    if (!block.contains(0, vbe_mode_info_bytes) || !block.read_u16(mode_attributes, attributes) ||
        !block.read_u16(mode_bytes_per_line, bytes_per_line) || !block.read_u16(mode_width, width) ||
        !block.read_u16(mode_height, height) || !block.read_u8(mode_planes, planes) ||
        !block.read_u8(mode_bits_per_pixel, bits) || !block.read_u8(mode_memory_model, model) ||
        !block.read_u32(mode_physical_base, base) ||
        !block.read_u16(mode_linear_bytes_per_line, linear_bytes_per_line)) {
        return VbeModeStatus::malformed;
    }

    for (uint32_t index = 0; index < 8; ++index) {
        block.read_u8(mode_red_size + index, fields[index]);
    }

    const uint16_t required = attribute_supported | attribute_information | attribute_color | attribute_graphics;

    if ((attributes & required) != required) {
        return VbeModeStatus::not_supported;
    }

    if ((attributes & attribute_linear) == 0 || base == 0) {
        return VbeModeStatus::no_linear_buffer;
    }

    // Red 8 at 16, green 8 at 8, blue 8 at 0; reserved absent or 8 at 24.
    const bool reserved_ok = (fields[6] == 0 && fields[7] == 0) || (fields[6] == 8 && fields[7] == 24);

    if (planes != 1 || bits != 32 || model != memory_model_direct || fields[0] != 8 || fields[1] != 16 ||
        fields[2] != 8 || fields[3] != 8 || fields[4] != 8 || fields[5] != 0 || !reserved_ok) {
        return VbeModeStatus::unsupported_format;
    }

    const uint32_t stride = version >= 0x0300 ? linear_bytes_per_line : bytes_per_line;

    if (width < vbe_minimum_width || height < vbe_minimum_height || width > vbe_maximum_dimension ||
        height > vbe_maximum_dimension || stride < static_cast<uint32_t>(width) * 4u || stride % 4u != 0) {
        return VbeModeStatus::unsupported_geometry;
    }

    const uint64_t bytes = static_cast<uint64_t>(stride) * height;

    if (base % page_bytes != 0 || base < aperture.base || aperture.bytes == 0 ||
        base - aperture.base > aperture.bytes || bytes > aperture.bytes - (base - aperture.base) ||
        bytes > total_memory_bytes) {
        return VbeModeStatus::outside_aperture;
    }

    output = {mode, width, height, stride, base, bytes};
    return VbeModeStatus::usable;
}

void vbe_preference_from_sink(const output::SinkSnapshot& sink, bool sink_valid, VbePreference& output) noexcept {
    output = {vbe_fallback_width, vbe_fallback_height, VbePreferenceSource::fallback};
    Timing timing{};

    if (!sink_valid || sink.preferred_mode == output::no_preferred_mode ||
        sink.detailed_modes.get(sink.preferred_mode, timing) != Status::success ||
        (timing.flags & interlaced) != 0 || timing.width == 0 || timing.height == 0) {
        return;
    }

    output = {timing.width, timing.height, VbePreferenceSource::edid_preferred};
}

bool vbe_mode_preferred(const VbeModeInfo& candidate, bool has_best, const VbeModeInfo& best,
                        const VbePreference& preference) noexcept {
    if (candidate.width > preference.width || candidate.height > preference.height) {
        return false;
    }

    if (!has_best) {
        return true;
    }

    const bool candidate_exact = candidate.width == preference.width && candidate.height == preference.height;
    const bool best_exact = best.width == preference.width && best.height == preference.height;

    if (candidate_exact != best_exact) {
        return candidate_exact;
    }

    const uint64_t candidate_area = static_cast<uint64_t>(candidate.width) * candidate.height;
    const uint64_t best_area = static_cast<uint64_t>(best.width) * best.height;

    if (candidate_area != best_area) {
        return candidate_area > best_area;
    }

    return candidate.width > best.width;
}

VbeStatus set_vbe_linear_mode(const VbeBios& bios, const VbeAperture& aperture, VbeWorkspace& workspace,
                              VbeReport& report) noexcept {
    reset_report(report);

    if (!valid_bios(bios) || aperture.bytes == 0) {
        return report.status;
    }

    Sequence sequence(bios, report);
    auto finish = [&report](VbeStatus status) noexcept {
        report.status = status;
        return status;
    };

    // 1. The current mode. A linear mode is left alone; so is a BIOS without VBE.
    VbeRegisters current{};
    VbeStatus status = sequence.call(function_current_mode, current);

    if (status != VbeStatus::mode_set) {
        return finish(status);
    }

    report.previous_mode = low_word(current.ebx);

    if ((report.previous_mode & mode_linear) != 0) {
        return finish(VbeStatus::already_linear);
    }

    // 2. Controller information, requesting the VBE 2.0+ block.
    workspace.controller[0] = 'V';
    workspace.controller[1] = 'B';
    workspace.controller[2] = 'E';
    workspace.controller[3] = '2';

    for (uint32_t index = 4; index < vbe_controller_info_bytes; ++index) {
        workspace.controller[index] = 0;
    }

    VbeRegisters info = sequence.at_scratch(0);

    if (!sequence.write(sequence.scratch(0), workspace.controller, vbe_controller_info_bytes)) {
        return finish(sequence.fail(function_controller_info, 0, VbeStatus::bios_unavailable));
    }

    status = sequence.call(function_controller_info, info);

    if (status != VbeStatus::mode_set) {
        return finish(status);
    }

    if (!sequence.read(sequence.scratch(0), workspace.controller, vbe_controller_info_bytes)) {
        return finish(sequence.fail(function_controller_info, 0, VbeStatus::bios_unavailable));
    }

    VbeControllerInfo controller{};
    status = parse_vbe_controller_info(ByteView(workspace.controller, vbe_controller_info_bytes), controller);

    if (status != VbeStatus::mode_set) {
        return finish(status);
    }

    report.version = controller.version;

    // 3. The mode list, copied before any further call can overwrite it.
    const uint32_t available = vbe_real_mode_limit - controller.mode_list_linear;
    const uint32_t list_bytes = available < sizeof(workspace.list) ? available : sizeof(workspace.list);
    uint32_t mode_count = 0;

    if (!sequence.read(controller.mode_list_linear, workspace.list, list_bytes)) {
        return finish(sequence.fail(function_controller_info, 0, VbeStatus::bios_unavailable));
    }

    status = parse_vbe_mode_list(ByteView(workspace.list, list_bytes), workspace.modes, vbe_mode_list_capacity,
                                 mode_count);

    if (status != VbeStatus::mode_set) {
        return finish(status);
    }

    report.listed_modes = mode_count;

    // 4. The sink preference; an unreadable EDID is not an error. parse_edid
    // writes the sink only on success, so a stale sink is never consulted.
    report.edid_read = read_edid(sequence, workspace);
    report.failed_function = 0;
    report.failed_ax = 0;
    vbe_preference_from_sink(workspace.sink, report.edid_read, report.preference);

    // 5. Every listed mode; keep the best usable one.
    bool chosen = false;

    for (uint32_t index = 0; index < mode_count; ++index) {
        VbeRegisters query = sequence.at_scratch(vbe_mode_info_offset);
        query.ecx = workspace.modes[index];

        for (uint32_t byte = 0; byte < vbe_mode_info_bytes; ++byte) {
            workspace.mode_block[byte] = 0;
        }

        if (!sequence.write(sequence.scratch(vbe_mode_info_offset), workspace.mode_block, vbe_mode_info_bytes)) {
            return finish(sequence.fail(function_mode_info, 0, VbeStatus::bios_unavailable));
        }

        status = sequence.call(function_mode_info, query);

        if (status == VbeStatus::bios_unavailable) {
            return finish(status);
        }

        VbeModeStatus mode_status = VbeModeStatus::not_supported;
        VbeModeInfo mode{};

        if (status == VbeStatus::mode_set) {
            if (!sequence.read(sequence.scratch(vbe_mode_info_offset), workspace.mode_block, vbe_mode_info_bytes)) {
                return finish(sequence.fail(function_mode_info, 0, VbeStatus::bios_unavailable));
            }

            mode_status = parse_vbe_mode_info(ByteView(workspace.mode_block, vbe_mode_info_bytes), controller.version,
                                              workspace.modes[index], aperture, controller.total_memory_bytes, mode);
        }

        if (mode_status == VbeModeStatus::usable &&
            (mode.width > report.preference.width || mode.height > report.preference.height)) {
            mode_status = VbeModeStatus::above_preference;
        }

        ++report.refusals[static_cast<uint32_t>(mode_status)];

        if (mode_status == VbeModeStatus::usable && vbe_mode_preferred(mode, chosen, report.chosen, report.preference)) {
            report.chosen = mode;
            chosen = true;
        }
    }

    // Individual 4F01 failures only disqualify their mode.
    report.failed_function = 0;
    report.failed_ax = 0;

    if (!chosen) {
        return finish(VbeStatus::no_usable_mode);
    }

    // 6. The only state change: the chosen mode with its linear framebuffer.
    VbeRegisters set{};
    set.ebx = static_cast<uint16_t>(report.chosen.mode | mode_linear);

    if (sequence.call(function_set_mode, set) != VbeStatus::mode_set) {
        report.restored = restore_previous(sequence, report.previous_mode);
        return finish(VbeStatus::set_failed);
    }

    VbeRegisters confirm{};

    if (sequence.call(function_current_mode, confirm) != VbeStatus::mode_set ||
        (low_word(confirm.ebx) & (mode_number_mask | mode_linear)) != (report.chosen.mode | mode_linear)) {
        report.restored = restore_previous(sequence, report.previous_mode);
        return finish(VbeStatus::verify_failed);
    }

    return finish(VbeStatus::mode_set);
}

} // namespace rx7900::display

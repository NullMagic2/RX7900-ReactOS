/*
Module: rx7900.opengl.gfx11
Purpose: Checked AMD PM4 address registers and nonindexed triangle commands.
Created: 2026-10-21
Architecture: Reuses the managed GPUVA owner and bounded SDMA byte writer.
Adapted from Linux v6.12 soc15d.h and Mesa 22.3.6 RadeonSI draw/shader code.
Copyright 2011, 2012, 2014 Advanced Micro Devices, Inc.
Preserved permission notices and pinned source hashes: licenses/OPENGL.txt.
*/
#include "gfx11.hpp"

namespace rx7900::opengl::gfx11 {
namespace {

constexpr uint32_t packet3_type = 3u << 30;
constexpr uint32_t set_sh_reg = 0x76;
constexpr uint32_t set_uconfig_reg = 0x79;
constexpr uint32_t num_instances = 0x2f;
constexpr uint32_t draw_index_auto = 0x2d;
constexpr uint32_t shader_register_base = 0xb000;
constexpr uint32_t uconfig_register_base = 0x30000;
constexpr uint32_t program_lo_es = 0xb320;
constexpr uint32_t program_lo_ps = 0xb020;
constexpr uint32_t user_data_gs = 0xb230;
constexpr uint32_t user_data_ps = 0xb030;
constexpr uint32_t vgt_primitive_type = 0x30908;
constexpr uint32_t primitive_restart_enable = 0x3092c;
constexpr uint32_t triangle_list = 4;
constexpr uint32_t auto_index = 2;
constexpr uint32_t program_alignment = 256;
constexpr uint32_t maximum_user_sgprs = 32;

constexpr uint32_t packet3(uint32_t opcode, uint32_t count) noexcept {
    return packet3_type | (count << 16) | (opcode << 8);
}

bool known_stage(Stage stage) noexcept {
    return stage == Stage::primitive_ngg || stage == Stage::pixel;
}

Status shader_words(const memory::Manager& manager, const ShaderCode& shader,
                    uint32_t (&words)[4]) noexcept {
    if (!known_stage(shader.stage)) { return Status::invalid_stage; }
    if (shader.padded_byte_count == 0 ||
        (shader.padded_byte_count & (program_alignment-1u)) != 0) {
        return Status::invalid_shader_size;
    }
    sdma::GpuRange range{};
    if (manager.resolve(shader.code, shader.padded_byte_count, memory::Access::read, range) !=
        memory::Status::success) { return Status::invalid_memory; }
    const uint64_t address = range.allocation.virtual_address + range.byte_offset;
    if ((address & (program_alignment-1u)) != 0) { return Status::invalid_alignment; }
    const uint32_t reg = shader.stage == Stage::primitive_ngg ? program_lo_es : program_lo_ps;
    words[0] = packet3(set_sh_reg, 2);
    words[1] = (reg-shader_register_base)/sizeof(uint32_t);
    words[2] = static_cast<uint32_t>(address >> 8);
    words[3] = static_cast<uint32_t>(address >> 40);
    return Status::success;
}

Status pointer_words(const memory::Manager& manager, const ShaderPointer& pointer,
                     uint32_t (&words)[4]) noexcept {
    if (!known_stage(pointer.stage)) { return Status::invalid_stage; }
    if (pointer.compiled_user_sgpr_count < 2 ||
        pointer.compiled_user_sgpr_count > maximum_user_sgprs ||
        pointer.first_user_sgpr > pointer.compiled_user_sgpr_count-2) {
        return Status::invalid_shader_abi;
    }
    sdma::GpuRange range{};
    if (manager.resolve(pointer.data, pointer.byte_count, memory::Access::read, range) !=
        memory::Status::success) { return Status::invalid_memory; }
    const uint64_t address = range.allocation.virtual_address + range.byte_offset;
    if ((address & (sizeof(uint32_t)-1u)) != 0) { return Status::invalid_alignment; }
    const uint32_t reg = pointer.stage == Stage::primitive_ngg ? user_data_gs : user_data_ps;
    words[0] = packet3(set_sh_reg, 2);
    words[1] = (reg-shader_register_base)/sizeof(uint32_t) + pointer.first_user_sgpr;
    words[2] = static_cast<uint32_t>(address);
    words[3] = static_cast<uint32_t>(address >> 32);
    return Status::success;
}

Status draw_words(const TriangleDraw& draw, uint32_t (&words)[11]) noexcept {
    if (draw.vertex_count == 0 || draw.vertex_count%3 != 0 || draw.instance_count == 0) {
        return Status::invalid_draw;
    }
    words[0] = packet3(set_uconfig_reg, 1);
    words[1] = (vgt_primitive_type-uconfig_register_base)/sizeof(uint32_t);
    words[2] = triangle_list;
    words[3] = packet3(set_uconfig_reg, 1);
    words[4] = (primitive_restart_enable-uconfig_register_base)/sizeof(uint32_t);
    words[5] = 0;
    words[6] = packet3(num_instances, 0);
    words[7] = draw.instance_count;
    words[8] = packet3(draw_index_auto, 1);
    words[9] = draw.vertex_count;
    words[10] = auto_index;
    return Status::success;
}

template<size_t Count>
void write_words(sdma::CommandBuffer output, size_t byte_offset,
                 const uint32_t (&words)[Count]) noexcept {
    // The public encoders preflight the complete output before the first store.
    for (size_t index = 0; index < Count; ++index) {
        output.write_u32(byte_offset + index*sizeof(uint32_t), words[index]);
    }
}

template<size_t Count>
Status finish(sdma::CommandBuffer output, const uint32_t (&words)[Count],
              sdma::EncodeResult& result) noexcept {
    result.bytes_required = Count*sizeof(uint32_t);
    if (output.size() < result.bytes_required) { return Status::output_too_small; }
    write_words(output, 0, words);
    result.bytes_written = result.bytes_required;
    return Status::success;
}

} // namespace

Status encode_shader_address(const memory::Manager& manager, const ShaderCode& shader,
                             sdma::CommandBuffer output, sdma::EncodeResult& result) noexcept {
    result = {};
    uint32_t words[4]{};
    const Status status = shader_words(manager, shader, words);
    return status == Status::success ? finish(output, words, result) : status;
}

Status encode_shader_pointer(const memory::Manager& manager, const ShaderPointer& pointer,
                             sdma::CommandBuffer output, sdma::EncodeResult& result) noexcept {
    result = {};
    uint32_t words[4]{};
    const Status status = pointer_words(manager, pointer, words);
    return status == Status::success ? finish(output, words, result) : status;
}

Status encode_triangle_draw(const TriangleDraw& draw, sdma::CommandBuffer output,
                            sdma::EncodeResult& result) noexcept {
    result = {};
    uint32_t words[11]{};
    const Status status = draw_words(draw, words);
    return status == Status::success ? finish(output, words, result) : status;
}

Status encode_triangle_commands(const memory::Manager& manager,
                                const TriangleCommands& commands, sdma::CommandBuffer output,
                                sdma::EncodeResult& result) noexcept {
    result = {};
    if (commands.primitive_shader.stage != Stage::primitive_ngg ||
        commands.pixel_shader.stage != Stage::pixel ||
        commands.vertex_pointer.stage != Stage::primitive_ngg) { return Status::invalid_stage; }
    if (commands.vertex_stride_bytes < 4*sizeof(float) ||
        commands.vertex_stride_bytes%sizeof(uint32_t) != 0 ||
        commands.vertex_pointer.byte_count <
            static_cast<uint64_t>(commands.draw.vertex_count)*commands.vertex_stride_bytes) {
        return Status::invalid_draw;
    }
    uint32_t primitive[4]{}, pixel[4]{}, pointer[4]{}, draw[11]{};
    Status status = shader_words(manager, commands.primitive_shader, primitive);
    if (status != Status::success) { return status; }
    status = shader_words(manager, commands.pixel_shader, pixel);
    if (status != Status::success) { return status; }
    status = pointer_words(manager, commands.vertex_pointer, pointer);
    if (status != Status::success) { return status; }
    status = draw_words(commands.draw, draw);
    if (status != Status::success) { return status; }
    result.bytes_required = triangle_commands_bytes;
    if (output.size() < result.bytes_required) { return Status::output_too_small; }
    write_words(output, 0, primitive);
    write_words(output, shader_address_bytes, pixel);
    write_words(output, 2*shader_address_bytes, pointer);
    write_words(output, 2*shader_address_bytes+shader_pointer_bytes, draw);
    result.bytes_written = result.bytes_required;
    return Status::success;
}

} // namespace rx7900::opengl::gfx11

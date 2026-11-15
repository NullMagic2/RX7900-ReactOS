/*
Module: rx7900.output.dcn32_aux
Purpose: Bounded DCN3.2 software AUX transfer and engine arbitration.
Created: 2026-10-22
Architecture: Exact v6.12 DCN3.2 offsets/masks and dce_aux.c order; no
clock, DDC GPIO, DMUB or display-stream ownership is inferred here.
Copyright 2012-15, 2022 Advanced Micro Devices, Inc.
Linux notice and provenance: licenses/AMDGPU-OUTPUT.txt.
*/
#include "dcn32_aux.hpp"

namespace rx7900::output {
namespace {

constexpr uint8_t engine_count = 5;
constexpr uint32_t engine_stride = 0x1cu;
constexpr uint32_t aux_control = 0x1f50u;
constexpr uint32_t sw_control = 0x1f51u;
constexpr uint32_t arb_control = 0x1f52u;
constexpr uint32_t interrupt_control = 0x1f53u;
constexpr uint32_t sw_status = 0x1f54u;
constexpr uint32_t sw_data = 0x1f56u;
constexpr uint32_t aux_enabled = 0x1u;
constexpr uint32_t aux_reset = 0x10u;
constexpr uint32_t aux_reset_done = 0x20u;
constexpr uint32_t ignore_hpd = 0x10000u;
constexpr uint32_t hpd_select_mask = 0x700000u;
constexpr uint32_t arb_status_mask = 0xcu;
constexpr uint32_t sw_access = 1u;
constexpr uint32_t dmub_access = 2u;
constexpr uint32_t sw_request = 0x10000u;
constexpr uint32_t sw_done_using = 0x20000u;
constexpr uint32_t sw_done_ack = 0x2u;
constexpr uint32_t other_ack_bits = 0x2220u;
constexpr uint32_t sw_done = 0x1u;
constexpr uint32_t sw_go = 0x1u;
constexpr uint32_t sw_start_delay_mask = 0xf0u;
constexpr uint32_t sw_write_bytes_mask = 0x1f0000u;
constexpr uint32_t data_control_mask = 0x801fff01u;
constexpr uint32_t reply_byte_count_mask = 0x1f000000u;
constexpr uint32_t receive_errors = 0x00de4ff0u;
constexpr uint32_t acquisition_polls = 50;
constexpr uint32_t reset_polls = 11;
constexpr uint32_t clear_polls = 50;
constexpr uint32_t completion_polls = 700;
constexpr uint32_t poll_delay_microseconds = 10;

struct AuxRegisters final {
    uint32_t control = 0;
    uint32_t sw_control = 0;
    uint32_t arb = 0;
    uint32_t interrupt = 0;
    uint32_t status = 0;
    uint32_t data = 0;
};

bool address(const display::Dcn32RegisterSpace& space, uint8_t engine,
             uint32_t index, uint32_t& output) noexcept {
    const uint64_t byte = (static_cast<uint64_t>(space.segment2_dword_base) +
                           index + static_cast<uint64_t>(engine) * engine_stride) * 4u;
    if (byte > UINT32_MAX || byte > space.aperture_bytes ||
        sizeof(uint32_t) > space.aperture_bytes - byte) {
        return false;
    }
    output = static_cast<uint32_t>(byte);
    return true;
}

bool resolve(const display::Dcn32RegisterSpace& space, uint8_t engine,
             AuxRegisters& regs) noexcept {
    return address(space, engine, aux_control, regs.control) &&
           address(space, engine, sw_control, regs.sw_control) &&
           address(space, engine, arb_control, regs.arb) &&
           address(space, engine, interrupt_control, regs.interrupt) &&
           address(space, engine, sw_status, regs.status) &&
           address(space, engine, sw_data, regs.data);
}

bool read(Dcn32AuxIo io, uint32_t offset, uint32_t& value) noexcept {
    return io.read(io.context, offset, value) && value != UINT32_MAX;
}

bool write(Dcn32AuxIo io, uint32_t offset, uint32_t value) noexcept {
    return io.write(io.context, offset, value);
}

bool wait_field(Dcn32AuxIo io, uint32_t offset, uint32_t mask,
                uint32_t expected, uint32_t attempts) noexcept {
    for (uint32_t index = 0; index < attempts; ++index) {
        uint32_t value = 0;
        if (!read(io, offset, value)) {
            return false;
        }
        if ((value & mask) == expected) {
            return true;
        }
        if (index + 1u != attempts &&
            !io.wait_microseconds(io.context, poll_delay_microseconds)) {
            return false;
        }
    }
    return false;
}

bool release_engine(Dcn32AuxIo io, const AuxRegisters& regs) noexcept {
    uint32_t value = 0;
    if (!read(io, regs.arb, value)) {
        return false;
    }
    value = (value & ~(sw_request | sw_done_using)) | sw_done_using;
    if (!write(io, regs.arb, value)) {
        return false;
    }
    for (uint32_t index = 0; index < acquisition_polls; ++index) {
        if (!read(io, regs.arb, value)) {
            return false;
        }
        if ((value & sw_request) == 0 &&
            ((value & arb_status_mask) >> 2) != sw_access) {
            return true;
        }
        if (index + 1u != acquisition_polls &&
            !io.wait_microseconds(io.context, poll_delay_microseconds)) {
            return false;
        }
    }
    return false;
}

bool enable_engine(Dcn32AuxIo io, const AuxRegisters& regs,
                   uint32_t control) noexcept {
    if ((control & aux_enabled) != 0) {
        return true;
    }
    // Linux v6.12 dce_aux acquire_engine enables AUX, pulses reset and waits
    // for RESET_DONE on both edges. The caller already proved route, clocks,
    // DMUB arbitration and exclusive ownership before entering this path.
    const uint32_t enabled = (control & ~aux_reset_done) | aux_enabled;
    if (!write(io, regs.control, enabled | aux_reset) ||
        !wait_field(io, regs.control, aux_reset_done,
                    aux_reset_done, reset_polls) ||
        !write(io, regs.control, enabled) ||
        !wait_field(io, regs.control, aux_reset_done, 0, reset_polls)) {
        return false;
    }
    uint32_t verified = 0;
    return read(io, regs.control, verified) &&
           (verified & (aux_enabled | aux_reset | aux_reset_done |
                        ignore_hpd | hpd_select_mask)) ==
               (enabled & (aux_enabled | hpd_select_mask));
}

bool valid_request(const AuxRequest& request) noexcept {
    if (request.operation != AuxOperation::native_read &&
        request.operation != AuxOperation::native_write &&
        request.operation != AuxOperation::i2c_read &&
        request.operation != AuxOperation::i2c_write) {
        return false;
    }
    const bool i2c = request.operation == AuxOperation::i2c_read ||
                     request.operation == AuxOperation::i2c_write;
    const bool reading = request.operation == AuxOperation::native_read ||
                         request.operation == AuxOperation::i2c_read;
    return request.bytes <= dp_aux_payload_bytes &&
           request.address <= (i2c ? 0x7fu : 0xfffffu) &&
           (request.bytes == 0 || (reading ? request.read_data != nullptr :
                                             request.write_data != nullptr)) &&
           (i2c || request.bytes != 0) &&
           (i2c || !request.middle_of_transaction);
}

uint8_t request_action(const AuxRequest& request) noexcept {
    switch (request.operation) {
    case AuxOperation::native_read: return 0x9u;
    case AuxOperation::native_write: return 0x8u;
    case AuxOperation::i2c_read:
        return static_cast<uint8_t>(0x1u | (request.middle_of_transaction ? 0x4u : 0));
    case AuxOperation::i2c_write:
        return static_cast<uint8_t>(request.middle_of_transaction ? 0x4u : 0);
    }
    return 0xffu;
}

bool stage_request(Dcn32AuxIo io, const AuxRegisters& regs,
                   const AuxRequest& request) noexcept {
    uint32_t control = 0;
    uint32_t data = 0;
    if (!read(io, regs.sw_control, control) || !read(io, regs.data, data) ||
        (control & sw_go) != 0) {
        return false;
    }

    const bool writing = request.operation == AuxOperation::native_write ||
                         request.operation == AuxOperation::i2c_write;
    const uint32_t length = (request.bytes == 0 ? 3u : 4u) +
                            (writing ? request.bytes : 0u);
    control = (control & ~(sw_start_delay_mask | sw_write_bytes_mask | sw_go)) |
              (length << 16);
    if (!write(io, regs.sw_control, control)) {
        return false;
    }

    const uint8_t header[]{
        static_cast<uint8_t>((request_action(request) << 4) |
                             ((request.address >> 16) & 0xfu)),
        static_cast<uint8_t>(request.address >> 8),
        static_cast<uint8_t>(request.address),
        static_cast<uint8_t>(request.bytes == 0 ? 0 : request.bytes - 1u)
    };
    const uint32_t header_bytes = request.bytes == 0 ? 3u : 4u;
    const uint32_t count = header_bytes + (writing ? request.bytes : 0u);
    const uint32_t base = data & ~data_control_mask;
    for (uint32_t index = 0; index < count; ++index) {
        const uint8_t byte = index < header_bytes ? header[index] :
                             request.write_data[index - header_bytes];
        const uint32_t value = base | (index == 0 ? 0x80000000u : 0) |
                               (static_cast<uint32_t>(byte) << 8);
        if (!write(io, regs.data, value)) {
            return false;
        }
    }

    return write(io, regs.sw_control, control | sw_go);
}

bool read_reply(Dcn32AuxIo io, const AuxRegisters& regs,
                const AuxRequest& request, AuxReply& reply) noexcept {
    uint32_t status = 0;
    uint32_t data = 0;
    if (!read(io, regs.status, status) || (status & sw_done) == 0 ||
        (status & receive_errors) != 0) {
        return false;
    }

    const uint32_t count = (status & reply_byte_count_mask) >> 24;
    if (count == 0 || count > static_cast<uint32_t>(request.bytes) + 1u ||
        !read(io, regs.data, data)) {
        return false;
    }
    const uint32_t base = data & ~data_control_mask;
    if (!write(io, regs.data, base | 0x80000001u) ||
        !read(io, regs.data, data)) {
        return false;
    }

    const uint8_t result = static_cast<uint8_t>((data >> 12) & 0xfu);
    if ((result & 0xau) != 0) {
        reply.result = AuxResult::defer;
    } else if ((result & 0x5u) != 0) {
        reply.result = AuxResult::nack;
    } else if (result == 0) {
        reply.result = AuxResult::ack;
    } else {
        return false;
    }

    if (reply.result != AuxResult::ack) {
        reply.bytes = 0;
        return true;
    }

    const bool reading = request.operation == AuxOperation::native_read ||
                         request.operation == AuxOperation::i2c_read;
    if (reading) {
        const uint32_t received = count - 1u;
        // Index zero is the status byte. Select index one explicitly so the
        // first payload read cannot repeat the status byte.
        if (!write(io, regs.data, base | 0x10001u)) {
            return false;
        }
        for (uint32_t index = 0; index < received; ++index) {
            if (!read(io, regs.data, data)) {
                return false;
            }
            request.read_data[index] = static_cast<uint8_t>((data >> 8) & 0xffu);
        }
        reply.bytes = static_cast<uint8_t>(received);
    } else {
        reply.bytes = request.bytes;
    }
    return true;
}

} // namespace

bool Dcn32AuxEngine::transfer(const AuxRequest& request, AuxReply& reply,
                              const Dcn32AuxPrerequisites& prerequisites) noexcept {
    reply = {};
    if (quarantined_ || !valid_request(request) ||
        !prerequisites.exclusive_register_lease ||
        !prerequisites.exact_ddc_route_in_aux_mode ||
        !prerequisites.aux_clock_and_phy_ready ||
        !prerequisites.dmub_arbitration_owned ||
        !space_.discovery_verified || space_.dmu_major != 3 ||
        space_.dmu_minor != 2 || space_.dmu_revision != 0 ||
        engine_ >= engine_count || hpd_pin_ >= engine_count ||
        io_.read == nullptr || io_.write == nullptr ||
        io_.wait_microseconds == nullptr || io_.connected == nullptr) {
        return false;
    }

    AuxRegisters regs{};
    if (!resolve(space_, engine_, regs) || !io_.connected(io_.context)) {
        return false;
    }

    uint32_t control = 0;
    uint32_t arb = 0;
    if (!read(io_, regs.control, control) || !read(io_, regs.arb, arb) ||
        (control & (aux_reset | ignore_hpd)) != 0 ||
        ((control & hpd_select_mask) >> 20) != hpd_pin_ ||
        ((arb & arb_status_mask) >> 2) == dmub_access ||
        (arb & sw_request) != 0) {
        return false;
    }

    if ((control & aux_enabled) == 0 && !enable_engine(io_, regs, control)) {
        // A failed MMIO write can already have changed the engine. Do not
        // publish another transaction without an actual reset proof.
        quarantined_ = true;
        return false;
    }

    bool claimed = false;
    bool submitted = false;
    bool completed = false;
    bool success = false;
    do {
        claimed = true;
        if (!write(io_, regs.arb, (arb & ~sw_done_using) | sw_request)) {
            break;
        }
        if (!wait_field(io_, regs.arb, arb_status_mask,
                        sw_access << 2, acquisition_polls)) {
            break;
        }

        uint32_t interrupt = 0;
        if (!read(io_, regs.interrupt, interrupt) ||
            !write(io_, regs.interrupt,
                   (interrupt & ~other_ack_bits) | sw_done_ack) ||
            !wait_field(io_, regs.status, sw_done, 0, clear_polls)) {
            break;
        }

        // stage_request's final write is GO. If any stage write fails, hardware
        // may already have seen GO; quarantine instead of guessing completion.
        submitted = true;
        if (!stage_request(io_, regs, request) ||
            !wait_field(io_, regs.status, sw_done, sw_done, completion_polls)) {
            break;
        }
        completed = true;
        success = read_reply(io_, regs, request, reply) &&
                  io_.connected(io_.context);
    } while (false);

    const bool released = !claimed || release_engine(io_, regs);
    if (!released || (submitted && !completed)) {
        quarantined_ = true;
    }
    if (!success || !released) {
        reply = {};
        return false;
    }
    return true;
}

} // namespace rx7900::output

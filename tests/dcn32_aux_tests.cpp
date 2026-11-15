/*
Module: rx7900.tests.dcn32_aux
Purpose: Exercise bounded DCN3.2 AUX software transactions and quarantine.
Created: 2026-10-22
Architecture: Synthetic register model only; no physical GPU access.
*/
#include <cstdio>

#include "dcn32_aux.hpp"

namespace {
using namespace rx7900::output;
using rx7900::display::Dcn32RegisterSpace;

unsigned checks = 0;
unsigned failures = 0;
void expect(bool condition, const char* contract) {
    ++checks;
    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", contract);
    }
}

struct Model final {
    uint32_t base = 0x1f50u * 4u;
    uint32_t control = 1;
    uint32_t arb = 0;
    uint32_t interrupt = 0;
    uint32_t status = 0;
    uint32_t sw_control = 0;
    uint32_t data_control = 0;
    uint8_t request[20]{};
    uint8_t reply[17]{0, 0x12, 0x34};
    uint8_t staged = 0;
    uint8_t reply_count = 3;
    uint8_t reply_index = 0;
    unsigned writes = 0;
    unsigned waits = 0;
    bool connected = true;
    bool complete = true;
    bool refuse_release = false;
};

bool read(void* context, uint32_t offset, uint32_t& value) noexcept {
    auto& model = *static_cast<Model*>(context);
    if (offset < model.base) {
        return false;
    }
    switch (offset - model.base) {
    case 0: value = model.control; return true;
    case 4: value = model.sw_control; return true;
    case 8: value = model.arb; return true;
    case 12: value = model.interrupt; return true;
    case 16: value = model.status; return true;
    case 24:
        value = model.data_control;
        if ((value & 1u) != 0 && model.reply_index < model.reply_count) {
            value = (value & ~0xff00u) |
                    (static_cast<uint32_t>(model.reply[model.reply_index]) << 8);
            if ((value & 0x80000000u) == 0) {
                ++model.reply_index;
            }
        }
        return true;
    default: return false;
    }
}

bool write(void* context, uint32_t offset, uint32_t value) noexcept {
    auto& model = *static_cast<Model*>(context);
    if (offset < model.base) {
        return false;
    }
    ++model.writes;
    switch (offset - model.base) {
    case 0:
        model.control = value;
        if ((value & 0x10u) != 0) {
            model.control |= 0x20u;
        } else {
            model.control &= ~0x20u;
        }
        return true;
    case 4:
        model.sw_control = value;
        if ((value & 1u) != 0 && model.complete) {
            model.status = 1u | (static_cast<uint32_t>(model.reply_count) << 24);
        }
        return true;
    case 8:
        if ((value & 0x10000u) != 0) {
            model.arb = (value & ~0xcu) | 4u;
        } else if (!model.refuse_release) {
            model.arb = value & ~0xcu;
        }
        return true;
    case 12:
        model.interrupt = value;
        if ((value & 2u) != 0) model.status = 0;
        return true;
    case 24:
        model.data_control = value;
        if ((value & 1u) == 0) {
            if ((value & 0x80000000u) != 0) {
                model.staged = 0;
            }
            if (model.staged < sizeof(model.request)) {
                model.request[model.staged++] = static_cast<uint8_t>(value >> 8);
            }
        } else {
            model.reply_index = static_cast<uint8_t>((value >> 16) & 0x1fu);
        }
        return true;
    default: return false;
    }
}

bool wait(void* context, uint32_t microseconds) noexcept {
    auto& model = *static_cast<Model*>(context);
    ++model.waits;
    return microseconds <= 10;
}

bool connected(void* context) noexcept {
    return static_cast<Model*>(context)->connected;
}

Dcn32AuxEngine engine(Model& model, Dcn32RegisterSpace space =
                       {0, 0x10000, 3, 2, 0, true}, uint8_t number = 0,
                       uint8_t pin = 0) {
    model.base = (space.segment2_dword_base + 0x1f50u + number * 0x1cu) * 4u;
    model.control = 1u | (static_cast<uint32_t>(pin) << 20);
    return Dcn32AuxEngine({&model, read, write, wait, connected}, space, number, pin);
}

constexpr Dcn32AuxPrerequisites ready{true, true, true, true};

void success_and_framing() {
    Model model{};
    auto aux = engine(model);
    uint8_t bytes[2]{};
    const AuxRequest request{AuxOperation::native_read, 0x12345, nullptr, bytes, 2, false};
    AuxReply reply{};
    expect(aux.transfer(request, reply, ready), "native read transaction succeeds");
    expect(reply.result == AuxResult::ack && reply.bytes == 2,
           "ACK reports actual read length");
    expect(bytes[0] == 0x12 && bytes[1] == 0x34, "read bytes copied within capacity");
    expect(model.staged == 4 && model.request[0] == 0x91 &&
           model.request[1] == 0x23 && model.request[2] == 0x45 &&
           model.request[3] == 1, "native DPCD header and length framed");
    expect((model.arb & 0x10000u) == 0 && !aux.quarantined(),
           "successful transaction releases arbitration");
}

void write_and_i2c_framing() {
    Model model{};
    model.reply_count = 1;
    auto aux = engine(model, {0, 0x10000, 3, 2, 0, true}, 0, 0);
    const uint8_t data[2]{0xaa, 0x55};
    const AuxRequest request{AuxOperation::native_write, 0xfedcb,
                             data, nullptr, 2, false};
    AuxReply reply{};
    expect(aux.transfer(request, reply, ready), "native write ACK succeeds");
    expect(reply.result == AuxResult::ack && reply.bytes == 2,
           "write ACK reports payload length to policy");
    expect(model.staged == 6 && model.request[0] == 0x8f &&
           model.request[1] == 0xed && model.request[2] == 0xcb &&
           model.request[3] == 1 && model.request[4] == 0xaa &&
           model.request[5] == 0x55, "write header and payload framed");

    Model i2c{};
    i2c.reply_count = 1;
    auto i2c_aux = engine(i2c);
    const AuxRequest stop{AuxOperation::i2c_write, 0x50,
                          nullptr, nullptr, 0, false};
    expect(i2c_aux.transfer(stop, reply, ready), "bare I2C stop succeeds");
    expect(i2c.staged == 3 && i2c.request[0] == 0x00 &&
           i2c.request[1] == 0x00 && i2c.request[2] == 0x50,
           "bare I2C stop has no length byte");
}

void owned_engine_enable() {
    Model model{};
    model.control = 0;
    auto aux = engine(model);
    model.control = 0;
    model.reply_count = 2;
    uint8_t byte = 0;
    const AuxRequest request{AuxOperation::native_read, 0, nullptr, &byte, 1, false};
    AuxReply reply{};
    expect(aux.transfer(request, reply, ready),
           "owned disabled AUX engine enables and resets before request");
    expect((model.control & 0x31u) == 1u && !aux.quarantined(),
           "AUX reset is deasserted and completion confirmed");
}

void refusals_and_quarantine() {
    Model model{};
    auto aux = engine(model);
    uint8_t value = 0;
    const AuxRequest request{AuxOperation::native_read, 0, nullptr, &value, 1, false};
    AuxReply reply{};
    expect(!aux.transfer(request, reply, {}), "missing owner proof refused");
    expect(model.writes == 0, "missing owner proof does not write MMIO");
    model.control |= 0x10000u;
    expect(!aux.transfer(request, reply, ready), "ignore-HPD route refused");
    expect(model.writes == 0, "unverified route does not write MMIO");
    model.control &= ~0x10000u;
    model.complete = false;
    expect(!aux.transfer(request, reply, ready), "completion timeout fails");
    expect(aux.quarantined(), "uncertain submission quarantines instance");
    const auto writes = model.writes;
    expect(!aux.transfer(request, reply, ready) && model.writes == writes,
           "quarantine refuses a second transaction");

    Model truncated{};
    auto bad_space = engine(truncated, {0, 0x7d58, 3, 2, 0, true});
    expect(!bad_space.transfer(request, reply, ready) && truncated.writes == 0,
           "truncated BAR aperture refused before MMIO");
    Model denied{};
    auto denied_aux = engine(denied);
    denied.arb = 8;
    expect(!denied_aux.transfer(request, reply, ready) && denied.writes == 0,
           "DMCU ownership refused");
    Model absent{};
    auto absent_aux = engine(absent);
    absent.connected = false;
    expect(!absent_aux.transfer(request, reply, ready) && absent.writes == 0,
           "disconnected HPD refused");

    Model release{};
    release.reply_count = 2;
    release.refuse_release = true;
    auto release_aux = engine(release);
    expect(!release_aux.transfer(request, reply, ready),
           "failed arbitration release refuses publication");
    expect(release_aux.quarantined(), "failed arbitration release quarantines");

    Model unknown{};
    auto unknown_aux = engine(unknown);
    const AuxRequest unknown_request{static_cast<AuxOperation>(0xff), 0,
                                     nullptr, &value, 1, false};
    expect(!unknown_aux.transfer(unknown_request, reply, ready) &&
           unknown.writes == 0, "unknown operation refused before MMIO");
}

} // namespace

int main() {
    success_and_framing();
    write_and_i2c_framing();
    owned_engine_enable();
    refusals_and_quarantine();
    std::printf("dcn32_aux: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

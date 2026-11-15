/*
Module: rx7900.tests.dcn32_display_model
Purpose: Shared synthetic Navi31 DCN3.2 register file for the link-side display tests.
Created: 2026-10-24
Architecture: Test-only sparse BAR5 model with write log, read/write hooks and
fault injection behind the shared hardware::RegisterIo contract. Segment bases
are the Navi31 DCN 3.2.0 / MP1 13.0.0 values Linux uses; this is not hardware
evidence and is never linked into a driver. Reuses rx7900_test::Checks.
*/
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <utility>
#include <vector>

#include "asic_test_device.hpp"
#include "dcn32_otg.hpp"
#include "register_io.hpp"

namespace rx7900_test::dcn {

// DCN_BASE instance 0 segments 1 (DCCG) and 2 (DCN), MP1 segment 0.
constexpr uint32_t segment1 = 0x000000c0u;
constexpr uint32_t segment2 = 0x000034c0u;
constexpr uint32_t mp1_segment0 = 0x00016000u;
constexpr uint32_t aperture = 0x00100000u;

inline uint32_t byte_offset(uint32_t segment, uint32_t reg) { return (segment + reg) * 4u; }

inline rx7900::display::Dcn32RegisterSpace space() {
    return {segment2, aperture, 3, 2, 0, true};
}

struct Model final {
    std::map<uint32_t, uint32_t> values;
    std::vector<std::pair<uint32_t, uint32_t>> writes;
    std::function<void(Model&, uint32_t, uint32_t)> on_write;
    std::function<bool(Model&, uint32_t, uint32_t&)> on_read;
    // Zero-based index of the write that fails (and is not applied).
    size_t fail_write = SIZE_MAX;
    uint32_t fail_read_offset = UINT32_MAX;
    uint64_t delayed_us = 0;
    uint32_t delays = 0;

    uint32_t get(uint32_t segment, uint32_t reg) const {
        const auto found = values.find(byte_offset(segment, reg));
        return found == values.end() ? 0u : found->second;
    }
    void set(uint32_t segment, uint32_t reg, uint32_t value) {
        values[byte_offset(segment, reg)] = value;
    }
    // Writes to one register, in order.
    std::vector<uint32_t> written(uint32_t segment, uint32_t reg) const {
        std::vector<uint32_t> output;
        for (const auto& entry : writes) {
            if (entry.first == byte_offset(segment, reg)) {
                output.push_back(entry.second);
            }
        }
        return output;
    }
    // Index of the first write to a register at or after start, SIZE_MAX if none.
    size_t write_index(uint32_t segment, uint32_t reg, size_t start = 0) const {
        for (size_t index = start; index < writes.size(); ++index) {
            if (writes[index].first == byte_offset(segment, reg)) {
                return index;
            }
        }
        return SIZE_MAX;
    }

    static bool read32(void* context, uint32_t offset, uint32_t& value) noexcept {
        auto& model = *static_cast<Model*>(context);
        if (offset == model.fail_read_offset || offset >= aperture) {
            return false;
        }
        if (model.on_read && model.on_read(model, offset, value)) {
            return true;
        }
        const auto found = model.values.find(offset);
        value = found == model.values.end() ? 0u : found->second;
        return true;
    }
    static bool write32(void* context, uint32_t offset, uint32_t value) noexcept {
        auto& model = *static_cast<Model*>(context);
        if (model.writes.size() == model.fail_write || offset >= aperture) {
            model.fail_write = SIZE_MAX - 1u; // fail once
            return false;
        }
        model.writes.emplace_back(offset, value);
        model.values[offset] = value;
        if (model.on_write) {
            model.on_write(model, offset, value);
        }
        return true;
    }
    static void delay(void* context, uint32_t microseconds) noexcept {
        auto& model = *static_cast<Model*>(context);
        model.delayed_us += microseconds;
        ++model.delays;
    }

    rx7900::hardware::RegisterIo io() {
        return {this, aperture, read32, write32, delay};
    }
};

} // namespace rx7900_test::dcn

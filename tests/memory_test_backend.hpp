/*
Module: rx7900.tests.memory_backend
Purpose: Share deterministic backing fixtures across memory and queue unit tests.
Created: 2026-10-21
Architecture: Test-only callbacks; never linked into any driver or hardware backend.
*/
#pragma once

#include <array>
#include "manager.hpp"

namespace rx7900_test {

struct TestBacking final {
    struct Record final {
        bool used = false;
        std::array<uint8_t, 4096> bytes{};
    };

    std::array<Record, rx7900::memory::allocation_capacity> records{};
    bool fail_create = false;
    bool fail_destroy = false;
    bool invalid_cpu_descriptor = false;
    bool quarantine_create = false;
    unsigned creates = 0;
    unsigned destroys = 0;

    static rx7900::memory::CreateResult create(void* context, rx7900::sdma::GpuAllocation allocation,
                       rx7900::memory::Access, bool cpu_visible,
                       rx7900::memory::Backing& backing) noexcept {
        auto& self = *static_cast<TestBacking*>(context);
        ++self.creates;

        if (self.fail_create || (cpu_visible && allocation.byte_length > rx7900::memory::page_bytes)) {
            return rx7900::memory::CreateResult::no_resources;
        }

        for (auto& record : self.records) {
            if (!record.used) {
                record.used = true;
                backing = {&record, cpu_visible ? record.bytes.data() : nullptr,
                            cpu_visible ? record.bytes.size() : 0};

                if (self.invalid_cpu_descriptor) {
                    backing.cpu_byte_count = 0;
                }

                return self.quarantine_create ? rx7900::memory::CreateResult::quarantined
                                              : rx7900::memory::CreateResult::created;
            }
        }

        return rx7900::memory::CreateResult::no_resources;
    }

    static bool destroy(void* context, const rx7900::memory::Backing& backing,
                         rx7900::sdma::GpuAllocation) noexcept {
        auto& self = *static_cast<TestBacking*>(context);

        if (self.fail_destroy) {
            return false;
        }

        auto* record = static_cast<Record*>(backing.token);

        if (record == nullptr || !record->used) {
            return false;
        }

        record->used = false;
        ++self.destroys;
        return true;
    }

    rx7900::memory::Backend backend() noexcept { return {this, create, destroy}; }
};

} // namespace rx7900_test

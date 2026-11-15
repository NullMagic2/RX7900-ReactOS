/*
Module: rx7900.tests.psp_training
Purpose: Verify PSP13 long/resume training layout, ordering, timeout and data save/restore.
Created: 2026-10-21
Architecture: Synthetic register/VRAM machine; no DRAM training occurs.
*/
#include <algorithm>
#include <memory>

#include "asic_test_device.hpp"

namespace {

using namespace rx7900::hardware;
using rx7900_test::Event;
using rx7900_test::SimulatedNavi31;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

struct Fixture final {
    std::unique_ptr<SimulatedNavi31> device = std::make_unique<SimulatedNavi31>();
    Nbio nbio{};
    PspRegisters psp{};
    std::unique_ptr<PspTrainingData> data = std::make_unique<PspTrainingData>();
    PspTrainingReport report{};
    uint64_t vram = static_cast<uint64_t>(rx7900_test::memsize_mib) << 20;

    Fixture() {
        const auto binary = rx7900_test::navi31_discovery();
        NbioRegisters registers;
        validate_nbio_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, registers);
        validate_psp_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, psp);
        nbio.initialize(registers, device->io());
        nbio.program_common();
    }

    PspTrainingStatus run(uint32_t reserved = 4u << 20) {
        const auto io = device->io();
        return run_psp_long_training(psp, io, nbio, vram, reserved, device->training_io(), *data, report);
    }

    PspTrainingStatus resume(bool callbacks = true) {
        const auto io = device->io();
        return run_psp_resume_training(psp, io, nbio, vram, 4u << 20,
                                       callbacks ? device->training_io() : PspTrainingIo{}, *data, report);
    }

    std::vector<uint32_t> messages() const {
        std::vector<uint32_t> output;

        for (const auto& event : device->events) {
            if (event.kind == Event::Kind::psp_boot) {
                output.push_back(event.value);
            }
        }

        return output;
    }

    // MM_DATA stores into the C2P block, in order.
    size_t c2p_writes() const {
        size_t count = 0;

        for (const auto& write : device->vram_writes) {
            count += write.first >= report.c2p_offset && write.first < report.c2p_offset + 0x1000 ? 1 : 0;
        }

        return count;
    }

    size_t index(Event::Kind kind) const {
        for (size_t at = 0; at < device->events.size(); ++at) {
            if (device->events.at(at).kind == kind) {
                return at;
            }
        }

        return SIZE_MAX;
    }
};

void completed() {
    Fixture fixture;
    const auto flushes = fixture.device->hdp_flushes;
    expect(fixture.run() == PspTrainingStatus::completed, "cold-boot long training completes");
    const uint64_t c2p = fixture.vram - (5ULL << 20);
    expect(fixture.report.c2p_offset == c2p && fixture.report.p2c_offset == fixture.vram - 0x8000,
           "C2P is 1 MiB aligned below the reserved top region; P2C 0x8000 below VRAM top");
    expect(fixture.device->training_c2p == static_cast<uint32_t>(c2p >> 20) &&
           fixture.report.message_sent, "C2PMSG_36 carries the C2P offset in MiB");
    expect(fixture.index(Event::Kind::training_save) < fixture.index(Event::Kind::psp_boot) &&
           fixture.index(Event::Kind::psp_boot) < fixture.index(Event::Kind::training_restore),
           "encroached VRAM saved before the long-training message and restored after");
    expect(fixture.device->events.at(fixture.index(Event::Kind::psp_boot)).value == 0x100000,
           "PSP_BL__DRAM_LONG_TRAIN sent through C2PMSG_35");
    expect(fixture.device->hdp_flushes == flushes + 1, "HDP flushed after the restore");
    expect(fixture.report.p2c_header[0] == psp_training_signature, "P2C header read before training");
    expect(fixture.data->valid && fixture.data->bytes[0] == 0x42 && fixture.data->bytes[4] == 0x3c,
           "4 KiB P2C training data saved to the CPU cache");

    Fixture unaligned;
    unaligned.run(0x10000);
    expect(unaligned.report.c2p_offset == unaligned.vram - (1ULL << 20),
           "64 KiB default reservation rounds the C2P block up to 1 MiB alignment");
}

void refusals() {
    Fixture alive;
    alive.device->reg(rx7900_test::mp_base + 0x91) = 1;
    expect(alive.run() == PspTrainingStatus::skipped_sos_alive &&
           alive.index(Event::Kind::training_save) == SIZE_MAX, "running SOS skips training");

    Fixture save;
    save.device->save_ok = false;
    expect(save.run() == PspTrainingStatus::save_failed && !save.report.message_sent &&
           save.index(Event::Kind::psp_boot) == SIZE_MAX, "no message without a confirmed save");

    Fixture hang;
    hang.device->train_hang = true;
    expect(hang.run() == PspTrainingStatus::timeout && hang.report.polls == 3000000 &&
           hang.index(Event::Kind::training_restore) == SIZE_MAX && !hang.data->valid,
           "3 s timeout leaves the encroached range untouched, as Linux does");

    Fixture restore;
    restore.device->restore_ok = false;
    expect(restore.run() == PspTrainingStatus::restore_failed && !restore.data->valid,
           "restore failure reported");

    Fixture small;
    small.vram = 32ULL << 20;
    expect(small.run() == PspTrainingStatus::invalid_layout && small.device->events.size() == 2,
           "VRAM too small for the training layout refused before MMIO");

    Fixture missing;
    const auto io = missing.device->io();
    expect(run_psp_long_training(missing.psp, io, missing.nbio, missing.vram, 0x10000, {},
                                 *missing.data, missing.report) == PspTrainingStatus::unavailable,
           "missing save/restore callbacks refused");
}

// psp_v13_0_memory_training(PSP_MEM_TRAIN_RESUME): the ops escalation matrix.
void resume_operations() {
    constexpr uint32_t short_restore = psp_training_op_short | psp_training_op_restore;

    // Cold boot saved the cache; S3 lost the P2C block: restore, then short.
    Fixture lost;
    lost.run();
    std::fill(lost.device->training_data.begin(), lost.device->training_data.end(), uint8_t{0});
    lost.device->events.clear();
    lost.device->vram_writes.clear();
    expect(lost.resume() == PspTrainingStatus::completed && lost.report.operations == short_restore &&
           lost.messages() == std::vector<uint32_t>{0x200000} && lost.c2p_writes() == 0x400 &&
           lost.device->vram_writes.front().second == psp_training_signature &&
           lost.device->training_c2p == static_cast<uint32_t>(lost.report.c2p_offset >> 20),
           "valid cache, lost P2C: sys_cache written to C2P, then PSP_BL__DRAM_SHORT_TRAIN");

    // P2C still holds matching data: nothing to save.
    Fixture matching;
    matching.run();
    matching.device->events.clear();
    expect(matching.resume() == PspTrainingStatus::completed && matching.report.operations == short_restore,
           "a P2C header matching the cache adds no SAVE");

    // P2C holds newer data (dword 3 differs): SAVE, RESTORE, SHORT.
    Fixture newer;
    newer.run();
    rx7900::sdma::CommandBuffer(newer.device->training_data.data(), 16).write_u32(12, 0x77777777u);
    newer.device->events.clear();
    expect(newer.resume() == PspTrainingStatus::completed &&
           newer.report.operations == (short_restore | psp_training_op_save) &&
           newer.data->bytes[12] == 0x77 && newer.messages() == std::vector<uint32_t>{0x200000},
           "an out-of-date cache is refreshed from P2C before the restore");

    // Empty cache and no P2C data: LONG replaces SHORT (save, long, restore).
    Fixture empty;
    std::fill(empty.device->training_data.begin(), empty.device->training_data.end(), uint8_t{0});
    expect(empty.resume(false) == PspTrainingStatus::unavailable && empty.messages().empty() &&
           empty.index(Event::Kind::training_save) == SIZE_MAX && empty.device->vram_writes.empty() &&
           empty.report.operations == (psp_training_op_long | psp_training_op_save | psp_training_op_restore),
           "LONG without the encroached callbacks refused before any write");
    expect(empty.resume() == PspTrainingStatus::completed &&
           empty.messages() == std::vector<uint32_t>{0x100000} &&
           empty.index(Event::Kind::training_save) < empty.index(Event::Kind::psp_boot) &&
           empty.index(Event::Kind::psp_boot) < empty.index(Event::Kind::training_restore),
           "escalated LONG training saves, trains and restores the BIST range");

    // Empty cache with P2C data present: SAVE then RESTORE, no long training.
    Fixture fresh;
    expect(fresh.resume() == PspTrainingStatus::completed &&
           fresh.report.operations == (short_restore | psp_training_op_save) && fresh.data->valid &&
           fresh.messages() == std::vector<uint32_t>{0x200000}, "an empty cache is filled from P2C");

    Fixture alive;
    alive.device->reg(rx7900_test::mp_base + 0x91) = 1;
    expect(alive.resume() == PspTrainingStatus::skipped_sos_alive && alive.messages().empty() &&
           alive.device->vram_writes.empty(),
           "running SOS skips resume training");

    Fixture hang;
    hang.run();
    hang.device->train_hang = true;
    expect(hang.resume() == PspTrainingStatus::timeout && hang.report.polls == 3000000 &&
           hang.report.message == 0x200000, "short training waits the same 3 s budget");
}

} // namespace

int main() {
    completed();
    refusals();
    resume_operations();
    return checks.finish("psp_training");
}

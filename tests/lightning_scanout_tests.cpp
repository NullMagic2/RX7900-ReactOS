/*
Module: lightning.tests.scanout
Purpose: Exercise the guest protocol against the actual canonical host core.
Created: 2026-10-21
Architecture: Explicit simulated UI completion; no VM or physical GPU access.
*/
#include "lightning_scanout.hpp"
#include "lightning_staging.hpp"
#include "lightning_present_abi.hpp"
#include "hw/misc/lightning-display-host.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace {
using namespace lightning::reactos;
uint32_t checks = 0;
void check(bool condition, const char* description) {
    ++checks;
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", description);
        std::exit(1);
    }
}

struct Host final {
    LightningDisplayHost* host = lightning_display_host_new();
    std::vector<uint8_t> ram = std::vector<uint8_t>(maximum_scanout_bytes());
    uint32_t registers[0x4000 / 4]{};
    uint32_t fail_write = UINT32_MAX;
    uint32_t fail_read = UINT32_MAX;

    Host() {
        check(host != nullptr, "actual host allocated");
        registers[LIGHTNING_SCANOUT_REG_MAGIC / 4] = LIGHTNING_SCANOUT_MAGIC;
        registers[LIGHTNING_SCANOUT_REG_VERSION / 4] = LIGHTNING_SCANOUT_VERSION;
    }
    ~Host() { lightning_display_host_free(host); }
    static bool read(void* context, uint32_t offset, uint32_t& value) noexcept {
        auto& self = *static_cast<Host*>(context);
        if ((offset & 3U) != 0 || offset >= sizeof(self.registers) || self.fail_read == offset) return false;
        if (lightning_display_host_read(self.host, offset, &value)) return true;
        value = self.registers[offset / 4];
        return true;
    }
    static bool write(void* context, uint32_t offset, uint32_t value) noexcept {
        auto& self = *static_cast<Host*>(context);
        if ((offset & 3U) != 0 || offset >= sizeof(self.registers) || self.fail_write == offset) return false;
        if (offset == LIGHTNING_DISPLAY_REG_COMMAND) {
            lightning_display_host_submit(self.host, value,
                self.registers[LIGHTNING_SCANOUT_REG_WIDTH / 4],
                self.registers[LIGHTNING_SCANOUT_REG_HEIGHT / 4],
                self.registers[LIGHTNING_SCANOUT_REG_STRIDE / 4],
                self.registers[LIGHTNING_SCANOUT_REG_OFFSET / 4],
                self.ram.data(), self.ram.size(),
                reinterpret_cast<const uint8_t*>(self.registers) + LIGHTNING_DISPLAY_DAMAGE_OFFSET,
                LIGHTNING_DISPLAY_DAMAGE_BYTES, 0);
            return true;
        }
        if (lightning_display_host_write(self.host, offset, value)) return true;
        self.registers[offset / 4] = value;
        return true;
    }
    static bool clear(void* context, uint32_t bytes) noexcept {
        auto& self = *static_cast<Host*>(context);
        if (bytes > self.ram.size()) return false;
        std::memset(self.ram.data(), 0, bytes);
        return true;
    }
    ScanoutAccess access() noexcept { return {this, read, write, clear, ram.size()}; }
    void publish() { lightning_display_host_complete(host, true, 16666666); }
};

void protocol() {
    Host host;
    CpuScanout core;
    DisplayCompletion completed{};
    check(core.initialize(host.access()) == ScanoutStatus::success, "canonical capability negotiation");
    check(core.set_mode(1, false) == ScanoutStatus::success, "full generated1080p accepted");
    check(core.completion(completed) == ScanoutStatus::success && !completed.published,
          "acceptance is distinct from publication");
    check(core.set_mode(0, false) == ScanoutStatus::busy, "pending mode change rejected");
    host.publish();
    check(core.completion(completed) == ScanoutStatus::success && completed.published,
          "actual host publication retires accepted prefix");
    const LightningDisplayRect rectangle{3, 2, 2, 1};
    const size_t pixel = (2 * 1920 + 3) * 4;
    host.ram[pixel] = 0xa7;
    check(core.damage(&rectangle, 1) == ScanoutStatus::success, "partial CPU snapshot accepted");
    host.ram[pixel] = 0x45;
    LightningDisplayView view{};
    check(lightning_display_host_view(host.host, &view) && view.pixels[pixel] == 0xa7,
          "accepted host snapshot retains pixels independently of guest RAM");
    check(view.damage.x == 3 && view.damage.y == 2 && view.damage.width == 2 && view.damage.height == 1,
          "small dirty region remains partial");
    const LightningDisplayRect second{10, 8, 1, 1};
    check(core.damage(&second, 1) == ScanoutStatus::success, "pending frame coalesces another update");
    check(core.completion(completed) == ScanoutStatus::success && !completed.published &&
          completed.accepted_sequence == 3 && completed.completed_sequence == 1,
          "coalesced prefix remains pending before publication");
    host.publish();
    check(core.completion(completed) == ScanoutStatus::success && completed.published &&
          completed.completed_sequence == 3, "publication retires cumulative prefix");
    const LightningDisplayRect invalid{UINT32_MAX, 0, 2, 1};
    check(core.damage(&invalid, 1) == ScanoutStatus::invalid_backing, "overflowing damage rejected");
    check(core.disable() == ScanoutStatus::success, "disable accepted as sequenced blank");
    check(core.completion(completed) == ScanoutStatus::success && !completed.published,
          "accepted blank is not prematurely complete");
    host.publish();
    check(core.completion(completed) == ScanoutStatus::success && completed.published, "blank publication completes");
    check(core.damage(&rectangle, 1) == ScanoutStatus::unavailable, "damage requires full baseline after disable");
    check(core.set_mode(0, true) == ScanoutStatus::success, "new mode starts next full epoch");
    host.publish();
    check(core.completion(completed) == ScanoutStatus::success && completed.accepted_epoch == 2 &&
          completed.accepted_sequence == 1 && completed.published, "epoch transition starts sequence1");
    host.fail_write = LIGHTNING_DISPLAY_REG_COMMAND;
    check(core.present() == ScanoutStatus::state_unknown && core.quarantined(), "failed command quarantines uncertain state");
    check(core.present() == ScanoutStatus::unavailable, "quarantined core performs no further submission");
}

void leases() {
    StagingLedger ledger;
    uint8_t buffers[StagingLedger::capacity + 1][64]{};
    check(!ledger.install(nullptr, 64) && ledger.acquire() == nullptr, "empty lease owner rejects invalid buffer");
    check(ledger.install(buffers[0], sizeof(buffers[0])), "first cached allocation installed");
    check(ledger.acquire() == buffers[0] && ledger.acquire() == buffers[0], "separate map leases share active RAM");
    check(ledger.current_bytes() == 64, "mapped extent retained");
    ledger.retire();
    check(ledger.current() == nullptr && ledger.reclaim() == nullptr, "retired leased RAM never reclaimed");
    check(ledger.release(buffers[0]) && ledger.reclaim() == nullptr, "first unmap retains another lease");
    check(ledger.release(buffers[0]) && ledger.reclaim() == buffers[0], "last unmap releases retired allocation");
    check(!ledger.release(buffers[0]), "double unmap rejected");
    for (uint32_t i = 0; i < StagingLedger::capacity; ++i) {
        check(ledger.install(buffers[i], 64) && ledger.acquire() == buffers[i], "bounded retained generation installed");
        ledger.retire();
    }
    check(!ledger.install(buffers[StagingLedger::capacity], 64), "restart refuses retained-generation exhaustion");
    check(!ledger.can_replace(), "fully leased generations cannot be silently recycled");
    for (uint32_t i = 0; i < StagingLedger::capacity; ++i) {
        check(ledger.release(buffers[i]) && ledger.reclaim() == buffers[i], "late unmap safely reclaims retained generation");
    }
    check(ledger.install(buffers[0],64) && ledger.acquire()==buffers[0],"first PDEV owns its RAM lease");
    check(ledger.can_replace(),"a free slot permits another PDEV");
    ledger.retire();
    check(ledger.install(buffers[1],64) && ledger.acquire()==buffers[1],"second PDEV receives disjoint RAM");
    buffers[0][0]=0xa7;
    check(buffers[1][0]==0,"late old PDEV drawing cannot corrupt new staging");
    check(ledger.release(buffers[0]) && ledger.reclaim()==buffers[0],"old wrapper release retires only old RAM");
    check(ledger.current()==buffers[1] && ledger.reclaim()==nullptr,"active leased generation remains owned");
    check(sizeof(CursorShapeRequest)==52 && sizeof(CursorMoveRequest)==24 && sizeof(CpuScanoutInformation)==60,
          "paired cursor-generation ABI and stable read-only query");
}

void saturation_and_reset() {
    Host host;
    CpuScanout core;
    check(core.initialize(host.access()) == ScanoutStatus::success &&
        core.set_mode(1, false) == ScanoutStatus::success, "backpressure fixture initialized");
    host.publish();
    const LightningDisplayRect rectangle{1919, 1079, 1, 1};
    for (uint32_t i=0;i<LIGHTNING_DISPLAY_MAX_PENDING_UPDATES;++i) {
        check(core.damage(&rectangle,1)==ScanoutStatus::success,"bounded coalescing prefix accepted");
    }
    check(core.damage(&rectangle,1)==ScanoutStatus::busy && !core.quarantined(),
        "queue saturation retains last accepted prefix without false failure");
    host.publish();
    check(core.damage(&rectangle,1)==ScanoutStatus::success,"publication permits bounded retry");
    lightning_display_host_reset(host.host);
    DisplayCompletion completion{};
    check(core.completion(completion)==ScanoutStatus::state_unknown && core.quarantined(),
        "external reset cannot turn canceled publication into completed fence");
    check(core.recover()==ScanoutStatus::state_unknown && core.quarantined() &&
        core.recover()==ScanoutStatus::state_unknown,
        "a cleared host baseline is never adopted; quarantine stays permanent");
}

// Another guest writer advances the host prefix behind the core's back.
void foreign_submit(Host& host, uint32_t epoch, uint32_t sequence) {
    Host::write(&host, LIGHTNING_DISPLAY_REG_RECT_COUNT, 0);
    Host::write(&host, LIGHTNING_DISPLAY_REG_EPOCH, epoch);
    Host::write(&host, LIGHTNING_DISPLAY_REG_SUBMITTED_SEQ, sequence);
    Host::write(&host, LIGHTNING_DISPLAY_REG_COMMAND, LIGHTNING_DISPLAY_FULL);
}

void recovery() {
    const LightningDisplayRect rectangle{0, 0, 4, 4};
    DisplayCompletion completion{};
    {
        Host host;
        CpuScanout core;
        check(core.initialize(host.access()) == ScanoutStatus::success &&
            core.set_mode(1, false) == ScanoutStatus::success, "recovery fixture initialized");
        host.publish();
        check(core.recover() == ScanoutStatus::success, "healthy core needs no recovery");
        host.fail_write = LIGHTNING_DISPLAY_REG_COMMAND;
        check(core.damage(&rectangle, 1) == ScanoutStatus::state_unknown && core.quarantined(),
            "undelivered command is uncertain");
        host.fail_write = UINT32_MAX;
        check(core.recover() == ScanoutStatus::success && !core.quarantined(),
            "unchanged acknowledged prefix proves the command was not applied");
        check(core.damage(&rectangle, 1) == ScanoutStatus::unavailable && core.present() == ScanoutStatus::unavailable,
            "recovery assumes no mode: partial or repeated updates are refused");
        uint32_t mode = 0;
        check(core.current_mode(mode) == ScanoutStatus::unavailable, "recovered core reports no current mode");
        check(core.set_mode(1, true) == ScanoutStatus::success, "replay is a FULL in the next epoch");
        check(core.completion(completion) == ScanoutStatus::success && completion.accepted_epoch == 2 &&
            completion.accepted_sequence == 1 && !completion.published, "replay starts sequence1 of a new epoch");
        host.publish();
        check(core.damage(&rectangle, 1) == ScanoutStatus::success, "recovered core accepts ordinary damage");

        host.fail_read = LIGHTNING_DISPLAY_REG_RESULT;
        check(core.damage(&rectangle, 1) == ScanoutStatus::state_unknown && core.quarantined(),
            "applied command with a lost acknowledgement is uncertain");
        host.fail_read = UINT32_MAX;
        check(core.recover() == ScanoutStatus::busy && core.quarantined(),
            "applied but unpublished prefix keeps quarantine while the host still reads it");
        host.publish();
        check(core.recover() == ScanoutStatus::success, "published attempted command is adopted exactly");
        check(core.set_mode(1, true) == ScanoutStatus::success &&
            core.completion(completion) == ScanoutStatus::success && completion.accepted_epoch == 3,
            "adopted prefix continues the host's epoch sequence");
        host.publish();

        host.registers[LIGHTNING_SCANOUT_REG_RESOURCE_LO / 4] = 1;
        host.fail_write = LIGHTNING_DISPLAY_REG_COMMAND;
        check(core.damage(&rectangle, 1) == ScanoutStatus::state_unknown, "uncertain command beside a GPU scanout");
        host.fail_write = UINT32_MAX;
        check(core.recover() == ScanoutStatus::busy && core.quarantined(),
            "another scanout owner keeps quarantine without spending recovery");
        host.registers[LIGHTNING_SCANOUT_REG_RESOURCE_LO / 4] = 0;
        check(core.recover() == ScanoutStatus::success, "released scanout owner permits recovery");
        check(core.set_mode(1, true) == ScanoutStatus::success, "replay after owner release");
        host.publish();
        check(core.damage(&rectangle, 1) == ScanoutStatus::success, "ordinary acceptance resets the limit");
    }
    {
        Host host;
        CpuScanout core;
        check(core.initialize(host.access()) == ScanoutStatus::success &&
            core.set_mode(1, false) == ScanoutStatus::success, "failed-publication fixture initialized");
        host.publish();
        check(core.damage(&rectangle, 1) == ScanoutStatus::success, "accepted update awaits publication");
        lightning_display_host_complete(host.host, false, 33333332);
        check(core.completion(completion) == ScanoutStatus::state_unknown && core.quarantined(),
            "failed publication is uncertain");
        check(core.recover() == ScanoutStatus::success, "host dropped the prefix and is no longer pending");
        check(core.completion(completion) == ScanoutStatus::success && completion.published &&
            completion.completed_sequence == 1 && completion.accepted_sequence == 2,
            "dropped prefix settles without reporting its sequence completed");
        check(core.set_mode(1, true) == ScanoutStatus::success, "replay FULL follows the dropped prefix");
        check(core.completion(completion) == ScanoutStatus::success && !completion.published,
            "replay must itself publish");
        host.publish();
        check(core.completion(completion) == ScanoutStatus::success && completion.published &&
            completion.completed_epoch == 2, "replay publication completes normally");
        lightning_display_host_complete(host.host, false, 50000000);
        check(core.damage(&rectangle, 1) == ScanoutStatus::success, "next update after recovery accepted");
        lightning_display_host_complete(host.host, false, 66666666);
        check(core.completion(completion) == ScanoutStatus::state_unknown,
            "a later failed publication is uncertain again");
    }
    {
        Host host;
        CpuScanout core;
        check(core.initialize(host.access()) == ScanoutStatus::success &&
            core.set_mode(1, false) == ScanoutStatus::success, "limit fixture initialized");
        host.publish();
        host.fail_write = LIGHTNING_DISPLAY_REG_COMMAND;
        check(core.damage(&rectangle, 1) == ScanoutStatus::state_unknown, "first uncertain command");
        for (uint32_t attempt = 0; attempt < CpuScanout::recovery_limit; ++attempt) {
            check(core.recover() == ScanoutStatus::success, "bounded recovery succeeds");
            check(core.set_mode(1, true) == ScanoutStatus::state_unknown, "replay keeps failing");
        }
        host.fail_write = UINT32_MAX;
        check(core.recover() == ScanoutStatus::state_unknown && core.quarantined(),
            "consecutive recoveries are bounded; then quarantine is permanent");
    }
    {
        Host host;
        CpuScanout core;
        check(core.initialize(host.access()) == ScanoutStatus::success &&
            core.set_mode(1, false) == ScanoutStatus::success, "replay-limit fixture initialized");
        host.publish();
        for (uint32_t attempt = 0; attempt < CpuScanout::recovery_limit; ++attempt) {
            host.fail_write = LIGHTNING_DISPLAY_REG_COMMAND;
            check(core.damage(&rectangle, 1) == ScanoutStatus::state_unknown, "update after replay fails again");
            host.fail_write = UINT32_MAX;
            check(core.recover() == ScanoutStatus::success && core.set_mode(1, true) == ScanoutStatus::success,
                "recovery and replay succeed");
            host.publish();
        }
        host.fail_write = LIGHTNING_DISPLAY_REG_COMMAND;
        check(core.damage(&rectangle, 1) == ScanoutStatus::state_unknown, "fourth consecutive failure");
        host.fail_write = UINT32_MAX;
        check(core.recover() == ScanoutStatus::state_unknown,
            "a successful replay alone does not renew the recovery budget");
    }
    {
        Host host;
        CpuScanout core;
        check(core.initialize(host.access()) == ScanoutStatus::success &&
            core.set_mode(1, false) == ScanoutStatus::success, "pending-status fixture initialized");
        host.publish();
        check(core.damage(&rectangle, 1) == ScanoutStatus::success, "update awaits publication");
        lightning_display_host_complete(host.host, false, 33333332);
        check(core.completion(completion) == ScanoutStatus::state_unknown, "failed publication");
        host.registers[LIGHTNING_SCANOUT_REG_STATUS / 4] = LIGHTNING_SCANOUT_PENDING;
        check(core.recover() == ScanoutStatus::busy && core.quarantined(),
            "a failed result is not a dropped prefix while the scanout is still pending");
        host.registers[LIGHTNING_SCANOUT_REG_STATUS / 4] = LIGHTNING_SCANOUT_INVALID;
        check(core.recover() == ScanoutStatus::success, "invalid idle scanout proves the prefix dropped");
    }
    {
        Host host;
        CpuScanout core;
        check(core.initialize(host.access()) == ScanoutStatus::success &&
            core.set_mode(1, false) == ScanoutStatus::success, "refused-attempt fixture initialized");
        host.publish();
        for (uint32_t i = 0; i < LIGHTNING_DISPLAY_MAX_PENDING_UPDATES; ++i) core.damage(&rectangle, 1);
        check(core.damage(&rectangle, 1) == ScanoutStatus::busy && !core.quarantined(),
            "host refuses an update with its prefix unchanged");
        host.publish();
        foreign_submit(host, 1, LIGHTNING_DISPLAY_MAX_PENDING_UPDATES + 2);
        host.publish();
        check(core.completion(completion) == ScanoutStatus::state_unknown, "foreign prefix after a refusal");
        check(core.recover() == ScanoutStatus::state_unknown,
            "a refused command is never later mistaken for our applied attempt");
    }
    {
        Host host;
        CpuScanout core;
        check(core.initialize(host.access()) == ScanoutStatus::success &&
            core.set_mode(1, false) == ScanoutStatus::success, "foreign-writer fixture initialized");
        host.publish();
        foreign_submit(host, 1, 2);
        host.publish();
        check(core.completion(completion) == ScanoutStatus::state_unknown, "foreign prefix is uncertain");
        check(core.recover() == ScanoutStatus::state_unknown && core.quarantined(),
            "a prefix neither acknowledged nor attempted by this core is never adopted");
    }
    {
        Host host;
        CpuScanout core;
        check(core.initialize(host.access()) == ScanoutStatus::success &&
            core.set_mode(1, false) == ScanoutStatus::success, "identity fixture initialized");
        host.publish();
        host.fail_write = LIGHTNING_DISPLAY_REG_COMMAND;
        check(core.damage(&rectangle, 1) == ScanoutStatus::state_unknown, "uncertain command before identity loss");
        host.registers[LIGHTNING_SCANOUT_REG_MAGIC / 4] = 0;
        check(core.recover() == ScanoutStatus::state_unknown, "lost scanout identity is permanent");
        host.registers[LIGHTNING_SCANOUT_REG_MAGIC / 4] = LIGHTNING_SCANOUT_MAGIC;
        check(core.recover() == ScanoutStatus::state_unknown, "permanent quarantine is not retried");
    }
    {
        CpuScanout core;
        check(core.recover() == ScanoutStatus::unavailable, "uninitialized core has nothing to recover");
    }
}
} // namespace

int main() {
    protocol();
    leases();
    saturation_and_reset();
    recovery();
    std::printf("Lightning scanout/lease checks: %u PASS\n", checks);
}

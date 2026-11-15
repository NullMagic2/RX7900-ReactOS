/*
Module: lightning.tests.overlap_model
Purpose: Prove the retirement-channel admission/drain protocol by exhaustive interleaving.
Created: 2026-10-25
Architecture: Exact production Rundown (lightning_rundown.hpp) over std::atomic.
    Each actor is a small step machine whose steps are the atomic operations
    lightning_driver.cpp performs (one Rundown call, one guard acquire, one
    extension access). A depth-first search runs every interleaving of
    retirement dispatches (CLOSE, lease resolve, a dispatch stuck in the lower
    device) against retirement_teardown and asserts: no extension access after
    the teardown deleted it, every dispatch ends (completed or refused), and
    the teardown either deletes exactly once after a full drain or contains.
    The previous protocol (check retirement_only, then count; delete after a
    bounded poll regardless) is modelled as well and must be caught. A
    threaded stress run gives TSan the same protocol on real threads.
*/
#include "lightning_rundown.hpp"
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <thread>
#include <vector>

namespace {
using lightning::reactos::Rundown;
unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* description) {
    ++checks;

    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL %s\n", description);
    }
}

// Copyable so the exhaustive search can fork whole model states.
struct AtomicWord final {
    std::atomic<int32_t> value{0};
    AtomicWord() noexcept = default;
    AtomicWord(const AtomicWord& other) noexcept : value(other.value.load()) {}
    AtomicWord& operator=(const AtomicWord& other) noexcept {
        value.store(other.value.load());
        return *this;
    }
    int32_t load() const noexcept { return value.load(); }
    int32_t compare_exchange(int32_t desired, int32_t expected) noexcept {
        value.compare_exchange_strong(expected, desired);
        return expected;
    }
};
using Entry = Rundown<AtomicWord>;

void check_rundown_contract() {
    Entry entry;
    expect(entry.phase() == Entry::Phase::inactive && !entry.acquire() && entry.active() == 0,
        "an inactive channel admits nothing");
    expect(entry.open() && entry.open() && entry.phase() == Entry::Phase::open, "open is idempotent");
    expect(entry.acquire() && entry.acquire() && entry.active() == 2, "an open channel counts admissions");
    expect(entry.close() && !entry.close() && entry.phase() == Entry::Phase::closed,
        "close latches exactly once");
    expect(!entry.acquire() && entry.active() == 2 && !entry.drained(),
        "a closed channel refuses new work but keeps counting admitted work");
    expect(!entry.open() && entry.phase() == Entry::Phase::closed, "a closed channel never reopens");
    entry.release();
    expect(!entry.drained() && entry.active() == 1, "one admitted dispatch still holds the drain");
    entry.release();
    expect(entry.drained() && entry.active() == 0, "the last release drains a closed channel");
    entry.release();
    expect(entry.drained() && entry.active() == 0, "an unbalanced release never wraps the count");

    Entry closed_first;
    expect(closed_first.close() && closed_first.drained() && !closed_first.open() && !closed_first.acquire(),
        "closing before opening drains at once and stays closed");

    Entry open_drain;
    expect(open_drain.open() && open_drain.acquire(), "fixture admits one dispatch");
    open_drain.release();
    expect(!open_drain.drained(), "an open channel is never reported drained");
}

// ---- Exhaustive interleaving model -------------------------------------
// The protocol under test: new (production) or old (before lightning-overlap).
enum class Protocol { production, previous };

// A retirement dispatch (forward_retirement / close_retirement). duration is
// the number of steps it spends inside the lower device under the guard; a
// value above the teardown's poll budget is a stuck lower request.
struct Dispatch final {
    unsigned pc = 0;
    unsigned duration = 1;
    unsigned spent = 0;
    bool admitted = false;
    bool done = false;
    bool counted_close = false;
};

struct Teardown final {
    unsigned pc = 0;
    unsigned polls = 0;
    bool done = false;
};

struct World final {
    Entry entry;
    // previous protocol state
    bool retirement_only = true;
    int calls = 0;
    // shared
    bool guard_held = false;
    int guard_owner = -1;
    bool extension_live = true;
    unsigned deletes = 0;
    unsigned contained = 0;
    bool violation = false;
    const char* why = nullptr;
};

constexpr unsigned poll_budget = 2;

struct Model final {
    Protocol protocol;
    World world;
    Dispatch dispatches[3];
    unsigned dispatch_count = 0;
    Teardown teardown;
    bool teardown_present = true;

    void touch(const char* what) {
        if (!world.extension_live && !world.violation) {
            world.violation = true;
            world.why = what;
        }
    }

    // One atomic step of dispatch index; returns false when blocked.
    bool step_dispatch(unsigned index) {
        auto& d = dispatches[index];
        const int self = static_cast<int>(index);

        if (protocol == Protocol::production) {
            switch (d.pc) {
            case 0:
                // forward_foreign: acquire before reading anything else.
                touch("dispatch admission read a deleted extension");
                d.admitted = world.entry.acquire();
                d.pc = d.admitted ? 1 : 5;
                return true;
            case 1:
                if (world.guard_held) { return false; }
                touch("dispatch took the guard of a deleted extension");
                world.guard_held = true;
                world.guard_owner = self;
                d.pc = 2;
                return true;
            case 2:
                touch("dispatch forwarded through a deleted extension");
                if (++d.spent < d.duration) { return true; }
                d.pc = 3;
                return true;
            case 3:
                touch("dispatch released the guard of a deleted extension");
                world.guard_held = false;
                world.guard_owner = -1;
                d.pc = 4;
                return true;
            case 4:
                // release is the last access.
                touch("dispatch released admission of a deleted extension");
                world.entry.release();
                d.done = true;
                d.pc = 6;
                return true;
            case 5:
                // Refused: falls through to the cleanup-lock path, which the
                // REMOVE already closed; completes DELETE_PENDING.
                d.done = true;
                d.pc = 6;
                return true;
            default:
                return true;
            }
        }

        switch (d.pc) {
        case 0:
            touch("old dispatch read retirement_only of a deleted extension");
            d.pc = world.retirement_only ? 1 : 5;
            return true;
        case 1:
            touch("old dispatch incremented calls of a deleted extension");
            ++world.calls;
            d.pc = 2;
            return true;
        case 2:
            if (world.guard_held) { return false; }
            touch("old dispatch took the guard of a deleted extension");
            world.guard_held = true;
            world.guard_owner = self;
            d.pc = 3;
            return true;
        case 3:
            touch("old dispatch forwarded through a deleted extension");
            if (++d.spent < d.duration) { return true; }
            world.guard_held = false;
            world.guard_owner = -1;
            d.pc = 4;
            return true;
        case 4:
            touch("old dispatch decremented calls of a deleted extension");
            --world.calls;
            d.done = true;
            d.pc = 6;
            return true;
        case 5:
            d.done = true;
            d.pc = 6;
            return true;
        default:
            return true;
        }
    }

    bool step_teardown() {
        auto& t = teardown;

        if (protocol == Protocol::production) {
            switch (t.pc) {
            case 0:
                world.entry.close();
                t.pc = 1;
                return true;
            case 1:
                if (world.entry.drained()) { t.pc = 2; return true; }
                // KeDelayExecutionThread: a bounded poll, then containment.
                if (++t.polls >= poll_budget) { t.pc = 5; }
                return true;
            case 2:
                // The final pause (runnable() holds it while a dispatch is
                // still in an instruction-only state).
                t.pc = 3;
                return true;
            case 3:
                // Guarded bookkeeping, then IoDeleteDevice.
                t.pc = 4;
                return true;
            case 4:
                world.extension_live = false;
                ++world.deletes;
                t.done = true;
                t.pc = 6;
                return true;
            case 5:
                ++world.contained;
                t.done = true;
                t.pc = 6;
                return true;
            default:
                return true;
            }
        }

        switch (t.pc) {
        case 0:
            if (world.calls == 0 || ++t.polls >= poll_budget) { t.pc = 1; }
            return true;
        case 1:
            world.retirement_only = false;
            t.pc = 2;
            return true;
        case 2:
            t.pc = 3;
            return true;
        case 3:
            world.extension_live = false;
            ++world.deletes;
            t.done = true;
            t.pc = 4;
            return true;
        default:
            return true;
        }
    }

    // A dispatch that only executes instructions (no wait, no lower call):
    // the teardown's final 10 ms pause outlasts it. A guard wait or a lower
    // request is not covered by that pause.
    bool instruction_only(const Dispatch& d) const {
        if (d.done) { return false; }
        return protocol == Protocol::production ? (d.pc == 0 || d.pc == 5) :
            (d.pc == 0 || d.pc == 1 || d.pc == 4 || d.pc == 5);
    }

    bool pause_blocked() const {
        for (unsigned index = 0; index < dispatch_count; ++index) {
            if (instruction_only(dispatches[index])) { return true; }
        }
        return false;
    }

    bool runnable(unsigned actor) const {
        if (actor < dispatch_count) {
            const auto& d = dispatches[actor];
            if (d.done) { return false; }
            const bool wants_guard = protocol == Protocol::production ? d.pc == 1 : d.pc == 2;
            return !(wants_guard && world.guard_held);
        }

        if (!teardown_present || teardown.done) { return false; }
        const bool wants_guard = protocol == Protocol::production ? teardown.pc == 3 : teardown.pc == 1;
        const bool pausing = teardown.pc == 2;
        return !(wants_guard && world.guard_held) && !(pausing && pause_blocked());
    }

    bool step(unsigned actor) { return actor < dispatch_count ? step_dispatch(actor) : step_teardown(); }

    bool finished() const {
        for (unsigned index = 0; index < dispatch_count; ++index) {
            if (!dispatches[index].done) { return false; }
        }
        return !teardown_present || teardown.done;
    }
};

struct Exploration final {
    unsigned schedules = 0;
    unsigned violations = 0;
    unsigned hangs = 0;
    unsigned contained = 0;
    unsigned deleted = 0;
    unsigned double_deletes = 0;
    const char* first_violation = nullptr;
};

void explore(const Model& state, Exploration& result, unsigned depth) {
    if (depth > 200) {
        ++result.hangs;
        return;
    }

    if (state.world.violation) {
        ++result.schedules;
        ++result.violations;
        if (result.first_violation == nullptr) { result.first_violation = state.world.why; }
        return;
    }

    if (state.finished()) {
        ++result.schedules;
        result.contained += state.world.contained;
        result.deleted += state.world.deletes;
        if (state.world.deletes > 1) { ++result.double_deletes; }
        return;
    }

    bool any = false;

    for (unsigned actor = 0; actor <= state.dispatch_count; ++actor) {
        if (!state.runnable(actor)) { continue; }
        any = true;
        Model next = state;
        next.step(actor);
        explore(next, result, depth + 1);
    }

    if (!any) {
        ++result.schedules;
        ++result.hangs;
    }
}

Model scenario(Protocol protocol, std::initializer_list<unsigned> durations) {
    Model model{protocol, {}, {}, 0, {}, true};
    model.world.entry.open();

    for (const unsigned duration : durations) {
        model.dispatches[model.dispatch_count++].duration = duration;
    }

    return model;
}

void check_exhaustive_production() {
    // Two quick dispatches (win32k CLOSE and the last lease resolve) race the
    // teardown their own completion queued.
    {
        Exploration result;
        explore(scenario(Protocol::production, {1, 1}), result, 0);
        expect(result.schedules > 100 && result.violations == 0 && result.hangs == 0 && result.double_deletes == 0,
            "production: no interleaving of two retirement dispatches and teardown touches a deleted extension");
        expect(result.deleted + result.contained == result.schedules,
            "production: every schedule either deletes exactly once or contains");
        std::printf("production quick: %u schedules, %u deleted, %u contained\n",
            result.schedules, result.deleted, result.contained);
    }
    // A dispatch stuck in the lower device beyond the poll budget.
    {
        Exploration result;
        explore(scenario(Protocol::production, {1, poll_budget + 3}), result, 0);
        expect(result.violations == 0 && result.hangs == 0 && result.double_deletes == 0,
            "production: a stuck lower request never sees its extension deleted");
        expect(result.contained > 0 && result.deleted + result.contained == result.schedules,
            "production: a stuck request contains the channel instead of deleting it");
        std::printf("production stuck: %u schedules, %u deleted, %u contained\n",
            result.schedules, result.deleted, result.contained);
    }
    // Three dispatches, one of them refused after close.
    {
        Exploration result;
        explore(scenario(Protocol::production, {1, 2, 1}), result, 0);
        expect(result.violations == 0 && result.hangs == 0 && result.double_deletes == 0,
            "production: three concurrent retirement dispatches stay contained in every order");
        std::printf("production three: %u schedules, %u deleted, %u contained\n",
            result.schedules, result.deleted, result.contained);
    }
}

void check_exhaustive_previous() {
    {
        Exploration result;
        explore(scenario(Protocol::previous, {1, 1}), result, 0);
        expect(result.violations > 0,
            "previous protocol: check-then-count lets a dispatch run after the teardown deleted the extension");
        std::printf("previous quick: %u schedules, %u use-after-delete (%s)\n",
            result.schedules, result.violations, result.first_violation != nullptr ? result.first_violation : "-");
    }
    {
        Exploration result;
        explore(scenario(Protocol::previous, {poll_budget + 3}), result, 0);
        expect(result.violations > 0,
            "previous protocol: a bounded poll deletes the extension under a stuck lower request");
        std::printf("previous stuck: %u schedules, %u use-after-delete (%s)\n",
            result.schedules, result.violations, result.first_violation != nullptr ? result.first_violation : "-");
    }
}

// Real threads for TSan: admitted readers of a plain payload against a closer
// that writes it only after the drain. Any missing happens-before edge in the
// Rundown protocol is a reported data race.
void check_threaded_drain() {
    for (unsigned round = 0; round < 64; ++round) {
        Entry entry;
        entry.open();
        int payload = 1;
        std::atomic<unsigned> admitted{0};
        std::atomic<unsigned> refused{0};
        std::atomic<bool> bad{false};
        std::vector<std::thread> threads;

        for (unsigned worker = 0; worker < 4; ++worker) {
            threads.emplace_back([&] {
                for (unsigned iteration = 0; iteration < 200; ++iteration) {
                    if (!entry.acquire()) {
                        refused.fetch_add(1);
                        continue;
                    }

                    if (payload != 1) { bad.store(true); }
                    admitted.fetch_add(1);
                    entry.release();
                }
            });
        }

        std::thread closer([&] {
            std::this_thread::yield();
            entry.close();

            while (!entry.drained()) {
                std::this_thread::yield();
            }

            payload = 0;
        });

        for (auto& thread : threads) { thread.join(); }
        closer.join();
        expect(!bad.load() && admitted.load() + refused.load() == 800 && entry.drained() && payload == 0,
            "threaded: no admitted dispatch observes the retired payload");
    }
}
} // namespace

int main() {
    check_rundown_contract();
    check_exhaustive_production();
    check_exhaustive_previous();
    check_threaded_drain();
    std::printf("%u overlap model checks; %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

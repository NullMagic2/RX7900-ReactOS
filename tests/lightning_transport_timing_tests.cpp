/*
Module: lightning.tests.transport_timing
Purpose: Exercise the actual bounded wait policy and counts-only diagnostics.
Created: 2026-10-22
Architecture: Portable policy tests, not an NT scheduling or GPU completion claim.
*/
#include "lightning_transport.hpp"
#include <stdio.h>

namespace {
using namespace lightning::reactos;
using namespace lightning::reactos::transport_timing;
unsigned checks = 0, failures = 0, clock_reads = 0;
void check(bool value, const char* text) {
    ++checks;
    if (!value) { ++failures; printf("FAIL %s\n", text); }
}
}

// A call here would reproduce the intrusive PIT diagnostic. Counts must not
// depend on a clock even when enabled or when the native call returns an error.
extern "C" LARGE_INTEGER KeQueryPerformanceCounter(LARGE_INTEGER*) {
    ++clock_reads;
    return {-1};
}

int main() {
    using transport_wait::Budget;
    using transport_wait::Pause;
    constexpr uint16_t operation = static_cast<uint16_t>(QLV_SUBMIT);
    Aggregates totals;
    {
        // Production checks COMPLETE before touching its pause budget.
        Scope sample(totals, operation, 17);
        sample.after_submit(); sample.complete();
        check(sample.finish(0) == 0, "immediate completion preserves native success");
    }
    const auto& row = totals.rows[operation];
    check(row.calls == 1 && row.succeeded == 1 && row.submitted == 1 && row.immediate == 1 &&
        row.pending == 0 && row.stalls == 0 && row.sleeps == 0, "immediate reply consumes no stall or yield");
    {
        Budget budget;
        Scope sample(totals, operation, 17);
        sample.after_submit();
        sample.pending();
        check(budget.next(0) == Pause::stall, "first pending reply receives a short poll before a scheduler yield");
        sample.stall();
        sample.pending();
        check(budget.next(250) == Pause::stall, "second pending reply remains in the bounded short-poll phase");
        sample.stall();
        sample.complete(); sample.finish(0);
    }
    check(row.calls == 2 && row.submitted == 2 && row.pending == 2 && row.stalls == 2 &&
        row.short_complete == 1 && row.sleeps == 0, "short-poll completion has one submission and no sleep");
    {
        Budget budget;
        Scope sample(totals, operation, 17);
        sample.after_submit();
        bool bounded = true;
        for (ULONG index = 0; index < transport_wait::short_polls; ++index) {
            sample.pending();
            bounded = bounded && budget.next(uint64_t{index} * 250) == Pause::stall;
            sample.stall();
        }
        check(bounded && transport_wait::short_polls * transport_wait::short_poll_microseconds == 200,
            "actual policy bounds requested initial stalls to eight times twenty-five microseconds");
        sample.pending();
        check(budget.next(2000) == Pause::sleep, "ninth pending reply yields using the original sleep path");
        sample.before_sleep(); sample.complete(); sample.finish(0);
    }
    check(row.calls == 3 && row.submitted == 3 && row.sleeps == 1 && row.stalls == 10 &&
        row.short_complete == 1 && row.succeeded == 3, "completion after yield is distinct from immediate/short completion");

    Budget exhausted;
    bool bounded = true;
    for (ULONG index = 0; index < transport_wait::short_polls; ++index) {
        bounded = bounded && exhausted.next(0) == Pause::stall;
    }
    for (ULONG index = 0; index < transport_wait::maximum_sleeps; ++index) {
        bounded = bounded && exhausted.next(0) == Pause::sleep;
    }
    check(bounded && exhausted.next(0) == Pause::expired && exhausted.next(0) == Pause::expired,
        "a frozen interrupt clock permits only eight stalls and the original six-thousand sleeps");
    Budget deadline;
    check(deadline.next(QLV_KERNEL_EXCHANGE_TIMEOUT_100NS) == Pause::expired,
        "six-second elapsed deadline prevents even the first short poll");
    check(deadline.next(UINT64_MAX) == Pause::expired,
        "wrapped or excessive elapsed time fails closed instead of extending a timeout");
    Budget between_polls;
    check(between_polls.next(QLV_KERNEL_EXCHANGE_TIMEOUT_100NS - 1) == Pause::stall &&
        between_polls.next(QLV_KERNEL_EXCHANGE_TIMEOUT_100NS) == Pause::expired,
        "real elapsed deadline is rechecked during short polling");
    Budget during_yield;
    for (ULONG index = 0; index < transport_wait::short_polls; ++index) { during_yield.next(0); }
    check(during_yield.next(2000) == Pause::sleep &&
        during_yield.next(QLV_KERNEL_EXCHANGE_TIMEOUT_100NS) == Pause::expired,
        "real elapsed deadline still terminates the yielding phase");

    {
        Scope sample(totals, operation, 29);
        check(sample.finish(-19) == -19, "early native rejection preserves its error");
    }
    check(row.calls == 4 && row.failed == 1 && row.submitted == 3 && row.mixed_contexts,
        "pre-submit rejection adds no fabricated submit, poll or completion");
    {
        Scope sample(totals, operation, 17);
        sample.after_submit(); sample.pending(); sample.stall(); sample.finish(-7);
    }
    check(row.calls == 5 && row.failed == 2 && row.submitted == 4 && row.short_complete == 1 && row.immediate == 1,
        "disconnect or protocol error after pending cannot claim completion or a second submit");
    {
        Scope sample(totals, operation, 17);
        sample.after_submit(); sample.pending(); sample.before_sleep(); sample.finish(-9);
    }
    check(row.calls == 6 && row.failed == 3 && row.submitted == 5 && row.sleeps == 2 && row.short_complete == 1,
        "timeout after scheduler yield remains a measured error without completion");
    { Scope sample(totals, operation, 17); }
    check(row.calls == 7 && row.failed == 4, "all early exits retain a default failure status");

    Aggregates bounded_counts;
    uint64_t value = UINT64_MAX - 2;
    bounded_counts.add(value, 2);
    check(value == UINT64_MAX && !bounded_counts.saturated, "exact maximum counter remains representable");
    bounded_counts.add(value, 1);
    check(value == UINT64_MAX && bounded_counts.saturated, "counter overflow saturates with a sticky flag");
    bounded_counts.rows[0].calls = UINT64_MAX;
    { Scope sample(bounded_counts, UINT16_MAX, 0); sample.finish(-1); }
    check(bounded_counts.rows[0].calls == UINT64_MAX && bounded_counts.rows[0].failed == 1,
        "unknown operation remains in bounded row zero without counter wrap");
    check(clock_reads == 0, "enabled counts never invoke PIT QPC in any success, pending, failure or overflow path");
    printf("Lightning transport wait/counts: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

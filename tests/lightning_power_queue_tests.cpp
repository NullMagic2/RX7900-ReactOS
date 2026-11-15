/*
Module: lightning.tests.power_queue
Purpose: Verify allocation-free native power admission, FIFO wrap and stop drain.
Created: 2026-10-21
Architecture: Uses the exact native queue; does not emulate NT thread completion.
*/
#include <cstdio>
#include <initializer_list>
#include "lightning_power_queue.hpp"
#include "lightning_display_gate.hpp"
#include "lightning_lifecycle.hpp"

namespace {
unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* description) {
    ++checks;

    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", description);
    }
}

struct Request final { unsigned sequence = 0; };
using Queue = lightning::reactos::PowerQueue<Request*>;

void check_admission_and_wrap() {
    Queue queue;
    Request requests[Queue::capacity]{};
    Request extra;
    Request* output = &extra;
    expect(!queue.pop(output) && output == nullptr, "empty pop has no stale request ownership");
    expect(!queue.push(nullptr) && queue.count() == 0, "null cannot consume an admission slot");

    for (unsigned cycle = 0; cycle < 128; ++cycle) {
        for (size_t slot = 0; slot < Queue::capacity; ++slot) {
            requests[slot].sequence = cycle;
            expect(queue.push(&requests[slot]), "all reserved slots accept without allocation");
        }

        expect(queue.count() == Queue::capacity, "full queue reports exact bounded count");
        expect(!queue.push(&extra) && queue.count() == Queue::capacity,
            "overflow cannot displace an already accepted request");

        for (size_t slot = 0; slot < Queue::capacity; ++slot) {
            expect(queue.pop(output) && output == &requests[slot] && output->sequence == cycle,
                "full wrap preserves request identity and FIFO order");
        }

        expect(queue.count() == 0, "accepted prefix drains exactly once");
        expect(queue.push(&extra) && queue.pop(output) && output == &extra,
            "partial wrap does not require a fresh allocation or reset");
    }
}

void check_stop_preserves_prefix() {
    for (size_t accepted = 0; accepted <= Queue::capacity; ++accepted) {
        Queue queue;
        Request requests[Queue::capacity]{};

        for (size_t slot = 0; slot < accepted; ++slot) {
            expect(queue.push(&requests[slot]), "prefix accepts before consumer stop");
        }

        queue.stop();
        queue.stop();
        expect(queue.stopping(), "stop is idempotent");
        expect(!queue.push(&requests[0]) && queue.count() == accepted,
            "stop rejects admission without discarding owned requests");

        for (size_t slot = 0; slot < accepted; ++slot) {
            Request* output = nullptr;
            expect(queue.pop(output) && output == &requests[slot], "stop still permits exact FIFO drain");
        }

        Request* output = &requests[0];
        expect(!queue.pop(output) && output == nullptr && queue.count() == 0,
            "stopped and drained queue owns no request");
        expect(!queue.push(&requests[0]), "drain never reopens a stopped queue");
    }
}

void check_display_power_admission() {
    using namespace lightning::reactos;
    for (bool recognized : {false, true}) {
        for (bool kernel : {false, true}) {
            expect(display_admission(DisplayRequest::open, recognized, kernel, false) == DisplayAdmission::not_ready,
                "no foreign OPEN can initialize an unstarted or suspended lower device");
            expect(display_admission(DisplayRequest::open, recognized, kernel, true) == DisplayAdmission::allow,
                "owned D0 OPEN retains the original lower initialization path");
        }
    }

    expect(display_admission(DisplayRequest::hardware, true, true, false) == DisplayAdmission::not_ready,
        "late kernel SET_MODE/PRESENT/MAP/WAIT cannot reach hardware after quiesce");
    expect(display_admission(DisplayRequest::hardware, true, true, true) == DisplayAdmission::allow,
        "kernel graphics work remains permitted in owned D0");
    for (bool active : {false, true}) {
        expect(display_admission(DisplayRequest::inventory, true, false, active) == DisplayAdmission::allow,
            "pointer-free cached display inventory remains available across D3");
        expect(display_admission(DisplayRequest::retirement, true, true, active) == DisplayAdmission::allow,
            "kernel RAM UNMAP and software ABANDON remain valid across D3");
        expect(display_admission(DisplayRequest::hardware, true, false, active) == DisplayAdmission::access_denied,
            "user callers cannot obtain or mutate kernel staging leases even in D0");
        expect(display_admission(DisplayRequest::retirement, true, false, active) == DisplayAdmission::access_denied,
            "user callers cannot retire another kernel owner's lease in any power state");
    }

    for (auto request : {DisplayRequest::inventory, DisplayRequest::retirement, DisplayRequest::hardware}) {
        expect(display_admission(request, false, true, false) == DisplayAdmission::not_ready,
            "unrecognized lower operations have no inferred offline safety");
        expect(display_admission(request, false, true, true) == DisplayAdmission::allow,
            "active standalone transport keeps its existing lower control behavior");
    }
}
// lightning-lifecycle (2026-10-25): the owned FIFO also carries the final
// system shutdown SET; device and system requests keep their arrival order.
void check_owned_routes_share_the_fifo() {
    using namespace lightning::reactos;
    struct Power final { bool system; uint32_t state; };
    const Power arrivals[] = {{false, power_state::device_d3}, {true, power_state::system_shutdown},
        {false, power_state::device_d0}, {true, power_state::system_working}, {false, power_state::device_d3}};
    lightning::reactos::PowerQueue<const Power*> queue;

    for (const auto& request : arrivals) {
        if (power_route(true, request.system, request.state) == PowerRoute::owned) {
            expect(queue.push(&request), "an owned SET is queued");
        }
    }

    expect(queue.count() == 4, "S0 is forwarded, the shutdown SET is owned");
    const Power* output = nullptr;
    expect(queue.pop(output) && output == &arrivals[0], "device D3 first");
    expect(queue.pop(output) && output == &arrivals[1], "then the shutdown SET in arrival order");
    expect(queue.pop(output) && output == &arrivals[2], "then D0");
    expect(queue.pop(output) && output == &arrivals[4], "then the last D3");
}

// The request gate's active input comes from the lifecycle state machine:
// started and never contained, with the display neither quiesced nor lost.
void check_gate_follows_lifecycle() {
    using namespace lightning::reactos;

    for (int contained = 0; contained < 2; ++contained) {
        for (int step = 0; step < 6; ++step) {
            DeviceLifecycle lifecycle{};

            if (step >= 1) {
                lifecycle.admit_start();
                lifecycle.started();
            }

            if (step == 2) {
                lifecycle.powered_down();
            }

            if (step == 3) {
                lifecycle.admit_query(false);
            }

            if (step == 4) {
                lifecycle.stopped();
            }

            if (step == 5) {
                lifecycle.surprise_removed();
            }

            if (contained != 0) {
                lifecycle.contain();
            }

            const bool active = lifecycle.accepts_work();
            expect(active == (contained == 0 && step == 1), "only an uncontained started device accepts work");
            expect(display_admission(DisplayRequest::hardware, true, true, active) ==
                (active ? DisplayAdmission::allow : DisplayAdmission::not_ready),
                "kernel display hardware requests follow the lifecycle");
            expect(display_admission(DisplayRequest::hardware, true, false, active) == DisplayAdmission::access_denied,
                "user display hardware requests are refused in every lifecycle phase");
            expect(display_admission(DisplayRequest::retirement, true, true, active) == DisplayAdmission::allow,
                "kernel lease retirement survives every lifecycle phase and containment");
        }
    }
}
} // namespace

int main() {
    check_admission_and_wrap();
    check_stop_preserves_prefix();
    check_display_power_admission();
    check_owned_routes_share_the_fifo();
    check_gate_follows_lifecycle();
    std::printf("Lightning power queue: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

/*
Module: rx7900.tests.gpu_process_restart
Purpose: Reject native GPU-process restart while any old owner remains.
Created: 2026-10-22
Architecture: Exact NT SDK fixture calls the production service admission,
UserQueues report and ProcessMemory ownership inspection without hardware I/O.
*/
#include "../src/reactos/gpu_process.hpp"
#include <stdio.h>

namespace {
rx7900::reactos::GpuProcessService service{};
unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* description) {
    ++checks;
    if (!condition) {
        ++failures;
        printf("FAIL: %s\n", description);
    }
}
}

int main() {
    expect(!service.can_restart_after_reset(), "uninitialized service has no reset proof");
    service.initialized = true;
    service.registered = true;
    service.failed = true;
    service.parked = true;
    expect(!service.can_restart_after_reset(),
        "failed parked state alone never establishes native resource retirement");
    service.reset_retired = true;
    service.next_handle = 0xfedcba9876543210ULL;
    expect(service.can_restart_after_reset(),
        "confirmed retired service with no remaining owners permits a real ASIC readiness check");
    expect(service.next_handle == 0xfedcba9876543210ULL,
        "admission does not recycle any previously issued native capability");

    bool* const required[]{&service.initialized, &service.registered,
        &service.failed, &service.parked, &service.reset_retired};
    for (bool* flag : required) {
        *flag = false;
        expect(!service.can_restart_after_reset(),
            "every lifecycle prerequisite is required before reinitialization");
        *flag = true;
    }
    service.reset_locked = true;
    expect(!service.can_restart_after_reset(),
        "reset callback must return ownership before a new service incarnation is admitted");
    service.reset_locked = false;

    // Exercise every actual session slot, including a hole after a live owner.
    // Sentinel native objects are compared only; the admission method must not
    // dereference process pointers or inspect stale hardware memory.
    for (auto& session : service.sessions) {
        session.process = reinterpret_cast<PEPROCESS>(static_cast<ULONG_PTR>(1));
        expect(!service.can_restart_after_reset(), "referenced NT process excludes restart");
        session.process = nullptr;
        uint64_t* const identities[]{&session.handle, &session.tables.generation,
            &session.table_backup.generation, &session.context.generation,
            &session.gang_context.generation};
        for (uint64_t* identity : identities) {
            *identity = 1;
            expect(!service.can_restart_after_reset(),
                "native session and backing identities remain owned until actually retired");
            *identity = 0;
        }
        session.pasid = 1;
        expect(!service.can_restart_after_reset(), "PASID excludes restart");
        session.pasid = 0;
        session.gang = 1;
        expect(!service.can_restart_after_reset(), "MES gang excludes restart");
        session.gang = 0;
        session.quarantined = true;
        expect(!service.can_restart_after_reset(), "uncertain session retirement excludes restart");
        session.quarantined = false;
        session.exiting = true;
        expect(!service.can_restart_after_reset(), "pending process-exit cleanup excludes restart");
        session.exiting = false;

        for (auto& queue : session.queues) {
            queue.handle = 1;
            expect(!service.can_restart_after_reset(), "public queue capability excludes restart");
            queue.handle = 0;
            queue.id = 1;
            expect(!service.can_restart_after_reset(), "native MES queue identity excludes restart");
            queue.id = 0;
            queue.ring.generation = 1;
            expect(!service.can_restart_after_reset(), "retained queue ring backing excludes restart");
            queue.ring.generation = 0;
            queue.mqd.generation = 1;
            expect(!service.can_restart_after_reset(), "retained queue MQD backing excludes restart");
            queue.mqd.generation = 0;
            queue.quarantined = true;
            expect(!service.can_restart_after_reset(), "uncertain queue retirement excludes restart");
            queue.quarantined = false;
        }
        expect(service.can_restart_after_reset(),
            "retiring exact owners restores admission without resetting handle generation");
    }
    expect(service.next_handle == 0xfedcba9876543210ULL,
        "failed and successful admission checks preserve the monotonic capability counter");
    printf("gpu-process-restart: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

/*
Module: lightning.tests.lifecycle_native_platform
Purpose: Real Win32 threads, locks and child processes for the native lifecycle fixture.
Created: 2026-10-25
Architecture: Compiled with the MinGW Win32 headers, separate from the ReactOS SDK
translation unit (tests/lightning_lifecycle_native_tests.cpp) whose NT declarations
conflict with windows.h. Only this narrow C-linkage surface crosses between them. Every
wait is bounded by the caller's timeout; children are killed when they overrun.
*/
#include <windows.h>
#include <stdio.h>

namespace {
CRITICAL_SECTION locks[2];
CONDITION_VARIABLE dispatcher_signal;
INIT_ONCE initialized = INIT_ONCE_STATIC_INIT;

BOOL CALLBACK initialize(PINIT_ONCE, PVOID, PVOID*) {
    InitializeCriticalSection(&locks[0]);
    InitializeCriticalSection(&locks[1]);
    InitializeConditionVariable(&dispatcher_signal);
    return TRUE;
}

void ensure() {
    InitOnceExecuteOnce(&initialized, initialize, nullptr, nullptr);
}

struct Start final {
    void (*routine)(void*);
    void* context;
};

DWORD WINAPI trampoline(LPVOID parameter) {
    Start start = *static_cast<Start*>(parameter);
    HeapFree(GetProcessHeap(), 0, parameter);
    start.routine(start.context);
    return 0;
}
} // namespace

extern "C" {

void* lifecycle_platform_thread(void (*routine)(void*), void* context) {
    ensure();
    auto* start = static_cast<Start*>(HeapAlloc(GetProcessHeap(), 0, sizeof(Start)));

    if (start == nullptr) {
        return nullptr;
    }

    start->routine = routine;
    start->context = context;
    HANDLE thread = CreateThread(nullptr, 0, trampoline, start, 0, nullptr);

    if (thread == nullptr) {
        HeapFree(GetProcessHeap(), 0, start);
    }

    return thread;
}

int lifecycle_platform_join(void* thread, unsigned milliseconds) {
    return WaitForSingleObject(static_cast<HANDLE>(thread), milliseconds) == WAIT_OBJECT_0 ? 1 : 0;
}

void lifecycle_platform_close(void* thread) {
    CloseHandle(static_cast<HANDLE>(thread));
}

void lifecycle_platform_exit_thread() {
    ExitThread(0);
}

unsigned long lifecycle_platform_thread_id() {
    return GetCurrentThreadId();
}

void lifecycle_platform_lock(int which) {
    ensure();
    EnterCriticalSection(&locks[which]);
}

void lifecycle_platform_unlock(int which) {
    LeaveCriticalSection(&locks[which]);
}

// Caller holds lock 0 (the dispatcher lock). 1: woken, 0: timed out.
int lifecycle_platform_wait(unsigned milliseconds) {
    return SleepConditionVariableCS(&dispatcher_signal, &locks[0], milliseconds) ? 1 : 0;
}

void lifecycle_platform_broadcast() {
    ensure();
    WakeAllConditionVariable(&dispatcher_signal);
}

void lifecycle_platform_sleep(unsigned milliseconds) {
    Sleep(milliseconds);
}

unsigned long long lifecycle_platform_ticks() {
    return GetTickCount64();
}

// Run this executable again with arguments; -1 when it could not start or
// overran the timeout (it is then terminated), else its exit code.
int lifecycle_platform_child(const char* arguments, unsigned milliseconds) {
    char path[MAX_PATH]{};

    if (GetModuleFileNameA(nullptr, path, MAX_PATH) == 0) {
        return -1;
    }

    char command[2 * MAX_PATH]{};
    snprintf(command, sizeof(command), "\"%s\" %s", path, arguments);
    STARTUPINFOA startup{};
    startup.cb = sizeof(startup);
    PROCESS_INFORMATION process{};

    if (!CreateProcessA(nullptr, command, nullptr, nullptr, FALSE, 0, nullptr, nullptr, &startup, &process)) {
        return -1;
    }

    int result = -1;

    if (WaitForSingleObject(process.hProcess, milliseconds) == WAIT_OBJECT_0) {
        DWORD code = 0;

        if (GetExitCodeProcess(process.hProcess, &code)) {
            result = static_cast<int>(code);
        }
    } else {
        TerminateProcess(process.hProcess, 99);
        WaitForSingleObject(process.hProcess, 5000);
    }

    CloseHandle(process.hThread);
    CloseHandle(process.hProcess);
    return result;
}

void lifecycle_platform_exit(int code) {
    fflush(stdout);
    fflush(stderr);
    ExitProcess(static_cast<UINT>(code));
}

} // extern "C"

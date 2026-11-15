/*
Module: rx7900.reactos.gpu_process
Purpose: Bind bounded GPUVM and queue ownership to real NT process lifetimes.
Created: 2026-10-22
Architecture: VideoPort service boundary over the portable GpuProcessCore; this
file owns only NT objects: the process-exit notification and its registry, the
service mutex, the PASSIVE_LEVEL reaper thread, process object references,
captured METHOD_BUFFERED requests and the AsicBringup reset callbacks.
NT notification registration owns the callback code lifetime.
*/
#include <ntifs.h>
#include "gpu_process.hpp"
#include "../opengl/gl_shader_gfx11.hpp"

namespace rx7900::reactos {
namespace {
using process_abi::Operation;
using process_abi::Outcome;

// PsSetCreateProcessNotifyRoutine has no caller context. This one driver-wide,
// fixed registry is its explicit owner; its mutex precedes every service mutex.
// Callback and registration/removal run at PASSIVE_LEVEL. Removal waits for any
// callback after dropping the registry mutex, so no callback can wait on itself.
KMUTEX registry_lock{};
GpuProcessService* services[hardware::user_queue_max_processes]{};
bool driver_started = false;

class Lock final {
public:
    explicit Lock(KMUTEX& mutex) noexcept : mutex_(mutex) {
        KeWaitForSingleObject(&mutex_, Executive, KernelMode, FALSE, nullptr);
    }
    ~Lock() noexcept { KeReleaseMutex(&mutex_, FALSE); }
    Lock(const Lock&) = delete;
    Lock& operator=(const Lock&) = delete;
private:
    KMUTEX& mutex_;
};

void acquire_service(void* context) noexcept {
    KeWaitForSingleObject(&static_cast<GpuProcessService*>(context)->lock, Executive, KernelMode,
                          FALSE, nullptr);
}

void release_service(void* context) noexcept {
    KeReleaseMutex(&static_cast<GpuProcessService*>(context)->lock, FALSE);
}

bool dma_bus(void* context, memory::Handle handle, uint64_t& bus) noexcept {
    return static_cast<AsicBringup*>(context)->asic.describe_gtt_bus(handle, bus);
}

bool queue_config(void* context, hardware::PasidAllocator& pasids,
                  hardware::UserQueueConfig& config) noexcept {
    return static_cast<AsicBringup*>(context)->asic.user_queue_config(pasids, config);
}

uint64_t now_us(void*) noexcept { return KeQueryInterruptTime() / 10; }

// The session's EPROCESS stays referenced while the session names it, so a
// recycled object address can never be authorized as the old process.
void reference_process(void*, void* process) noexcept {
    ObReferenceObject(static_cast<PEPROCESS>(process));
}

void dereference_process(void*, void* process) noexcept {
    ObDereferenceObject(static_cast<PEPROCESS>(process));
}

// IB construction computes viewport/guard-band/offset floats. NT kernel code
// must save the caller's floating-point state around any x87/SSE use.
bool run_with_fpu(void*, bool (*body)(void*) noexcept, void* argument) noexcept {
    KFLOATING_SAVE save{};
    if (!NT_SUCCESS(KeSaveFloatingPointState(&save))) return false;
    const bool result = body(argument);
    KeRestoreFloatingPointState(&save);
    return result;
}

// Bounded PASSIVE_LEVEL wait for the synchronous present fence.
void wait_us(void*, uint32_t microseconds) noexcept {
    LARGE_INTEGER interval{};
    interval.QuadPart = -static_cast<LONGLONG>(microseconds) * 10;
    KeDelayExecutionThread(KernelMode, FALSE, &interval);
}

// gl-shaders' kernel-safe generators; the key was validated by the decoder.
bool generate_vertex(void* context, const opengl::gfx11_abi::ShaderKey& key, sdma::CommandBuffer output,
                     opengl::gfx11_abi::ProgramInfo& info) noexcept {
    auto& workspace = *static_cast<opengl::shader::GeneratorWorkspace*>(context);
    return opengl::shader::generate_vertex_shader(key, output, info, workspace) == opengl::shader::Status::success;
}

bool generate_pixel(void* context, const opengl::gfx11_abi::ShaderKey& key, sdma::CommandBuffer output,
                    opengl::gfx11_abi::ProgramInfo& info) noexcept {
    auto& workspace = *static_cast<opengl::shader::GeneratorWorkspace*>(context);
    return opengl::shader::generate_pixel_shader(key, output, info, workspace) == opengl::shader::Status::success;
}

bool begin_reset(void* context) noexcept;
void end_reset(void* context) noexcept;
bool retire_reset(void* context) noexcept;
bool retire_shutdown(void* context) noexcept;

// Runs in the exiting process's context at PASSIVE_LEVEL with process-manager
// locks held: it only marks the sessions exiting (no new request is admitted)
// and wakes the reaper. MES removal, waits and releases never run here.
void NTAPI process_notify(HANDLE, HANDLE process_id, BOOLEAN create) {
    if (create) return;
    // Comparing the referenced object as well as the ID prevents PID reuse from
    // closing a new process's session; no identity is taken from request data.
    PEPROCESS process = PsGetCurrentProcess();
    if (PsGetCurrentProcessId() != process_id) return;
    Lock registry(registry_lock);
    for (auto* service : services) {
        if (service == nullptr) continue;
        bool matched = false;
        {
            Lock lock(service->lock);
            matched = process_core_mark_exit(*service, process);
        }
        if (matched) KeSetEvent(&service->exit_reap_event, IO_NO_INCREMENT, FALSE);
    }
}

// The reaper: each pass takes the service mutex, reaps exited sessions (bounded
// idle wait, MES eviction, unmap, VM destroy, release) and drops the mutex
// before an escalation reaches the device owner. It sleeps until a process
// exit, resume, rearm or reset retirement signals it, or polls every
// process_reap_retry_us while an exited session still waits for its fences.
void NTAPI reap_exited_sessions(void* context) {
    auto& service = *static_cast<GpuProcessService*>(context);
    const ProcessLock lock{&service, acquire_service, release_service};
    for (;;) {
        const auto pass = process_core_reaper_pass(service, lock, service.exit_reap_stop);
        if (pass.stopped) break;
        LARGE_INTEGER retry{};
        retry.QuadPart = -static_cast<LONGLONG>(pass.retry_us) * 10;
        KeWaitForSingleObject(&service.exit_reap_event, Executive, KernelMode,
                              FALSE, pass.retry_us != 0 ? &retry : nullptr);
    }
    PsTerminateSystemThread(STATUS_SUCCESS);
}

bool start_exit_reaper(GpuProcessService& service) noexcept {
    if (service.exit_reap_thread != nullptr) return true;
    KeInitializeEvent(&service.exit_reap_event, SynchronizationEvent, FALSE);
    service.exit_reap_stop = false;
    OBJECT_ATTRIBUTES attributes{};
    InitializeObjectAttributes(&attributes, nullptr, OBJ_KERNEL_HANDLE, nullptr, nullptr);
    HANDLE thread = nullptr;
    if (!NT_SUCCESS(PsCreateSystemThread(&thread, SYNCHRONIZE, &attributes,
        nullptr, nullptr, reap_exited_sessions, &service))) return false;
    service.exit_reap_thread = thread;
    return true;
}

// KernelGraphicsBridge::reset_confirmed: AsicInit's reset proof, which holds
// while release_after_reset retires this service (retire_reset below).
bool asic_reset_proven(void* context) noexcept {
    return static_cast<AsicBringup*>(context)->asic.reset_proven();
}

// The bridge follows the current ASIC incarnation; null fences (submission not
// enabled) make every kernel-ring submit unavailable.
void bind_kernel_bridge(GpuProcessService& service, AsicBringup& bringup) noexcept {
    service.kernel_bridge.queues = &bringup.asic.kernel_queues();
    service.kernel_bridge.fences = bringup.asic.kernel_fences();
    service.kernel_bridge.reset_context = &bringup;
    service.kernel_bridge.reset_confirmed = asic_reset_proven;
}

ProcessPlatform native_platform(GpuProcessService& service, AsicBringup& bringup) noexcept {
    ProcessPlatform platform{};
    platform.context = &bringup;
    platform.gtt = &bringup.asic.gtt_manager();
    platform.vram = &bringup.vram;
    platform.dma_bus = dma_bus;
    platform.queue_config = queue_config;
    platform.now_us = now_us;
    platform.reference = reference_process;
    platform.dereference = dereference_process;
    platform.kernel_ring = service.kernel_bridge.submitter();
    return platform;
}
} // namespace

NTSTATUS gpu_process_driver_start() noexcept {
    if (driver_started) return STATUS_SUCCESS;
    KeInitializeMutex(&registry_lock, 0);
    const NTSTATUS status = PsSetCreateProcessNotifyRoutine(process_notify, FALSE);
    if (NT_SUCCESS(status)) driver_started = true;
    return status;
}

void gpu_process_driver_stop() noexcept {
    if (!driver_started) return;
    PsSetCreateProcessNotifyRoutine(process_notify, TRUE);
    driver_started = false;
    // The owner calls shutdown on devices first and may unload only if all
    // services and the physical IRQ/hardware callbacks are retired.
}

bool gpu_process_initialize(GpuProcessService& service, AsicBringup& bringup) noexcept {
    if (!driver_started || VideoPortGetCurrentIrql() != PASSIVE_LEVEL) return false;
    bool ready = false;
    bool escalate = false;
    ProcessRecoveryHook hook{};
    {
        Lock registry(registry_lock);
        if (service.initialized) {
            Lock lock(service.lock);
            if (service.bringup != &bringup) return false;
            bind_kernel_bridge(service, bringup);
            // A running service only confirms the ASIC configuration; a reset-
            // retired one reconstructs its live sessions on the new incarnation.
            ready = process_core_rearm(service);
            escalate = process_core_take_escalation(service, hook);
            if (ready) KeSetEvent(&service.exit_reap_event, IO_NO_INCREMENT, FALSE);
        } else {
            GpuProcessService** vacant = nullptr;
            for (auto& slot : services) if (slot == nullptr) { vacant = &slot; break; }
            if (vacant == nullptr) return false;
            KeInitializeMutex(&service.lock, 0);
            service.bringup = &bringup;
            service.graphics.generator = {&service.shader_workspace, generate_vertex, generate_pixel};
            service.graphics.fpu = {nullptr, run_with_fpu};
            service.graphics.wait_us = wait_us;
            bind_kernel_bridge(service, bringup);
            if (!process_core_bind(service, native_platform(service, bringup)) || !start_exit_reaper(service))
                return false;
            service.failed = false;
            service.parked = false;
            service.reset_locked = false;
            service.reset_retired = false;
            service.initialized = true;
            service.registered = true;
            bringup.process_owner = &service;
            bringup.process_reset_begin = begin_reset;
            bringup.process_reset_retire = retire_reset;
            bringup.process_reset_end = end_reset;
            bringup.process_shutdown = retire_shutdown;
            *vacant = &service;
            ready = true;
        }
    }
    // A reconstruction whose teardown stayed uncertain asks for another
    // destructive recovery; the owner's hook never runs under a process lock.
    if (escalate) hook.request(hook.context);
    return ready;
}

void gpu_process_set_recovery_hook(GpuProcessService& service, void* context,
                                   void (*request)(void*) noexcept) noexcept {
    if (!service.initialized || VideoPortGetCurrentIrql() != PASSIVE_LEVEL) return;
    Lock lock(service.lock);
    service.recovery = {request != nullptr ? context : nullptr, request};
}

VP_STATUS gpu_process_start_io(GpuProcessService& service, VIDEO_REQUEST_PACKET& packet) noexcept {
    packet.StatusBlock->Information = 0;
    if (!service.initialized || VideoPortGetCurrentIrql() != PASSIVE_LEVEL)
        return ERROR_INVALID_FUNCTION;
    if (packet.InputBuffer == nullptr || packet.OutputBuffer == nullptr ||
        packet.InputBufferLength < sizeof(process_abi::Request) ||
        packet.OutputBufferLength < sizeof(process_abi::Reply)) return ERROR_INVALID_PARAMETER;
    Lock lock(service.lock);
    auto* input = static_cast<uint8_t*>(packet.InputBuffer);
    if (!process_abi::decode_request(ByteView(input, packet.InputBufferLength), service.request) ||
        !process_abi::valid_request_fields(service.request)) return ERROR_INVALID_PARAMETER;
    const auto op = static_cast<Operation>(service.request.operation);
    const bool transfer = op == Operation::copy || op == Operation::fill;
    // Version-3 submit/upload payloads are consumed in place from the captured
    // system buffer (bounded by bulk_bytes); everything else is copied first.
    const bool in_place = op == Operation::graphics_submit || op == Operation::buffer_upload;
    const bool readback = op == Operation::buffer_readback;
    const uint32_t payload = op == Operation::write || in_place ?
        static_cast<uint32_t>(service.request.bytes) :
        transfer ? static_cast<uint32_t>(sizeof(process_abi::Transfer)) : 0;
    if (packet.InputBufferLength != sizeof(process_abi::Request) + payload ||
        ((op == Operation::read || readback) && packet.OutputBufferLength < sizeof(process_abi::Reply) +
         service.request.bytes)) return ERROR_INSUFFICIENT_BUFFER;
    service.bulk_input = ByteView(nullptr, 0);
    service.bulk_output = nullptr;
    service.bulk_output_bytes = 0;
    if (in_place) {
        service.bulk_input = ByteView(input + sizeof(process_abi::Request), payload);
    } else if (payload != 0) {
        VideoPortMoveMemory(service.payload, input + sizeof(process_abi::Request), payload);
    }
    if (readback) {
        // The request header was captured above; the reply region may alias it.
        service.bulk_output = static_cast<uint8_t*>(packet.OutputBuffer) + sizeof(process_abi::Reply);
        service.bulk_output_bytes = static_cast<uint32_t>(service.request.bytes);
    }
    service.reply = {sizeof(process_abi::Reply), service.request.abi_version, 0, 0, 0, 0, 0, 0};
    service.reply.outcome = static_cast<uint32_t>(process_core_execute(service, PsGetCurrentProcess()));
    service.bulk_input = ByteView(nullptr, 0);
    service.bulk_output = nullptr;
    // System buffers may alias: all request bytes were captured before output.
    // A refused request (buffer_lost included) returns no payload bytes.
    VideoPortMoveMemory(packet.OutputBuffer, &service.reply, sizeof(service.reply));
    if (service.reply.payload_bytes != 0 && !readback) {
        auto* output = static_cast<uint8_t*>(packet.OutputBuffer);
        VideoPortMoveMemory(output + sizeof(service.reply), service.payload, service.reply.payload_bytes);
    }
    packet.StatusBlock->Information = sizeof(service.reply) + service.reply.payload_bytes;
    return NO_ERROR;
}

bool gpu_process_suspend(GpuProcessService& service) noexcept {
    if (!service.initialized) return true;
    Lock lock(service.lock);
    return process_core_suspend(service);
}

bool gpu_process_resume(GpuProcessService& service) noexcept {
    if (!service.initialized) return true;
    bool resumed = false;
    {
        Lock lock(service.lock);
        resumed = process_core_resume(service);
    }
    // Sessions that exited while parked are reaped now.
    KeSetEvent(&service.exit_reap_event, IO_NO_INCREMENT, FALSE);
    return resumed;
}

namespace {
bool reset_confirmed(void*) noexcept { return true; }
// mode1_reset_asic: begin before the reset (the service mutex stays held until
// end_reset, so neither the reaper nor an IOCTL observes the reset window),
// retire inside AsicInit::release_after_reset's confirmed-reset window.
bool begin_reset(void* context) noexcept {
    auto& service = *static_cast<GpuProcessService*>(context);
    if (VideoPortGetCurrentIrql() != PASSIVE_LEVEL) return false;
    KeWaitForSingleObject(&service.lock, Executive, KernelMode, FALSE, nullptr);
    process_core_begin_reset(service);
    return true;
}
void end_reset(void* context) noexcept {
    auto& service = *static_cast<GpuProcessService*>(context);
    if (service.reset_locked) {
        service.reset_locked = false;
        KeReleaseMutex(&service.lock, FALSE);
    }
}
bool retire_reset(void* context) noexcept {
    auto& service = *static_cast<GpuProcessService*>(context);
    // Called by AsicInit only after its reset proof: confirmation holds here.
    const bool released = process_core_retire_reset(service, reset_confirmed, nullptr);
    KeSetEvent(&service.exit_reap_event, IO_NO_INCREMENT, FALSE);
    return released;
}
bool retire_shutdown(void* context) noexcept {
    return gpu_process_shutdown(*static_cast<GpuProcessService*>(context));
}
} // namespace

bool gpu_process_shutdown(GpuProcessService& service) noexcept {
    if (!service.initialized) return true;
    if (VideoPortGetCurrentIrql() != PASSIVE_LEVEL) return false;
    Lock registry(registry_lock);
    {
        Lock lock(service.lock);
        if (!process_core_close_all(service)) return false;
        service.exit_reap_stop = true;
        KeSetEvent(&service.exit_reap_event, IO_NO_INCREMENT, FALSE);
    }
    if (service.exit_reap_thread != nullptr) {
        if (!NT_SUCCESS(ZwWaitForSingleObject(service.exit_reap_thread, FALSE, nullptr))) return false;
        ZwClose(service.exit_reap_thread);
        service.exit_reap_thread = nullptr;
    }
    Lock lock(service.lock);
    for (auto& slot : services) if (slot == &service) slot = nullptr;
    service.registered = false;
    service.recovery = {};
    service.bringup->process_owner = nullptr;
    service.bringup->process_reset_begin = nullptr;
    service.bringup->process_reset_retire = nullptr;
    service.bringup->process_reset_end = nullptr;
    service.bringup->process_shutdown = nullptr;
    service.initialized = false;
    service.reset_retired = false;
    return true;
}

uint32_t gpu_process_quarantine_vram(GpuProcessService& service, uint64_t mc_address, uint64_t bytes,
                                     uint32_t* owners) noexcept {
    if (owners != nullptr) *owners = 0;
    if (!service.initialized || VideoPortGetCurrentIrql() != PASSIVE_LEVEL) return 0;
    Lock lock(service.lock);
    return process_core_quarantine_vram(service, mc_address, bytes, owners);
}

bool gpu_process_configure_scanout(GpuProcessService& service, const GraphicsScanout* scanout) noexcept {
    if (!service.initialized || VideoPortGetCurrentIrql() != PASSIVE_LEVEL) return false;
    Lock lock(service.lock);
    return process_core_configure_scanout(service, scanout);
}

VP_STATUS gpu_process_present(GpuProcessService& service, VIDEO_REQUEST_PACKET& packet) noexcept {
    packet.StatusBlock->Information = 0;
    if (!service.initialized || VideoPortGetCurrentIrql() != PASSIVE_LEVEL)
        return ERROR_INVALID_FUNCTION;
    if (packet.InputBuffer == nullptr || packet.OutputBuffer == nullptr ||
        packet.InputBufferLength != graphics_abi::present_request_bytes ||
        packet.OutputBufferLength < graphics_abi::present_reply_bytes) return ERROR_INVALID_PARAMETER;
    Lock lock(service.lock);
    // Decoded into device-owned storage (the request is about 1 KiB).
    if (!graphics_abi::decode_present_request(
            ByteView(static_cast<uint8_t*>(packet.InputBuffer), packet.InputBufferLength),
            service.present_request))
        return ERROR_INVALID_PARAMETER;
    uint32_t copied = 0;
    const Outcome result = process_core_present(service, PsGetCurrentProcess(), copied);
    uint8_t reply[graphics_abi::present_reply_bytes]{};
    for (uint32_t byte = 0; byte < 4; ++byte) {
        reply[byte] = static_cast<uint8_t>(static_cast<uint32_t>(result) >> (byte * 8));
        reply[4 + byte] = static_cast<uint8_t>(copied >> (byte * 8));
    }
    VideoPortMoveMemory(packet.OutputBuffer, reply, sizeof(reply));
    packet.StatusBlock->Information = sizeof(reply);
    return NO_ERROR;
}
} // namespace rx7900::reactos

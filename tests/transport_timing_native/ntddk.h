/*
Module: lightning.tests.transport_timing_boundary
Purpose: Supply only opaque native declarations for the portable timing fixture.
Created: 2026-10-22
Architecture: Not an NT ABI substitute; production is separately SDK compiled.
*/
#pragma once
#include <stdint.h>

using ULONG = uint32_t;
using NTSTATUS = int32_t;
using PVOID = void*;
using PUCHAR = uint8_t*;
using PDEVICE_OBJECT = void*;
using PCM_RESOURCE_LIST = const void*;
using PDMA_ADAPTER = void*;
struct LARGE_INTEGER { int64_t QuadPart = 0; };
using PHYSICAL_ADDRESS = LARGE_INTEGER;
struct BUS_INTERFACE_STANDARD { uintptr_t opaque = 0; };
inline constexpr NTSTATUS STATUS_UNSUCCESSFUL = static_cast<NTSTATUS>(-1073741823);
#define NT_SUCCESS(status) ((status) >= 0)
extern "C" LARGE_INTEGER KeQueryPerformanceCounter(LARGE_INTEGER* frequency);

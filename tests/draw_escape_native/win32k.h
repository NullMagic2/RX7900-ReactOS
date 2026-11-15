/*
Module: lightning.tests.draw_escape_native
Purpose: Declare the narrow native boundary for actual-source ownership tests.
Created: 2026-10-22
Architecture: Test callbacks replace OS services, never the production escape.
*/
#pragma once
#include <stdint.h>
#include <stddef.h>
#define APIENTRY
#define FASTCALL
typedef int32_t INT;
typedef uint32_t ULONG;
typedef int32_t NTSTATUS;
typedef void* HDC;
typedef void* PVOID;
typedef void VOID;
constexpr NTSTATUS STATUS_SUCCESS = 0;
constexpr NTSTATUS STATUS_INSUFFICIENT_RESOURCES = -1;
constexpr NTSTATUS STATUS_ACCESS_VIOLATION = -2;
#define NT_SUCCESS(status) ((status) >= 0)
constexpr ULONG ERROR_INVALID_PARAMETER = 87;
constexpr ULONG ERROR_INVALID_HANDLE = 6;
constexpr ULONG ERROR_NOT_SUPPORTED = 50;
constexpr ULONG PDEV_DISABLED = 1;
constexpr ULONG PDEV_DRIVER_PUNTED_CALL = 2;
constexpr ULONG PagedPool = 1;
struct RECTL { int32_t left, top, right, bottom; };
struct CLIPOBJ { RECTL rclBounds; ULONG unique; };
struct XCLIPOBJ : CLIPOBJ {};
struct SURFOBJ { ULONG identity; };
struct SURFACE { SURFOBJ SurfObj; };
typedef SURFACE* PSURFACE;
struct PDEVOBJ {
    struct Functions {
        ULONG (*DrawEscape)(SURFOBJ*, ULONG, CLIPOBJ*, RECTL*, ULONG, PVOID);
    } DriverFunctions;
    ULONG flFlags;
};
typedef PDEVOBJ* PPDEVOBJ;
struct DC {
    struct Level { PSURFACE pSurface; } dclevel;
    PPDEVOBJ ppdev;
    RECTL erclWindow;
    struct { int32_t x, y; } ptlDCOrig;
    XCLIPOBJ co;
};
typedef DC* PDC;
PVOID ExAllocatePoolWithTag(ULONG pool, ULONG size, ULONG tag);
VOID ExFreePoolWithTag(PVOID pointer, ULONG tag);
VOID EngSetLastError(ULONG error);
VOID SetLastNtError(NTSTATUS status);
PDC DC_LockDc(HDC handle);
VOID DC_UnlockDc(PDC dc);
VOID DC_vPrepareDCsForBlit(PDC dc, const RECTL*, PDC, const RECTL*);
VOID DC_vFinishBlit(PDC dc, PDC);

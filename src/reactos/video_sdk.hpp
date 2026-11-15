/*
Module: rx7900.reactos.video_sdk
Purpose: Share the native VideoPort SDK include boundary between miniports.
Created: 2026-10-21
Architecture: OS declarations only; no device or firmware dependency.
*/
#pragma once
#include <ntddk.h>
#include <dderror.h>
// Match videoprt: suppress miniport.h duplicates of WDM resource/IRQ types.
#ifndef __BROKEN__
#define __BROKEN__
#define RX7900_UNDEFINE_MINIPORT_WORKAROUND
#endif
#include <miniport.h>
#ifdef RX7900_UNDEFINE_MINIPORT_WORKAROUND
#undef __BROKEN__
#undef RX7900_UNDEFINE_MINIPORT_WORKAROUND
#endif
#include <video.h>

/*
Module: lightning.reactos.cursor_snapshot
Purpose: Retain one owned canonical cursor without borrowed engine pointers.
Created: 2026-10-21
Architecture: Shared storage for the display client and native cursor owner.
*/
#pragma once
#include "lightning_present_abi.hpp"
#include "lightning-memory.h"

namespace lightning::reactos {
struct CursorShapeSnapshot final {
    CursorShapeRequest header{};
    uint8_t pixels[LIGHTNING_SCANOUT_CURSOR_SHAPE_BYTES]{};
};
static_assert(sizeof(CursorShapeSnapshot)==sizeof(CursorShapeRequest)+
    LIGHTNING_SCANOUT_CURSOR_SHAPE_BYTES,"Contiguous bounded cursor packet");
} // namespace lightning::reactos

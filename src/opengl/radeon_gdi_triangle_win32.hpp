/*
Module: rx7900.opengl.radeon_gdi_triangle_win32
Purpose: Bind a current Win32 HDC to the restricted GDI triangle backend.
Created: 2026-10-23
Architecture: Standard msimg32 GradientFill preserves the caller's DC clip.
*/
#pragma once

#include "radeon_gdi_triangle.hpp"
#include <windows.h>

namespace rx7900::opengl {

// The caller keeps dc alive and authorized for the whole context lifetime.
// Rebind on DC recreation; no DC ownership or thread-affinity transfer occurs.
GdiTriangleTransport win32_gdi_transport(HDC dc) noexcept;

} // namespace rx7900::opengl

/*
Module: lightning.tests.draw_escape_user
Purpose: Substitute only USER lock calls at the production native boundary.
Created: 2026-10-22
Architecture: Real C++ escape implementation is compiled without source changes.
*/
#pragma once
VOID FASTCALL UserEnterExclusive(VOID);
VOID FASTCALL UserLeave(VOID);

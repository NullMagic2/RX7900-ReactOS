/*
Module: lightning.tests.draw_escape_capture
Purpose: Inject native capture errors into the production ownership path.
Created: 2026-10-22
Architecture: Matches pinned _MmCopyFromCaller ownership and failure semantics.
*/
#pragma once
NTSTATUS _MmCopyFromCaller(PVOID destination, PVOID source, ULONG bytes);

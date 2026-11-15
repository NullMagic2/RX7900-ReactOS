/*
Module: rx7900.common.status
Purpose: Shared explicit binary-validation results.
Created: 2026-10-21
Architecture: Common parsers do not depend on NTSTATUS or exceptions.
*/
#pragma once

namespace rx7900 {

enum class ParseStatus {
    success,
    truncated_input,
    unsupported_device,
    malformed_header,
    invalid_rom_signature,
    invalid_atom_header
};

} // namespace rx7900

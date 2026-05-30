// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Single-translation-unit build of sokol_gfx's implementation.
// Backend macro (SOKOL_D3D11 / SOKOL_METAL / SOKOL_GLCORE) is provided by
// the `sokol` INTERFACE target. On macOS this file is compiled as
// Objective-C with ARC.

#define SOKOL_IMPL
#include "sokol_gfx.h"

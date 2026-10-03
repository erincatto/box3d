// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#define B3_SIMD_WIDTH 8

#include "core.h"

#if defined( B3_SIMD_AVX2 )

#if defined( _MSC_VER ) && !defined( __clang__ ) && !defined( __AVX2__ )
#error "MSVC must compile this file with /arch:AVX2, or define BOX3D_DISABLE_AVX2"
#endif

#include <immintrin.h>

B3_AVX2_BEGIN

#include "contact_solver_wide.inl"
#include "convex_manifold_wide.inl"

B3_AVX2_END

#endif

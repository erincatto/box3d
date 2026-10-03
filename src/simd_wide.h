// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "simd.h"

// Macros to avoid name clashes between W4 and W8 functions existing in the same code base.
// At the same time this helps to avoid code duplication.

#if B3_SIMD_WIDTH == 8
#define B3_WIDE( name ) name##W8
#elif B3_SIMD_WIDTH == 4
#define B3_WIDE( name ) name##W4
#else
#error "B3_SIMD_WIDTH must be 4 or 8"
#endif

#define B3_WIDE_ALIGNMENT ( 4 * B3_SIMD_WIDTH )

#define b3FloatW B3_WIDE( b3Float )
#define b3Vec3W B3_WIDE( b3Vec3 )
#define b3ZeroW B3_WIDE( b3Zero )
#define b3SplatW B3_WIDE( b3Splat )
#define b3SetW B3_WIDE( b3Set )
#define b3LoadW B3_WIDE( b3Load )
#define b3StoreW B3_WIDE( b3Store )
#define b3NegW B3_WIDE( b3Neg )
#define b3AbsW B3_WIDE( b3Abs )
#define b3AddW B3_WIDE( b3Add )
#define b3SubW B3_WIDE( b3Sub )
#define b3MulW B3_WIDE( b3Mul )
#define b3DivW B3_WIDE( b3Div )
#define b3SqrtW B3_WIDE( b3Sqrt )
#define b3MulAddW B3_WIDE( b3MulAdd )
#define b3MinW B3_WIDE( b3Min )
#define b3MaxW B3_WIDE( b3Max )
#define b3SymClampW B3_WIDE( b3SymClamp )
#define b3AndW B3_WIDE( b3And )
#define b3OrW B3_WIDE( b3Or )
#define b3AndNotW B3_WIDE( b3AndNot )
#define b3SoftMaskW B3_WIDE( b3SoftMask )
#define b3GreaterThanW B3_WIDE( b3GreaterThan )
#define b3LessThanW B3_WIDE( b3LessThan )
#define b3EqualsW B3_WIDE( b3Equals )
#define b3AllZeroW B3_WIDE( b3AllZero )
#define b3AnyTrueW B3_WIDE( b3AnyTrue )
#define b3BlendW B3_WIDE( b3Blend )
#define b3EmbedIndexW B3_WIDE( b3EmbedIndex )
#define b3MinIndexW B3_WIDE( b3MinIndex )
#define b3TransposeW B3_WIDE( b3Transpose )
#define b3LoadVW B3_WIDE( b3LoadV )
#define b3StoreVW B3_WIDE( b3StoreV )
#define b3SplatVW B3_WIDE( b3SplatV )
#define b3NegVW B3_WIDE( b3NegV )
#define b3MulSVW B3_WIDE( b3MulSV )
#define b3MulSubSVW B3_WIDE( b3MulSubSV )
#define b3MulAddSVW B3_WIDE( b3MulAddSV )
#define b3SubVW B3_WIDE( b3SubV )
#define b3AddVW B3_WIDE( b3AddV )
#define b3DotW B3_WIDE( b3Dot )
#define b3CrossW B3_WIDE( b3Cross )

// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

#include "box3d/types.h"

#include <immintrin.h>

// wide float holds 8 numbers
typedef __m256 b3FloatW8;

static inline b3FloatW8 b3ZeroW8( void )
{
	return _mm256_setzero_ps();
}

static inline b3FloatW8 b3SplatW8( float scalar )
{
	return _mm256_set1_ps( scalar );
}

static inline b3FloatW8 b3SetW8( float a, float b, float c, float d, float e, float f, float g, float h )
{
	return _mm256_setr_ps( a, b, c, d, e, f, g, h );
}

static inline b3FloatW8 b3LoadW8( const float* data )
{
	return _mm256_loadu_ps( data );
}

static inline void b3StoreW8( float* data, b3FloatW8 a )
{
	_mm256_storeu_ps( data, a );
}

static inline b3FloatW8 b3NegW8( b3FloatW8 a )
{
	// Create a mask with the sign bit set for each element.
	__m256 mask = _mm256_set1_ps( -0.0f );

	// XOR the input with the mask to negate each element.
	return _mm256_xor_ps( a, mask );
}

static inline b3FloatW8 b3AbsW8( b3FloatW8 a )
{
	return _mm256_andnot_ps( _mm256_set1_ps( -0.0f ), a );
}

static inline b3FloatW8 b3AddW8( b3FloatW8 a, b3FloatW8 b )
{
	return _mm256_add_ps( a, b );
}

static inline b3FloatW8 b3SubW8( b3FloatW8 a, b3FloatW8 b )
{
	return _mm256_sub_ps( a, b );
}

static inline b3FloatW8 b3MulW8( b3FloatW8 a, b3FloatW8 b )
{
	return _mm256_mul_ps( a, b );
}

static inline b3FloatW8 b3DivW8( b3FloatW8 a, b3FloatW8 b )
{
	return _mm256_div_ps( a, b );
}

static inline b3FloatW8 b3SqrtW8( b3FloatW8 a )
{
	return _mm256_sqrt_ps( a );
}

// a + b * c
static inline b3FloatW8 b3MulAddW8( b3FloatW8 a, b3FloatW8 b, b3FloatW8 c )
{
	return _mm256_add_ps( a, _mm256_mul_ps( b, c ) );
}

static inline b3FloatW8 b3MinW8( b3FloatW8 a, b3FloatW8 b )
{
	return _mm256_min_ps( a, b );
}

static inline b3FloatW8 b3MaxW8( b3FloatW8 a, b3FloatW8 b )
{
	return _mm256_max_ps( a, b );
}

// Clamp a to [-b, b]
static inline b3FloatW8 b3SymClampW8( b3FloatW8 a, b3FloatW8 b )
{
	b3FloatW8 nb = b3NegW8( b );
	b3FloatW8 c = b3MaxW8( nb, a );
	return b3MinW8( c, b );
}

static inline b3FloatW8 b3AndW8( b3FloatW8 a, b3FloatW8 b )
{
	return _mm256_and_ps( a, b );
}

static inline b3FloatW8 b3OrW8( b3FloatW8 a, b3FloatW8 b )
{
	return _mm256_or_ps( a, b );
}

// a & ~b
static inline b3FloatW8 b3AndNotW8( b3FloatW8 a, b3FloatW8 b )
{
	// Arguments are reversed
	return _mm256_andnot_ps( b, a );
}

// This is used to optimize selection of contact softness.
static inline b3FloatW8 b3SoftMaskW8( const int* indexA, const int* indexB )
{
	__m256i zero = _mm256_setzero_si256();
	__m256i a = _mm256_cmpeq_epi32( _mm256_loadu_si256( (const __m256i*)indexA ), zero );
	__m256i b = _mm256_cmpeq_epi32( _mm256_loadu_si256( (const __m256i*)indexB ), zero );
	return _mm256_castsi256_ps( _mm256_or_si256( a, b ) );
}

static inline b3FloatW8 b3GreaterThanW8( b3FloatW8 a, b3FloatW8 b )
{
	return _mm256_cmp_ps( a, b, _CMP_GT_OQ );
}

static inline b3FloatW8 b3LessThanW8( b3FloatW8 a, b3FloatW8 b )
{
	return _mm256_cmp_ps( a, b, _CMP_LT_OQ );
}

static inline b3FloatW8 b3EqualsW8( b3FloatW8 a, b3FloatW8 b )
{
	return _mm256_cmp_ps( a, b, _CMP_EQ_OQ );
}

static inline bool b3AllZeroW8( b3FloatW8 a )
{
	// Compare each element with zero
	b3FloatW8 zero = _mm256_setzero_ps();
	b3FloatW8 cmp = _mm256_cmp_ps( a, zero, _CMP_EQ_OQ );

	// Create a mask from the comparison results
	int mask = _mm256_movemask_ps( cmp );

	// If all elements are zero, the mask will be 0xFF (11111111 in binary)
	return mask == 0xFF;
}

static inline bool b3AnyTrueW8( b3FloatW8 mask )
{
	return _mm256_movemask_ps( mask ) != 0;
}

// Component-wise returns mask ? b : a
static inline b3FloatW8 b3BlendW8( b3FloatW8 a, b3FloatW8 b, b3FloatW8 mask )
{
	return _mm256_or_ps( _mm256_and_ps( mask, b ), _mm256_andnot_ps( mask, a ) );
}

// Replace the low bitCount mantissa bits of each lane with baseIndex + lane. The value must be
// positive so the embedded index sorts with the value, and ties fall to the lower index.
static inline b3FloatW8 b3EmbedIndexW8( b3FloatW8 value, int baseIndex, int bitCount )
{
	int mask = ( 1 << bitCount ) - 1;
	__m256i index = _mm256_add_epi32( _mm256_set1_epi32( baseIndex ), _mm256_setr_epi32( 0, 1, 2, 3, 4, 5, 6, 7 ) );
	__m256 clearLow = _mm256_castsi256_ps( _mm256_set1_epi32( ~mask ) );
	return _mm256_or_ps( _mm256_and_ps( value, clearLow ), _mm256_castsi256_ps( index ) );
}

// Recovers the index embedded by b3EmbedIndexW8 from the lane holding the minimum.
static inline int b3MinIndexW8( b3FloatW8 a, int bitCount )
{
	__m128 min = _mm_min_ps( _mm256_castps256_ps128( a ), _mm256_extractf128_ps( a, 1 ) );
	min = _mm_min_ps( _mm_shuffle_ps( min, min, _MM_SHUFFLE( 2, 3, 0, 1 ) ), min );
	min = _mm_min_ps( _mm_shuffle_ps( min, min, _MM_SHUFFLE( 1, 0, 3, 2 ) ), min );
	return _mm_cvtsi128_si32( _mm_castps_si128( min ) ) & ( ( 1 << bitCount ) - 1 );
}

B3_FORCE_INLINE void b3TransposeW8( b3FloatW8 r0, b3FloatW8 r1, b3FloatW8 r2, b3FloatW8 r3, b3FloatW8 r4, b3FloatW8 r5,
									b3FloatW8 r6, b3FloatW8 r7, b3FloatW8* c0, b3FloatW8* c1, b3FloatW8* c2, b3FloatW8* c3,
									b3FloatW8* c4, b3FloatW8* c5, b3FloatW8* c6, b3FloatW8* c7 )
{
	b3FloatW8 t0 = _mm256_unpacklo_ps( r0, r1 );
	b3FloatW8 t1 = _mm256_unpackhi_ps( r0, r1 );
	b3FloatW8 t2 = _mm256_unpacklo_ps( r2, r3 );
	b3FloatW8 t3 = _mm256_unpackhi_ps( r2, r3 );
	b3FloatW8 t4 = _mm256_unpacklo_ps( r4, r5 );
	b3FloatW8 t5 = _mm256_unpackhi_ps( r4, r5 );
	b3FloatW8 t6 = _mm256_unpacklo_ps( r6, r7 );
	b3FloatW8 t7 = _mm256_unpackhi_ps( r6, r7 );
	b3FloatW8 tt0 = _mm256_shuffle_ps( t0, t2, _MM_SHUFFLE( 1, 0, 1, 0 ) );
	b3FloatW8 tt1 = _mm256_shuffle_ps( t0, t2, _MM_SHUFFLE( 3, 2, 3, 2 ) );
	b3FloatW8 tt2 = _mm256_shuffle_ps( t1, t3, _MM_SHUFFLE( 1, 0, 1, 0 ) );
	b3FloatW8 tt3 = _mm256_shuffle_ps( t1, t3, _MM_SHUFFLE( 3, 2, 3, 2 ) );
	b3FloatW8 tt4 = _mm256_shuffle_ps( t4, t6, _MM_SHUFFLE( 1, 0, 1, 0 ) );
	b3FloatW8 tt5 = _mm256_shuffle_ps( t4, t6, _MM_SHUFFLE( 3, 2, 3, 2 ) );
	b3FloatW8 tt6 = _mm256_shuffle_ps( t5, t7, _MM_SHUFFLE( 1, 0, 1, 0 ) );
	b3FloatW8 tt7 = _mm256_shuffle_ps( t5, t7, _MM_SHUFFLE( 3, 2, 3, 2 ) );

	*c0 = _mm256_permute2f128_ps( tt0, tt4, 0x20 );
	*c1 = _mm256_permute2f128_ps( tt1, tt5, 0x20 );
	*c2 = _mm256_permute2f128_ps( tt2, tt6, 0x20 );
	*c3 = _mm256_permute2f128_ps( tt3, tt7, 0x20 );
	*c4 = _mm256_permute2f128_ps( tt0, tt4, 0x31 );
	*c5 = _mm256_permute2f128_ps( tt1, tt5, 0x31 );
	*c6 = _mm256_permute2f128_ps( tt2, tt6, 0x31 );
	*c7 = _mm256_permute2f128_ps( tt3, tt7, 0x31 );
}

// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

#include "box3d/types.h"

#include <arm_neon.h>

// wide float holds 4 numbers
typedef float32x4_t b3FloatW4;

static inline b3FloatW4 b3ZeroW4( void )
{
	return vdupq_n_f32( 0.0f );
}

static inline b3FloatW4 b3SplatW4( float scalar )
{
	return vdupq_n_f32( scalar );
}

static inline b3FloatW4 b3SetW4( float a, float b, float c, float d )
{
	float32_t array[4] = { a, b, c, d };
	return vld1q_f32( array );
}

static inline b3FloatW4 b3LoadW4( const float* data )
{
	return vld1q_f32( data );
}

static inline void b3StoreW4( float* data, b3FloatW4 a )
{
	vst1q_f32( data, a );
}

static inline b3FloatW4 b3NegW4( b3FloatW4 a )
{
	return vnegq_f32( a );
}

static inline b3FloatW4 b3AbsW4( b3FloatW4 a )
{
	return vabsq_f32( a );
}

static inline b3FloatW4 b3AddW4( b3FloatW4 a, b3FloatW4 b )
{
	return vaddq_f32( a, b );
}

static inline b3FloatW4 b3SubW4( b3FloatW4 a, b3FloatW4 b )
{
	return vsubq_f32( a, b );
}

static inline b3FloatW4 b3MulW4( b3FloatW4 a, b3FloatW4 b )
{
	return vmulq_f32( a, b );
}

static inline b3FloatW4 b3DivW4( b3FloatW4 a, b3FloatW4 b )
{
	return vdivq_f32( a, b );
}

static inline b3FloatW4 b3SqrtW4( b3FloatW4 a )
{
	return vsqrtq_f32( a );
}

// Cannot use real FMA because it doesn't match the non-SIMD path
static inline b3FloatW4 b3MulAddW4( b3FloatW4 a, b3FloatW4 b, b3FloatW4 c )
{
	return vaddq_f32( a, vmulq_f32( b, c ) );
}

static inline b3FloatW4 b3MinW4( b3FloatW4 a, b3FloatW4 b )
{
	return vminq_f32( a, b );
}

static inline b3FloatW4 b3MaxW4( b3FloatW4 a, b3FloatW4 b )
{
	return vmaxq_f32( a, b );
}

// clamp a to [-b, b]
static inline b3FloatW4 b3SymClampW4( b3FloatW4 a, b3FloatW4 b )
{
	b3FloatW4 nb = b3NegW4( b );
	b3FloatW4 c = b3MaxW4( nb, a );
	return b3MinW4( c, b );
}

static inline b3FloatW4 b3AndW4( b3FloatW4 a, b3FloatW4 b )
{
	return vreinterpretq_f32_u32( vandq_u32( vreinterpretq_u32_f32( a ), vreinterpretq_u32_f32( b ) ) );
}

static inline b3FloatW4 b3OrW4( b3FloatW4 a, b3FloatW4 b )
{
	return vreinterpretq_f32_u32( vorrq_u32( vreinterpretq_u32_f32( a ), vreinterpretq_u32_f32( b ) ) );
}

// a & ~b
static inline b3FloatW4 b3AndNotW4( b3FloatW4 a, b3FloatW4 b )
{
	return vreinterpretq_f32_u32( vbicq_u32( vreinterpretq_u32_f32( a ), vreinterpretq_u32_f32( b ) ) );
}

static inline b3FloatW4 b3SoftMaskW4( const int* indexA, const int* indexB )
{
	int32x4_t zero = vdupq_n_s32( 0 );
	uint32x4_t a = vceqq_s32( vld1q_s32( (const int32_t*)indexA ), zero );
	uint32x4_t b = vceqq_s32( vld1q_s32( (const int32_t*)indexB ), zero );
	return vreinterpretq_f32_u32( vorrq_u32( a, b ) );
}

static inline b3FloatW4 b3GreaterThanW4( b3FloatW4 a, b3FloatW4 b )
{
	return vreinterpretq_f32_u32( vcgtq_f32( a, b ) );
}

static inline b3FloatW4 b3LessThanW4( b3FloatW4 a, b3FloatW4 b )
{
	return vreinterpretq_f32_u32( vcltq_f32( a, b ) );
}

static inline b3FloatW4 b3EqualsW4( b3FloatW4 a, b3FloatW4 b )
{
	return vreinterpretq_f32_u32( vceqq_f32( a, b ) );
}

static inline bool b3AllZeroW4( b3FloatW4 a )
{
	// Create a zero vector for comparison
	b3FloatW4 zero = vdupq_n_f32( 0.0f );

	// Compare the input vector with zero
	uint32x4_t cmp_result = vceqq_f32( a, zero );

// Check if all comparison results are non-zero using vminvq
#ifdef __ARM_FEATURE_SVE
	// ARM v8.2+ has horizontal minimum instruction
	return vminvq_u32( cmp_result ) != 0;
#else
	// For older ARM architectures, we need to manually check all lanes
	return vgetq_lane_u32( cmp_result, 0 ) != 0 && vgetq_lane_u32( cmp_result, 1 ) != 0 && vgetq_lane_u32( cmp_result, 2 ) != 0 &&
		   vgetq_lane_u32( cmp_result, 3 ) != 0;
#endif
}

// _mm_movemask_ps equivalent, compatible with ARM v7.
static inline bool b3AnyTrueW4( b3FloatW4 mask )
{
	uint32x4_t m = vreinterpretq_u32_f32( mask );
	uint32x2_t p = vorr_u32( vget_low_u32( m ), vget_high_u32( m ) );
	return ( vget_lane_u32( p, 0 ) | vget_lane_u32( p, 1 ) ) != 0;
}

// component-wise returns mask ? b : a
static inline b3FloatW4 b3BlendW4( b3FloatW4 a, b3FloatW4 b, b3FloatW4 mask )
{
	uint32x4_t mask32 = vreinterpretq_u32_f32( mask );
	return vbslq_f32( mask32, b, a );
}

static inline b3FloatW4 b3EmbedIndexW4( b3FloatW4 value, int baseIndex, int bitCount )
{
	uint32_t mask = ( 1u << bitCount ) - 1;
	const int32_t lanes[4] = { 0, 1, 2, 3 };
	int32x4_t index = vaddq_s32( vdupq_n_s32( baseIndex ), vld1q_s32( lanes ) );
	uint32x4_t clearLow = vdupq_n_u32( ~mask );
	uint32x4_t bits = vorrq_u32( vandq_u32( vreinterpretq_u32_f32( value ), clearLow ), vreinterpretq_u32_s32( index ) );
	return vreinterpretq_f32_u32( bits );
}

static inline int b3MinIndexW4( b3FloatW4 a, int bitCount )
{
	float32x2_t m = vmin_f32( vget_low_f32( a ), vget_high_f32( a ) );
	m = vpmin_f32( m, m );
	uint32_t bits = vget_lane_u32( vreinterpret_u32_f32( m ), 0 );
	return (int)( bits & ( ( 1u << bitCount ) - 1 ) );
}

typedef struct b3AABBV
{
	float32x4_t lower;
	float32x4_t upper;
} b3AABBV;

B3_FORCE_INLINE b3AABBV b3LoadAABBV( const b3AABB* aabb )
{
	const float* base = &aabb->lowerBound.x;

	// Offset to avoid reading off the end (avoid UB).
	// [lz ux uy uz]
	float32x4_t v1 = vld1q_f32( base + 2 );
	b3AABBV result;
	// [lx ly lz -]
	result.lower = vld1q_f32( base );
	// [ux uy uz -]
	result.upper = vextq_f32( v1, v1, 1 );
	return result;
}

B3_FORCE_INLINE bool b3OverlapAABBV( b3AABBV a, b3AABBV b )
{
	static const uint32_t laneMask[4] = { 0, 0, 0, 0xFFFFFFFFu };
	uint32x4_t test = vandq_u32( vcleq_f32( a.lower, b.upper ), vcleq_f32( b.lower, a.upper ) );
	return vminvq_u32( vorrq_u32( test, vld1q_u32( laneMask ) ) ) != 0;
}

B3_FORCE_INLINE bool b3OverlapNode( b3AABBV av, const b3TreeNode* node )
{
	return b3OverlapAABBV( av, b3LoadAABBV( &node->aabb ) );
}

B3_FORCE_INLINE bool b3OverlapV( const b3AABB* a, const b3AABB* b )
{
	return b3OverlapAABBV( b3LoadAABBV( a ), b3LoadAABBV( b ) );
}

B3_FORCE_INLINE b3AABBV b3UnionAABBV( b3AABBV a, b3AABBV b )
{
	b3AABBV result;
	result.lower = vminq_f32( a.lower, b.lower );
	result.upper = vmaxq_f32( a.upper, b.upper );
	return result;
}

B3_FORCE_INLINE b3AABBV b3UnionPairV( const b3TreeNode* pair )
{
	return b3UnionAABBV( b3LoadAABBV( &pair[0].aabb ), b3LoadAABBV( &pair[1].aabb ) );
}

B3_FORCE_INLINE void b3StoreAABBV( b3AABB* aabb, b3AABBV value, bool condition )
{
	float32x4_t raw0 = vsetq_lane_f32( vgetq_lane_f32( value.upper, 0 ), value.lower, 3 );
	float32x4_t rotated = vextq_f32( value.upper, value.upper, 1 );
	float32x4_t raw1 = vcombine_f32( vget_high_f32( raw0 ), vget_low_f32( rotated ) );

	float* base = &aabb->lowerBound.x;
	uint32x4_t mask = vdupq_n_u32( condition ? 0xFFFFFFFFu : 0u );
	vst1q_f32( base, vbslq_f32( mask, raw0, vld1q_f32( base ) ) );
	vst1q_f32( base + 2, vbslq_f32( mask, raw1, vld1q_f32( base + 2 ) ) );
}

B3_FORCE_INLINE void b3TransposeW4( b3FloatW4 r0, b3FloatW4 r1, b3FloatW4 r2, b3FloatW4 r3, b3FloatW4* c0, b3FloatW4* c1,
									b3FloatW4* c2, b3FloatW4* c3 )
{
	b3FloatW4 t0 = vzip1q_f32( r0, r2 );
	b3FloatW4 t1 = vzip1q_f32( r1, r3 );
	b3FloatW4 t2 = vzip2q_f32( r0, r2 );
	b3FloatW4 t3 = vzip2q_f32( r1, r3 );
	*c0 = vzip1q_f32( t0, t1 );
	*c1 = vzip2q_f32( t0, t1 );
	*c2 = vzip1q_f32( t2, t3 );
	*c3 = vzip2q_f32( t2, t3 );
}

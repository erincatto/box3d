// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

#include "box3d/types.h"

#include <math.h>
#include <string.h>

// scalar math
typedef struct b3FloatW4
{
	float x, y, z, w;
} b3FloatW4;

static inline b3FloatW4 b3ZeroW4( void )
{
	return (b3FloatW4){ 0.0f, 0.0f, 0.0f, 0.0f };
}

static inline b3FloatW4 b3SplatW4( float scalar )
{
	return (b3FloatW4){ scalar, scalar, scalar, scalar };
}

static inline b3FloatW4 b3SetW4( float a, float b, float c, float d )
{
	return (b3FloatW4){ a, b, c, d };
}

static inline b3FloatW4 b3LoadW4( const float* data )
{
	return (b3FloatW4){ data[0], data[1], data[2], data[3] };
}

static inline void b3StoreW4( float* data, b3FloatW4 a )
{
	data[0] = a.x;
	data[1] = a.y;
	data[2] = a.z;
	data[3] = a.w;
}

static inline b3FloatW4 b3NegW4( b3FloatW4 a )
{
	return (b3FloatW4){ -a.x, -a.y, -a.z, -a.w };
}

static inline b3FloatW4 b3AbsW4( b3FloatW4 a )
{
	return (b3FloatW4){ a.x < 0.0f ? -a.x : a.x, a.y < 0.0f ? -a.y : a.y, a.z < 0.0f ? -a.z : a.z, a.w < 0.0f ? -a.w : a.w };
}

static inline b3FloatW4 b3AddW4( b3FloatW4 a, b3FloatW4 b )
{
	return (b3FloatW4){ a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w };
}

static inline b3FloatW4 b3SubW4( b3FloatW4 a, b3FloatW4 b )
{
	return (b3FloatW4){ a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w };
}

static inline b3FloatW4 b3MulW4( b3FloatW4 a, b3FloatW4 b )
{
	return (b3FloatW4){ a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w };
}

static inline b3FloatW4 b3DivW4( b3FloatW4 a, b3FloatW4 b )
{
	return (b3FloatW4){ a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w };
}

static inline b3FloatW4 b3SqrtW4( b3FloatW4 a )
{
	return (b3FloatW4){ sqrtf( a.x ), sqrtf( a.y ), sqrtf( a.z ), sqrtf( a.w ) };
}

static inline b3FloatW4 b3MulAddW4( b3FloatW4 a, b3FloatW4 b, b3FloatW4 c )
{
	return (b3FloatW4){ a.x + b.x * c.x, a.y + b.y * c.y, a.z + b.z * c.z, a.w + b.w * c.w };
}

static inline b3FloatW4 b3MinW4( b3FloatW4 a, b3FloatW4 b )
{
	b3FloatW4 r;
	r.x = a.x <= b.x ? a.x : b.x;
	r.y = a.y <= b.y ? a.y : b.y;
	r.z = a.z <= b.z ? a.z : b.z;
	r.w = a.w <= b.w ? a.w : b.w;
	return r;
}

static inline b3FloatW4 b3MaxW4( b3FloatW4 a, b3FloatW4 b )
{
	b3FloatW4 r;
	r.x = a.x >= b.x ? a.x : b.x;
	r.y = a.y >= b.y ? a.y : b.y;
	r.z = a.z >= b.z ? a.z : b.z;
	r.w = a.w >= b.w ? a.w : b.w;
	return r;
}

// clamp a to [-b, b]
static inline b3FloatW4 b3SymClampW4( b3FloatW4 a, b3FloatW4 b )
{
	b3FloatW4 r;
	r.x = a.x <= b.x ? a.x : b.x;
	r.y = a.y <= b.y ? a.y : b.y;
	r.z = a.z <= b.z ? a.z : b.z;
	r.w = a.w <= b.w ? a.w : b.w;
	r.x = r.x <= -b.x ? -b.x : r.x;
	r.y = r.y <= -b.y ? -b.y : r.y;
	r.z = r.z <= -b.z ? -b.z : r.z;
	r.w = r.w <= -b.w ? -b.w : r.w;
	return r;
}

// Logical operations on the scalar path are 0/1 float values. Not bit-wise like SIMD.

static inline b3FloatW4 b3AndW4( b3FloatW4 a, b3FloatW4 b )
{
	b3FloatW4 r;
	r.x = a.x != 0.0f && b.x != 0.0f ? 1.0f : 0.0f;
	r.y = a.y != 0.0f && b.y != 0.0f ? 1.0f : 0.0f;
	r.z = a.z != 0.0f && b.z != 0.0f ? 1.0f : 0.0f;
	r.w = a.w != 0.0f && b.w != 0.0f ? 1.0f : 0.0f;
	return r;
}

static inline b3FloatW4 b3OrW4( b3FloatW4 a, b3FloatW4 b )
{
	b3FloatW4 r;
	r.x = a.x != 0.0f || b.x != 0.0f ? 1.0f : 0.0f;
	r.y = a.y != 0.0f || b.y != 0.0f ? 1.0f : 0.0f;
	r.z = a.z != 0.0f || b.z != 0.0f ? 1.0f : 0.0f;
	r.w = a.w != 0.0f || b.w != 0.0f ? 1.0f : 0.0f;
	return r;
}

// a & ~b
static inline b3FloatW4 b3AndNotW4( b3FloatW4 a, b3FloatW4 b )
{
	b3FloatW4 r;
	r.x = a.x != 0.0f && b.x == 0.0f ? 1.0f : 0.0f;
	r.y = a.y != 0.0f && b.y == 0.0f ? 1.0f : 0.0f;
	r.z = a.z != 0.0f && b.z == 0.0f ? 1.0f : 0.0f;
	r.w = a.w != 0.0f && b.w == 0.0f ? 1.0f : 0.0f;
	return r;
}

static inline b3FloatW4 b3SoftMaskW4( const int* indexA, const int* indexB )
{
	b3FloatW4 r;
	r.x = indexA[0] == 0 || indexB[0] == 0 ? 1.0f : 0.0f;
	r.y = indexA[1] == 0 || indexB[1] == 0 ? 1.0f : 0.0f;
	r.z = indexA[2] == 0 || indexB[2] == 0 ? 1.0f : 0.0f;
	r.w = indexA[3] == 0 || indexB[3] == 0 ? 1.0f : 0.0f;
	return r;
}

static inline b3FloatW4 b3GreaterThanW4( b3FloatW4 a, b3FloatW4 b )
{
	b3FloatW4 r;
	r.x = a.x > b.x ? 1.0f : 0.0f;
	r.y = a.y > b.y ? 1.0f : 0.0f;
	r.z = a.z > b.z ? 1.0f : 0.0f;
	r.w = a.w > b.w ? 1.0f : 0.0f;
	return r;
}

static inline b3FloatW4 b3LessThanW4( b3FloatW4 a, b3FloatW4 b )
{
	b3FloatW4 r;
	r.x = a.x < b.x ? 1.0f : 0.0f;
	r.y = a.y < b.y ? 1.0f : 0.0f;
	r.z = a.z < b.z ? 1.0f : 0.0f;
	r.w = a.w < b.w ? 1.0f : 0.0f;
	return r;
}

static inline b3FloatW4 b3EqualsW4( b3FloatW4 a, b3FloatW4 b )
{
	b3FloatW4 r;
	r.x = a.x == b.x ? 1.0f : 0.0f;
	r.y = a.y == b.y ? 1.0f : 0.0f;
	r.z = a.z == b.z ? 1.0f : 0.0f;
	r.w = a.w == b.w ? 1.0f : 0.0f;
	return r;
}

static inline bool b3AllZeroW4( b3FloatW4 a )
{
	return a.x == 0.0f && a.y == 0.0f && a.z == 0.0f && a.w == 0.0f;
}

static inline bool b3AnyTrueW4( b3FloatW4 mask )
{
	return mask.x != 0.0f || mask.y != 0.0f || mask.z != 0.0f || mask.w != 0.0f;
}

static inline int b3MoveMaskW4( b3FloatW4 mask )
{
	return ( mask.x != 0.0f ? 1 : 0 ) | ( mask.y != 0.0f ? 2 : 0 ) | ( mask.z != 0.0f ? 4 : 0 ) | ( mask.w != 0.0f ? 8 : 0 );
}

// component-wise returns mask ? b : a
static inline b3FloatW4 b3BlendW4( b3FloatW4 a, b3FloatW4 b, b3FloatW4 mask )
{
	b3FloatW4 r;
	r.x = mask.x != 0.0f ? b.x : a.x;
	r.y = mask.y != 0.0f ? b.y : a.y;
	r.z = mask.z != 0.0f ? b.z : a.z;
	r.w = mask.w != 0.0f ? b.w : a.w;
	return r;
}

static inline b3FloatW4 b3EmbedIndexW4( b3FloatW4 value, int baseIndex, int bitCount )
{
	uint32_t mask = ( 1u << bitCount ) - 1;
	float lanes[4] = { value.x, value.y, value.z, value.w };

	for ( int i = 0; i < 4; ++i )
	{
		uint32_t bits;
		memcpy( &bits, lanes + i, sizeof( bits ) );
		bits = ( bits & ~mask ) | (uint32_t)( baseIndex + i );
		memcpy( lanes + i, &bits, sizeof( bits ) );
	}

	return (b3FloatW4){ lanes[0], lanes[1], lanes[2], lanes[3] };
}

static inline int b3MinIndexW4( b3FloatW4 a, int bitCount )
{
	float m = a.x;
	m = a.y < m ? a.y : m;
	m = a.z < m ? a.z : m;
	m = a.w < m ? a.w : m;

	uint32_t bits;
	memcpy( &bits, &m, sizeof( bits ) );
	return (int)( bits & ( ( 1u << bitCount ) - 1 ) );
}

typedef b3AABB b3AABBV;

B3_FORCE_INLINE b3AABBV b3LoadAABBV( const b3AABB* aabb )
{
	return *aabb;
}

B3_FORCE_INLINE bool b3OverlapAABBV( b3AABBV a, b3AABBV b )
{
	return a.lowerBound.x <= b.upperBound.x && a.lowerBound.y <= b.upperBound.y && a.lowerBound.z <= b.upperBound.z &&
		   b.lowerBound.x <= a.upperBound.x && b.lowerBound.y <= a.upperBound.y && b.lowerBound.z <= a.upperBound.z;
}

B3_FORCE_INLINE bool b3OverlapNode( b3AABBV av, const b3TreeNode* node )
{
	return b3OverlapAABBV( av, node->aabb );
}

B3_FORCE_INLINE bool b3OverlapV( const b3AABB* a, const b3AABB* b )
{
	return b3OverlapAABBV( *a, *b );
}

B3_FORCE_INLINE b3AABBV b3UnionAABBV( b3AABBV a, b3AABBV b )
{
	return b3AABB_Union( a, b );
}

B3_FORCE_INLINE b3AABBV b3UnionPairV( const b3TreeNode* pair )
{
	return b3AABB_Union( pair[0].aabb, pair[1].aabb );
}

B3_FORCE_INLINE void b3StoreAABBV( b3AABB* aabb, b3AABBV value, bool condition )
{
	if ( condition )
	{
		*aabb = value;
	}
}

B3_FORCE_INLINE void b3TransposeW4( b3FloatW4 r0, b3FloatW4 r1, b3FloatW4 r2, b3FloatW4 r3, b3FloatW4* c0, b3FloatW4* c1,
									b3FloatW4* c2, b3FloatW4* c3 )
{
	b3FloatW4 t0 = (b3FloatW4){ r0.x, r2.x, r0.y, r2.y };
	b3FloatW4 t1 = (b3FloatW4){ r1.x, r3.x, r1.y, r3.y };
	b3FloatW4 t2 = (b3FloatW4){ r0.z, r2.z, r0.w, r2.w };
	b3FloatW4 t3 = (b3FloatW4){ r1.z, r3.z, r1.w, r3.w };
	*c0 = (b3FloatW4){ t0.x, t1.x, t0.y, t1.y };
	*c1 = (b3FloatW4){ t0.z, t1.z, t0.w, t1.w };
	*c2 = (b3FloatW4){ t2.x, t3.x, t2.y, t3.y };
	*c3 = (b3FloatW4){ t2.z, t3.z, t2.w, t3.w };
}

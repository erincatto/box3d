// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Dirk Gregorius contributed portions of this code

#include "simd.h"

#include "platform.h"

#include <stdbool.h>
#include <stdint.h>

static b3AtomicInt b3_simdWidth;

#if defined( B3_SIMD_HAS_WIDTH_8 )

#if defined( _MSC_VER )
#include <intrin.h>
#endif

// This code enables runtime dispatch to AVX2.
// Inspired by https://github.com/simdjson/simdjson/blob/master/src/internal/isadetection.h

static void b3CpuId( unsigned int leaf, unsigned int subLeaf, unsigned int registers[4] )
{
#if defined( _MSC_VER )
	int info[4];
	__cpuidex( info, (int)leaf, (int)subLeaf );
	registers[0] = (unsigned int)info[0];
	registers[1] = (unsigned int)info[1];
	registers[2] = (unsigned int)info[2];
	registers[3] = (unsigned int)info[3];
#else
	// clang cpuid.h can give the rbx swap register the same register as the leaf input, clobbering rbx
	__asm__ volatile( "cpuid"
					  : "=a"( registers[0] ), "=b"( registers[1] ), "=c"( registers[2] ), "=d"( registers[3] )
					  : "a"( leaf ), "c"( subLeaf ) );
#endif
}

static uint64_t b3GetExtendedControlRegister( void )
{
#if defined( _MSC_VER ) && !defined( __clang__ )
	return _xgetbv( 0 );
#else
	uint32_t low, high;
	__asm__ volatile( "xgetbv" : "=a"( low ), "=d"( high ) : "c"( 0 ) );
	return ( (uint64_t)high << 32 ) | low;
#endif
}

static bool b3HasAVX2( void )
{
	unsigned int registers[4];
	b3CpuId( 0, 0, registers );
	if ( registers[0] < 7 )
	{
		return false;
	}

	b3CpuId( 1, 0, registers );
	unsigned int osxsaveAndAvx = ( 1u << 27 ) | ( 1u << 28 );
	if ( ( registers[2] & osxsaveAndAvx ) != osxsaveAndAvx )
	{
		return false;
	}

	if ( ( b3GetExtendedControlRegister() & 6 ) != 6 )
	{
		return false;
	}

	b3CpuId( 7, 0, registers );
	return ( registers[1] & ( 1u << 5 ) ) != 0;
}

#endif

static int b3DetectSIMDWidth( void )
{
#if defined( B3_SIMD_HAS_WIDTH_8 )
	return b3HasAVX2() ? 8 : 4;
#else
	return 4;
#endif
}

int b3GetSIMDWidth( void )
{
	int width = b3AtomicLoadInt( &b3_simdWidth );
	if ( width == 0 )
	{
		width = b3DetectSIMDWidth();
		b3AtomicStoreInt( &b3_simdWidth, width );
	}

	return width;
}

bool b3IsAVX2Available( void )
{
	return b3DetectSIMDWidth() == 8;
}

void b3SetSIMDWidth( int width )
{
	B3_ASSERT( width == 0 || width == 4 || width == 8 );
	if ( width == 8 && b3DetectSIMDWidth() != 8 )
	{
		width = 4;
	}

	b3AtomicStoreInt( &b3_simdWidth, width );
}

#if defined( B3_SIMD_SSE2 )

#define B3_TRANSPOSE3( C1, C2, C3 )                                                                                              \
	{                                                                                                                            \
		b3V32 T1 = _mm_unpacklo_ps( ( C1 ), ( C2 ) );                                                                            \
		b3V32 T2 = _mm_unpackhi_ps( ( C1 ), ( C2 ) );                                                                            \
		( C1 ) = _mm_shuffle_ps( ( T1 ), ( C3 ), _MM_SHUFFLE( 0, 0, 1, 0 ) );                                                    \
		( C2 ) = _mm_shuffle_ps( ( T1 ), ( C3 ), _MM_SHUFFLE( 1, 1, 3, 2 ) );                                                    \
		( C3 ) = _mm_shuffle_ps( ( T2 ), ( C3 ), _MM_SHUFFLE( 2, 2, 1, 0 ) );                                                    \
	}

static inline b3V32 b3SplatXV( b3V32 v )
{
	return _mm_shuffle_ps( v, v, _MM_SHUFFLE( 0, 0, 0, 0 ) );
}

static inline b3V32 b3SplatYV( b3V32 v )
{
	return _mm_shuffle_ps( v, v, _MM_SHUFFLE( 1, 1, 1, 1 ) );
}

static inline b3V32 b3SplatZV( b3V32 v )
{
	return _mm_shuffle_ps( v, v, _MM_SHUFFLE( 2, 2, 2, 2 ) );
}

static inline bool b3AnyGreaterEq3V( b3V32 a, b3V32 b )
{
	b3V32 v = _mm_cmpge_ps( a, b );
	return ( _mm_movemask_ps( v ) & 0x07 ) != 0;
}

static inline b3V32 b3Dot3V( b3V32 a, b3V32 b )
{
	b3V32 m = _mm_mul_ps( a, b );
	b3V32 x = _mm_shuffle_ps( m, m, _MM_SHUFFLE( 0, 0, 0, 0 ) );
	b3V32 y = _mm_shuffle_ps( m, m, _MM_SHUFFLE( 1, 1, 1, 1 ) );
	b3V32 z = _mm_shuffle_ps( m, m, _MM_SHUFFLE( 2, 2, 2, 2 ) );

	return _mm_add_ps( _mm_add_ps( x, y ), z );
}

#else

#define B3_TRANSPOSE3( C1, C2, C3 )                                                                                              \
	{                                                                                                                            \
		float temp1 = C1.y;                                                                                                      \
		float temp2 = C1.z;                                                                                                      \
		float temp3 = C2.z;                                                                                                      \
                                                                                                                                 \
		C1.y = C2.x;                                                                                                             \
		C1.z = C3.x;                                                                                                             \
		C2.z = C3.y;                                                                                                             \
                                                                                                                                 \
		C2.x = temp1;                                                                                                            \
		C3.x = temp2;                                                                                                            \
		C3.y = temp3;                                                                                                            \
	}

static inline b3V32 b3SplatXV( b3V32 a )
{
	return B3_LITERAL( b3V32 ){ a.x, a.x, a.x };
}

static inline b3V32 b3SplatYV( b3V32 a )
{
	return B3_LITERAL( b3V32 ){ a.y, a.y, a.y };
}

static inline b3V32 b3SplatZV( b3V32 a )
{
	return B3_LITERAL( b3V32 ){ a.z, a.z, a.z };
}

static inline bool b3AnyGreaterEq3V( b3V32 a, b3V32 b )
{
	return a.x >= b.x || a.y >= b.y || a.z >= b.z;
}

static inline b3V32 b3Dot3V( b3V32 a, b3V32 b )
{
	float d = a.x * b.x + a.y * b.y + a.z * b.z;
	return B3_LITERAL( b3V32 ){ d, d, d };
}

#endif

bool b3TestBoundsTriangleOverlap( b3V32 nodeCenter, b3V32 nodeExtent, b3V32 vertex1, b3V32 vertex2, b3V32 vertex3 )
{
	b3V32 two = b3SplatV( 2.0f );

	// Setup triangle
	vertex1 = b3SubV( vertex1, nodeCenter );
	vertex2 = b3SubV( vertex2, nodeCenter );
	vertex3 = b3SubV( vertex3, nodeCenter );

	// Face separation
	b3V32 triangleMin = b3MinV( vertex1, b3MinV( vertex2, vertex3 ) );
	b3V32 triangleMax = b3MaxV( vertex1, b3MaxV( vertex2, vertex3 ) );

	b3V32 separation1 = b3SubV( triangleMin, nodeExtent );
	b3V32 separation2 = b3AddV( triangleMax, nodeExtent );

	b3V32 faceSeparation = b3MaxV( separation1, b3NegV( separation2 ) );
	if ( b3AnyGreater3V( faceSeparation, b3_zeroV ) )
	{
		return false;
	}

	// SAT: Face separation
	b3V32 edge1 = b3SubV( vertex2, vertex1 );
	b3V32 edge2 = b3SubV( vertex3, vertex2 );
	b3V32 edge3 = b3SubV( vertex1, vertex3 );

	b3V32 normal = b3CrossV( edge1, edge2 );

	b3V32 triangleSeparation = b3SubV( b3AbsV( b3Dot3V( normal, vertex1 ) ), b3Dot3V( b3AbsV( normal ), nodeExtent ) );
	if ( b3AnyGreater3V( triangleSeparation, b3_zeroV ) )
	{
		return false;
	}

	// SAT: Edge separation
	b3V32 edgeSeparation1 = b3SubV( b3SubV( b3AbsV( b3CrossV( edge1, b3AddV( vertex1, vertex3 ) ) ), b3AbsV( b3CrossV( edge1, edge3 ) ) ),
									b3MulV( two, b3ModifiedCrossV( b3AbsV( edge1 ), nodeExtent ) ) );
	if ( b3AnyGreater3V( edgeSeparation1, b3_zeroV ) )
	{
		return false;
	}

	b3V32 edgeSeparation2 = b3SubV( b3SubV( b3AbsV( b3CrossV( edge2, b3AddV( vertex1, vertex2 ) ) ), b3AbsV( b3CrossV( edge2, edge1 ) ) ),
									b3MulV( two, b3ModifiedCrossV( b3AbsV( edge2 ), nodeExtent ) ) );
	if ( b3AnyGreater3V( edgeSeparation2, b3_zeroV ) )
	{
		return false;
	}

	b3V32 edgeSeparation3 = b3SubV( b3SubV( b3AbsV( b3CrossV( edge3, b3AddV( vertex2, vertex3 ) ) ), b3AbsV( b3CrossV( edge3, edge2 ) ) ),
									b3MulV( two, b3ModifiedCrossV( b3AbsV( edge3 ), nodeExtent ) ) );
	if ( b3AnyGreater3V( edgeSeparation3, b3_zeroV ) )
	{
		return false;
	}

	return true;
}

float b3IntersectRayTriangle( b3V32 rayStart, b3V32 rayDelta, b3V32 vertex1, b3V32 vertex2, b3V32 vertex3 )
{
	// Test if ray intersects this triangle sharing same calculations for each triangle
	{
		b3V32 edge1 = b3SubV( vertex3, vertex2 );
		b3V32 edge2 = b3SubV( vertex1, vertex3 );
		b3V32 edge3 = b3SubV( vertex2, vertex1 );

		b3V32 midPoint1 = b3MulV( b3_halfV, b3AddV( vertex2, vertex3 ) );
		b3V32 midPoint2 = b3MulV( b3_halfV, b3AddV( vertex3, vertex1 ) );
		b3V32 midPoint3 = b3MulV( b3_halfV, b3AddV( vertex1, vertex2 ) );

		b3V32 normal1 = b3CrossV( edge1, b3SubV( midPoint1, rayStart ) );
		b3V32 normal2 = b3CrossV( edge2, b3SubV( midPoint2, rayStart ) );
		b3V32 normal3 = b3CrossV( edge3, b3SubV( midPoint3, rayStart ) );
		B3_TRANSPOSE3( normal1, normal2, normal3 );

		b3V32 rayDeltaX = b3SplatXV( rayDelta );
		b3V32 rayDeltaY = b3SplatYV( rayDelta );
		b3V32 rayDeltaZ = b3SplatZV( rayDelta );

		b3V32 volumes = b3AddV( b3AddV( b3MulV( normal1, rayDeltaX ), b3MulV( normal2, rayDeltaY ) ), b3MulV( normal3, rayDeltaZ ) );
		if ( b3AnyLess3V( volumes, b3_zeroV ) )
		{
			return 1.0f;
		}
	}

	// Compute intersection with triangle plane
	b3V32 edge1 = b3SubV( vertex2, vertex1 );
	b3V32 edge2 = b3SubV( vertex3, vertex1 );
	b3V32 normal = b3CrossV( edge1, edge2 );

	b3V32 denominator = b3Dot3V( normal, rayDelta );
	if ( b3AnyGreaterEq3V( denominator, b3_zeroV ) )
	{
		return 1.0f;
	}

	b3V32 lambda = b3DivV( b3Dot3V( normal, b3SubV( vertex1, rayStart ) ), denominator );
	if ( b3AnyLessEq3V( lambda, b3_zeroV ) )
	{
		return 1.0f;
	}

	lambda = b3MinV( lambda, b3_oneV );
	return b3GetXV( lambda );
}

// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box3d/math_functions.h"

#define RAND_LIMIT 32767
#define RAND_SEED 12345

// Global seed for simple random number generator.

#ifdef __cplusplus
extern "C"
{
#endif

extern uint32_t g_randomSeed;

int GetNumberOfCores( void );

#ifdef __cplusplus
}
#endif

// Simple random number generator. Using this instead of rand() for cross platform determinism.
B3_INLINE int RandomInt()
{
	// XorShift32 algorithm
	uint32_t x = g_randomSeed;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	g_randomSeed = x;

	// Map the 32-bit value to the range 0 to RAND_LIMIT
	return (int)( x % ( RAND_LIMIT + 1 ) );
}

// Random integer in range [lo, hi]
B3_INLINE float RandomIntRange( int lo, int hi )
{
	return lo + RandomInt() % ( hi - lo + 1 );
}

// Random number in range [-1,1]
B3_INLINE float RandomFloat()
{
	float r = (float)( RandomInt() & ( RAND_LIMIT ) );
	r /= RAND_LIMIT;
	r = 2.0f * r - 1.0f;
	return r;
}

// Random floating point number in range [lo, hi]
B3_INLINE float RandomFloatRange( float lo, float hi )
{
	float r = (float)( RandomInt() & ( RAND_LIMIT ) );
	r /= RAND_LIMIT;
	r = ( hi - lo ) * r + lo;
	return r;
}

// Random vector with coordinates in range [lo, hi]
B3_INLINE b3Vec3 RandomVec3( b3Vec3 lo, b3Vec3 hi )
{
	b3Vec3 v;
	v.x = RandomFloatRange( lo.x, hi.x );
	v.y = RandomFloatRange( lo.y, hi.y );
	v.z = RandomFloatRange( lo.z, hi.z );
	return v;
}

// Random world position with coordinates in range [lo, hi]
B3_INLINE b3Pos RandomPos( b3Vec3 lo, b3Vec3 hi )
{
	b3Pos v;
	v.x = RandomFloatRange( lo.x, hi.x );
	v.y = RandomFloatRange( lo.y, hi.y );
	v.z = RandomFloatRange( lo.z, hi.z );
	return v;
}

B3_INLINE b3Vec3 RandomVec3Uniform( float lo, float hi )
{
	b3Vec3 v;
	v.x = RandomFloatRange( lo, hi );
	v.y = RandomFloatRange( lo, hi );
	v.z = RandomFloatRange( lo, hi );
	return v;
}

B3_INLINE b3Vec3 RandomUnitVector()
{
	// Generate uniformly distributed random quaternion using Shoemake's method
	// Reference: "Uniform Random Rotations", Ken Shoemake, Graphics Gems III, 1992

	float u1 = RandomFloatRange( 0.0f, 1.0f );
	float u2 = RandomFloatRange( 0.0f, 2.0f * B3_PI );
	float u3 = RandomFloatRange( 0.0f, 2.0f * B3_PI );

	float sqrt1MinusU1 = sqrtf( 1.0f - u1 );
	float sqrtU1 = sqrtf( u1 );

	b3Vec3 v;
	v.x = sqrt1MinusU1 * sinf( u2 );
	v.y = sqrt1MinusU1 * cosf( u2 );
	v.z = sqrtU1 * sinf( u3 );

	return v;
}

B3_INLINE b3Quat RandomQuat()
{
	// Generate uniformly distributed random quaternion using Shoemake's method
	// Reference: "Uniform Random Rotations", Ken Shoemake, Graphics Gems III, 1992

	float u1 = RandomFloatRange( 0.0f, 1.0f );
	float u2 = RandomFloatRange( 0.0f, 2.0f * B3_PI );
	float u3 = RandomFloatRange( 0.0f, 2.0f * B3_PI );

	float sqrt1MinusU1 = sqrtf( 1.0f - u1 );
	float sqrtU1 = sqrtf( u1 );

	b3Quat q;
	q.v.x = sqrt1MinusU1 * sinf( u2 );
	q.v.y = sqrt1MinusU1 * cosf( u2 );
	q.v.z = sqrtU1 * sinf( u3 );
	q.s = sqrtU1 * cosf( u3 );

	return q;
}

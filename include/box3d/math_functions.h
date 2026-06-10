// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "base.h"

#include <float.h>

// for sqrtf and remainderf
#include <math.h>
#include <stdbool.h>

/**
 * @defgroup math Math
 * @brief Vector math types and functions
 * @{
 */

/// https://en.wikipedia.org/wiki/Pi
#define B3_PI 3.14159265359f

/// Convenience macro to convert from degrees to radians.
#define B3_DEG_TO_RAD 0.01745329251f

/// Convenience macro to convert from radians to degrees.
#define B3_RAD_TO_DEG 57.2957795131f

/// Minimum scale used for scaling collision meshes, etc.
#define B3_MIN_SCALE 0.01f

/// A 2D vector.
typedef struct b3Vec2
{
	float x;
	float y;
} b3Vec2;

/// A 3D vector.
typedef struct b3Vec3
{
	float x;
	float y;
	float z;
} b3Vec3;

/// Cosine and sine pair.
/// This uses a custom implementation designed for cross-platform determinism.
typedef struct b3CosSin
{
	/// cosine and sine
	float cosine;
	float sine;
} b3CosSin;

/// A quaternion.
typedef struct b3Quat
{
	b3Vec3 v;
	float s;
} b3Quat;

/// A rigid transform.
typedef struct b3Transform
{
	b3Vec3 p;
	b3Quat q;
} b3Transform;

/// A 3x3 matrix.
typedef struct b3Matrix3
{
	b3Vec3 cx, cy, cz;
} b3Matrix3;

/// Axis aligned bounding box.
typedef struct b3AABB
{
	b3Vec3 lowerBound;
	b3Vec3 upperBound;
} b3AABB;

/// A plane.
/// separation = dot(normal, point) - offset
typedef struct b3Plane
{
	b3Vec3 normal;
	float offset;
} b3Plane;

static const b3Vec3 b3Vec3_zero = { 0.0f, 0.0f, 0.0f };
static const b3Vec3 b3Vec3_one = { 1.0f, 1.0f, 1.0f };
static const b3Vec3 b3Vec3_axisX = { 1.0f, 0.0f, 0.0f };
static const b3Vec3 b3Vec3_axisY = { 0.0f, 1.0f, 0.0f };
static const b3Vec3 b3Vec3_axisZ = { 0.0f, 0.0f, 1.0f };
static const b3Quat b3Quat_identity = { { 0.0f, 0.0f, 0.0f }, 1.0f };
static const b3Transform b3Transform_identity = { { 0.0f, 0.0f, 0.0f }, { { 0.0f, 0.0f, 0.0f }, 1.0f } };
static const b3Matrix3 b3Mat3_zero = {
	{ 0.0f, 0.0f, 0.0f },
	{ 0.0f, 0.0f, 0.0f },
	{ 0.0f, 0.0f, 0.0f },
};
static const b3Matrix3 b3Mat3_identity = {
	{ 1.0f, 0.0f, 0.0f },
	{ 0.0f, 1.0f, 0.0f },
	{ 0.0f, 0.0f, 1.0f },
};

/// @return the minimum of two integers.
B3_INLINE int b3MinInt( int a, int b )
{
	return a < b ? a : b;
}

/// @return the maximum of two integers.
B3_INLINE int b3MaxInt( int a, int b )
{
	return a > b ? a : b;
}

/// @return an integer clamped between a lower and upper bound.
B3_INLINE int b3ClampInt( int a, int lower, int upper )
{
	return a < lower ? lower : ( upper < a ? upper : a );
}

/// @return is this float valid (finite and not NaN).
B3_API bool b3IsValidFloat( float a );

/// @return the absolute value of a float.
B3_INLINE float b3AbsFloat( float a )
{
	return a < 0 ? -a : a;
}

/// @return the minimum of two floats.
B3_INLINE float b3MinFloat( float a, float b )
{
	return a < b ? a : b;
}

/// @return the maximum of two floats.
B3_INLINE float b3MaxFloat( float a, float b )
{
	return a > b ? a : b;
}

/// @return a float clamped between a lower and upper bound.
B3_INLINE float b3ClampFloat( float a, float lower, float upper )
{
	return a < lower ? lower : ( upper < a ? upper : a );
}

/// Interpolate a scalar.
B3_INLINE float b3LerpFloat( float a, float b, float alpha )
{
	return ( 1.0f - alpha ) * a + alpha * b;
}

/// Compute an approximate arctangent in the range [-pi, pi]
/// This is hand coded for cross-platform determinism. The atan2f
/// function in the standard library is not cross-platform deterministic.
///	Accurate to around 0.0023 degrees.
B3_API float b3Atan2( float y, float x );

/// Compute the cosine and sine of an angle in radians. Implemented
/// for cross-platform determinism.
B3_API b3CosSin b3ComputeCosSin( float radians );

/// @deprecated 
B3_INLINE float b3Sin( float radians )
{
	b3CosSin cs = b3ComputeCosSin( radians );
	return cs.sine;
}

/// @deprecated 
B3_INLINE float b3Cos( float radians )
{
	b3CosSin cs = b3ComputeCosSin( radians );
	return cs.cosine;
}

/// Convert any angle into the range [-pi, pi].
B3_INLINE float b3UnwindAngle( float radians )
{
	// Assuming this is deterministic
	return remainderf( radians, 2.0f * B3_PI );
}

/// Vector addition.
B3_INLINE b3Vec3 b3Add( b3Vec3 a, b3Vec3 b )
{
	return B3_LITERAL( b3Vec3 ){ a.x + b.x, a.y + b.y, a.z + b.z };
}

/// Vector subtraction.
B3_INLINE b3Vec3 b3Sub( b3Vec3 a, b3Vec3 b )
{
	return B3_LITERAL( b3Vec3 ){ a.x - b.x, a.y - b.y, a.z - b.z };
}

/// Vector component-wise multiplication.
B3_INLINE b3Vec3 b3Mul( b3Vec3 a, b3Vec3 b )
{
	return B3_LITERAL( b3Vec3 ){ a.x * b.x, a.y * b.y, a.z * b.z };
}

/// Vector negation.
B3_INLINE b3Vec3 b3Neg( b3Vec3 a )
{
	return B3_LITERAL( b3Vec3 ){ -a.x, -a.y, -a.z };
}

/// Vector dot product.
B3_INLINE float b3Dot( b3Vec3 a, b3Vec3 b )
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// Vector length.
B3_INLINE float b3Length( b3Vec3 v )
{
	return sqrtf( b3Dot( v, v ) );
}

/// Vector length squared.
B3_INLINE float b3LengthSquared( b3Vec3 a )
{
	return a.x * a.x + a.y * a.y + a.z * a.z;
}

/// Distance between two points.
B3_INLINE float b3Distance( b3Vec3 a, b3Vec3 b )
{
	b3Vec3 dv = { b.x - a.x, b.y - a.y, b.z - a.z };
	return b3Length( dv );
}

/// Squared distance between two points.
B3_INLINE float b3DistanceSquared( b3Vec3 a, b3Vec3 b )
{
	b3Vec3 dv = { b.x - a.x, b.y - a.y, b.z - a.z };
	return dv.x * dv.x + dv.y * dv.y + dv.z * dv.z;
}

/// Normalize a vector. Returns a zero vector if the input vector is very small.
B3_INLINE b3Vec3 b3Normalize( b3Vec3 a )
{
	float lengthSquared = a.x * a.x + a.y * a.y + a.z * a.z;

	if ( lengthSquared > 1000.0f * FLT_MIN )
	{
		float s = 1.0f / sqrtf( lengthSquared );
		b3Vec3 u = { s * a.x, s * a.y, s * a.z };
		return u;
	}

	return B3_LITERAL( b3Vec3 ){ 0.0f, 0.0f, 0.0f };
}

/// Normalize a vector and return the length. Returns a zero vector
/// if the input is very small.
B3_INLINE b3Vec3 b3GetLengthAndNormalize( float* length, b3Vec3 a )
{
	*length = b3Length( a );
	if ( *length < FLT_EPSILON )
	{
		return b3Vec3_zero;
	}

	float invLength = 1.0f / *length;
	b3Vec3 n = { invLength * a.x, invLength * a.y, invLength * a.z };
	return n;
}

/// Get a unit vector that is perpendicular to the supplied vector.
B3_INLINE b3Vec3 b3Perp( b3Vec3 a )
{
	// Suppose vector a has all equal components and is a unit vector: a = (s, s, s)
	// Then 3*s*s = 1, s = sqrt(1/3) = 0.57735. This means that at least one component
	// of a unit vector must be greater or equal to 0.57735.
	b3Vec3 p;
	if ( a.x < -0.5f || 0.5f < a.x )
	{
		p = B3_LITERAL( b3Vec3 ){ a.y, -a.x, 0.0f };
	}
	else
	{
		p = B3_LITERAL( b3Vec3 ){ 0.0f, a.z, -a.y };
	}

	return b3Normalize( p );
}

/// Is a vector normalized? In other words, does it have unit length?
B3_INLINE bool b3IsNormalized( b3Vec3 a )
{
	float aa = b3Dot( a, a );
	return b3AbsFloat( 1.0f - aa ) < 100.0f * FLT_EPSILON;
}

/// a + s * b
B3_INLINE b3Vec3 b3MulAdd( b3Vec3 a, float s, b3Vec3 b )
{
	return B3_LITERAL( b3Vec3 ){ a.x + s * b.x, a.y + s * b.y, a.z + s * b.z };
}

/// a - s * b
B3_INLINE b3Vec3 b3MulSub( b3Vec3 a, float s, b3Vec3 b )
{
	return B3_LITERAL( b3Vec3 ){ a.x - s * b.x, a.y - s * b.y, a.z - s * b.z };
}

/// a + s * b
B3_INLINE b3Vec3 b3MulSV( float s, b3Vec3 a )
{
	return B3_LITERAL( b3Vec3 ){ s * a.x, s * a.y, s * a.z };
}

/// https://en.wikipedia.org/wiki/Cross_product
B3_INLINE b3Vec3 b3Cross( b3Vec3 a, b3Vec3 b )
{
	b3Vec3 c;
	c.x = a.y * b.z - a.z * b.y;
	c.y = a.z * b.x - a.x * b.z;
	c.z = a.x * b.y - a.y * b.x;
	return c;
}

/// Linearly interpolation between two vectors.
B3_INLINE b3Vec3 b3Lerp( b3Vec3 a, b3Vec3 b, float alpha )
{
	B3_ASSERT( 0.0f <= alpha && alpha <= 1.0f );

	b3Vec3 c = {
		( 1.0f - alpha ) * a.x + alpha * b.x,
		( 1.0f - alpha ) * a.y + alpha * b.y,
		( 1.0f - alpha ) * a.z + alpha * b.z,
	};
	return c;
}

/// Blend two vectors: s * a + t * b
B3_INLINE b3Vec3 b3Blend2( float s, b3Vec3 a, float t, b3Vec3 b )
{
	b3Vec3 d = {
		s * a.x + t * b.x,
		s * a.y + t * b.y,
		s * a.z + t * b.z,
	};
	return d;
}

/// Component-wise absolute value.
B3_INLINE b3Vec3 b3Abs( b3Vec3 a )
{
	return B3_LITERAL( b3Vec3 ){
		b3AbsFloat( a.x ),
		b3AbsFloat( a.y ),
		b3AbsFloat( a.z ),
	};
}

/// Component-wise -1 or 1 (1 if zero).
B3_INLINE b3Vec3 b3Sign( b3Vec3 a )
{
	return B3_LITERAL( b3Vec3 ){
		a.x >= 0.0f ? 1.0f : -1.0f,
		a.y >= 0.0f ? 1.0f : -1.0f,
		a.z >= 0.0f ? 1.0f : -1.0f,
	};
}

/// Component-wise minimum value.
B3_INLINE b3Vec3 b3Min( b3Vec3 a, b3Vec3 b )
{
	return B3_LITERAL( b3Vec3 ){
		b3MinFloat( a.x, b.x ),
		b3MinFloat( a.y, b.y ),
		b3MinFloat( a.z, b.z ),
	};
}

/// Component-wise maximum value.
B3_INLINE b3Vec3 b3Max( b3Vec3 a, b3Vec3 b )
{
	return B3_LITERAL( b3Vec3 ){
		b3MaxFloat( a.x, b.x ),
		b3MaxFloat( a.y, b.y ),
		b3MaxFloat( a.z, b.z ),
	};
}

/// Component-wise clamped value.
B3_INLINE b3Vec3 b3Clamp( b3Vec3 a, b3Vec3 lower, b3Vec3 upper )
{
	b3Vec3 b;
	b.x = b3ClampFloat( a.x, lower.x, upper.x );
	b.y = b3ClampFloat( a.y, lower.y, upper.y );
	b.z = b3ClampFloat( a.z, lower.z, upper.z );
	return b;
}

/// Create a safe scaling value for scaling collision. This allows
/// negative scale, but keeps scale sufficiently far from zero.
B3_INLINE b3Vec3 b3SafeScale( b3Vec3 a )
{
	b3Vec3 absScale = b3Abs( a );
	b3Vec3 minScale = { B3_MIN_SCALE, B3_MIN_SCALE, B3_MIN_SCALE };
	b3Vec3 safeScale = b3Mul( b3Sign( a ), b3Max( absScale, minScale ) );
	return safeScale;
}

/// Does the supplied quaternion have unit length?
B3_INLINE bool b3IsNormalizedQuat( b3Quat q )
{
	float qq = q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z + q.s * q.s;
	return 1.0f - 20.0f * FLT_EPSILON < qq && qq < 1.0f + 20.0f * FLT_EPSILON;
}

/// Rotate a vector.
B3_INLINE b3Vec3 b3RotateVector( b3Quat q, b3Vec3 v )
{
	// v + 2 * cross(q.v, cross(q.v, v) + q.s * v)
	// B3_ASSERT( b3IsNormalizedQuat( q ) );
	b3Vec3 t1 = b3Cross( q.v, v );
	b3Vec3 t2 = b3MulAdd( t1, q.s, v );
	b3Vec3 t3 = b3Cross( q.v, t2 );
	return b3MulAdd( v, 2.0f, t3 );
}

/// Inverse rotate a vector.
B3_INLINE b3Vec3 b3InvRotateVector( b3Quat q, b3Vec3 v )
{
	// v + 2 * cross(q.v, cross(q.v, v) - q.s * v)
	// B3_ASSERT( b3IsNormalizedQuat( q ) );
	b3Vec3 t1 = b3Cross( q.v, v );
	b3Vec3 t2 = b3MulSub( t1, q.s, v );
	b3Vec3 t3 = b3Cross( q.v, t2 );
	return b3MulAdd( v, 2.0f, t3 );
}

/// Compute dot product of two quaternions. Useful for polarity tests.
B3_INLINE float b3DotQuat( b3Quat a, b3Quat b )
{
	return a.v.x * b.v.x + a.v.y * b.v.y + a.v.z * b.v.z + a.s * b.s;
}

/// Multiply two quaternions.
B3_INLINE b3Quat b3MulQuat( b3Quat q1, b3Quat q2 )
{
	b3Vec3 t1 = b3Cross( q1.v, q2.v );
	b3Vec3 t2 = b3MulAdd( t1, q1.s, q2.v );
	b3Vec3 t3 = b3MulAdd( t2, q2.s, q1.v );
	b3Quat q = { t3, q1.s * q2.s - b3Dot( q1.v, q2.v ) };
	return q;
}

/// Compute a relative quaternion.
/// inv(q1) * q2
B3_INLINE b3Quat b3InvMulQuat( b3Quat q1, b3Quat q2 )
{
	b3Vec3 t1 = b3Cross( q2.v, q1.v );
	b3Vec3 t2 = b3MulAdd( t1, q1.s, q2.v );
	b3Vec3 t3 = b3MulSub( t2, q2.s, q1.v );
	b3Quat q = { t3, q1.s * q2.s + b3Dot( q1.v, q2.v ) };
	return q;
}

/// Quaternion conjugate (cheap inverse).
B3_INLINE b3Quat b3Conjugate( b3Quat q )
{
	return B3_LITERAL( b3Quat ){ { -q.v.x, -q.v.y, -q.v.z }, q.s };
}

/// Component-wise quaternion negation.
B3_INLINE b3Quat b3NegateQuat( b3Quat q )
{
	return B3_LITERAL( b3Quat ){ { -q.v.x, -q.v.y, -q.v.z }, -q.s };
}

/// Normalize a quaternion.
B3_INLINE b3Quat b3NormalizeQuat( b3Quat q )
{
	float lengthSq = b3DotQuat( q, q );
	if ( lengthSq > 1000.0f * FLT_MIN )
	{
		float s = 1.0f / sqrtf( lengthSq );
		b3Quat qn = { { s * q.v.x, s * q.v.y, s * q.v.z }, s * q.s };
		return qn;
	}

	return b3Quat_identity;
}

/// Make a quaternion that is equivalent to rotating around a axis by a specified angle.
B3_INLINE b3Quat b3MakeQuatFromAxisAngle( b3Vec3 axis, float radians )
{
	B3_ASSERT( b3IsNormalized( axis ) );
	b3CosSin cs = b3ComputeCosSin( 0.5f * radians );
	b3Quat q = { { cs.sine * axis.x, cs.sine * axis.y, cs.sine * axis.z }, cs.cosine };
	return q;
}

/// Get the axis and angle from a quaternion. Assumes the quaternion is normalized.
B3_INLINE b3Vec3 b3GetAxisAngle( float* radians, b3Quat q )
{
	float length = sqrtf( q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z );
	*radians = 2.0f * b3Atan2( length, q.s );
	if ( length > 0.0f )
	{
		float invLength = 1.0f / length;
		b3Vec3 axis = { invLength * q.v.x, invLength * q.v.y, invLength * q.v.z };
		return axis;
	}

	return b3Vec3_zero;
}

/// Get the angle for a quaternion in radians
B3_INLINE float b3GetQuatAngle( b3Quat q )
{
	float length = sqrtf( q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z );
	return 2.0f * b3Atan2( length, q.s );
}

/// Extract a quaternion from a rotation matrix.
B3_API b3Quat b3MakeQuatFromMatrix( const b3Matrix3* m );

/// Find a quaternion that rotations one vector to another.
B3_API b3Quat b3ComputeQuatBetweenUnitVectors( b3Vec3 v1, b3Vec3 v2 );

/// Linearly interpolate and normalize between two quaternions
B3_INLINE b3Quat b3NLerp( b3Quat q1, b3Quat q2, float alpha )
{
	B3_VALIDATE( 0.0f <= alpha && alpha <= 1.0f );
	if ( b3DotQuat( q1, q2 ) < 0.0f )
	{
		q1 = B3_LITERAL( b3Quat ){ { -q1.v.x, -q1.v.y, -q1.v.z }, -q1.s };
	}

	b3Quat q;
	q.v = b3Lerp( q1.v, q2.v, alpha );
	q.s = ( 1.0f - alpha ) * q1.s + alpha * q2.s;

	return b3NormalizeQuat( q );
}

/// Multiply two transforms. If the result is applied to a point p local to frame B,
/// the transform would first convert p to a point local to frame A, then into a point
/// in the world frame. This is useful if frame B is a child of frame A.
B3_INLINE b3Transform b3MulTransforms( b3Transform a, b3Transform b )
{
	b3Transform out;
	out.p = b3Add( b3RotateVector( a.q, b.p ), a.p );
	out.q = b3MulQuat( a.q, b.q );
	return out;
}

/// Creates a transform that converts a local point in frame B to a local point in frame A.
/// This is useful for transforming points between the local spaces of two frames that are
/// in world space.
B3_FORCE_INLINE b3Transform b3InvMulTransforms( b3Transform a, b3Transform b )
{
	b3Transform out;
	out.p = b3InvRotateVector( a.q, b3Sub( b.p, a.p ) );
	out.q = b3InvMulQuat( a.q, b.q );
	return out;
}

/// Get the inverse of a transform.
B3_INLINE b3Transform b3InvertTransform( b3Transform t )
{
	b3Transform out;
	out.p = b3InvRotateVector( t.q, b3Neg( t.p ) );
	out.q = b3Conjugate( t.q );
	return out;
}

/// Transform a point.
B3_INLINE b3Vec3 b3TransformPoint( b3Transform t, b3Vec3 v )
{
	b3Vec3 rv = b3RotateVector( t.q, v );
	return b3Add( rv, t.p );
}

/// Inverse transform a point.
B3_INLINE b3Vec3 b3InvTransformPoint( b3Transform t, b3Vec3 v )
{
	return b3InvRotateVector( t.q, b3Sub( v, t.p ) );
}

/// Compute the determinant of a 3-by-3 matrix.
B3_INLINE float b3Det( b3Matrix3 m )
{
	return b3Dot( m.cx, b3Cross( m.cy, m.cz ) );
}

/// Multiply a matrix times a column vector.
B3_INLINE b3Vec3 b3MulMV( b3Matrix3 m, b3Vec3 a )
{
	b3Vec3 b = {
		m.cx.x * a.x + m.cy.x * a.y + m.cz.x * a.z,
		m.cx.y * a.x + m.cy.y * a.y + m.cz.y * a.z,
		m.cx.z * a.x + m.cy.z * a.y + m.cz.z * a.z,
	};
	return b;
}

/// Negate a matrix.
B3_INLINE b3Matrix3 b3NegateMat3( b3Matrix3 a )
{
	return B3_LITERAL( b3Matrix3 ){
		{ -a.cx.x, -a.cx.y, -a.cx.z },
		{ -a.cy.x, -a.cy.y, -a.cy.z },
		{ -a.cz.x, -a.cz.y, -a.cz.z },
	};
}

/// Matrix addition.
/// @return a + b
B3_INLINE b3Matrix3 b3AddMM( b3Matrix3 a, b3Matrix3 b )
{
	return B3_LITERAL( b3Matrix3 ){
		{ a.cx.x + b.cx.x, a.cx.y + b.cx.y, a.cx.z + b.cx.z },
		{ a.cy.x + b.cy.x, a.cy.y + b.cy.y, a.cy.z + b.cy.z },
		{ a.cz.x + b.cz.x, a.cz.y + b.cz.y, a.cz.z + b.cz.z },
	};
}

/// Matrix subtraction.
/// @return a - b
B3_INLINE b3Matrix3 b3SubMM( b3Matrix3 a, b3Matrix3 b )
{
	return B3_LITERAL( b3Matrix3 ){
		{ a.cx.x - b.cx.x, a.cx.y - b.cx.y, a.cx.z - b.cx.z },
		{ a.cy.x - b.cy.x, a.cy.y - b.cy.y, a.cy.z - b.cy.z },
		{ a.cz.x - b.cz.x, a.cz.y - b.cz.y, a.cz.z - b.cz.z },
	};
}

/// Multiply a matrix by a scalar, component-wise.
B3_INLINE b3Matrix3 b3MulSM( float s, b3Matrix3 a )
{
	return B3_LITERAL( b3Matrix3 ){
		{ s * a.cx.x, s * a.cx.y, s * a.cx.z },
		{ s * a.cy.x, s * a.cy.y, s * a.cy.z },
		{ s * a.cz.x, s * a.cz.y, s * a.cz.z },
	};
}

/// Matrix multiplication.
/// @return a * b
B3_INLINE b3Matrix3 b3MulMM( b3Matrix3 a, b3Matrix3 b )
{
	b3Matrix3 out;
	out.cx = b3MulMV( a, b.cx );
	out.cy = b3MulMV( a, b.cy );
	out.cz = b3MulMV( a, b.cz );
	return out;
}

/// Matrix transpose.
B3_INLINE b3Matrix3 b3Transpose( b3Matrix3 m )
{
	b3Matrix3 out;
	out.cx = B3_LITERAL( b3Vec3 ){ m.cx.x, m.cy.x, m.cz.x };
	out.cy = B3_LITERAL( b3Vec3 ){ m.cx.y, m.cy.y, m.cz.y };
	out.cz = B3_LITERAL( b3Vec3 ){ m.cx.z, m.cy.z, m.cz.z };

	return out;
}

/// General matrix inverse.
B3_INLINE b3Matrix3 b3InvertMatrix( b3Matrix3 m )
{
	float det = b3Det( m );
	if ( b3AbsFloat( det ) > 1000.0f * FLT_MIN )
	{
		float invDet = 1.0f / det;
		b3Matrix3 out;
		out.cx = b3MulSV( invDet, b3Cross( m.cy, m.cz ) );
		out.cy = b3MulSV( invDet, b3Cross( m.cz, m.cx ) );
		out.cz = b3MulSV( invDet, b3Cross( m.cx, m.cy ) );

		return b3Transpose( out );
	}

	return b3Mat3_zero;
}

/// Solve a matrix equation.
/// @return inv(m) * a
B3_INLINE b3Vec3 b3Solve3( b3Matrix3 m, b3Vec3 a )
{
	float det = b3Det( m );
	if ( b3AbsFloat( det ) > 1000.0f * FLT_MIN )
	{
		float invDet = 1.0f / det;
		b3Matrix3 s;
		s.cx = b3Cross( m.cy, m.cz );
		s.cy = b3Cross( m.cz, m.cx );
		s.cz = b3Cross( m.cx, m.cy );

		b3Vec3 b = {
			invDet * b3Dot( s.cx, a ),
			invDet * b3Dot( s.cy, a ),
			invDet * b3Dot( s.cz, a ),
		};

		return b;
	}

	return b3Vec3_zero;
}

/// Invert a matrix.
B3_INLINE b3Matrix3 b3InvertT( b3Matrix3 m )
{
	float det = b3Det( m );
	if ( b3AbsFloat( det ) > 1000.0f * FLT_MIN )
	{
		float invDet = 1.0f / det;
		b3Matrix3 out;
		out.cx = b3MulSV( invDet, b3Cross( m.cy, m.cz ) );
		out.cy = b3MulSV( invDet, b3Cross( m.cz, m.cx ) );
		out.cz = b3MulSV( invDet, b3Cross( m.cx, m.cy ) );
		return out;
	}

	return b3Mat3_zero;
}

/// Get the component-wise absolute value of a matrix.
B3_INLINE b3Matrix3 b3AbsMatrix3( b3Matrix3 m )
{
	b3Matrix3 out;
	out.cx = b3Abs( m.cx );
	out.cy = b3Abs( m.cy );
	out.cz = b3Abs( m.cz );

	return out;
}

/// Make a matrix from a quaternion. This is useful if you need to
/// rotate many vectors.
/// The force inline improves the performance of b3ShapeDistance.
B3_FORCE_INLINE b3Matrix3 b3MakeMatrixFromQuat( b3Quat q )
{
	float xx = q.v.x * q.v.x;
	float yy = q.v.y * q.v.y;
	float zz = q.v.z * q.v.z;
	float xy = q.v.x * q.v.y;
	float xz = q.v.x * q.v.z;
	float xw = q.v.x * q.s;
	float yz = q.v.y * q.v.z;
	float yw = q.v.y * q.s;
	float zw = q.v.z * q.s;

	return B3_LITERAL( b3Matrix3 ){
		{ 1.0f - 2.0f * ( yy + zz ), 2.0f * ( xy + zw ), 2.0f * ( xz - yw ) },
		{ 2.0f * ( xy - zw ), 1.0f - 2.0f * ( xx + zz ), 2.0f * ( yz + xw ) },
		{ 2.0f * ( xz + yw ), 2.0f * ( yz - xw ), 1.0f - 2.0f * ( xx + yy ) },
	};
}

/// Get the inertia tensor of an offset point.
/// https://en.wikipedia.org/wiki/Parallel_axis_theorem
B3_API b3Matrix3 b3Steiner( float mass, b3Vec3 origin );

/// Get the AABB of a point cloud.
B3_INLINE b3AABB b3MakeAABB( const b3Vec3* points, int count, float radius )
{
	B3_ASSERT( count > 0 );
	b3AABB a = { points[0], points[0] };
	for ( int i = 1; i < count; ++i )
	{
		a.lowerBound = b3Min( a.lowerBound, points[i] );
		a.upperBound = b3Max( a.upperBound, points[i] );
	}

	b3Vec3 r = { radius, radius, radius };
	a.lowerBound = b3Sub( a.lowerBound, r );
	a.upperBound = b3Add( a.upperBound, r );

	return a;
}

/// Does a fully contain b?
B3_INLINE bool b3AABB_Contains( b3AABB a, b3AABB b )
{
	if ( a.lowerBound.x > b.lowerBound.x || b.upperBound.x > a.upperBound.x )
		return false;
	if ( a.lowerBound.y > b.lowerBound.y || b.upperBound.y > a.upperBound.y )
		return false;
	if ( a.lowerBound.z > b.lowerBound.z || b.upperBound.z > a.upperBound.z )
		return false;

	return true;
}

/// Get the surface area of an axis-aligned bounding box.
B3_INLINE float b3AABB_Area( b3AABB a )
{
	b3Vec3 delta = b3Sub( a.upperBound, a.lowerBound );
	return 2.0f * ( delta.x * delta.y + delta.y * delta.z + delta.z * delta.x );
}

/// Get the center of an axis-aligned bounding box.
B3_INLINE b3Vec3 b3AABB_Center( b3AABB a )
{
	return b3MulSV( 0.5f, b3Add( a.upperBound, a.lowerBound ) );
}

/// Get the extents (half-widths) of an axis-aligned bounding box.
B3_INLINE b3Vec3 b3AABB_Extents( b3AABB a )
{
	return b3MulSV( 0.5f, b3Sub( a.upperBound, a.lowerBound ) );
}

/// Get the union of two axis-aligned bounding boxes.
B3_INLINE b3AABB b3AABB_Union( b3AABB a, b3AABB b )
{
	b3AABB out;
	out.lowerBound = b3Min( a.lowerBound, b.lowerBound );
	out.upperBound = b3Max( a.upperBound, b.upperBound );
	return out;
}

/// Add uniform padding to an axis-aligned bounding box.
B3_INLINE b3AABB b3AABB_Inflate( b3AABB a, float extension )
{
	b3Vec3 radius = { extension, extension, extension };

	b3AABB out;
	out.lowerBound = b3Sub( a.lowerBound, radius );
	out.upperBound = b3Add( a.upperBound, radius );
	return out;
}

/// Do two axis-aligned boxes overlap?
B3_INLINE bool b3AABB_Overlaps( b3AABB a, b3AABB b )
{
	// No intersection if separated along one axis
	if ( a.upperBound.x < b.lowerBound.x || a.lowerBound.x > b.upperBound.x )
		return false;
	if ( a.upperBound.y < b.lowerBound.y || a.lowerBound.y > b.upperBound.y )
		return false;
	if ( a.upperBound.z < b.lowerBound.z || a.lowerBound.z > b.upperBound.z )
		return false;

	// Overlapping on all axis means bounds are intersecting
	return true;
}

/// Transform an axis-aligned bounding box. This can create a large box
/// than if you recomputed the AABB of the original shape with the transform
/// applied.
B3_INLINE b3AABB b3AABB_Transform( b3Transform transform, b3AABB a )
{
	b3Vec3 center = b3TransformPoint( transform, b3AABB_Center( a ) );
	b3Matrix3 m = b3MakeMatrixFromQuat( transform.q );
	b3Vec3 extent = b3MulMV( b3AbsMatrix3( m ), b3AABB_Extents( a ) );
	b3AABB out = { b3Sub( center, extent ), b3Add( center, extent ) };
	return out;
}

/// Get the closest point on an axis-aligned bounding box.
B3_INLINE b3Vec3 b3ClosestPointToAABB( b3Vec3 point, b3AABB a )
{
	return b3Clamp( point, a.lowerBound, a.upperBound );
}

/// Is this a valid number? Not NaN or infinity.
B3_API bool b3IsValidFloat( float a );

/// Is this a valid vector? Not NaN or infinity.
B3_API bool b3IsValidVec3( b3Vec3 a );

/// Is this a valid quaternion? Not NaN or infinity. Is normalized.
B3_API bool b3IsValidQuat( b3Quat q );

/// Is this a valid transform? Not NaN or infinity. Is normalized.
B3_API bool b3IsValidTransform( b3Transform a );

/// Is this a valid matrix? Not NaN or infinity.
B3_API bool b3IsValidMatrix3( b3Matrix3 a );

/// Is this a valid bounding box? Not Nan or infinity. Upper bound greater than or equal to lower bound.
B3_API bool b3IsValidAABB( b3AABB a );

/// Is this AABB valid and not huge or very far from the origin?
B3_API bool b3IsSaneAABB( b3AABB a );

/// Is this a valid plane? Normal is a unit vector. Not Nan or infinity.
B3_API bool b3IsValidPlane( b3Plane a );

/**@}*/ // math

/**
 * @defgroup math_cpp C++ Math
 * @brief Math operator overloads for C++
 * Some of the simpler ones are expanded to improve debug performance.
 * See math_functions.h for details.
 * @{
 */

#ifdef __cplusplus

/// Vector addition.
B3_FORCE_INLINE b3Vec3& operator+=( b3Vec3& a, b3Vec3 b )
{
	a.x += b.x;
	a.y += b.y;
	a.z += b.z;
	return a;
}

/// Vector subtraction.
B3_FORCE_INLINE b3Vec3& operator-=( b3Vec3& a, b3Vec3 b )
{
	a.x -= b.x;
	a.y -= b.y;
	a.z -= b.z;
	return a;
}

/// Vector scaling.
B3_FORCE_INLINE b3Vec3& operator*=( b3Vec3& a, float s )
{
	a.x *= s;
	a.y *= s;
	a.z *= s;
	return a;
}

/// Vector negation.
B3_FORCE_INLINE b3Vec3 operator-( b3Vec3 a )
{
	return { -a.x, -a.y, -a.z };
}

/// Vector scaling.
B3_FORCE_INLINE b3Vec3 operator*( float s, b3Vec3 a )
{
	return { s * a.x, s * a.y, s * a.z };
}

/// Vector scaling.
B3_FORCE_INLINE b3Vec3 operator*( b3Vec3 a, float s )
{
	return { s * a.x, s * a.y, s * a.z };
}

/// Component-wise vector multiplication.
B3_FORCE_INLINE b3Vec3 operator*( b3Vec3 a, b3Vec3 b )
{
	return { a.x * b.x, a.y * b.y, a.z * b.z };
}

/// Vector addition.
B3_FORCE_INLINE b3Vec3 operator+( b3Vec3 a, b3Vec3 b )
{
	return { a.x + b.x, a.y + b.y, a.z + b.z };
}

/// Vector subtraction.
B3_FORCE_INLINE b3Vec3 operator-( b3Vec3 a, b3Vec3 b )
{
	return { a.x - b.x, a.y - b.y, a.z - b.z };
}

#endif

/**@}*/ // math_cpp

// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "contact_solver.h"
#include "platform.h"
#include "simd.h"
#include "simd_wide.h"

// Wide vec2
typedef struct B3_WIDE( b3Vec2 )
{
	b3FloatW x, y;
} b3Vec2W;

// Wide quaternion
typedef struct B3_WIDE( b3Quat )
{
	b3Vec3W V;
	b3FloatW S;
} b3QuatW;

// Wide symmetric matrix2
typedef struct B3_WIDE( b3SymMatrix2 )
{
	b3FloatW cxx, cxy, cyy;
} b3SymMatrix2W;

// Wide symmetric matrix3
typedef struct B3_WIDE( b3SymMatrix3 )
{
	b3FloatW cxx, cxy, cxz, cyy, cyz, czz;
} b3SymMatrix3W;

typedef struct B3_WIDE( b3Matrix3 )
{
	b3Vec3W cx, cy, cz;
} b3Matrix3W;

// a + b
static inline b3Vec2W b3AddV2W( b3Vec2W a, b3Vec2W b )
{
	return (b3Vec2W){
		b3AddW( a.x, b.x ),
		b3AddW( a.y, b.y ),
	};
}

// m * a
static inline b3Vec2W b3MulMV2W( b3SymMatrix2W m, b3Vec2W a )
{
	b3Vec2W b = {
		b3AddW( b3MulW( m.cxx, a.x ), b3MulW( m.cxy, a.y ) ),
		b3AddW( b3MulW( m.cxy, a.x ), b3MulW( m.cyy, a.y ) ),
	};

	return b;
}

// m * a
static inline b3Vec3W b3MulMVW( b3SymMatrix3W m, b3Vec3W a )
{
	b3Vec3W b = {
		b3AddW( b3MulW( m.cxx, a.X ), b3AddW( b3MulW( m.cxy, a.Y ), b3MulW( m.cxz, a.Z ) ) ),
		b3AddW( b3MulW( m.cxy, a.X ), b3AddW( b3MulW( m.cyy, a.Y ), b3MulW( m.cyz, a.Z ) ) ),
		b3AddW( b3MulW( m.cxz, a.X ), b3AddW( b3MulW( m.cyz, a.Y ), b3MulW( m.czz, a.Z ) ) ),
	};

	return b;
}

// a - m * b
static inline b3Vec3W b3MulSubMVW( b3Vec3W a, b3SymMatrix3W m, b3Vec3W b )
{
	b3Vec3W c = {
		b3AddW( b3MulW( m.cxx, b.X ), b3AddW( b3MulW( m.cxy, b.Y ), b3MulW( m.cxz, b.Z ) ) ),
		b3AddW( b3MulW( m.cxy, b.X ), b3AddW( b3MulW( m.cyy, b.Y ), b3MulW( m.cyz, b.Z ) ) ),
		b3AddW( b3MulW( m.cxz, b.X ), b3AddW( b3MulW( m.cyz, b.Y ), b3MulW( m.czz, b.Z ) ) ),
	};

	return (b3Vec3W){ b3SubW( a.X, c.X ), b3SubW( a.Y, c.Y ), b3SubW( a.Z, c.Z ) };
}

// a + m * b
static inline b3Vec3W b3MulAddMVW( b3Vec3W a, b3SymMatrix3W m, b3Vec3W b )
{
	b3Vec3W c = {
		b3AddW( b3MulW( m.cxx, b.X ), b3AddW( b3MulW( m.cxy, b.Y ), b3MulW( m.cxz, b.Z ) ) ),
		b3AddW( b3MulW( m.cxy, b.X ), b3AddW( b3MulW( m.cyy, b.Y ), b3MulW( m.cyz, b.Z ) ) ),
		b3AddW( b3MulW( m.cxz, b.X ), b3AddW( b3MulW( m.cyz, b.Y ), b3MulW( m.czz, b.Z ) ) ),
	};

	return (b3Vec3W){ b3AddW( a.X, c.X ), b3AddW( a.Y, c.Y ), b3AddW( a.Z, c.Z ) };
}

static inline b3Vec3W b3MulM3VW( b3Matrix3W m, b3Vec3W a )
{
	b3Vec3W b = {
		b3AddW( b3MulW( m.cx.X, a.X ), b3AddW( b3MulW( m.cy.X, a.Y ), b3MulW( m.cz.X, a.Z ) ) ),
		b3AddW( b3MulW( m.cx.Y, a.X ), b3AddW( b3MulW( m.cy.Y, a.Y ), b3MulW( m.cz.Y, a.Z ) ) ),
		b3AddW( b3MulW( m.cx.Z, a.X ), b3AddW( b3MulW( m.cy.Z, a.Y ), b3MulW( m.cz.Z, a.Z ) ) ),
	};

	return b;
}

static inline b3Vec3W b3InvRotateVectorW( b3QuatW q, b3Vec3W a )
{
	b3Vec3W t = b3CrossW( q.V, a );
	t = (b3Vec3W){ b3AddW( t.X, t.X ), b3AddW( t.Y, t.Y ), b3AddW( t.Z, t.Z ) };
	b3Vec3W u = b3CrossW( q.V, t );
	return (b3Vec3W){
		b3AddW( b3SubW( a.X, b3MulW( q.S, t.X ) ), u.X ),
		b3AddW( b3SubW( a.Y, b3MulW( q.S, t.Y ) ), u.Y ),
		b3AddW( b3SubW( a.Z, b3MulW( q.S, t.Z ) ), u.Z ),
	};
}

// Soft contact constraints with sub-stepping support
// Uses fixed anchors for Jacobians for better behavior on rolling shapes (circles & capsules)
// http://mmacklin.com/smallsteps.pdf
// https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf

typedef struct B3_WIDE( b3ContactConstraintPoint )
{
	b3Vec3W anchorAs, anchorBs;
	b3FloatW baseSeparations;
	b3FloatW normalImpulses;
	b3FloatW totalNormalImpulses;
	b3FloatW normalMasses;
	b3FloatW leverArms;
	b3FloatW relativeVelocities;
	b3FloatW restitutionImpulses;
} b3ContactConstraintPointWide;

typedef struct B3_WIDE( b3ContactManifold )
{
	_Alignas( B3_WIDE_ALIGNMENT ) b3Vec3W normal;

	b3Vec3W tangent1;
	b3Vec3W tangent2;

	// Friction centers
	b3Vec3W centerA, centerB;
	b3FloatW twistMass;
	b3FloatW twistImpulse;
	b3SymMatrix2W tangentMass;
	b3Vec2W frictionImpulse;
	b3Vec3W rollingImpulse;
	b3FloatW rollingMask;
	b3FloatW tangentVelocity1;
	b3FloatW tangentVelocity2;

	int pointCount;

	b3ContactConstraintPointWide points[B3_MAX_MANIFOLD_POINTS];
} b3ContactManifoldWide;

// Solves one contact per lane
typedef struct B3_WIDE( b3ContactConstraint )
{
	// These are base 1
	_Alignas( B3_WIDE_ALIGNMENT ) int indexA[B3_SIMD_WIDTH];
	int indexB[B3_SIMD_WIDTH];

	b3Contact* contacts[B3_SIMD_WIDTH];

	b3FloatW invMassA, invMassB;
	b3SymMatrix3W invIA, invIB;
	b3SymMatrix3W rollingMass;
	b3FloatW friction;
	b3FloatW rollingResistance;
	b3FloatW restitution;

	int manifoldCount;
} b3ContactConstraintWide;

typedef struct B3_WIDE( b3ConvexConstraint )
{
	b3ContactConstraintWide base;
	b3ContactManifoldWide manifold;
} b3ConvexConstraintWide;

_Static_assert( offsetof( b3ConvexConstraintWide, manifold ) == sizeof( b3ContactConstraintWide ), "wide constraint layout" );
_Static_assert( sizeof( b3ContactConstraintWide ) % B3_WIDE_ALIGNMENT == 0, "wide constraint layout" );
_Static_assert( sizeof( b3ContactManifoldWide ) % B3_WIDE_ALIGNMENT == 0, "wide constraint layout" );

int B3_WIDE( b3GetWideContactConstraintByteCount )( void )
{
	return sizeof( b3ConvexConstraintWide );
}

int B3_WIDE( b3GetWideMeshConstraintByteCount )( void )
{
	return sizeof( b3ContactConstraintWide );
}

int B3_WIDE( b3GetWideMeshManifoldByteCount )( void )
{
	return sizeof( b3ContactManifoldWide );
}

// wide version of b3BodyState
typedef struct B3_WIDE( b3BodyState )
{
	b3Vec3W v;
	b3Vec3W w;
	b3Vec3W dp;
	b3QuatW dq;
} b3BodyStateW;

#if defined( B3_SIMD_NONE )

B3_FORCE_INLINE b3BodyStateW b3GatherBodies( const b3BodyState* B3_RESTRICT states, const int* B3_RESTRICT indices )
{
	b3BodyState identity = b3_identityBodyState;

	b3BodyState s1 = indices[0] == 0 ? identity : states[indices[0] - 1];
	b3BodyState s2 = indices[1] == 0 ? identity : states[indices[1] - 1];
	b3BodyState s3 = indices[2] == 0 ? identity : states[indices[2] - 1];
	b3BodyState s4 = indices[3] == 0 ? identity : states[indices[3] - 1];

	b3BodyStateW simdBody;
	simdBody.v.X = (b3FloatW){ s1.linearVelocity.x, s2.linearVelocity.x, s3.linearVelocity.x, s4.linearVelocity.x };
	simdBody.v.Y = (b3FloatW){ s1.linearVelocity.y, s2.linearVelocity.y, s3.linearVelocity.y, s4.linearVelocity.y };
	simdBody.v.Z = (b3FloatW){ s1.linearVelocity.z, s2.linearVelocity.z, s3.linearVelocity.z, s4.linearVelocity.z };
	simdBody.w.X = (b3FloatW){ s1.angularVelocity.x, s2.angularVelocity.x, s3.angularVelocity.x, s4.angularVelocity.x };
	simdBody.w.Y = (b3FloatW){ s1.angularVelocity.y, s2.angularVelocity.y, s3.angularVelocity.y, s4.angularVelocity.y };
	simdBody.w.Z = (b3FloatW){ s1.angularVelocity.z, s2.angularVelocity.z, s3.angularVelocity.z, s4.angularVelocity.z };
	simdBody.dp.X = (b3FloatW){ s1.deltaPosition.x, s2.deltaPosition.x, s3.deltaPosition.x, s4.deltaPosition.x };
	simdBody.dp.Y = (b3FloatW){ s1.deltaPosition.y, s2.deltaPosition.y, s3.deltaPosition.y, s4.deltaPosition.y };
	simdBody.dp.Z = (b3FloatW){ s1.deltaPosition.z, s2.deltaPosition.z, s3.deltaPosition.z, s4.deltaPosition.z };
	simdBody.dq.V.X = (b3FloatW){ s1.deltaRotation.v.x, s2.deltaRotation.v.x, s3.deltaRotation.v.x, s4.deltaRotation.v.x };
	simdBody.dq.V.Y = (b3FloatW){ s1.deltaRotation.v.y, s2.deltaRotation.v.y, s3.deltaRotation.v.y, s4.deltaRotation.v.y };
	simdBody.dq.V.Z = (b3FloatW){ s1.deltaRotation.v.z, s2.deltaRotation.v.z, s3.deltaRotation.v.z, s4.deltaRotation.v.z };
	simdBody.dq.S = (b3FloatW){ s1.deltaRotation.s, s2.deltaRotation.s, s3.deltaRotation.s, s4.deltaRotation.s };

	return simdBody;
}

// This writes only the velocities back to the solver bodies
B3_FORCE_INLINE void b3ScatterBodies( b3BodyState* B3_RESTRICT states, const int* B3_RESTRICT indices,
									  const b3BodyStateW* B3_RESTRICT simdBody )
{
	int index1 = indices[0] - 1;
	if ( index1 != -1 && ( states[index1].flags & b3_dynamicFlag ) != 0 )
	{
		b3BodyState* state = states + index1;
		state->linearVelocity.x = simdBody->v.X.x;
		state->linearVelocity.y = simdBody->v.Y.x;
		state->linearVelocity.z = simdBody->v.Z.x;
		state->angularVelocity.x = simdBody->w.X.x;
		state->angularVelocity.y = simdBody->w.Y.x;
		state->angularVelocity.z = simdBody->w.Z.x;
	}

	int index2 = indices[1] - 1;
	if ( index2 != -1 && ( states[index2].flags & b3_dynamicFlag ) != 0 )
	{
		b3BodyState* state = states + index2;
		state->linearVelocity.x = simdBody->v.X.y;
		state->linearVelocity.y = simdBody->v.Y.y;
		state->linearVelocity.z = simdBody->v.Z.y;
		state->angularVelocity.x = simdBody->w.X.y;
		state->angularVelocity.y = simdBody->w.Y.y;
		state->angularVelocity.z = simdBody->w.Z.y;
	}

	int index3 = indices[2] - 1;
	if ( index3 != -1 && ( states[index3].flags & b3_dynamicFlag ) != 0 )
	{
		b3BodyState* state = states + index3;
		state->linearVelocity.x = simdBody->v.X.z;
		state->linearVelocity.y = simdBody->v.Y.z;
		state->linearVelocity.z = simdBody->v.Z.z;
		state->angularVelocity.x = simdBody->w.X.z;
		state->angularVelocity.y = simdBody->w.Y.z;
		state->angularVelocity.z = simdBody->w.Z.z;
	}

	int index4 = indices[3] - 1;
	if ( index4 != -1 && ( states[index4].flags & b3_dynamicFlag ) != 0 )
	{
		b3BodyState* state = states + index4;
		state->linearVelocity.x = simdBody->v.X.w;
		state->linearVelocity.y = simdBody->v.Y.w;
		state->linearVelocity.z = simdBody->v.Z.w;
		state->angularVelocity.x = simdBody->w.X.w;
		state->angularVelocity.y = simdBody->w.Y.w;
		state->angularVelocity.z = simdBody->w.Z.w;
	}
}

#else

_Static_assert( sizeof( b3BodyState ) == 64, "body state layout" );
_Static_assert( offsetof( b3BodyState, linearVelocity ) == 0 && offsetof( b3BodyState, angularVelocity ) == 16 &&
					offsetof( b3BodyState, deltaPosition ) == 32 && offsetof( b3BodyState, deltaRotation ) == 48,
				"body state layout" );

#if B3_SIMD_WIDTH == 8

B3_FORCE_INLINE b3BodyStateW b3GatherBodies( const b3BodyState* B3_RESTRICT states, const int* B3_RESTRICT indices )
{
	const float* identity = (const float*)&b3_identityBodyState;

	// Indices are 0 for null
	const float* p1 = indices[0] == 0 ? identity : (const float*)( states + indices[0] - 1 );
	const float* p2 = indices[1] == 0 ? identity : (const float*)( states + indices[1] - 1 );
	const float* p3 = indices[2] == 0 ? identity : (const float*)( states + indices[2] - 1 );
	const float* p4 = indices[3] == 0 ? identity : (const float*)( states + indices[3] - 1 );
	const float* p5 = indices[4] == 0 ? identity : (const float*)( states + indices[4] - 1 );
	const float* p6 = indices[5] == 0 ? identity : (const float*)( states + indices[5] - 1 );
	const float* p7 = indices[6] == 0 ? identity : (const float*)( states + indices[6] - 1 );
	const float* p8 = indices[7] == 0 ? identity : (const float*)( states + indices[7] - 1 );

	b3BodyStateW s;
	b3FloatW pad;

	b3TransposeW( b3LoadW( p1 ), b3LoadW( p2 ), b3LoadW( p3 ), b3LoadW( p4 ), b3LoadW( p5 ), b3LoadW( p6 ), b3LoadW( p7 ),
				  b3LoadW( p8 ), &s.v.X, &s.v.Y, &s.v.Z, &pad, &s.w.X, &s.w.Y, &s.w.Z, &pad );
	b3TransposeW( b3LoadW( p1 + 8 ), b3LoadW( p2 + 8 ), b3LoadW( p3 + 8 ), b3LoadW( p4 + 8 ), b3LoadW( p5 + 8 ),
				  b3LoadW( p6 + 8 ), b3LoadW( p7 + 8 ), b3LoadW( p8 + 8 ), &s.dp.X, &s.dp.Y, &s.dp.Z, &pad, &s.dq.V.X, &s.dq.V.Y,
				  &s.dq.V.Z, &s.dq.S );

	return s;
}

B3_FORCE_INLINE void b3StoreBodyVelocity( b3BodyState* B3_RESTRICT states, int index, b3FloatW vw )
{
	// Indices are 0 for null
	if ( index == 0 )
	{
		return;
	}

	b3BodyState* state = states + index - 1;
	uint32_t flags = state->flags;
	if ( ( flags & b3_dynamicFlag ) == 0 )
	{
		return;
	}

	b3StoreW( (float*)state, vw );
}

// This writes only the velocities back to the solver bodies
B3_FORCE_INLINE void b3ScatterBodies( b3BodyState* B3_RESTRICT states, const int* B3_RESTRICT indices,
									  const b3BodyStateW* B3_RESTRICT simdBody )
{
	// I don't use any dummy body in the body array because this will lead to multithreaded sharing and the
	// associated cache flushing.
	b3FloatW zero = b3ZeroW();
	b3FloatW vw1, vw2, vw3, vw4, vw5, vw6, vw7, vw8;
	b3TransposeW( simdBody->v.X, simdBody->v.Y, simdBody->v.Z, zero, simdBody->w.X, simdBody->w.Y, simdBody->w.Z, zero, &vw1,
				  &vw2, &vw3, &vw4, &vw5, &vw6, &vw7, &vw8 );

	b3StoreBodyVelocity( states, indices[0], vw1 );
	b3StoreBodyVelocity( states, indices[1], vw2 );
	b3StoreBodyVelocity( states, indices[2], vw3 );
	b3StoreBodyVelocity( states, indices[3], vw4 );
	b3StoreBodyVelocity( states, indices[4], vw5 );
	b3StoreBodyVelocity( states, indices[5], vw6 );
	b3StoreBodyVelocity( states, indices[6], vw7 );
	b3StoreBodyVelocity( states, indices[7], vw8 );
}

#else

B3_FORCE_INLINE b3BodyStateW b3GatherBodies( const b3BodyState* B3_RESTRICT states, const int* B3_RESTRICT indices )
{
	const float* identity = (const float*)&b3_identityBodyState;

	// Indices are 0 for null
	const float* p1 = indices[0] == 0 ? identity : (const float*)( states + indices[0] - 1 );
	const float* p2 = indices[1] == 0 ? identity : (const float*)( states + indices[1] - 1 );
	const float* p3 = indices[2] == 0 ? identity : (const float*)( states + indices[2] - 1 );
	const float* p4 = indices[3] == 0 ? identity : (const float*)( states + indices[3] - 1 );

	b3BodyStateW s;
	b3FloatW pad;
	b3TransposeW( b3LoadW( p1 ), b3LoadW( p2 ), b3LoadW( p3 ), b3LoadW( p4 ), &s.v.X, &s.v.Y, &s.v.Z, &pad );
	b3TransposeW( b3LoadW( p1 + 4 ), b3LoadW( p2 + 4 ), b3LoadW( p3 + 4 ), b3LoadW( p4 + 4 ), &s.w.X, &s.w.Y, &s.w.Z, &pad );
	b3TransposeW( b3LoadW( p1 + 8 ), b3LoadW( p2 + 8 ), b3LoadW( p3 + 8 ), b3LoadW( p4 + 8 ), &s.dp.X, &s.dp.Y, &s.dp.Z, &pad );
	b3TransposeW( b3LoadW( p1 + 12 ), b3LoadW( p2 + 12 ), b3LoadW( p3 + 12 ), b3LoadW( p4 + 12 ), &s.dq.V.X, &s.dq.V.Y, &s.dq.V.Z,
				  &s.dq.S );
	return s;
}

B3_FORCE_INLINE void b3StoreBodyVelocity( b3BodyState* B3_RESTRICT states, int index, b3FloatW v, b3FloatW w )
{
	// Indices are 0 for null
	if ( index == 0 )
	{
		return;
	}

	b3BodyState* state = states + index - 1;
	uint32_t flags = state->flags;
	if ( ( flags & b3_dynamicFlag ) == 0 )
	{
		return;
	}

	b3StoreW( (float*)state, v );
	b3StoreW( (float*)state + 4, w );
}

// This writes only the velocities back to the solver bodies
B3_FORCE_INLINE void b3ScatterBodies( b3BodyState* B3_RESTRICT states, const int* B3_RESTRICT indices,
									  const b3BodyStateW* B3_RESTRICT simdBody )
{
	// I don't use any dummy body in the body array because this will lead to multithreaded sharing and the
	// associated cache flushing.
	b3FloatW zero = b3ZeroW();
	b3FloatW v1, v2, v3, v4;
	b3TransposeW( simdBody->v.X, simdBody->v.Y, simdBody->v.Z, zero, &v1, &v2, &v3, &v4 );
	b3FloatW w1, w2, w3, w4;
	b3TransposeW( simdBody->w.X, simdBody->w.Y, simdBody->w.Z, zero, &w1, &w2, &w3, &w4 );

	b3StoreBodyVelocity( states, indices[0], v1, w1 );
	b3StoreBodyVelocity( states, indices[1], v2, w2 );
	b3StoreBodyVelocity( states, indices[2], v3, w3 );
	b3StoreBodyVelocity( states, indices[3], v4, w4 );
}

#endif

#endif

_Static_assert( offsetof( b3ManifoldPoint, anchorA ) == 0, "manifold point layout" );
_Static_assert( offsetof( b3ManifoldPoint, anchorB ) == 12, "manifold point layout" );
_Static_assert( offsetof( b3ManifoldPoint, separation ) == 24, "manifold point layout" );
_Static_assert( offsetof( b3ManifoldPoint, normalImpulse ) == 28, "manifold point layout" );
_Static_assert( offsetof( b3Manifold, twistImpulse ) == offsetof( b3Manifold, normal ) + 12, "manifold layout" );
_Static_assert( offsetof( b3Manifold, frictionImpulse ) == offsetof( b3Manifold, normal ) + 16, "manifold layout" );
_Static_assert( offsetof( b3Manifold, rollingImpulse ) == offsetof( b3Manifold, normal ) + 28, "manifold layout" );
_Static_assert( offsetof( b3Manifold, pointCount ) == offsetof( b3Manifold, normal ) + 40, "manifold layout" );
_Static_assert( offsetof( b3Matrix3, cy ) == 12 && offsetof( b3Matrix3, cz ) == 24 && sizeof( b3Matrix3 ) == 36,
				"matrix layout" );

static const b3Contact b3_zeroContact = { 0 };
static const b3Manifold b3_zeroManifold = { 0 };
static const b3BodySim b3_zeroBodySim = { 0 };

#if B3_SIMD_WIDTH == 8
#define B3_GATHER_LANES( wide, lanes, field )                                                                                    \
	wide = b3SetW( lanes[0]->field, lanes[1]->field, lanes[2]->field, lanes[3]->field, lanes[4]->field, lanes[5]->field,         \
				   lanes[6]->field, lanes[7]->field )
#else
#define B3_GATHER_LANES( wide, lanes, field ) wide = b3SetW( lanes[0]->field, lanes[1]->field, lanes[2]->field, lanes[3]->field )
#endif

static inline b3SymMatrix3W b3GatherInvInertiaW( const b3BodySim* simLanes[B3_SIMD_WIDTH] )
{
	const float* i0 = &simLanes[0]->invInertiaWorld.cx.x;
	const float* i1 = &simLanes[1]->invInertiaWorld.cx.x;
	const float* i2 = &simLanes[2]->invInertiaWorld.cx.x;
	const float* i3 = &simLanes[3]->invInertiaWorld.cx.x;
#if B3_SIMD_WIDTH == 8
	const float* i4 = &simLanes[4]->invInertiaWorld.cx.x;
	const float* i5 = &simLanes[5]->invInertiaWorld.cx.x;
	const float* i6 = &simLanes[6]->invInertiaWorld.cx.x;
	const float* i7 = &simLanes[7]->invInertiaWorld.cx.x;
#endif

	b3SymMatrix3W m;
	b3FloatW unused;
#if B3_SIMD_WIDTH == 8
	b3TransposeW( b3LoadW( i0 ), b3LoadW( i1 ), b3LoadW( i2 ), b3LoadW( i3 ), b3LoadW( i4 ), b3LoadW( i5 ), b3LoadW( i6 ),
				  b3LoadW( i7 ), &m.cxx, &m.cxy, &m.cxz, &unused, &m.cyy, &m.cyz, &unused, &unused );
	m.czz = b3SetW( i0[8], i1[8], i2[8], i3[8], i4[8], i5[8], i6[8], i7[8] );
#else
	b3TransposeW( b3LoadW( i0 ), b3LoadW( i1 ), b3LoadW( i2 ), b3LoadW( i3 ), &m.cxx, &m.cxy, &m.cxz, &unused );
	b3TransposeW( b3LoadW( i0 + 4 ), b3LoadW( i1 + 4 ), b3LoadW( i2 + 4 ), b3LoadW( i3 + 4 ), &m.cyy, &m.cyz, &unused, &unused );
	m.czz = b3SetW( i0[8], i1[8], i2[8], i3[8] );
#endif
	return m;
}

static inline b3SymMatrix3W b3AddSymW( b3SymMatrix3W a, b3SymMatrix3W b )
{
	return (b3SymMatrix3W){
		b3AddW( a.cxx, b.cxx ), b3AddW( a.cxy, b.cxy ), b3AddW( a.cxz, b.cxz ),
		b3AddW( a.cyy, b.cyy ), b3AddW( a.cyz, b.cyz ), b3AddW( a.czz, b.czz ),
	};
}

static inline b3SymMatrix3W b3InvertSymW( b3SymMatrix3W m )
{
	b3FloatW cxx = b3SubW( b3MulW( m.cyy, m.czz ), b3MulW( m.cyz, m.cyz ) );
	b3FloatW cxy = b3SubW( b3MulW( m.cxz, m.cyz ), b3MulW( m.cxy, m.czz ) );
	b3FloatW cxz = b3SubW( b3MulW( m.cxy, m.cyz ), b3MulW( m.cxz, m.cyy ) );
	b3FloatW cyy = b3SubW( b3MulW( m.cxx, m.czz ), b3MulW( m.cxz, m.cxz ) );
	b3FloatW cyz = b3SubW( b3MulW( m.cxy, m.cxz ), b3MulW( m.cxx, m.cyz ) );
	b3FloatW czz = b3SubW( b3MulW( m.cxx, m.cyy ), b3MulW( m.cxy, m.cxy ) );

	b3FloatW det = b3AddW( b3MulW( m.cxx, cxx ), b3AddW( b3MulW( m.cxy, cxy ), b3MulW( m.cxz, cxz ) ) );
	b3FloatW valid = b3GreaterThanW( b3AbsW( det ), b3SplatW( 1000.0f * FLT_MIN ) );
	b3FloatW invDet = b3BlendW( b3ZeroW(), b3DivW( b3SplatW( 1.0f ), det ), valid );

	return (b3SymMatrix3W){
		b3MulW( invDet, cxx ), b3MulW( invDet, cxy ), b3MulW( invDet, cxz ),
		b3MulW( invDet, cyy ), b3MulW( invDet, cyz ), b3MulW( invDet, czz ),
	};
}

static inline b3Vec3W b3PerpW( b3Vec3W a )
{
	b3FloatW zero = b3ZeroW();
	b3FloatW half = b3SplatW( 0.5f );
	b3FloatW mask = b3OrW( b3LessThanW( a.X, b3NegW( half ) ), b3GreaterThanW( a.X, half ) );

	b3Vec3W p;
	p.X = b3BlendW( zero, a.Y, mask );
	p.Y = b3BlendW( a.Z, b3NegW( a.X ), mask );
	p.Z = b3BlendW( b3NegW( a.Y ), zero, mask );

	b3FloatW lengthSquared = b3DotW( p, p );
	b3FloatW valid = b3GreaterThanW( lengthSquared, b3SplatW( 1000.0f * FLT_MIN ) );
	b3FloatW s = b3BlendW( zero, b3DivW( b3SplatW( 1.0f ), b3SqrtW( lengthSquared ) ), valid );
	return b3MulSVW( s, p );
}
static inline b3FloatW b3IntsToFloatW( const int* v )
{
#if B3_SIMD_WIDTH == 8
	return b3SetW( (float)v[0], (float)v[1], (float)v[2], (float)v[3], (float)v[4], (float)v[5], (float)v[6], (float)v[7] );
#else
	return b3SetW( (float)v[0], (float)v[1], (float)v[2], (float)v[3] );
#endif
}

// Get the contiguous manifold array that trails the constraint.
static inline b3ContactManifoldWide* b3GetManifoldsW( b3ContactConstraintWide* c )
{
	return (b3ContactManifoldWide*)( c + 1 );
}

// fixedManifoldCount is 1 to indicate a convex contact with one manifold.
B3_FORCE_INLINE b3ContactConstraintWide* b3GetConstraintW( void* base, const int* manifoldStarts, int index,
														   int fixedManifoldCount )
{
	if ( fixedManifoldCount > 0 )
	{
		size_t stride = sizeof( b3ContactConstraintWide ) + fixedManifoldCount * sizeof( b3ContactManifoldWide );
		return (b3ContactConstraintWide*)( (uint8_t*)base + index * stride );
	}

	size_t offset = index * sizeof( b3ContactConstraintWide ) +
					( manifoldStarts[index] - manifoldStarts[0] ) * sizeof( b3ContactManifoldWide );
	return (b3ContactConstraintWide*)( (uint8_t*)base + offset );
}

B3_FORCE_INLINE b3ContactConstraintWide* b3NextConstraintW( b3ContactConstraintWide* c, int manifoldCount )
{
	return (b3ContactConstraintWide*)( (uint8_t*)c + sizeof( b3ContactConstraintWide ) +
									   manifoldCount * sizeof( b3ContactManifoldWide ) );
}

// Contact constraint data is stored in a heterogious stream. Constraints followed by an array of constraint manifolds.
// [c1 m11 m12 c2 m21 c3 m31 m32 m33 ... ]
B3_FORCE_INLINE bool b3PrepareConstraintW( b3ContactConstraintWide* c, b3Contact* const* contacts, int manifoldCount,
										   b3StepContext* context, b3FloatW warmStartScale )
{
	b3BodySim* sims = context->sims;
	b3BodyState* states = context->states;
#if B3_ENABLE_VALIDATION
	b3Body* bodies = context->world->bodies.data;
#endif

	b3FloatW zeroW = b3ZeroW();
	b3FloatW oneW = b3SplatW( 1.0f );
	b3FloatW twoW = b3SplatW( 2.0f );
	b3FloatW invTau = b3SplatW( 1.0f / B3_SPECULATIVE_DISTANCE );
	b3FloatW minFrictionWeight = b3SplatW( B3_MIN_FRICTION_WEIGHT );
	b3FloatW minDet = b3SplatW( 1000.0f * FLT_MIN );

	const b3Contact* contactLanes[B3_SIMD_WIDTH];
	const b3BodySim* simLanesA[B3_SIMD_WIDTH];
	const b3BodySim* simLanesB[B3_SIMD_WIDTH];
	int manifoldCounts[B3_SIMD_WIDTH];
	int hitEventLanes = 0;
	int maxManifoldCount = 0;

	for ( int lane = 0; lane < B3_SIMD_WIDTH; ++lane )
	{
		b3Contact* contact = contacts[lane];
		c->contacts[lane] = contact;

		if ( contact != NULL )
		{
			int indexA = b3DecodeAwakeIndex( contact->encodedBodySimA );
			int indexB = b3DecodeAwakeIndex( contact->encodedBodySimB );

#if B3_ENABLE_VALIDATION
			b3Body* bodyA = bodies + contact->edges[0].bodyId;
			b3Body* bodyB = bodies + contact->edges[1].bodyId;
			B3_ASSERT( contact->encodedBodySimA == b3EncodeBodySimIndex( bodyA ) );
			B3_ASSERT( contact->encodedBodySimB == b3EncodeBodySimIndex( bodyB ) );
#endif

			c->indexA[lane] = indexA + 1;
			c->indexB[lane] = indexB + 1;
			manifoldCounts[lane] = contact->manifoldCount;

			contactLanes[lane] = contact;
			simLanesA[lane] = indexA == B3_NULL_INDEX ? &b3_zeroBodySim : sims + indexA;
			simLanesB[lane] = indexB == B3_NULL_INDEX ? &b3_zeroBodySim : sims + indexB;
			hitEventLanes |= ( contact->flags & b3_simEnableHitEvent ) != 0 ? 1 << lane : 0;
			maxManifoldCount = b3MaxInt( maxManifoldCount, contact->manifoldCount );
		}
		else
		{
			c->indexA[lane] = 0;
			c->indexB[lane] = 0;
			manifoldCounts[lane] = 0;

			contactLanes[lane] = &b3_zeroContact;
			simLanesA[lane] = &b3_zeroBodySim;
			simLanesB[lane] = &b3_zeroBodySim;
		}
	}

	B3_VALIDATE( maxManifoldCount == manifoldCount );
	B3_UNUSED( maxManifoldCount );
	c->manifoldCount = manifoldCount;

	b3FloatW mA, mB;
	B3_GATHER_LANES( mA, simLanesA, invMass );
	B3_GATHER_LANES( mB, simLanesB, invMass );
	b3SymMatrix3W iA = b3GatherInvInertiaW( simLanesA );
	b3SymMatrix3W iB = b3GatherInvInertiaW( simLanesB );
	c->invMassA = mA;
	c->invMassB = mB;
	c->invIA = iA;
	c->invIB = iB;

	b3Vec3W tangentVelocity;
	B3_GATHER_LANES( c->friction, contactLanes, friction );
	B3_GATHER_LANES( c->rollingResistance, contactLanes, rollingResistance );
	B3_GATHER_LANES( c->restitution, contactLanes, restitution );
	B3_GATHER_LANES( tangentVelocity.X, contactLanes, tangentVelocity.x );
	B3_GATHER_LANES( tangentVelocity.Y, contactLanes, tangentVelocity.y );
	B3_GATHER_LANES( tangentVelocity.Z, contactLanes, tangentVelocity.z );

	b3SymMatrix3W invIAB = b3AddSymW( iA, iB );
	if ( b3AllZeroW( c->rollingResistance ) == false )
	{
		c->rollingMass = b3InvertSymW( invIAB );
	}
	else
	{
		c->rollingMass = (b3SymMatrix3W){ zeroW, zeroW, zeroW, zeroW, zeroW, zeroW };
	}

	b3FloatW rollingResistanceMask = b3GreaterThanW( c->rollingResistance, zeroW );

	b3ContactManifoldWide* manifolds = b3GetManifoldsW( c );
	for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
	{
		b3ContactManifoldWide* cm = manifolds + manifoldIndex;

		const b3Manifold* manifoldLanes[B3_SIMD_WIDTH];
		int activeLanes[B3_SIMD_WIDTH];
		int pointCounts[B3_SIMD_WIDTH];
		int pointCount = 0;
		for ( int lane = 0; lane < B3_SIMD_WIDTH; ++lane )
		{
			if ( manifoldIndex < manifoldCounts[lane] )
			{
				const b3Manifold* manifold = contactLanes[lane]->manifolds + manifoldIndex;
				manifoldLanes[lane] = manifold;
				activeLanes[lane] = 1;
				pointCounts[lane] = manifold->pointCount;
			}
			else
			{
				manifoldLanes[lane] = &b3_zeroManifold;
				activeLanes[lane] = 0;
				pointCounts[lane] = 0;
			}

			pointCount = b3MaxInt( pointCount, pointCounts[lane] );
		}

		B3_VALIDATE( 0 <= pointCount && pointCount <= B3_MAX_MANIFOLD_POINTS );
		cm->pointCount = pointCount;
		cm->rollingMask = b3AndW( rollingResistanceMask, b3GreaterThanW( b3IntsToFloatW( activeLanes ), zeroW ) );

		b3Vec3W normal, frictionImpulse, rollingImpulse;
		b3FloatW twistImpulse;
		{
			const float* m0 = &manifoldLanes[0]->normal.x;
			const float* m1 = &manifoldLanes[1]->normal.x;
			const float* m2 = &manifoldLanes[2]->normal.x;
			const float* m3 = &manifoldLanes[3]->normal.x;
#if B3_SIMD_WIDTH == 8
			const float* m4 = &manifoldLanes[4]->normal.x;
			const float* m5 = &manifoldLanes[5]->normal.x;
			const float* m6 = &manifoldLanes[6]->normal.x;
			const float* m7 = &manifoldLanes[7]->normal.x;
			b3TransposeW( b3LoadW( m0 ), b3LoadW( m1 ), b3LoadW( m2 ), b3LoadW( m3 ), b3LoadW( m4 ), b3LoadW( m5 ), b3LoadW( m6 ),
						  b3LoadW( m7 ), &normal.X, &normal.Y, &normal.Z, &twistImpulse, &frictionImpulse.X, &frictionImpulse.Y,
						  &frictionImpulse.Z, &rollingImpulse.X );
			rollingImpulse.Y = b3SetW( m0[8], m1[8], m2[8], m3[8], m4[8], m5[8], m6[8], m7[8] );
			rollingImpulse.Z = b3SetW( m0[9], m1[9], m2[9], m3[9], m4[9], m5[9], m6[9], m7[9] );
#else
			b3TransposeW( b3LoadW( m0 ), b3LoadW( m1 ), b3LoadW( m2 ), b3LoadW( m3 ), &normal.X, &normal.Y, &normal.Z,
						  &twistImpulse );
			b3TransposeW( b3LoadW( m0 + 4 ), b3LoadW( m1 + 4 ), b3LoadW( m2 + 4 ), b3LoadW( m3 + 4 ), &frictionImpulse.X,
						  &frictionImpulse.Y, &frictionImpulse.Z, &rollingImpulse.X );
			rollingImpulse.Y = b3SetW( m0[8], m1[8], m2[8], m3[8] );
			rollingImpulse.Z = b3SetW( m0[9], m1[9], m2[9], m3[9] );
#endif
		}

		b3Vec3W tangent1 = b3PerpW( normal );
		b3Vec3W tangent2 = b3CrossW( tangent1, normal );
		cm->normal = normal;
		cm->tangent1 = tangent1;
		cm->tangent2 = tangent2;
		cm->tangentVelocity1 = b3DotW( tangentVelocity, tangent1 );
		cm->tangentVelocity2 = b3DotW( tangentVelocity, tangent2 );

		cm->twistImpulse = b3MulW( warmStartScale, twistImpulse );
		cm->rollingImpulse.X = b3BlendW( zeroW, b3MulW( warmStartScale, rollingImpulse.X ), cm->rollingMask );
		cm->rollingImpulse.Y = b3BlendW( zeroW, b3MulW( warmStartScale, rollingImpulse.Y ), cm->rollingMask );
		cm->rollingImpulse.Z = b3BlendW( zeroW, b3MulW( warmStartScale, rollingImpulse.Z ), cm->rollingMask );
		cm->frictionImpulse.x = b3MulW( warmStartScale, b3DotW( frictionImpulse, tangent1 ) );
		cm->frictionImpulse.y = b3MulW( warmStartScale, b3DotW( frictionImpulse, tangent2 ) );

		b3FloatW pointCountW = b3IntsToFloatW( pointCounts );

		b3Vec3W centerA = { zeroW, zeroW, zeroW };
		b3Vec3W centerB = { zeroW, zeroW, zeroW };
		b3FloatW totalFrictionWeight = zeroW;

		for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
		{
			b3ContactConstraintPointWide* cp = cm->points + pointIndex;
			b3FloatW pointMask = b3GreaterThanW( pointCountW, b3SplatW( (float)pointIndex ) );

			const float* p0 = (const float*)( manifoldLanes[0]->points + pointIndex );
			const float* p1 = (const float*)( manifoldLanes[1]->points + pointIndex );
			const float* p2 = (const float*)( manifoldLanes[2]->points + pointIndex );
			const float* p3 = (const float*)( manifoldLanes[3]->points + pointIndex );
#if B3_SIMD_WIDTH == 8
			const float* p4 = (const float*)( manifoldLanes[4]->points + pointIndex );
			const float* p5 = (const float*)( manifoldLanes[5]->points + pointIndex );
			const float* p6 = (const float*)( manifoldLanes[6]->points + pointIndex );
			const float* p7 = (const float*)( manifoldLanes[7]->points + pointIndex );
#endif

			b3Vec3W rA, rB;
			b3FloatW separation, normalImpulse;
#if B3_SIMD_WIDTH == 8
			b3TransposeW( b3LoadW( p0 ), b3LoadW( p1 ), b3LoadW( p2 ), b3LoadW( p3 ), b3LoadW( p4 ), b3LoadW( p5 ), b3LoadW( p6 ),
						  b3LoadW( p7 ), &rA.X, &rA.Y, &rA.Z, &rB.X, &rB.Y, &rB.Z, &separation, &normalImpulse );
#else
			b3TransposeW( b3LoadW( p0 ), b3LoadW( p1 ), b3LoadW( p2 ), b3LoadW( p3 ), &rA.X, &rA.Y, &rA.Z, &rB.X );
			b3TransposeW( b3LoadW( p0 + 4 ), b3LoadW( p1 + 4 ), b3LoadW( p2 + 4 ), b3LoadW( p3 + 4 ), &rB.Y, &rB.Z, &separation,
						  &normalImpulse );
#endif

			rA.X = b3BlendW( zeroW, rA.X, pointMask );
			rA.Y = b3BlendW( zeroW, rA.Y, pointMask );
			rA.Z = b3BlendW( zeroW, rA.Z, pointMask );
			rB.X = b3BlendW( zeroW, rB.X, pointMask );
			rB.Y = b3BlendW( zeroW, rB.Y, pointMask );
			rB.Z = b3BlendW( zeroW, rB.Z, pointMask );
			separation = b3BlendW( zeroW, separation, pointMask );
			normalImpulse = b3BlendW( zeroW, normalImpulse, pointMask );

			// C0 friction center decay. Needed to prevent spinning top drift (GyroscopicPrecession sample).
			// See details in b3PrepareContacts_Mesh. This code should stay in sync.
			b3FloatW weight = b3MinW( b3MaxW( b3SubW( twoW, b3MulW( separation, invTau ) ), minFrictionWeight ), oneW );
			weight = b3BlendW( zeroW, weight, pointMask );
			centerA = b3MulAddSVW( centerA, weight, rA );
			centerB = b3MulAddSVW( centerB, weight, rB );
			totalFrictionWeight = b3AddW( totalFrictionWeight, weight );

			cp->anchorAs = rA;
			cp->anchorBs = rB;
			cp->baseSeparations = b3SubW( separation, b3DotW( b3SubVW( rB, rA ), normal ) );
			cp->normalImpulses = b3MulW( warmStartScale, normalImpulse );
			cp->totalNormalImpulses = zeroW;
			cp->relativeVelocities = zeroW;
			cp->restitutionImpulses = zeroW;

			b3Vec3W rnA = b3CrossW( rA, normal );
			b3Vec3W rnB = b3CrossW( rB, normal );
			b3FloatW kNormal = b3AddW( mA, mB );
			kNormal = b3AddW( kNormal, b3DotW( rnA, b3MulMVW( iA, rnA ) ) );
			kNormal = b3AddW( kNormal, b3DotW( rnB, b3MulMVW( iB, rnB ) ) );
			b3FloatW valid = b3AndW( b3GreaterThanW( kNormal, zeroW ), pointMask );
			cp->normalMasses = b3BlendW( zeroW, b3DivW( oneW, kNormal ), valid );
		}

		b3FloatW frictionValid = b3GreaterThanW( totalFrictionWeight, zeroW );
		b3FloatW invWeight = b3BlendW( zeroW, b3DivW( oneW, totalFrictionWeight ), frictionValid );
		centerA = b3MulSVW( invWeight, centerA );
		centerB = b3MulSVW( invWeight, centerB );
		cm->centerA = centerA;
		cm->centerB = centerB;

		for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
		{
			b3ContactConstraintPointWide* cp = cm->points + pointIndex;
			b3FloatW pointMask = b3GreaterThanW( pointCountW, b3SplatW( (float)pointIndex ) );
			b3Vec3W d = b3SubVW( cp->anchorAs, centerA );
			cp->leverArms = b3BlendW( zeroW, b3SqrtW( b3DotW( d, d ) ), pointMask );
		}

		{
			b3Vec3W rtA1 = b3CrossW( centerA, tangent1 );
			b3Vec3W rtA2 = b3CrossW( centerA, tangent2 );
			b3Vec3W rtB1 = b3CrossW( centerB, tangent1 );
			b3Vec3W rtB2 = b3CrossW( centerB, tangent2 );
			b3Vec3W iArtA1 = b3MulMVW( iA, rtA1 );
			b3Vec3W iArtA2 = b3MulMVW( iA, rtA2 );
			b3Vec3W iBrtB1 = b3MulMVW( iB, rtB1 );
			b3Vec3W iBrtB2 = b3MulMVW( iB, rtB2 );

			b3FloatW kxx = b3AddW( b3AddW( b3AddW( mA, mB ), b3DotW( rtA1, iArtA1 ) ), b3DotW( rtB1, iBrtB1 ) );
			b3FloatW kyy = b3AddW( b3AddW( b3AddW( mA, mB ), b3DotW( rtA2, iArtA2 ) ), b3DotW( rtB2, iBrtB2 ) );
			b3FloatW kxy = b3AddW( b3DotW( rtA1, iArtA2 ), b3DotW( rtB1, iBrtB2 ) );

			b3FloatW det = b3SubW( b3MulW( kxx, kyy ), b3MulW( kxy, kxy ) );
			b3FloatW valid = b3AndW( b3GreaterThanW( b3AbsW( det ), minDet ), frictionValid );
			b3FloatW invDet = b3BlendW( zeroW, b3DivW( oneW, det ), valid );
			cm->tangentMass.cxx = b3MulW( invDet, kyy );
			cm->tangentMass.cxy = b3NegW( b3MulW( invDet, kxy ) );
			cm->tangentMass.cyy = b3MulW( invDet, kxx );
		}

		{
			b3FloatW kTwist = b3DotW( normal, b3MulMVW( invIAB, normal ) );
			cm->twistMass = b3BlendW( zeroW, b3DivW( oneW, kTwist ), b3GreaterThanW( kTwist, zeroW ) );
		}
	}

	// Only sample contact point normal velocity if needed.
	bool haveRestitution = b3AnyTrueW( b3GreaterThanW( c->restitution, zeroW ) );
	if ( hitEventLanes != 0 || haveRestitution )
	{
		b3BodyStateW bA = b3GatherBodies( states, c->indexA );
		b3BodyStateW bB = b3GatherBodies( states, c->indexB );

		for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
		{
			b3ContactManifoldWide* cm = manifolds + manifoldIndex;

			for ( int pointIndex = 0; pointIndex < cm->pointCount; ++pointIndex )
			{
				b3ContactConstraintPointWide* cp = cm->points + pointIndex;

				b3Vec3W vrA = b3AddVW( bA.v, b3CrossW( bA.w, cp->anchorAs ) );
				b3Vec3W vrB = b3AddVW( bB.v, b3CrossW( bB.w, cp->anchorBs ) );
				b3FloatW vn = b3DotW( cm->normal, b3SubVW( vrB, vrA ) );
				cp->relativeVelocities = vn;

				if ( hitEventLanes != 0 )
				{
					float normalVelocities[B3_SIMD_WIDTH];
					b3StoreW( normalVelocities, vn );

					for ( int lane = 0; lane < B3_SIMD_WIDTH; ++lane )
					{
						if ( ( hitEventLanes & ( 1 << lane ) ) == 0 || manifoldIndex >= manifoldCounts[lane] )
						{
							continue;
						}

						b3Manifold* manifold = c->contacts[lane]->manifolds + manifoldIndex;
						if ( pointIndex < manifold->pointCount )
						{
							manifold->points[pointIndex].normalVelocity = normalVelocities[lane];
						}
					}
				}
			}
		}
	}

	return haveRestitution;
}

void B3_WIDE( b3PrepareContacts_Convex )( b3SolverBlock block, b3StepContext* context )
{
	b3TracyCZoneNC( prepare_contact, "Prepare Contact", b3_colorYellow, true );

	b3World* world = context->world;
	b3WidePrepareSpan* spans = context->widePrepareSpans;
	void* wideBase = context->wideConstraints;
	b3FloatW warmStartScale = world->enableWarmStarting ? b3SplatW( 1.0f ) : b3ZeroW();
	bool anyRestitution = false;

	int wideIndex = block.startIndex;
	int endWideIndex = block.startIndex + block.count;

	// Find color for start index. Linear search but fast.
	int colorIndex = 0;
	while ( spans[colorIndex + 1].start <= wideIndex )
	{
		colorIndex += 1;
	}

	// Loop over block
	while ( wideIndex < endWideIndex )
	{
		int colorWideStart = spans[colorIndex].start;
		int colorWideEndIndex = b3MinInt( spans[colorIndex + 1].start, endWideIndex );
		int colorContactCount = spans[colorIndex].count;
		int* contactIds = spans[colorIndex].contacts;

		// Loop over color
		for ( ; wideIndex < colorWideEndIndex; ++wideIndex )
		{
			b3ContactConstraintWide* c = b3GetConstraintW( wideBase, NULL, wideIndex, 1 );
			int localWideIndex = wideIndex - colorWideStart;

			b3Contact* contacts[B3_SIMD_WIDTH];
			for ( int lane = 0; lane < B3_SIMD_WIDTH; ++lane )
			{
				int contactIndex = B3_SIMD_WIDTH * localWideIndex + lane;
				if ( contactIndex < colorContactCount )
				{
					contacts[lane] = b3Array_Get( world->contacts, contactIds[contactIndex] );
					B3_ASSERT( contacts[lane]->manifoldCount == 1 );
				}
				else
				{
					contacts[lane] = NULL;
				}
			}

			bool haveRestitution = b3PrepareConstraintW( c, contacts, 1, context, warmStartScale );
			anyRestitution = anyRestitution || haveRestitution;
		}

		// Advance to next color
		colorIndex += 1;
	}

	if ( anyRestitution )
	{
		b3AtomicStoreInt( &context->anyRestitution, 1 );
	}

	b3TracyCZoneEnd( prepare_contact );
}

void B3_WIDE( b3PrepareContacts_MeshWide )( b3SolverBlock block, b3StepContext* context )
{
	b3TracyCZoneNC( prepare_mesh_wide, "Prepare Mesh", b3_colorYellow, true );

	b3World* world = context->world;
	b3MeshPrepareSpan* spans = context->meshPrepareSpans;
	void* wideBase = context->wideMeshConstraints;
	const int* manifoldStarts = context->wideMeshManifoldStarts;
	b3FloatW warmStartScale = world->enableWarmStarting ? b3SplatW( 1.0f ) : b3ZeroW();
	bool anyRestitution = false;

	int wideIndex = block.startIndex;
	int endWideIndex = block.startIndex + block.count;

	int colorIndex = 0;
	while ( spans[colorIndex + 1].start <= wideIndex )
	{
		colorIndex += 1;
	}

	while ( wideIndex < endWideIndex )
	{
		int colorWideStart = spans[colorIndex].start;
		int colorWideEndIndex = b3MinInt( spans[colorIndex + 1].start, endWideIndex );
		int colorContactCount = spans[colorIndex].count;
		b3ContactSpec* specs = spans[colorIndex].contacts;

		// This order is used to group contacts with similar manifold counts to keep the lanes full.
		const int* order = spans[colorIndex].order;

		for ( ; wideIndex < colorWideEndIndex; ++wideIndex )
		{
			b3ContactConstraintWide* c = b3GetConstraintW( wideBase, manifoldStarts, wideIndex, 0 );
			int localWideIndex = wideIndex - colorWideStart;

			b3Contact* contacts[B3_SIMD_WIDTH];
			for ( int lane = 0; lane < B3_SIMD_WIDTH; ++lane )
			{
				int contactIndex = B3_SIMD_WIDTH * localWideIndex + lane;
				if ( contactIndex < colorContactCount )
				{
					b3ContactSpec* spec = specs + order[contactIndex];
					b3Contact* contact = b3Array_Get( world->contacts, spec->contactId );
					B3_ASSERT( contact->contactId == spec->contactId );
					B3_ASSERT( contact->manifoldCount == spec->manifoldCount );
					contacts[lane] = contact;
				}
				else
				{
					contacts[lane] = NULL;
				}
			}

			int manifoldCount = manifoldStarts[wideIndex + 1] - manifoldStarts[wideIndex];
			bool haveRestitution = b3PrepareConstraintW( c, contacts, manifoldCount, context, warmStartScale );
			anyRestitution = anyRestitution || haveRestitution;
		}

		colorIndex += 1;
	}

	if ( anyRestitution )
	{
		b3AtomicStoreInt( &context->anyRestitution, 1 );
	}

	b3TracyCZoneEnd( prepare_mesh_wide );
}

B3_FORCE_INLINE void b3WarmStartContactsW( b3SolverBlock block, b3StepContext* context, void* base, const int* manifoldStarts,
										   int fixedManifoldCount )
{
	b3BodyState* states = context->states;
	b3FloatW zeroW = b3ZeroW();

	b3ContactConstraintWide* c = b3GetConstraintW( base, manifoldStarts, block.startIndex, fixedManifoldCount );
	for ( int i = 0; i < block.count; ++i )
	{
		int manifoldCount = fixedManifoldCount > 0 ? fixedManifoldCount : c->manifoldCount;
		bool anyRolling = b3AllZeroW( c->rollingResistance ) == false;

		b3Vec3W linearImpulse = { zeroW, zeroW, zeroW };
		b3Vec3W angularImpulseA = { zeroW, zeroW, zeroW };
		b3Vec3W angularImpulseB = { zeroW, zeroW, zeroW };

		const b3ContactManifoldWide* manifolds = b3GetManifoldsW( c );
		for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
		{
			const b3ContactManifoldWide* cm = manifolds + manifoldIndex;

			b3FloatW totalNormalImpulse = zeroW;
			b3Vec3W momentA = { zeroW, zeroW, zeroW };
			b3Vec3W momentB = { zeroW, zeroW, zeroW };

			for ( int pointIndex = 0; pointIndex < cm->pointCount; ++pointIndex )
			{
				const b3ContactConstraintPointWide* cp = cm->points + pointIndex;
				b3FloatW normalImpulse = cp->normalImpulses;
				totalNormalImpulse = b3AddW( totalNormalImpulse, normalImpulse );
				momentA = b3MulAddSVW( momentA, normalImpulse, cp->anchorAs );
				momentB = b3MulAddSVW( momentB, normalImpulse, cp->anchorBs );
			}

			b3Vec3W normal = cm->normal;
			b3Vec3W frictionImpulse = b3MulSVW( cm->frictionImpulse.x, cm->tangent1 );
			frictionImpulse = b3MulAddSVW( frictionImpulse, cm->frictionImpulse.y, cm->tangent2 );

			b3Vec3W linear = b3MulAddSVW( frictionImpulse, totalNormalImpulse, normal );

			b3Vec3W twistImpulse = b3MulSVW( cm->twistImpulse, normal );
			b3Vec3W angularA =
				b3AddVW( b3AddVW( b3CrossW( momentA, normal ), b3CrossW( cm->centerA, frictionImpulse ) ), twistImpulse );
			b3Vec3W angularB =
				b3AddVW( b3AddVW( b3CrossW( momentB, normal ), b3CrossW( cm->centerB, frictionImpulse ) ), twistImpulse );

			if ( anyRolling )
			{
				angularA = b3AddVW( angularA, cm->rollingImpulse );
				angularB = b3AddVW( angularB, cm->rollingImpulse );
			}

			linearImpulse = b3AddVW( linearImpulse, linear );
			angularImpulseA = b3AddVW( angularImpulseA, angularA );
			angularImpulseB = b3AddVW( angularImpulseB, angularB );
		}

		b3BodyStateW bA = b3GatherBodies( states, c->indexA );
		b3BodyStateW bB = b3GatherBodies( states, c->indexB );

		bA.w = b3MulSubMVW( bA.w, c->invIA, angularImpulseA );
		bA.v = b3MulSubSVW( bA.v, c->invMassA, linearImpulse );
		bB.w = b3MulAddMVW( bB.w, c->invIB, angularImpulseB );
		bB.v = b3MulAddSVW( bB.v, c->invMassB, linearImpulse );

		b3ScatterBodies( states, c->indexA, &bA );
		b3ScatterBodies( states, c->indexB, &bB );

		c = b3NextConstraintW( c, manifoldCount );
	}
}

// Solve the non-penetration constraints with the soft bias. No friction and no restitution.
B3_FORCE_INLINE void b3PushContactsW( b3SolverBlock block, b3StepContext* context, void* base, const int* manifoldStarts,
									  int fixedManifoldCount )
{
	b3BodyState* states = context->states;
	b3FloatW inv_h = b3SplatW( context->inv_h );
	b3FloatW contactSpeed = b3SplatW( -context->world->contactSpeed );
	b3FloatW oneW = b3SplatW( 1.0f );
	b3FloatW zeroW = b3ZeroW();

	// Stiffer for static contacts to avoid bodies getting pushed through the ground. Selected per
	// lane from the null body index instead of stored per constraint.
	b3FloatW dynamicBiasRate = b3SplatW( context->contactSoftness.massScale * context->contactSoftness.biasRate );
	b3FloatW dynamicMassScale = b3SplatW( context->contactSoftness.massScale );
	b3FloatW dynamicImpulseScale = b3SplatW( context->contactSoftness.impulseScale );
	b3FloatW staticBiasRate = b3SplatW( context->staticSoftness.massScale * context->staticSoftness.biasRate );
	b3FloatW staticMassScale = b3SplatW( context->staticSoftness.massScale );
	b3FloatW staticImpulseScale = b3SplatW( context->staticSoftness.impulseScale );

	b3ContactConstraintWide* c = b3GetConstraintW( base, manifoldStarts, block.startIndex, fixedManifoldCount );
	for ( int i = 0; i < block.count; ++i )
	{
		int manifoldCount = fixedManifoldCount > 0 ? fixedManifoldCount : c->manifoldCount;
		b3BodyStateW bA = b3GatherBodies( states, c->indexA );
		b3BodyStateW bB = b3GatherBodies( states, c->indexB );

		b3FloatW softMask = b3SoftMaskW( c->indexA, c->indexB );
		b3FloatW biasRate = b3BlendW( dynamicBiasRate, staticBiasRate, softMask );
		b3FloatW massScale = b3BlendW( dynamicMassScale, staticMassScale, softMask );
		b3FloatW impulseScale = b3BlendW( dynamicImpulseScale, staticImpulseScale, softMask );

		b3Vec3W dp = b3SubVW( bB.dp, bA.dp );

		b3ContactManifoldWide* manifolds = b3GetManifoldsW( c );
		for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
		{
			b3ContactManifoldWide* cm = manifolds + manifoldIndex;

			// Convert to normals to local space to reduce transform math.
			// Read the normal rather than put on stack to avoid register spills.
			b3FloatW normalSeparation = b3DotW( cm->normal, dp );
			b3Vec3W normalA = b3InvRotateVectorW( bA.dq, cm->normal );
			b3Vec3W normalB = b3InvRotateVectorW( bB.dq, cm->normal );

			for ( int pointIndex = 0; pointIndex < cm->pointCount; ++pointIndex )
			{
				b3ContactConstraintPointWide* cp = cm->points + pointIndex;

				// Fixed anchor points for applying impulses
				b3Vec3W rA = cp->anchorAs;
				b3Vec3W rB = cp->anchorBs;

				b3FloatW s = b3AddW( b3AddW( normalSeparation, b3SubW( b3DotW( normalB, rB ), b3DotW( normalA, rA ) ) ),
									 cp->baseSeparations );

				// Apply speculative bias if separation is greater than zero, otherwise apply soft constraint bias
				b3FloatW separated = b3GreaterThanW( s, zeroW );

				// Speculative bias - positive
				b3FloatW specBias = b3MulW( s, inv_h );

				// Overlap bias - negative
				b3FloatW overlapBias = b3MaxW( b3MulW( biasRate, s ), contactSpeed );
				b3FloatW velocityBias = b3BlendW( overlapBias, specBias, separated );

				b3FloatW pointMassScale = b3BlendW( massScale, oneW, separated );
				b3FloatW pointImpulseScale = b3BlendW( impulseScale, zeroW, separated );

				// Relative velocity at contact
				b3Vec3W vrA = b3AddVW( bA.v, b3CrossW( bA.w, rA ) );
				b3Vec3W vrB = b3AddVW( bB.v, b3CrossW( bB.w, rB ) );
				b3FloatW vn = b3DotW( b3SubVW( vrB, vrA ), cm->normal );

				// Compute normal impulse
				b3FloatW negImpulse = b3AddW( b3MulW( cp->normalMasses, b3AddW( b3MulW( pointMassScale, vn ), velocityBias ) ),
											  b3MulW( pointImpulseScale, cp->normalImpulses ) );

				// Clamp the accumulated impulse
				b3FloatW newImpulse = b3MaxW( b3SubW( cp->normalImpulses, negImpulse ), zeroW );
				b3FloatW deltaImpulse = b3SubW( newImpulse, cp->normalImpulses );
				cp->normalImpulses = newImpulse;

				// Apply contact impulse
				b3Vec3W P = b3MulSVW( deltaImpulse, cm->normal );
				bA.w = b3MulSubMVW( bA.w, c->invIA, b3CrossW( rA, P ) );
				bA.v = b3MulSubSVW( bA.v, c->invMassA, P );
				bB.w = b3MulAddMVW( bB.w, c->invIB, b3CrossW( rB, P ) );
				bB.v = b3MulAddSVW( bB.v, c->invMassB, P );
			}
		}

		b3ScatterBodies( states, c->indexA, &bA );
		b3ScatterBodies( states, c->indexB, &bB );

		c = b3NextConstraintW( c, manifoldCount );
	}
}

// Solve the normal constraint, friction, and rolling resistance.
B3_FORCE_INLINE void b3SolveContactsW( b3SolverBlock block, b3StepContext* context, void* base, const int* manifoldStarts,
									   int fixedManifoldCount )
{
	b3BodyState* states = context->states;
	b3FloatW inv_h = b3SplatW( context->inv_h );
	b3FloatW oneW = b3SplatW( 1.0f );
	b3FloatW zeroW = b3ZeroW();
	b3FloatW epsilonW = b3SplatW( FLT_EPSILON );

	b3ContactConstraintWide* c = b3GetConstraintW( base, manifoldStarts, block.startIndex, fixedManifoldCount );
	for ( int i = 0; i < block.count; ++i )
	{
		int manifoldCount = fixedManifoldCount > 0 ? fixedManifoldCount : c->manifoldCount;
		b3BodyStateW bA = b3GatherBodies( states, c->indexA );
		b3BodyStateW bB = b3GatherBodies( states, c->indexB );

		bool anyRolling = b3AllZeroW( c->rollingResistance ) == false;
		b3Vec3W dp = b3SubVW( bB.dp, bA.dp );

		b3ContactManifoldWide* manifolds = b3GetManifoldsW( c );
		for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
		{
			b3ContactManifoldWide* cm = manifolds + manifoldIndex;

			// Read the normal rather than put on stack to avoid register spills.
			b3FloatW normalSeparation = b3DotW( cm->normal, dp );
			b3Vec3W normalA = b3InvRotateVectorW( bA.dq, cm->normal );
			b3Vec3W normalB = b3InvRotateVectorW( bB.dq, cm->normal );

			b3FloatW totalNormalImpulse = zeroW;
			b3FloatW totalTwistLimit = zeroW;

			// Normal contraints
			for ( int pointIndex = 0; pointIndex < cm->pointCount; ++pointIndex )
			{
				b3ContactConstraintPointWide* cp = cm->points + pointIndex;

				// Fixed anchor points for applying impulses
				b3Vec3W rA = cp->anchorAs;
				b3Vec3W rB = cp->anchorBs;

				b3FloatW s = b3AddW( b3AddW( normalSeparation, b3SubW( b3DotW( normalB, rB ), b3DotW( normalA, rA ) ) ),
									 cp->baseSeparations );

				// Speculative bias, positive if separated and zero if overlapped
				b3FloatW velocityBias = b3MaxW( zeroW, b3MulW( s, inv_h ) );

				// Relative velocity at contact
				b3Vec3W vrA = b3AddVW( bA.v, b3CrossW( bA.w, rA ) );
				b3Vec3W vrB = b3AddVW( bB.v, b3CrossW( bB.w, rB ) );
				b3FloatW vn = b3DotW( b3SubVW( vrB, vrA ), cm->normal );

				// Compute normal impulse
				b3FloatW negImpulse = b3MulW( cp->normalMasses, b3AddW( vn, velocityBias ) );

				// Clamp the accumulated impulse
				b3FloatW newImpulse = b3MaxW( b3SubW( cp->normalImpulses, negImpulse ), zeroW );
				b3FloatW deltaImpulse = b3SubW( newImpulse, cp->normalImpulses );
				cp->normalImpulses = newImpulse;
				cp->totalNormalImpulses = b3AddW( cp->totalNormalImpulses, newImpulse );
				totalNormalImpulse = b3AddW( totalNormalImpulse, newImpulse );
				totalTwistLimit = b3AddW( totalTwistLimit, b3MulW( cp->leverArms, newImpulse ) );

				// Apply contact impulse
				b3Vec3W P = b3MulSVW( deltaImpulse, cm->normal );
				bA.w = b3MulSubMVW( bA.w, c->invIA, b3CrossW( rA, P ) );
				bA.v = b3MulSubSVW( bA.v, c->invMassA, P );
				bB.w = b3MulAddMVW( bB.w, c->invIB, b3CrossW( rB, P ) );
				bB.v = b3MulAddSVW( bB.v, c->invMassB, P );
			}

			// Rolling resistance. After normal constraints because it uses the normal impulse.
			if ( anyRolling )
			{
				// flip A/B order to negate
				b3Vec3W deltaImpulse = b3MulMVW( c->rollingMass, b3SubVW( bA.w, bB.w ) );
				b3Vec3W oldImpulse = cm->rollingImpulse;
				cm->rollingImpulse = b3AddVW( oldImpulse, deltaImpulse );

				b3FloatW maxImpulse = b3MulW( c->rollingResistance, totalNormalImpulse );
				b3FloatW lengthSquared = b3DotW( cm->rollingImpulse, cm->rollingImpulse );

				// if ( magSqr > maxLambda * maxLambda + FLT_EPSILON )
				//{
				//	c->rollingImpulse *= maxLambda / sqrtf( magSqr );
				// }

				b3FloatW mask = b3GreaterThanW( lengthSquared, b3MulAddW( epsilonW, maxImpulse, maxImpulse ) );

				// No approximate _mm_rsqrt_ps here to maintain cross-platform determinism
				b3FloatW normalize = b3DivW( maxImpulse, b3AddW( b3SqrtW( lengthSquared ), epsilonW ) );
				b3FloatW scale = b3BlendW( oneW, normalize, mask );

				// Blend the result to ensure there are no +/-0 determinism problems, since 0 * -1 == -0.
				b3Vec3W scaledImpulse = b3MulSVW( scale, cm->rollingImpulse );
				cm->rollingImpulse.X = b3BlendW( zeroW, scaledImpulse.X, cm->rollingMask );
				cm->rollingImpulse.Y = b3BlendW( zeroW, scaledImpulse.Y, cm->rollingMask );
				cm->rollingImpulse.Z = b3BlendW( zeroW, scaledImpulse.Z, cm->rollingMask );

				deltaImpulse = b3SubVW( cm->rollingImpulse, oldImpulse );

				bA.w = b3MulSubMVW( bA.w, c->invIA, deltaImpulse );
				bB.w = b3MulAddMVW( bB.w, c->invIB, deltaImpulse );
			}

			// Central twist friction. Friction goes last to improve stacking stability.
			{
				b3FloatW twistSpeed = b3DotW( cm->normal, b3SubVW( bB.w, bA.w ) );
				b3FloatW maxLambda = b3MulW( c->friction, totalTwistLimit );
				b3FloatW deltaImpulse = b3NegW( b3MulW( cm->twistMass, twistSpeed ) );
				b3FloatW oldImpulse = cm->twistImpulse;
				cm->twistImpulse = b3SymClampW( b3AddW( oldImpulse, deltaImpulse ), maxLambda );
				deltaImpulse = b3SubW( cm->twistImpulse, oldImpulse );

				b3Vec3W L = b3MulSVW( deltaImpulse, cm->normal );
				bA.w = b3MulSubMVW( bA.w, c->invIA, L );
				bB.w = b3MulAddMVW( bB.w, c->invIB, L );
			}

			// Central friction. Friction goes last to improve stacking stability.
			{
				b3Vec3W tangent1 = cm->tangent1;
				b3Vec3W tangent2 = cm->tangent2;

				// Fixed anchor points for applying impulses
				b3Vec3W rA = cm->centerA;
				b3Vec3W rB = cm->centerB;

				// Relative tangent velocity at contact
				b3Vec3W vrA = b3AddVW( bA.v, b3CrossW( bA.w, rA ) );
				b3Vec3W vrB = b3AddVW( bB.v, b3CrossW( bB.w, rB ) );
				b3Vec3W vr = b3SubVW( vrB, vrA );
				b3Vec2W vt = {
					b3SubW( b3DotW( vr, tangent1 ), cm->tangentVelocity1 ),
					b3SubW( b3DotW( vr, tangent2 ), cm->tangentVelocity2 ),
				};

				// Incremental tangent impulse
				b3Vec2W deltaImpulse = b3MulMV2W( cm->tangentMass, vt );
				deltaImpulse = (b3Vec2W){ b3NegW( deltaImpulse.x ), b3NegW( deltaImpulse.y ) };
				b3Vec2W newImpulse = b3AddV2W( cm->frictionImpulse, deltaImpulse );

				b3FloatW maxImpulse = b3MulW( c->friction, totalNormalImpulse );

				// Clamp the accumulated impulse
				b3FloatW lengthSquared = b3AddW( b3MulW( newImpulse.x, newImpulse.x ), b3MulW( newImpulse.y, newImpulse.y ) );

				// Max impulse can be zero
				b3FloatW mask = b3GreaterThanW( lengthSquared, b3MulW( maxImpulse, maxImpulse ) );

				// No approximate _mm_rsqrt_ps here to maintain cross-platform determinism. Add epsilon to avoid divide by
				// zero.
				b3FloatW normalize = b3DivW( maxImpulse, b3AddW( b3SqrtW( lengthSquared ), epsilonW ) );
				b3FloatW scale = b3BlendW( oneW, normalize, mask );
				newImpulse = (b3Vec2W){
					b3MulW( scale, newImpulse.x ),
					b3MulW( scale, newImpulse.y ),
				};

				deltaImpulse = (b3Vec2W){
					b3SubW( newImpulse.x, cm->frictionImpulse.x ),
					b3SubW( newImpulse.y, cm->frictionImpulse.y ),
				};

				cm->frictionImpulse = newImpulse;

				// Apply delta impulse
				b3Vec3W P = b3AddVW( b3MulSVW( deltaImpulse.x, tangent1 ), b3MulSVW( deltaImpulse.y, tangent2 ) );
				bA.w = b3MulSubMVW( bA.w, c->invIA, b3CrossW( rA, P ) );
				bA.v = b3MulSubSVW( bA.v, c->invMassA, P );
				bB.w = b3MulAddMVW( bB.w, c->invIB, b3CrossW( rB, P ) );
				bB.v = b3MulAddSVW( bB.v, c->invMassB, P );
			}
		}

		b3ScatterBodies( states, c->indexA, &bA );
		b3ScatterBodies( states, c->indexB, &bB );

		c = b3NextConstraintW( c, manifoldCount );
	}
}

B3_FORCE_INLINE void b3ApplyRestitutionW( b3SolverBlock block, b3StepContext* context, void* base, const int* manifoldStarts,
										  int fixedManifoldCount )
{
	b3BodyState* states = context->states;
	b3FloatW inv_h = b3SplatW( context->inv_h );
	b3FloatW negRestitutionThreshold = b3SplatW( -context->world->restitutionThreshold );
	b3FloatW zeroW = b3ZeroW();
	bool propagate = context->world->enableRestitutionPropagation;

	b3ContactConstraintWide* next = b3GetConstraintW( base, manifoldStarts, block.startIndex, fixedManifoldCount );
	for ( int i = 0; i < block.count; ++i )
	{
		b3ContactConstraintWide* c = next;
		int manifoldCount = fixedManifoldCount > 0 ? fixedManifoldCount : c->manifoldCount;
		next = b3NextConstraintW( c, manifoldCount );

		if ( propagate == false && b3AllZeroW( c->restitution ) )
		{
			continue;
		}

		b3BodyStateW bA = b3GatherBodies( states, c->indexA );
		b3BodyStateW bB = b3GatherBodies( states, c->indexB );

		b3FloatW restitutionMask = b3GreaterThanW( c->restitution, zeroW );
		b3Vec3W dp = b3SubVW( bB.dp, bA.dp );

		b3ContactManifoldWide* manifolds = b3GetManifoldsW( c );
		for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
		{
			b3ContactManifoldWide* cm = manifolds + manifoldIndex;

			b3FloatW normalSeparation = b3DotW( cm->normal, dp );
			b3Vec3W normalA = b3InvRotateVectorW( bA.dq, cm->normal );
			b3Vec3W normalB = b3InvRotateVectorW( bB.dq, cm->normal );

			// Read the point count rather than put on stack to avoid register spills.
			for ( int pointIndex = 0; pointIndex < cm->pointCount; ++pointIndex )
			{
				b3ContactConstraintPointWide* cp = cm->points + pointIndex;

				b3Vec3W rA = cp->anchorAs;
				b3Vec3W rB = cp->anchorBs;

				b3FloatW s = b3AddW( b3AddW( normalSeparation, b3SubW( b3DotW( normalB, rB ), b3DotW( normalA, rA ) ) ),
									 cp->baseSeparations );

				b3FloatW normalMass = propagate ? cp->normalMasses : b3BlendW( zeroW, cp->normalMasses, restitutionMask );

				b3FloatW compressionImpulse = b3SubW( cp->totalNormalImpulses, cp->restitutionImpulses );
				b3FloatW armed =
					b3AndW( b3AndW( restitutionMask, b3LessThanW( cp->relativeVelocities, negRestitutionThreshold ) ),
							b3GreaterThanW( compressionImpulse, zeroW ) );

				b3FloatW specBias = b3MaxW( zeroW, b3MulW( s, inv_h ) );
				b3FloatW velocityBias = b3BlendW( specBias, b3MulW( c->restitution, cp->relativeVelocities ), armed );

				b3Vec3W vrA = b3AddVW( bA.v, b3CrossW( bA.w, rA ) );
				b3Vec3W vrB = b3AddVW( bB.v, b3CrossW( bB.w, rB ) );
				b3FloatW vn = b3DotW( b3SubVW( vrB, vrA ), cm->normal );

				b3FloatW negImpulse = b3MulW( normalMass, b3AddW( vn, velocityBias ) );

				b3FloatW newImpulse = b3MaxW( b3SubW( cp->normalImpulses, negImpulse ), zeroW );
				b3FloatW impulse = b3SubW( newImpulse, cp->normalImpulses );

				b3FloatW approachImpulse =
					b3MinW( b3MaxW( b3NegW( b3MulW( normalMass, vn ) ), zeroW ), b3MaxW( impulse, zeroW ) );
				b3FloatW allowance =
					b3SubW( b3MulW( c->restitution, b3AddW( compressionImpulse, approachImpulse ) ), cp->restitutionImpulses );
				b3FloatW maxImpulse = b3AddW( approachImpulse, b3MaxW( allowance, zeroW ) );
				impulse = b3BlendW( impulse, b3MinW( impulse, maxImpulse ), armed );

				cp->normalImpulses = b3AddW( cp->normalImpulses, impulse );
				cp->restitutionImpulses = b3AddW( cp->restitutionImpulses, b3SubW( impulse, approachImpulse ) );
				cp->totalNormalImpulses = b3AddW( cp->totalNormalImpulses, impulse );

				b3Vec3W P = b3MulSVW( impulse, cm->normal );
				bA.w = b3MulSubMVW( bA.w, c->invIA, b3CrossW( rA, P ) );
				bA.v = b3MulSubSVW( bA.v, c->invMassA, P );
				bB.w = b3MulAddMVW( bB.w, c->invIB, b3CrossW( rB, P ) );
				bB.v = b3MulAddSVW( bB.v, c->invMassB, P );
			}
		}

		b3ScatterBodies( states, c->indexA, &bA );
		b3ScatterBodies( states, c->indexB, &bB );
	}
}

// Store impulses by contact constraint
B3_FORCE_INLINE void b3StoreImpulsesW( b3SolverBlock block, b3StepContext* context, int workerIndex, void* base,
									   const int* manifoldStarts, int fixedManifoldCount )
{
	b3World* world = context->world;
	b3TaskContext* taskContext = world->taskContexts.data + workerIndex;
	b3BitSet* hitEventBitSet = &taskContext->hitEventBitSet;
	bool hasHitEvents = taskContext->hasHitEvents;
	float negHitThreshold = -world->hitEventThreshold;

	b3ContactConstraintWide* c = b3GetConstraintW( base, manifoldStarts, block.startIndex, fixedManifoldCount );
	for ( int i = 0; i < block.count; ++i )
	{
		int manifoldCount = fixedManifoldCount > 0 ? fixedManifoldCount : c->manifoldCount;
		const b3ContactManifoldWide* manifolds = b3GetManifoldsW( c );

		// Pay the cache misses up front.
		b3Manifold* laneManifolds[B3_SIMD_WIDTH];
		for ( int lane = 0; lane < B3_SIMD_WIDTH; ++lane )
		{
			b3Contact* contact = c->contacts[lane];
			laneManifolds[lane] = contact != NULL ? contact->manifolds : NULL;
		}

		for ( int lane = 0; lane < B3_SIMD_WIDTH; ++lane )
		{
			b3Contact* contact = c->contacts[lane];
			if ( contact == NULL )
			{
				continue;
			}

			int laneManifoldCount = fixedManifoldCount > 0 ? fixedManifoldCount : contact->manifoldCount;
			B3_ASSERT( laneManifoldCount == contact->manifoldCount && laneManifoldCount <= manifoldCount );

			for ( int manifoldIndex = 0; manifoldIndex < laneManifoldCount; ++manifoldIndex )
			{
				const b3ContactManifoldWide* cm = manifolds + manifoldIndex;
				b3Manifold* m = laneManifolds[lane] + manifoldIndex;

				const float* frictionImpulse1 = (const float*)&cm->frictionImpulse.x;
				const float* frictionImpulse2 = (const float*)&cm->frictionImpulse.y;
				const float* tangent1X = (const float*)&cm->tangent1.X;
				const float* tangent1Y = (const float*)&cm->tangent1.Y;
				const float* tangent1Z = (const float*)&cm->tangent1.Z;
				const float* tangent2X = (const float*)&cm->tangent2.X;
				const float* tangent2Y = (const float*)&cm->tangent2.Y;
				const float* tangent2Z = (const float*)&cm->tangent2.Z;
				const float* twistImpulse = (const float*)&cm->twistImpulse;
				const float* rollingImpulseX = (const float*)&cm->rollingImpulse.X;
				const float* rollingImpulseY = (const float*)&cm->rollingImpulse.Y;
				const float* rollingImpulseZ = (const float*)&cm->rollingImpulse.Z;

				float f1 = frictionImpulse1[lane];
				float f2 = frictionImpulse2[lane];
				m->frictionImpulse = (b3Vec3){
					f1 * tangent1X[lane] + f2 * tangent2X[lane],
					f1 * tangent1Y[lane] + f2 * tangent2Y[lane],
					f1 * tangent1Z[lane] + f2 * tangent2Z[lane],
				};
				m->twistImpulse = twistImpulse[lane];
				m->rollingImpulse = (b3Vec3){
					rollingImpulseX[lane],
					rollingImpulseY[lane],
					rollingImpulseZ[lane],
				};

				int pointCount = m->pointCount;
				B3_ASSERT( pointCount <= cm->pointCount );
				for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
				{
					const b3ContactConstraintPointWide* cp = cm->points + pointIndex;
					const float* normalImpulse = (const float*)&cp->normalImpulses;
					const float* totalNormalImpulse = (const float*)&cp->totalNormalImpulses;

					b3ManifoldPoint* mp = m->points + pointIndex;
					mp->normalImpulse = normalImpulse[lane];
					mp->totalNormalImpulse = totalNormalImpulse[lane];
				}
			}

			if ( ( contact->flags & b3_simEnableHitEvent ) != 0 )
			{
				bool flagged = false;
				for ( int manifoldIndex = 0; manifoldIndex < laneManifoldCount && flagged == false; ++manifoldIndex )
				{
					const b3Manifold* m = contact->manifolds + manifoldIndex;
					for ( int pointIndex = 0; pointIndex < m->pointCount; ++pointIndex )
					{
						const b3ManifoldPoint* mp = m->points + pointIndex;

						// Need to check total impulse because the point may be speculative and not colliding
						if ( mp->normalVelocity < negHitThreshold && mp->totalNormalImpulse > 0.0f )
						{
							b3SetBit( hitEventBitSet, contact->contactId );
							hasHitEvents = true;
							flagged = true;
							break;
						}
					}
				}
			}
		}

		c = b3NextConstraintW( c, manifoldCount );
	}

	taskContext->hasHitEvents = hasHitEvents;
}

// These wrappers allow the implementation be shared between convex and mesh contacts.

void B3_WIDE( b3WarmStartContacts_Convex )( b3SolverBlock block, b3StepContext* context )
{
	b3TracyCZoneNC( warm_start_contact, "Warm Start", b3_colorGreen, true );
	b3WarmStartContactsW( block, context, context->graph->colors[block.colorIndex].wideConstraints, NULL, 1 );
	b3TracyCZoneEnd( warm_start_contact );
}

void B3_WIDE( b3PushContacts_Convex )( b3SolverBlock block, b3StepContext* context )
{
	b3TracyCZoneNC( push_contact, "Push Contact", b3_colorAliceBlue, true );
	b3PushContactsW( block, context, context->graph->colors[block.colorIndex].wideConstraints, NULL, 1 );
	b3TracyCZoneEnd( push_contact );
}

void B3_WIDE( b3SolveContacts_Convex )( b3SolverBlock block, b3StepContext* context )
{
	b3TracyCZoneNC( solve_contact, "Solve Contact", b3_colorAliceBlue, true );
	b3SolveContactsW( block, context, context->graph->colors[block.colorIndex].wideConstraints, NULL, 1 );
	b3TracyCZoneEnd( solve_contact );
}

void B3_WIDE( b3ApplyRestitution_Convex )( b3SolverBlock block, b3StepContext* context )
{
	b3TracyCZoneNC( restitution, "Restitution", b3_colorDodgerBlue, true );
	b3ApplyRestitutionW( block, context, context->graph->colors[block.colorIndex].wideConstraints, NULL, 1 );
	b3TracyCZoneEnd( restitution );
}

void B3_WIDE( b3StoreImpulses_Convex )( b3SolverBlock block, b3StepContext* context, int workerIndex )
{
	b3TracyCZoneNC( store_impulses, "Store", b3_colorFireBrick, true );
	b3StoreImpulsesW( block, context, workerIndex, context->wideConstraints, NULL, 1 );
	b3TracyCZoneEnd( store_impulses );
}

void B3_WIDE( b3WarmStartContacts_MeshWide )( b3SolverBlock block, b3StepContext* context )
{
	b3TracyCZoneNC( warm_start_mesh_wide, "Warm Start Mesh", b3_colorGreen, true );
	b3GraphColor* color = context->graph->colors + block.colorIndex;
	b3WarmStartContactsW( block, context, color->wideMeshConstraints, color->wideMeshManifoldStarts, 0 );
	b3TracyCZoneEnd( warm_start_mesh_wide );
}

void B3_WIDE( b3PushContacts_MeshWide )( b3SolverBlock block, b3StepContext* context )
{
	b3TracyCZoneNC( push_mesh_wide, "Push Mesh", b3_colorAliceBlue, true );
	b3GraphColor* color = context->graph->colors + block.colorIndex;
	b3PushContactsW( block, context, color->wideMeshConstraints, color->wideMeshManifoldStarts, 0 );
	b3TracyCZoneEnd( push_mesh_wide );
}

void B3_WIDE( b3SolveContacts_MeshWide )( b3SolverBlock block, b3StepContext* context )
{
	b3TracyCZoneNC( solve_mesh_wide, "Solve Mesh", b3_colorAliceBlue, true );
	b3GraphColor* color = context->graph->colors + block.colorIndex;
	b3SolveContactsW( block, context, color->wideMeshConstraints, color->wideMeshManifoldStarts, 0 );
	b3TracyCZoneEnd( solve_mesh_wide );
}

void B3_WIDE( b3ApplyRestitution_MeshWide )( b3SolverBlock block, b3StepContext* context )
{
	b3TracyCZoneNC( restitution_mesh_wide, "Restitution Mesh", b3_colorDodgerBlue, true );
	b3GraphColor* color = context->graph->colors + block.colorIndex;
	b3ApplyRestitutionW( block, context, color->wideMeshConstraints, color->wideMeshManifoldStarts, 0 );
	b3TracyCZoneEnd( restitution_mesh_wide );
}

void B3_WIDE( b3StoreImpulses_MeshWide )( b3SolverBlock block, b3StepContext* context, int workerIndex )
{
	b3TracyCZoneNC( store_mesh_wide, "Store Mesh", b3_colorFireBrick, true );
	b3StoreImpulsesW( block, context, workerIndex, context->wideMeshConstraints, context->wideMeshManifoldStarts, 0 );
	b3TracyCZoneEnd( store_mesh_wide );
}

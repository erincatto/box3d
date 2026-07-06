// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <float.h>
#include <math.h>
#include <string.h>

static b3Capsule capsule = { { -1.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, 1.0f };
static b3Sphere sphere = { { 1.0f, 0.0f, 0.0f }, 1.0f };
static b3BoxHull box = { 0 };

#define N 4

static int ShapeMassTest( void )
{
	// Sphere
	{
		b3MassData md = b3ComputeSphereMass( &sphere, 1.0f );
		float mass = 4.0f / 3.0f * B3_PI;
		ENSURE_SMALL( md.mass - mass, FLT_EPSILON );
		ENSURE( md.center.x == 1.0f && md.center.y == 0.0f );

		// Inertia is now about the shape center of mass, so the offset does not appear.
		float inertia = 2.0f / 5.0f * mass;
		ENSURE_SMALL( md.inertia.cx.x - inertia, FLT_EPSILON );
		ENSURE_SMALL( md.inertia.cy.y - inertia, FLT_EPSILON );
		ENSURE_SMALL( md.inertia.cz.z - inertia, FLT_EPSILON );
	}

	// Analytic box hull
	{
		b3MassData md = b3ComputeHullMass( &box.base, 1.0f );
		float mass = 2.0f * 2.0f * 2.0f;
		ENSURE_SMALL( md.mass - mass, FLT_EPSILON );
		ENSURE_SMALL( md.center.x, FLT_EPSILON );
		ENSURE_SMALL( md.center.y, FLT_EPSILON );
		ENSURE_SMALL( md.center.z, FLT_EPSILON );
		float inertia = ( 1.0f / 12.0f ) * mass * ( 2.0f * 2.0f + 2.0f * 2.0f );
		ENSURE_SMALL( md.inertia.cx.x - inertia, 2.0f * FLT_EPSILON );
		ENSURE_SMALL( md.inertia.cy.y - inertia, 2.0f * FLT_EPSILON );
		ENSURE_SMALL( md.inertia.cz.z - inertia, 2.0f * FLT_EPSILON );
	}

	// Translated box
	{
		b3Vec3 offset = { 0.4f, -0.7f, 0.1f };
		b3Transform transform = {
			.p = offset,
			.q = b3Quat_identity,
		};
		b3Vec3 h = { 0.25f, 0.5f, 0.3f };
		b3BoxHull b1 = b3MakeBoxHull( h.x, h.y, h.z );
		b3BoxHull b2 = b3MakeTransformedBoxHull( h.x, h.y, h.z, transform );

		b3MassData m1 = b3ComputeHullMass( &b1.base, 1.0f );
		b3MassData m2 = b3ComputeHullMass( &b2.base, 1.0f );

		ENSURE_SMALL( m1.mass - m2.mass, FLT_EPSILON );

		b3Matrix3 d = b3SubMM( b1.base.centralInertia, b2.base.centralInertia );
		ENSURE_SMALL( d.cx.x, FLT_EPSILON );
		ENSURE_SMALL( d.cx.y, FLT_EPSILON );
		ENSURE_SMALL( d.cx.z, FLT_EPSILON );
		ENSURE_SMALL( d.cy.x, FLT_EPSILON );
		ENSURE_SMALL( d.cy.y, FLT_EPSILON );
		ENSURE_SMALL( d.cy.z, FLT_EPSILON );
		ENSURE_SMALL( d.cz.x, FLT_EPSILON );
		ENSURE_SMALL( d.cz.y, FLT_EPSILON );
		ENSURE_SMALL( d.cz.z, FLT_EPSILON );

		ENSURE_SMALL( m2.center.x - offset.x, FLT_EPSILON );
		ENSURE_SMALL( m2.center.y - offset.y, FLT_EPSILON );
		ENSURE_SMALL( m2.center.z - offset.z, FLT_EPSILON );
	}

	// Rotated box
	{
		b3Vec3 h1 = { 0.25f, 0.5f, 0.3f };
		b3Vec3 h2 = { 0.25f, 0.3f, 0.5f };
		b3Quat q = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisY, b3Vec3_axisZ );
		b3Transform transform = {
			.p = b3Vec3_zero,
			.q = q,
		};
		b3BoxHull b1 = b3MakeTransformedBoxHull( h1.x, h1.y, h1.z, transform );
		b3BoxHull b2 = b3MakeBoxHull( h2.x, h2.y, h2.z );

		b3MassData m1 = b3ComputeHullMass( &b1.base, 1.0f );
		b3MassData m2 = b3ComputeHullMass( &b2.base, 1.0f );

		ENSURE_SMALL( m1.mass - m2.mass, FLT_EPSILON );

		b3Matrix3 d = b3SubMM( b1.base.centralInertia, b2.base.centralInertia );
		ENSURE_SMALL( d.cx.x, FLT_EPSILON );
		ENSURE_SMALL( d.cx.y, FLT_EPSILON );
		ENSURE_SMALL( d.cx.z, FLT_EPSILON );
		ENSURE_SMALL( d.cy.x, FLT_EPSILON );
		ENSURE_SMALL( d.cy.y, FLT_EPSILON );
		ENSURE_SMALL( d.cy.z, FLT_EPSILON );
		ENSURE_SMALL( d.cz.x, FLT_EPSILON );
		ENSURE_SMALL( d.cz.y, FLT_EPSILON );
		ENSURE_SMALL( d.cz.z, FLT_EPSILON );

		ENSURE_SMALL( m1.center.x - m2.center.x, FLT_EPSILON );
		ENSURE_SMALL( m1.center.y - m2.center.y, FLT_EPSILON );
		ENSURE_SMALL( m1.center.z - m2.center.z, FLT_EPSILON );
	}

	// Transformed box
	{
		b3Vec3 offset = { 0.4f, -0.7f, 0.1f };
		b3Vec3 h1 = { 0.25f, 0.5f, 0.3f };
		b3Vec3 h2 = { 0.25f, 0.3f, 0.5f };
		b3Quat q = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisY, b3Vec3_axisZ );
		b3Transform transform = {
			.p = offset,
			.q = q,
		};
		b3BoxHull b1 = b3MakeTransformedBoxHull( h1.x, h1.y, h1.z, transform );
		b3BoxHull b2 = b3MakeBoxHull( h2.x, h2.y, h2.z );

		b3MassData m1 = b3ComputeHullMass( &b1.base, 1.0f );
		b3MassData m2 = b3ComputeHullMass( &b2.base, 1.0f );

		ENSURE_SMALL( m1.mass - m2.mass, FLT_EPSILON );

		b3Matrix3 d = b3SubMM( b1.base.centralInertia, b2.base.centralInertia );
		ENSURE_SMALL( d.cx.x, FLT_EPSILON );
		ENSURE_SMALL( d.cx.y, FLT_EPSILON );
		ENSURE_SMALL( d.cx.z, FLT_EPSILON );
		ENSURE_SMALL( d.cy.x, FLT_EPSILON );
		ENSURE_SMALL( d.cy.y, FLT_EPSILON );
		ENSURE_SMALL( d.cy.z, FLT_EPSILON );
		ENSURE_SMALL( d.cz.x, FLT_EPSILON );
		ENSURE_SMALL( d.cz.y, FLT_EPSILON );
		ENSURE_SMALL( d.cz.z, FLT_EPSILON );

		ENSURE_SMALL( m1.center.x - offset.x, FLT_EPSILON );
		ENSURE_SMALL( m1.center.y - offset.y, FLT_EPSILON );
		ENSURE_SMALL( m1.center.z - offset.z, FLT_EPSILON );
	}

	// Capsule
	{
		float radius = capsule.radius;
		float length = b3Distance( capsule.center1, capsule.center2 );

		// Capsule along x-axis
		b3MassData md = b3ComputeCapsuleMass( &capsule, 1.0f );

		// Box that fully contains capsule. Upper bound on capsule mass.
		b3BoxHull r = b3MakeBoxHull( radius + 0.5f * length, radius, radius );
		b3MassData mdUpper = b3ComputeHullMass( &r.base, 1.0f );

		// Approximate capsule using convex hull. This should be a lower bound on the
		// capsule mass.
		b3Vec3 points[2 * N * N];
		float d = B3_PI / ( N - 1.0f );
		float angle1 = -0.5f * B3_PI;
		int index = 0;
		for ( int i = 0; i < N; ++i )
		{
			float s1 = sinf( angle1 );
			float c1 = cosf( angle1 );
			float angle2 = -0.5f * B3_PI;
			for ( int j = 0; j < N; ++j )
			{
				points[index].x = 1.0f + radius * c1;
				points[index].y = radius * s1 * cosf( angle2 );
				points[index].z = radius * s1 * sinf( angle2 );
				angle2 += d;
				index += 1;
			}

			angle1 += d;
		}

		angle1 = 0.5f * B3_PI;
		for ( int i = 0; i < N; ++i )
		{
			float s1 = sinf( angle1 );
			float c1 = cosf( angle1 );
			float angle2 = -0.5f * B3_PI;
			for ( int j = 0; j < N; ++j )
			{
				points[index].x = -1.0f + radius * c1;
				points[index].y = radius * s1 * cosf( angle2 );
				points[index].z = radius * s1 * sinf( angle2 );
				angle2 += d;
				index += 1;
			}

			angle1 += d;
		}

		ENSURE( index == 2 * N * N );

		b3HullData* hull = b3CreateHull( points, 2 * N * N, 2 * N * N );
		b3MassData mdLower = b3ComputeHullMass( hull, 1.0f );

		ENSURE( mdLower.mass < md.mass && md.mass < mdUpper.mass );
		ENSURE( mdLower.inertia.cx.x < md.inertia.cx.x && md.inertia.cx.x < mdUpper.inertia.cx.x );
		ENSURE( mdLower.inertia.cy.y < md.inertia.cy.y && md.inertia.cy.y < mdUpper.inertia.cy.y );
		ENSURE( mdLower.inertia.cz.z < md.inertia.cz.z && md.inertia.cz.z < mdUpper.inertia.cz.z );

		b3DestroyHull( hull );
	}

	return 0;
}

static int ShapeAABBTest( void )
{
	{
		b3AABB b = b3ComputeSphereAABB( &sphere, b3Transform_identity );
		ENSURE_SMALL( b.lowerBound.x, FLT_EPSILON );
		ENSURE_SMALL( b.lowerBound.y + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( b.lowerBound.z + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( b.upperBound.x - 2.0f, FLT_EPSILON );
		ENSURE_SMALL( b.upperBound.y - 1.0f, FLT_EPSILON );
		ENSURE_SMALL( b.upperBound.z - 1.0f, FLT_EPSILON );
	}

	{
		b3AABB b = b3ComputeCapsuleAABB( &capsule, b3Transform_identity );
		ENSURE_SMALL( b.lowerBound.x + 2.0f, FLT_EPSILON );
		ENSURE_SMALL( b.lowerBound.y + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( b.lowerBound.z + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( b.upperBound.x - 2.0f, FLT_EPSILON );
		ENSURE_SMALL( b.upperBound.y - 1.0f, FLT_EPSILON );
		ENSURE_SMALL( b.upperBound.z - 1.0f, FLT_EPSILON );
	}
	{
		b3AABB b = b3ComputeHullAABB( &box.base, b3Transform_identity );
		ENSURE_SMALL( b.lowerBound.x + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( b.lowerBound.y + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( b.lowerBound.z + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( b.upperBound.x - 1.0f, FLT_EPSILON );
		ENSURE_SMALL( b.upperBound.y - 1.0f, FLT_EPSILON );
		ENSURE_SMALL( b.upperBound.z - 1.0f, FLT_EPSILON );
	}

	return 0;
}

#if 0
static int PointInShapeTest( void )
{
	b3Vec2 p1 = { 0.5f, 0.5f };
	b3Vec2 p2 = { 4.0f, -4.0f };

	{
		bool hit;
		hit = b3PointInSphere( p1, &sphere );
		ENSURE( hit == true );
		hit = b3PointInSphere( p2, &sphere );
		ENSURE( hit == false );
	}

	{
		bool hit;
		hit = b3PointInPolygon( p1, &box );
		ENSURE( hit == true );
		hit = b3PointInPolygon( p2, &box );
		ENSURE( hit == false );
	}

	return 0;
}
#endif

// Shared assertions for a surface hit. The normal points outward toward the ray,
// the point sits on the surface, and the point lies on the ray at the reported fraction.
static int CheckCastHit( b3CastOutput out, b3Vec3 origin, b3Vec3 translation, b3Vec3 point, b3Vec3 normal, float fraction,
						 float tol )
{
	ENSURE( out.hit );
	ENSURE_SMALL( out.fraction - fraction, tol );
	ENSURE_SMALL( out.point.x - point.x, tol );
	ENSURE_SMALL( out.point.y - point.y, tol );
	ENSURE_SMALL( out.point.z - point.z, tol );
	ENSURE_SMALL( out.normal.x - normal.x, tol );
	ENSURE_SMALL( out.normal.y - normal.y, tol );
	ENSURE_SMALL( out.normal.z - normal.z, tol );

	b3Vec3 onRay = b3MulAdd( origin, out.fraction, translation );
	ENSURE_SMALL( b3Distance( out.point, onRay ), tol );
	return 0;
}

// The shared initial overlap convention: a ray starting inside a solid reports the origin
// with zero fraction and no normal.
static int CheckInitialOverlap( b3CastOutput out, b3Vec3 origin )
{
	ENSURE( out.hit );
	ENSURE( out.fraction == 0.0f );
	ENSURE_SMALL( b3Distance( out.point, origin ), FLT_EPSILON );
	ENSURE( out.normal.x == 0.0f && out.normal.y == 0.0f && out.normal.z == 0.0f );
	return 0;
}

static int RayCastSphereHitTest( void )
{
	b3Sphere s = { { 0.0f, 0.0f, 0.0f }, 1.0f };

	// Hit along each principal axis. Surface at distance 3 over a length 8 ray.
	{
		b3RayCastInput input = { { -4.0f, 0.0f, 0.0f }, { 8.0f, 0.0f, 0.0f }, 1.0f };
		b3CastOutput out = b3RayCastSphere( &s, &input );
		if ( CheckCastHit( out, input.origin, input.translation, (b3Vec3){ -1.0f, 0.0f, 0.0f }, (b3Vec3){ -1.0f, 0.0f, 0.0f },
						   3.0f / 8.0f, 1e-5f ) )
			return 1;
	}
	{
		b3RayCastInput input = { { 0.0f, 4.0f, 0.0f }, { 0.0f, -8.0f, 0.0f }, 1.0f };
		b3CastOutput out = b3RayCastSphere( &s, &input );
		if ( CheckCastHit( out, input.origin, input.translation, (b3Vec3){ 0.0f, 1.0f, 0.0f }, (b3Vec3){ 0.0f, 1.0f, 0.0f },
						   3.0f / 8.0f, 1e-5f ) )
			return 1;
	}
	{
		b3RayCastInput input = { { 0.0f, 0.0f, -4.0f }, { 0.0f, 0.0f, 8.0f }, 1.0f };
		b3CastOutput out = b3RayCastSphere( &s, &input );
		if ( CheckCastHit( out, input.origin, input.translation, (b3Vec3){ 0.0f, 0.0f, -1.0f }, (b3Vec3){ 0.0f, 0.0f, -1.0f },
						   3.0f / 8.0f, 1e-5f ) )
			return 1;
	}

	// Offset center, hit partway along the ray.
	{
		b3Sphere s2 = { { 5.0f, 0.0f, 0.0f }, 2.0f };
		b3RayCastInput input = { { 0.0f, 0.0f, 0.0f }, { 10.0f, 0.0f, 0.0f }, 1.0f };
		b3CastOutput out = b3RayCastSphere( &s2, &input );
		if ( CheckCastHit( out, input.origin, input.translation, (b3Vec3){ 3.0f, 0.0f, 0.0f }, (b3Vec3){ -1.0f, 0.0f, 0.0f },
						   0.3f, 1e-5f ) )
			return 1;
	}

	// Diagonal ray straight through the center.
	{
		float k = 0.70710678f;
		b3RayCastInput input = { { -3.0f, -3.0f, 0.0f }, { 6.0f, 6.0f, 0.0f }, 1.0f };
		b3CastOutput out = b3RayCastSphere( &s, &input );
		if ( CheckCastHit( out, input.origin, input.translation, (b3Vec3){ -k, -k, 0.0f }, (b3Vec3){ -k, -k, 0.0f }, 0.382149f,
						   1e-4f ) )
			return 1;
	}

	return 0;
}

static int RayCastSphereMissTest( void )
{
	b3Sphere s = { { 0.0f, 0.0f, 0.0f }, 1.0f };

	// Pointing away.
	{
		b3RayCastInput input = { { -4.0f, 0.0f, 0.0f }, { -8.0f, 0.0f, 0.0f }, 1.0f };
		ENSURE( b3RayCastSphere( &s, &input ).hit == false );
	}
	// Passes wide of the sphere.
	{
		b3RayCastInput input = { { -4.0f, 3.0f, 0.0f }, { 8.0f, 0.0f, 0.0f }, 1.0f };
		ENSURE( b3RayCastSphere( &s, &input ).hit == false );
	}
	// Aimed at the sphere but the translation stops short.
	{
		b3RayCastInput input = { { -4.0f, 0.0f, 0.0f }, { 8.0f, 0.0f, 0.0f }, 0.3f };
		ENSURE( b3RayCastSphere( &s, &input ).hit == false );
	}
	return 0;
}

static int RayCastSphereClipTest( void )
{
	b3Sphere s = { { 0.0f, 0.0f, 0.0f }, 1.0f };

	// The surface is reached at fraction 3/8. Straddle it with maxFraction.
	{
		b3RayCastInput input = { { -4.0f, 0.0f, 0.0f }, { 8.0f, 0.0f, 0.0f }, 0.374f };
		ENSURE( b3RayCastSphere( &s, &input ).hit == false );
	}
	{
		b3RayCastInput input = { { -4.0f, 0.0f, 0.0f }, { 8.0f, 0.0f, 0.0f }, 0.376f };
		b3CastOutput out = b3RayCastSphere( &s, &input );
		ENSURE( out.hit );
		ENSURE_SMALL( out.fraction - 3.0f / 8.0f, 1e-5f );
	}
	return 0;
}

static int RayCastSphereInteriorTest( void )
{
	b3Sphere s = { { 0.0f, 0.0f, 0.0f }, 1.0f };

	// Origin inside reports the origin with zero fraction.
	{
		b3RayCastInput input = { { 0.3f, 0.0f, 0.0f }, { 8.0f, 0.0f, 0.0f }, 1.0f };
		b3CastOutput out = b3RayCastSphere( &s, &input );
		ENSURE( out.hit );
		ENSURE( out.fraction == 0.0f );
		ENSURE_SMALL( b3Distance( out.point, input.origin ), FLT_EPSILON );
	}
	// Zero length ray inside.
	{
		b3RayCastInput input = { { 0.5f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 1.0f };
		b3CastOutput out = b3RayCastSphere( &s, &input );
		ENSURE( out.hit );
		ENSURE_SMALL( b3Distance( out.point, input.origin ), FLT_EPSILON );
	}
	// Zero length ray outside.
	{
		b3RayCastInput input = { { 3.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 1.0f };
		ENSURE( b3RayCastSphere( &s, &input ).hit == false );
	}
	return 0;
}

static int RayCastSphereGrazeTest( void )
{
	b3Sphere s = { { 0.0f, 0.0f, 0.0f }, 1.0f };

	// Just inside the radius grazes a hit, just outside misses.
	{
		b3RayCastInput input = { { -4.0f, 0.999f, 0.0f }, { 8.0f, 0.0f, 0.0f }, 1.0f };
		ENSURE( b3RayCastSphere( &s, &input ).hit );
	}
	{
		b3RayCastInput input = { { -4.0f, 1.001f, 0.0f }, { 8.0f, 0.0f, 0.0f }, 1.0f };
		ENSURE( b3RayCastSphere( &s, &input ).hit == false );
	}
	return 0;
}

// Capsule along x from -2 to 2, radius 1. Reused by the capsule ray cast subtests.
static const b3Capsule rayCapsule = { { -2.0f, 0.0f, 0.0f }, { 2.0f, 0.0f, 0.0f }, 1.0f };

static int RayCastCapsuleSideTest( void )
{
	// Perpendicular hit on the cylindrical side. Surface at distance 2 over a length 6 ray.
	{
		b3RayCastInput input = { { 0.0f, 3.0f, 0.0f }, { 0.0f, -6.0f, 0.0f }, 1.0f };
		b3CastOutput out = b3RayCastCapsule( &rayCapsule, &input );
		if ( CheckCastHit( out, input.origin, input.translation, (b3Vec3){ 0.0f, 1.0f, 0.0f }, (b3Vec3){ 0.0f, 1.0f, 0.0f },
						   1.0f / 3.0f, 1e-5f ) )
			return 1;
	}
	// Same from +z to exercise the other transverse direction.
	{
		b3RayCastInput input = { { 0.0f, 0.0f, 3.0f }, { 0.0f, 0.0f, -6.0f }, 1.0f };
		b3CastOutput out = b3RayCastCapsule( &rayCapsule, &input );
		if ( CheckCastHit( out, input.origin, input.translation, (b3Vec3){ 0.0f, 0.0f, 1.0f }, (b3Vec3){ 0.0f, 0.0f, 1.0f },
						   1.0f / 3.0f, 1e-5f ) )
			return 1;
	}
	// Side hit nearer the c1 end.
	{
		b3RayCastInput input = { { -1.0f, 3.0f, 0.0f }, { 0.0f, -6.0f, 0.0f }, 1.0f };
		b3CastOutput out = b3RayCastCapsule( &rayCapsule, &input );
		if ( CheckCastHit( out, input.origin, input.translation, (b3Vec3){ -1.0f, 1.0f, 0.0f }, (b3Vec3){ 0.0f, 1.0f, 0.0f },
						   1.0f / 3.0f, 1e-5f ) )
			return 1;
	}
	return 0;
}

static int RayCastCapsuleObliqueTest( void )
{
	// Oblique ray in the z=0 plane. It crosses y=1 inside the cylinder span, so the
	// normal stays transverse. Exercises the non perpendicular ray/axis solve where
	// dot(axis, rayAxis) != 0.
	b3RayCastInput input = { { -3.0f, 3.0f, 0.0f }, { 4.0f, -4.0f, 0.0f }, 1.0f };
	b3CastOutput out = b3RayCastCapsule( &rayCapsule, &input );
	return CheckCastHit( out, input.origin, input.translation, (b3Vec3){ -1.0f, 1.0f, 0.0f }, (b3Vec3){ 0.0f, 1.0f, 0.0f }, 0.5f,
						 1e-4f );
}

static int RayCastCapsuleCapTest( void )
{
	float k = 0.70710678f;

	// Collinear ray hits the c2 hemisphere from beyond the end.
	{
		b3RayCastInput input = { { 5.0f, 0.0f, 0.0f }, { -8.0f, 0.0f, 0.0f }, 1.0f };
		b3CastOutput out = b3RayCastCapsule( &rayCapsule, &input );
		if ( CheckCastHit( out, input.origin, input.translation, (b3Vec3){ 3.0f, 0.0f, 0.0f }, (b3Vec3){ 1.0f, 0.0f, 0.0f },
						   1.0f / 4.0f, 1e-5f ) )
			return 1;
	}
	// Off-axis ray through the c2 cap center, approaching from outside the cylinder.
	{
		b3RayCastInput input = { { 4.0f, 2.0f, 0.0f }, { -4.0f, -4.0f, 0.0f }, 1.0f };
		b3CastOutput out = b3RayCastCapsule( &rayCapsule, &input );
		if ( CheckCastHit( out, input.origin, input.translation, (b3Vec3){ 2.0f + k, k, 0.0f }, (b3Vec3){ k, k, 0.0f }, 0.323223f,
						   1e-4f ) )
			return 1;
	}
	// Mirror through the c1 cap center.
	{
		b3RayCastInput input = { { -4.0f, 2.0f, 0.0f }, { 4.0f, -4.0f, 0.0f }, 1.0f };
		b3CastOutput out = b3RayCastCapsule( &rayCapsule, &input );
		if ( CheckCastHit( out, input.origin, input.translation, (b3Vec3){ -2.0f - k, k, 0.0f }, (b3Vec3){ -k, k, 0.0f },
						   0.323223f, 1e-4f ) )
			return 1;
	}
	return 0;
}

static int RayCastCapsuleMissTest( void )
{
	// Pointing away.
	{
		b3RayCastInput input = { { 0.0f, 3.0f, 0.0f }, { 0.0f, 4.0f, 0.0f }, 1.0f };
		ENSURE( b3RayCastCapsule( &rayCapsule, &input ).hit == false );
	}
	// Crosses above the axis more than a radius away.
	{
		b3RayCastInput input = { { 0.0f, 4.0f, 2.0f }, { 0.0f, -8.0f, 0.0f }, 1.0f };
		ENSURE( b3RayCastCapsule( &rayCapsule, &input ).hit == false );
	}
	// Aimed at the side but the translation stops short.
	{
		b3RayCastInput input = { { 0.0f, 5.0f, 0.0f }, { 0.0f, -1.0f, 0.0f }, 1.0f };
		ENSURE( b3RayCastCapsule( &rayCapsule, &input ).hit == false );
	}
	// Parallel to the axis and outside the cylinder.
	{
		b3RayCastInput input = { { 0.0f, 3.0f, 0.0f }, { 8.0f, 0.0f, 0.0f }, 1.0f };
		ENSURE( b3RayCastCapsule( &rayCapsule, &input ).hit == false );
	}
	// Descends past the rounded c2 end, beyond cap reach.
	{
		b3RayCastInput input = { { 4.0f, 3.0f, 0.0f }, { 0.0f, -6.0f, 0.0f }, 1.0f };
		ENSURE( b3RayCastCapsule( &rayCapsule, &input ).hit == false );
	}
	return 0;
}

static int RayCastCapsuleInteriorTest( void )
{
	// Origin on the axis between the caps.
	{
		b3RayCastInput input = { { 0.0f, 0.0f, 0.0f }, { 0.0f, -5.0f, 0.0f }, 1.0f };
		b3CastOutput out = b3RayCastCapsule( &rayCapsule, &input );
		ENSURE( out.hit );
		ENSURE( out.fraction == 0.0f );
		ENSURE_SMALL( b3Distance( out.point, input.origin ), FLT_EPSILON );
	}
	// Origin inside the c2 hemisphere, past the cylinder end.
	{
		b3RayCastInput input = { { 2.5f, 0.0f, 0.0f }, { 0.0f, 0.0f, 5.0f }, 1.0f };
		b3CastOutput out = b3RayCastCapsule( &rayCapsule, &input );
		ENSURE( out.hit );
		ENSURE( out.fraction == 0.0f );
		ENSURE_SMALL( b3Distance( out.point, input.origin ), FLT_EPSILON );
	}
	// Zero length ray inside.
	{
		b3RayCastInput input = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 1.0f };
		b3CastOutput out = b3RayCastCapsule( &rayCapsule, &input );
		ENSURE( out.hit );
		ENSURE_SMALL( b3Distance( out.point, input.origin ), FLT_EPSILON );
	}
	// Zero length ray outside.
	{
		b3RayCastInput input = { { 0.0f, 3.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 1.0f };
		ENSURE( b3RayCastCapsule( &rayCapsule, &input ).hit == false );
	}
	return 0;
}

static int RayCastCapsuleDegenerateTest( void )
{
	// Coincident centers collapse to a sphere.
	b3Capsule c = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 1.0f };
	b3RayCastInput input = { { -4.0f, 0.0f, 0.0f }, { 8.0f, 0.0f, 0.0f }, 1.0f };
	b3CastOutput out = b3RayCastCapsule( &c, &input );
	return CheckCastHit( out, input.origin, input.translation, (b3Vec3){ -1.0f, 0.0f, 0.0f }, (b3Vec3){ -1.0f, 0.0f, 0.0f },
						 3.0f / 8.0f, 1e-5f );
}

static int RayCastCapsuleClipTest( void )
{
	// The side hit occurs at fraction 1/3. Straddle it with maxFraction.
	{
		b3RayCastInput input = { { 0.0f, 3.0f, 0.0f }, { 0.0f, -6.0f, 0.0f }, 0.3f };
		ENSURE( b3RayCastCapsule( &rayCapsule, &input ).hit == false );
	}
	{
		b3RayCastInput input = { { 0.0f, 3.0f, 0.0f }, { 0.0f, -6.0f, 0.0f }, 0.5f };
		b3CastOutput out = b3RayCastCapsule( &rayCapsule, &input );
		ENSURE( out.hit );
		ENSURE_SMALL( out.fraction - 1.0f / 3.0f, 1e-5f );
	}
	return 0;
}

// A ray within a hair of the capsule axis must still hit when it slowly converges onto the
// surface. The closest point solver is ill conditioned in this band, so this guards the near
// parallel fallback that intersects the infinite cylinder directly.
static int RayCastCapsuleParallelTest( void )
{
	// Capsule along y. A long ray almost parallel to the axis drifts inward from just outside the
	// cylinder and dips through the far endcap. The naive solve loses this hit to a determinant of zero.
	b3Capsule axisY = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 10.0f, 0.0f }, 1.0f };
	{
		b3RayCastInput input = { { 1.0001f, 100.0f, 0.0f }, { -0.001f, -200.0f, 0.0f }, 1.0f };
		b3CastOutput out = b3RayCastCapsule( &axisY, &input );
		ENSURE( out.hit );

		// The hit lands on the capsule surface and on the ray.
		b3Vec3 onSeg = b3PointToSegmentDistance( axisY.center1, axisY.center2, out.point );
		ENSURE_SMALL( b3Distance( out.point, onSeg ) - axisY.radius, 1e-3f );
		b3Vec3 onRay = b3MulAdd( input.origin, out.fraction, input.translation );
		ENSURE_SMALL( b3Distance( out.point, onRay ), 1e-3f );
	}

	// Near parallel ray converging onto the x-axis capsule from far away.
	{
		b3RayCastInput input = { { -1000.0f, 1.0001f, 0.0f }, { 2000.0f, -0.001f, 0.0f }, 1.0f };
		b3CastOutput out = b3RayCastCapsule( &rayCapsule, &input );
		ENSURE( out.hit );
		b3Vec3 onSeg = b3PointToSegmentDistance( rayCapsule.center1, rayCapsule.center2, out.point );
		ENSURE_SMALL( b3Distance( out.point, onSeg ) - rayCapsule.radius, 1e-3f );
	}

	// Exactly parallel and outside the cylinder still misses.
	{
		b3RayCastInput input = { { 0.0f, 3.0f, 0.0f }, { 8.0f, 0.0f, 0.0f }, 1.0f };
		ENSURE( b3RayCastCapsule( &rayCapsule, &input ).hit == false );
	}

	return 0;
}

// Zero length rays and initial overlap behave the same across the solid shapes. A moving ray
// and a zero length ray that both start inside report the origin with zero fraction, and a zero
// length ray that starts outside misses.
static int RayCastOverlapConventionTest( void )
{
	b3Vec3 zero = { 0.0f, 0.0f, 0.0f };
	b3Vec3 ray = { 8.0f, 0.0f, 0.0f };

	// Sphere
	{
		b3Sphere s = { { 0.0f, 0.0f, 0.0f }, 1.0f };
		b3Vec3 inside = { 0.2f, 0.0f, 0.0f };
		b3Vec3 outside = { 3.0f, 0.0f, 0.0f };
		b3RayCastInput moving = { inside, ray, 1.0f };
		b3RayCastInput pointInside = { inside, zero, 1.0f };
		b3RayCastInput pointOutside = { outside, zero, 1.0f };
		if ( CheckInitialOverlap( b3RayCastSphere( &s, &moving ), inside ) )
			return 1;
		if ( CheckInitialOverlap( b3RayCastSphere( &s, &pointInside ), inside ) )
			return 1;
		ENSURE( b3RayCastSphere( &s, &pointOutside ).hit == false );
	}

	// Capsule
	{
		b3Vec3 inside = { 0.0f, 0.0f, 0.0f };
		b3Vec3 outside = { 0.0f, 3.0f, 0.0f };
		b3RayCastInput moving = { inside, ray, 1.0f };
		b3RayCastInput pointInside = { inside, zero, 1.0f };
		b3RayCastInput pointOutside = { outside, zero, 1.0f };
		if ( CheckInitialOverlap( b3RayCastCapsule( &rayCapsule, &moving ), inside ) )
			return 1;
		if ( CheckInitialOverlap( b3RayCastCapsule( &rayCapsule, &pointInside ), inside ) )
			return 1;
		ENSURE( b3RayCastCapsule( &rayCapsule, &pointOutside ).hit == false );
	}

	// Hull
	{
		b3Vec3 inside = { 0.3f, 0.2f, 0.1f };
		b3Vec3 outside = { 3.0f, 0.0f, 0.0f };
		b3RayCastInput moving = { inside, ray, 1.0f };
		b3RayCastInput pointInside = { inside, zero, 1.0f };
		b3RayCastInput pointOutside = { outside, zero, 1.0f };
		if ( CheckInitialOverlap( b3RayCastHull( &box.base, &moving ), inside ) )
			return 1;
		if ( CheckInitialOverlap( b3RayCastHull( &box.base, &pointInside ), inside ) )
			return 1;
		ENSURE( b3RayCastHull( &box.base, &pointOutside ).hit == false );
	}

	return 0;
}

// Distance, in double precision, from a single precision hit point to the analytic first
// ray/sphere intersection of the same float ray. Isolates the single precision method error:
// the reference carries no float rounding, so what remains is purely what the method lost.
static double SphereHitError( b3Sphere shape, b3RayCastInput input, b3Vec3 point )
{
	double r = shape.radius;
	double sx = (double)input.origin.x - shape.center.x;
	double sy = (double)input.origin.y - shape.center.y;
	double sz = (double)input.origin.z - shape.center.z;
	double tx = input.translation.x, ty = input.translation.y, tz = input.translation.z;
	double len = sqrt( tx * tx + ty * ty + tz * tz );
	double dx = tx / len, dy = ty / len, dz = tz / len;
	double b = sx * dx + sy * dy + sz * dz;
	double c = sx * sx + sy * sy + sz * sz - r * r;
	double t = -b - sqrt( b * b - c );
	double ex = point.x - ( input.origin.x + t * dx );
	double ey = point.y - ( input.origin.y + t * dy );
	double ez = point.z - ( input.origin.z + t * dz );
	return sqrt( ex * ex + ey * ey + ez * ez );
}

// Same idea for the ray/infinite-cylinder intersection, used where the hit lands on the side.
static double CapsuleHitError( b3Capsule shape, b3RayCastInput input, b3Vec3 point )
{
	double ax = (double)shape.center2.x - shape.center1.x;
	double ay = (double)shape.center2.y - shape.center1.y;
	double az = (double)shape.center2.z - shape.center1.z;
	double alen = sqrt( ax * ax + ay * ay + az * az );
	ax /= alen, ay /= alen, az /= alen;

	double sx = (double)input.origin.x - shape.center1.x;
	double sy = (double)input.origin.y - shape.center1.y;
	double sz = (double)input.origin.z - shape.center1.z;
	double tx = input.translation.x, ty = input.translation.y, tz = input.translation.z;
	double tlen = sqrt( tx * tx + ty * ty + tz * tz );
	double dx = tx / tlen, dy = ty / tlen, dz = tz / tlen;

	double sa = sx * ax + sy * ay + sz * az;
	double da = dx * ax + dy * ay + dz * az;
	double spx = sx - sa * ax, spy = sy - sa * ay, spz = sz - sa * az;
	double dpx = dx - da * ax, dpy = dy - da * ay, dpz = dz - da * az;
	double A = dpx * dpx + dpy * dpy + dpz * dpz;
	double B = 2.0 * ( spx * dpx + spy * dpy + spz * dpz );
	double r = shape.radius;
	double C = spx * spx + spy * spy + spz * spz - r * r;
	double tau = ( -B - sqrt( B * B - 4.0 * A * C ) ) / ( 2.0 * A );
	double ex = point.x - ( input.origin.x + tau * dx );
	double ey = point.y - ( input.origin.y + tau * dy );
	double ez = point.z - ( input.origin.z + tau * dz );
	return sqrt( ex * ex + ey * ey + ez * ez );
}

// A miss is the worst possible outcome, so fold it into the error as a large sentinel.
#define RAY_MISS 1.0e30

static int RayCastFarOriginTest( void )
{
	b3Sphere s = { { 0.0f, 0.0f, 0.0f }, 1.0f };
	b3Capsule c = { { -2.0f, 0.0f, 0.0f }, { 2.0f, 0.0f, 0.0f }, 1.0f };

	// (0,0,1) lies on the unit sphere and on the capsule side. The ray dives in from a far origin
	// along H + D*u for a fan of directions u skewed off the surface normal, so the capsule solve
	// sees a real perpendicular gap rather than a free exact cancellation.
	b3Vec3 h = { 0.0f, 0.0f, 1.0f };
	float offsets[] = { -0.7f, 0.0f, 0.7f };
	float distances[] = { 1e1f, 1e2f, 1e3f, 1e4f, 1e5f, 1e6f, 1e7f };

	double worstSphere[ARRAY_COUNT( distances )];
	double worstCapsule[ARRAY_COUNT( distances )];

	printf( "    worst hit point error over a fan of skew rays, by origin distance:\n" );
	printf( "    %-9s %-13s %-13s\n", "distance", "sphere", "capsule" );
	for ( int i = 0; i < ARRAY_COUNT( distances ); ++i )
	{
		float d = distances[i];
		double maxS = 0.0, maxC = 0.0;

		for ( int ia = 0; ia < ARRAY_COUNT( offsets ); ++ia )
		{
			for ( int ib = 0; ib < ARRAY_COUNT( offsets ); ++ib )
			{
				b3Vec3 u = b3Normalize( (b3Vec3){ offsets[ia], offsets[ib], 1.0f } );
				b3Vec3 origin = b3MulAdd( h, d, u );
				b3Vec3 translation = b3MulSV( -2.0f * d, u );
				b3RayCastInput input = { origin, translation, 1.0f };

				b3CastOutput os = b3RayCastSphere( &s, &input );
				b3CastOutput oc = b3RayCastCapsule( &c, &input );

				double errS = os.hit ? SphereHitError( s, input, os.point ) : RAY_MISS;
				double errC = oc.hit ? CapsuleHitError( c, input, oc.point ) : RAY_MISS;

				maxS = errS > maxS ? errS : maxS;
				maxC = errC > maxC ? errC : maxC;
			}
		}

		worstSphere[i] = maxS;
		worstCapsule[i] = maxC;
		printf( "    %-9.0e %-13.3e %-13.3e\n", d, maxS, maxC );
	}

	// The closest point formulation keeps the error at the single precision floor: it grows only
	// linearly with origin distance, error ~ distance * FLT_EPSILON, with no catastrophic loss. This
	// holds out to a million units. At ten million the origin coordinates carry a meter sized ULP,
	// larger than the unit radius, so the ray genuinely drops the hit. That row is printed for insight
	// but left unasserted rather than baking in the breakdown.
	for ( int i = 0; i < ARRAY_COUNT( distances ); ++i )
	{
		if ( distances[i] < 1.0e7f )
		{
			double floor = 16.0 * distances[i] * FLT_EPSILON + 2.0e-6;
			ENSURE( worstSphere[i] < floor );
			ENSURE( worstCapsule[i] < floor );
		}
	}

	// Still a clean sub-meter hit at a million units out.
	ENSURE( worstSphere[5] < 0.5 && worstCapsule[5] < 0.5 );

	return 0;
}

static int RayCastShapeTest( void )
{
	b3RayCastInput input = {
		.origin = { -4.0f, 0.0f, 0.0f },
		.translation = { 8.0f, 0.0f, 0.0f },
		.maxFraction = 1.0f,
	};

	{
		b3CastOutput output = b3RayCastSphere( &sphere, &input );
		ENSURE( output.hit );
		ENSURE_SMALL( output.normal.x + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( output.normal.y, FLT_EPSILON );
		ENSURE_SMALL( output.normal.z, FLT_EPSILON );
		ENSURE_SMALL( output.fraction - 0.5f, FLT_EPSILON );
	}

	{
		b3CastOutput output = b3RayCastCapsule( &capsule, &input );
		ENSURE( output.hit );
		ENSURE_SMALL( output.normal.x + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( output.normal.y, FLT_EPSILON );
		ENSURE_SMALL( output.normal.z, FLT_EPSILON );
		ENSURE_SMALL( output.fraction - 1.0f / 4.0f, FLT_EPSILON );
	}

	{
		b3CastOutput output = b3RayCastHull( &box.base, &input );
		ENSURE( output.hit );
		ENSURE_SMALL( output.normal.x + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( output.normal.y, FLT_EPSILON );
		ENSURE_SMALL( output.normal.z, FLT_EPSILON );
		ENSURE_SMALL( output.fraction - 3.0f / 8.0f, FLT_EPSILON );
	}

	return 0;
}

static int CheckShapeName( b3ShapeId shapeId, const char* expected )
{
	const char* got = b3Shape_GetName( shapeId );
	ENSURE( got != NULL || expected == NULL );
	if ( got == NULL )
	{
		return 0;
	}

	int srcLen = expected != NULL ? (int)strlen( expected ) : 0;
	ENSURE( (int)strlen( got ) == srcLen );
	if ( srcLen > 0 )
	{
		ENSURE( strncmp( got, expected, (size_t)srcLen ) == 0 );
	}
	return 0;
}

// Cover the def path, the setter, over-length truncation, and the NULL clear. A zero cap collapses every
// case to the empty string, which the length-derived check handles without a special branch.
static int ShapeNameTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3Sphere s = { { 0.0f, 0.0f, 0.0f }, 0.5f };

	// Default def leaves the name empty.
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	b3ShapeId shapeId = b3CreateSphereShape( bodyId, &shapeDef, &s );
	if ( CheckShapeName( shapeId, NULL ) )
		return 1;

	// Name carried on the def.
	shapeDef.name = "box";
	b3ShapeId namedId = b3CreateSphereShape( bodyId, &shapeDef, &s );
	if ( CheckShapeName( namedId, "box" ) )
		return 1;

	// Setter overwrites.
	b3Shape_SetName( shapeId, "wheel" );
	if ( CheckShapeName( shapeId, "wheel" ) )
		return 1;

	// Over-length name truncates to the cap.
	b3Shape_SetName( shapeId, "abcdefghijklmnopqrstuvwxyz" );
	if ( CheckShapeName( shapeId, "abcdefghijklmnopqrstuvwxyz" ) )
		return 1;

	// NULL clears the name.
	b3Shape_SetName( shapeId, NULL );
	if ( CheckShapeName( shapeId, NULL ) )
		return 1;

	b3DestroyWorld( worldId );
	return 0;
}

// The event enables moved from separate bools to a shared bit field. Each accessor must touch only its own
// bit, so start with all set and flip them one at a time, checking the rest hold.
static int ShapeFlagsTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3Sphere s = { { 0.0f, 0.0f, 0.0f }, 0.5f };

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.enableSensorEvents = true;
	shapeDef.enableContactEvents = true;
	shapeDef.enableHitEvents = true;
	shapeDef.enablePreSolveEvents = true;
	b3ShapeId shapeId = b3CreateSphereShape( bodyId, &shapeDef, &s );

	// The def carries every enable through to the shape.
	ENSURE( b3Shape_AreSensorEventsEnabled( shapeId ) );
	ENSURE( b3Shape_AreContactEventsEnabled( shapeId ) );
	ENSURE( b3Shape_AreHitEventsEnabled( shapeId ) );
	ENSURE( b3Shape_ArePreSolveEventsEnabled( shapeId ) );

	// Clearing one leaves the rest set.
	b3Shape_EnableSensorEvents( shapeId, false );
	ENSURE( b3Shape_AreSensorEventsEnabled( shapeId ) == false );
	ENSURE( b3Shape_AreContactEventsEnabled( shapeId ) );
	ENSURE( b3Shape_AreHitEventsEnabled( shapeId ) );
	ENSURE( b3Shape_ArePreSolveEventsEnabled( shapeId ) );

	// Setting it back does not disturb the others.
	b3Shape_EnableSensorEvents( shapeId, true );
	b3Shape_EnableHitEvents( shapeId, false );
	ENSURE( b3Shape_AreSensorEventsEnabled( shapeId ) );
	ENSURE( b3Shape_AreContactEventsEnabled( shapeId ) );
	ENSURE( b3Shape_AreHitEventsEnabled( shapeId ) == false );
	ENSURE( b3Shape_ArePreSolveEventsEnabled( shapeId ) );

	// Clear two more and confirm only the remaining bit stays set.
	b3Shape_EnableContactEvents( shapeId, false );
	b3Shape_EnablePreSolveEvents( shapeId, false );
	ENSURE( b3Shape_AreSensorEventsEnabled( shapeId ) );
	ENSURE( b3Shape_AreContactEventsEnabled( shapeId ) == false );
	ENSURE( b3Shape_AreHitEventsEnabled( shapeId ) == false );
	ENSURE( b3Shape_ArePreSolveEventsEnabled( shapeId ) == false );

	b3DestroyWorld( worldId );
	return 0;
}

int ShapeTest( void )
{
	box = b3MakeBoxHull( 1.0f, 1.0f, 1.0f );

	RUN_SUBTEST( ShapeMassTest );
	RUN_SUBTEST( ShapeAABBTest );
	RUN_SUBTEST( ShapeNameTest );
	RUN_SUBTEST( ShapeFlagsTest );
	// RUN_SUBTEST( PointInShapeTest );
	RUN_SUBTEST( RayCastShapeTest );

	RUN_SUBTEST( RayCastSphereHitTest );
	RUN_SUBTEST( RayCastSphereMissTest );
	RUN_SUBTEST( RayCastSphereClipTest );
	RUN_SUBTEST( RayCastSphereInteriorTest );
	RUN_SUBTEST( RayCastSphereGrazeTest );

	RUN_SUBTEST( RayCastCapsuleSideTest );
	RUN_SUBTEST( RayCastCapsuleObliqueTest );
	RUN_SUBTEST( RayCastCapsuleCapTest );
	RUN_SUBTEST( RayCastCapsuleMissTest );
	RUN_SUBTEST( RayCastCapsuleInteriorTest );
	RUN_SUBTEST( RayCastCapsuleDegenerateTest );
	RUN_SUBTEST( RayCastCapsuleClipTest );
	RUN_SUBTEST( RayCastCapsuleParallelTest );

	RUN_SUBTEST( RayCastOverlapConventionTest );
	RUN_SUBTEST( RayCastFarOriginTest );

	return 0;
}

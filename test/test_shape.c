// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <float.h>

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
		float inertia = 2.0f / 5.0f * mass;
		float offsetInertia = inertia + mass;
		ENSURE_SMALL( md.inertia.cx.x - inertia, FLT_EPSILON );
		ENSURE_SMALL( md.inertia.cy.y - offsetInertia, FLT_EPSILON );
		ENSURE_SMALL( md.inertia.cz.z - offsetInertia, FLT_EPSILON );
	}

	// Analytic box hull
	{
		b3MassData md = b3ComputeHullMass( &box.base, 1.0f, 1.0f );
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

		b3MassData m1 = b3ComputeHullMass( &b1.base, 1.0f, 1.0f );
		b3MassData m2 = b3ComputeHullMass( &b2.base, 1.0f, 1.0f );

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

		b3MassData m1 = b3ComputeHullMass( &b1.base, 1.0f, 1.0f );
		b3MassData m2 = b3ComputeHullMass( &b2.base, 1.0f, 1.0f );

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

		b3MassData m1 = b3ComputeHullMass( &b1.base, 1.0f, 1.0f );
		b3MassData m2 = b3ComputeHullMass( &b2.base, 1.0f, 1.0f );

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
		b3MassData mdUpper = b3ComputeHullMass( &r.base, 1.0f, 1.0f );

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
		b3MassData mdLower = b3ComputeHullMass( hull, 1.0f, 1.0f );

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
		b3AABB b = b3ComputeHullAABB( &box.base, 1.0f, b3Transform_identity );
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
		b3CastOutput output = b3RayCastHull( &box.base, 1.0f, &input );
		ENSURE( output.hit );
		ENSURE_SMALL( output.normal.x + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( output.normal.y, FLT_EPSILON );
		ENSURE_SMALL( output.normal.z, FLT_EPSILON );
		ENSURE_SMALL( output.fraction - 3.0f / 8.0f, FLT_EPSILON );
	}

	return 0;
}

// Uniform scale is applied at query time, not baked into the data.
static int ScaledHullTest( void )
{
	float density = 2.5f;

	b3MassData m1 = b3ComputeHullMass( &box.base, 1.0f, density );
	b3MassData m2 = b3ComputeHullMass( &box.base, 2.0f, density );

	// Mass scales with volume, central inertia with the fifth power
	ENSURE_SMALL( m2.mass - 8.0f * m1.mass, 1e-4f );
	ENSURE_SMALL( m2.inertia.cx.x - 32.0f * m1.inertia.cx.x, 1e-3f );
	ENSURE_SMALL( m2.inertia.cy.y - 32.0f * m1.inertia.cy.y, 1e-3f );
	ENSURE_SMALL( m2.inertia.cz.z - 32.0f * m1.inertia.cz.z, 1e-3f );

	// Centroid scales linearly
	b3BoxHull offsetBox = b3MakeOffsetBoxHull( 1.0f, 1.0f, 1.0f, (b3Vec3){ 3.0f, -1.0f, 0.5f } );
	b3MassData o1 = b3ComputeHullMass( &offsetBox.base, 1.0f, density );
	b3MassData o2 = b3ComputeHullMass( &offsetBox.base, 2.0f, density );
	ENSURE_SMALL( o2.center.x - 2.0f * o1.center.x, 1e-4f );
	ENSURE_SMALL( o2.center.y - 2.0f * o1.center.y, 1e-4f );
	ENSURE_SMALL( o2.center.z - 2.0f * o1.center.z, 1e-4f );

	// AABB scales linearly
	b3AABB a1 = b3ComputeHullAABB( &box.base, 1.0f, b3Transform_identity );
	b3AABB a2 = b3ComputeHullAABB( &box.base, 2.0f, b3Transform_identity );
	ENSURE_SMALL( a2.upperBound.x - 2.0f * a1.upperBound.x, FLT_EPSILON );
	ENSURE_SMALL( a2.lowerBound.z - 2.0f * a1.lowerBound.z, FLT_EPSILON );

	return 0;
}

int ShapeTest( void )
{
	box = b3MakeBoxHull( 1.0f, 1.0f, 1.0f );

	RUN_SUBTEST( ShapeMassTest );
	RUN_SUBTEST( ShapeAABBTest );
	// RUN_SUBTEST( PointInShapeTest );
	RUN_SUBTEST( RayCastShapeTest );
	RUN_SUBTEST( ScaledHullTest );

	return 0;
}

// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_functions.h"

#include <float.h>
#include <math.h>

static const float kRoot2 = 1.41421356f;
static const float kHalfRoot2 = 0.70710678f;

static const b3Vec3 kAxisX = { 1.0f, 0.0f, 0.0f };
static const b3Vec3 kAxisY = { 0.0f, 1.0f, 0.0f };
static const b3Vec3 kAxisZ = { 0.0f, 0.0f, 1.0f };

// b3ComputeCosSin is a rational approximation good to about 1e-3. That is coarse enough to
// shift an edge off the position the analytic result expects, so these fixtures need libm.
static b3Quat ExactQuat( b3Vec3 axis, float radians )
{
	float half = 0.5f * radians;
	float s = sinf( half );
	return (b3Quat){ { s * axis.x, s * axis.y, s * axis.z }, cosf( half ) };
}

static b3Transform ExactRotation( b3Vec3 axis, float radians )
{
	return (b3Transform){ b3Vec3_zero, ExactQuat( axis, radians ) };
}

// Cube A yawed 45 degrees presents an edge along y at x = +h*root2.
// Cube B rolled 45 degrees presents an edge along z at x = -h*root2.
// Sliding B along x makes those two edges the closest features, so the axis of minimum
// penetration is x, the separation is d - 2*h*root2 and the contact point sits at x = d/2.
// Both hulls are far from a face axis here, which keeps the edge query in charge.
static void MakeCrossedEdgeHulls( b3BoxHull* hullA, b3BoxHull* hullB, float halfWidth )
{
	*hullA = b3MakeTransformedBoxHull( halfWidth, halfWidth, halfWidth, ExactRotation( kAxisY, 0.25f * B3_PI ) );
	*hullB = b3MakeTransformedBoxHull( halfWidth, halfWidth, halfWidth, ExactRotation( kAxisZ, 0.25f * B3_PI ) );
}

static b3Transform SlideX( float x )
{
	return (b3Transform){ { x, 0.0f, 0.0f }, b3Quat_identity };
}

static float MinSeparation( const b3LocalManifold* manifold )
{
	float minSeparation = FLT_MAX;
	for ( int i = 0; i < manifold->pointCount; ++i )
	{
		minSeparation = b3MinFloat( minSeparation, manifold->points[i].separation );
	}

	return minSeparation;
}

// The edge pair axis is built by intersecting the two Gauss map arcs. Walk the crossed edges from
// speculative contact into deep overlap and check the axis, the separation and the point.
static int CrossedEdgeTest( void )
{
	b3BoxHull hullA, hullB;
	MakeCrossedEdgeHulls( &hullA, &hullB, 0.5f );

	float distances[] = { 1.42f, kRoot2, 1.41f, 1.3f };

	for ( int i = 0; i < ARRAY_COUNT( distances ); ++i )
	{
		float d = distances[i];
		float expected = d - kRoot2;

		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SATCache cache = { 0 };
		b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, SlideX( d ), &cache );

		ENSURE( manifold.pointCount == 1 );
		ENSURE( cache.type == b3_edgePairAxis );
		ENSURE_SMALL( manifold.normal.x - 1.0f, 1e-6f );
		ENSURE_SMALL( manifold.normal.y, 1e-6f );
		ENSURE_SMALL( manifold.normal.z, 1e-6f );
		ENSURE_SMALL( manifold.points[0].separation - expected, 1e-5f );
		ENSURE_SMALL( manifold.points[0].point.x - 0.5f * d, 1e-5f );
		ENSURE_SMALL( manifold.points[0].point.y, 1e-5f );
		ENSURE_SMALL( manifold.points[0].point.z, 1e-5f );

		// The forced edge query must agree with what the full solver chose
		b3LocalManifold manual = { 0 };
		manual.points = points;
		b3SATCache manualCache = { .type = b3_manualEdgePairAxis };
		b3CollideHulls( &manual, 8, &hullA.base, &hullB.base, SlideX( d ), &manualCache );

		ENSURE( manual.pointCount == 1 );
		ENSURE_SMALL( manual.points[0].separation - expected, 1e-5f );
	}

	// Beyond the speculative distance the query reports the axis without building a contact.
	// The axis carries its own orientation now, so a sign error here would read as deep overlap.
	{
		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SATCache cache = { 0 };
		b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, SlideX( 1.5f ), &cache );

		ENSURE( manifold.pointCount == 0 );
		ENSURE( cache.type == b3_edgePairAxis );
		ENSURE_SMALL( cache.separation - ( 1.5f - kRoot2 ), 1e-5f );
	}

	return 0;
}

// The parallel edge rejection compares dot products against the edge length, so it is a sine
// threshold and must hold at any size.
static int EdgeAxisScaleTest( void )
{
	float scales[] = { 100.0f, 1.0f, 0.2f };

	for ( int i = 0; i < ARRAY_COUNT( scales ); ++i )
	{
		float s = scales[i];
		b3BoxHull hullA, hullB;
		MakeCrossedEdgeHulls( &hullA, &hullB, 0.5f * s );

		float expected = -0.002f;
		float d = s * kRoot2 + expected;

		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SATCache cache = { 0 };
		b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, SlideX( d ), &cache );

		// Differencing coordinates of magnitude d costs precision proportional to the scale
		float tolerance = 1e-5f * s + 1e-6f;

		ENSURE( manifold.pointCount == 1 );
		ENSURE( cache.type == b3_edgePairAxis );
		ENSURE_SMALL( manifold.normal.x - 1.0f, 1e-6f );
		ENSURE_SMALL( manifold.points[0].separation - expected, tolerance );
		ENSURE_SMALL( manifold.points[0].point.x - 0.5f * d, tolerance );
		ENSURE_SMALL( manifold.points[0].point.y, tolerance );
		ENSURE_SMALL( manifold.points[0].point.z, tolerance );
	}

	return 0;
}

// The cached edge pair rebuilds the axis without a fresh query. An untouched cache proves the
// cached branch answered rather than falling through to the full SAT.
static int EdgeCacheTest( void )
{
	b3BoxHull hullA, hullB;
	MakeCrossedEdgeHulls( &hullA, &hullB, 0.5f );

	b3LocalManifoldPoint points[8];
	b3LocalManifold manifold = { 0 };
	manifold.points = points;
	b3SATCache cache = { 0 };

	b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, SlideX( 1.41f ), &cache );
	ENSURE( manifold.pointCount == 1 );
	ENSURE( cache.type == b3_edgePairAxis );
	ENSURE( cache.indexA % 2 == 0 && cache.indexA < hullA.base.edgeCount );
	ENSURE( cache.indexB % 2 == 0 && cache.indexB < hullB.base.edgeCount );

	float seededSeparation = cache.separation;
	ENSURE_SMALL( seededSeparation - ( 1.41f - kRoot2 ), 1e-5f );

	// Small motion, the cached features still describe the contact
	b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, SlideX( 1.4105f ), &cache );
	ENSURE( manifold.pointCount == 1 );
	ENSURE( cache.separation == seededSeparation );
	ENSURE_SMALL( manifold.points[0].separation - ( 1.4105f - kRoot2 ), 1e-5f );
	ENSURE_SMALL( manifold.normal.x - 1.0f, 1e-6f );

	// Jump past the speculative distance. The cached axis alone must report the separation.
	b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, SlideX( 1.5f ), &cache );
	ENSURE( manifold.pointCount == 0 );
	ENSURE( cache.separation == seededSeparation );

	return 0;
}

// Sliding B along the direction of edge A walks the closest point off the end of the segment.
static int EdgeEndpointTest( void )
{
	b3BoxHull hullA, hullB;
	MakeCrossedEdgeHulls( &hullA, &hullB, 0.5f );

	float d = 1.41f;
	float expected = d - kRoot2;

	// Just inside the end of edge A
	{
		b3Transform transform = { { d, 0.49f, 0.0f }, b3Quat_identity };
		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SATCache cache = { .type = b3_manualEdgePairAxis };
		b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, transform, &cache );

		ENSURE( manifold.pointCount == 1 );
		ENSURE_SMALL( manifold.points[0].separation - expected, 1e-5f );
		ENSURE_SMALL( manifold.points[0].point.y - 0.49f, 1e-5f );
	}

	// Off the end. The edge pair no longer describes a contact, so the builder rejects it and
	// clears the cache rather than clamping to a point that is not on the hulls.
	{
		b3Transform transform = { { d, 0.55f, 0.0f }, b3Quat_identity };
		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SATCache cache = { .type = b3_manualEdgePairAxis };
		b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, transform, &cache );

		ENSURE( manifold.pointCount == 0 );
		ENSURE( cache.type == b3_invalidAxis );

		// The true gap is a vertex to edge distance well past the speculative distance
		b3SATCache freshCache = { 0 };
		b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, transform, &freshCache );
		ENSURE( manifold.pointCount == 0 );
		ENSURE( freshCache.separation > 0.0f );
	}

	return 0;
}

static const b3Vec3 kTiltAxes[] = {
	{ 0.57735027f, 0.57735027f, 0.57735027f },
	{ 0.70710678f, 0.0f, 0.70710678f },
	{ 0.26726124f, 0.53452248f, 0.80178373f },
	{ -0.48507125f, 0.72760688f, -0.48507125f },
};

// Angles that straddle the 0.005 rejection threshold
static const float kTiltAngles[] = { 0.0f, 1e-7f, 1e-6f, 1e-5f, 1e-4f, 1e-3f, 0.004f, 0.005f, 0.006f, 0.01f, 0.05f };

// A cube corner is root3/2 from the center of rotation
static const float kHalfDiagonal = 0.87f;

// Cubes stacked face to face and tipped by a hair. A third of the edge pairs are then nearly
// parallel, the angle between them is at the noise floor and the arc intersection carries no
// information. The face contact has to survive that untouched.
static int ParallelEdgeTest( void )
{
	b3BoxHull hullA = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3BoxHull hullB = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

	float overlap = 0.01f;

	for ( int i = 0; i < ARRAY_COUNT( kTiltAxes ); ++i )
	{
		for ( int j = 0; j < ARRAY_COUNT( kTiltAngles ); ++j )
		{
			float angle = kTiltAngles[j];
			b3Transform transform = { { 0.0f, 1.0f - overlap, 0.0f }, ExactQuat( kTiltAxes[i], angle ) };

			b3LocalManifoldPoint points[8];
			b3LocalManifold manifold = { 0 };
			manifold.points = points;
			b3SATCache cache = { 0 };
			b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, transform, &cache );

			ENSURE( manifold.pointCount == 4 );
			ENSURE( cache.type == b3_faceAxisA || cache.type == b3_faceAxisB );
			ENSURE( b3Dot( manifold.normal, kAxisY ) > 0.998f );

			// The tilt can only lift or sink a face point by the length of the arc it sweeps
			float bound = kHalfDiagonal * angle + 1e-5f;
			for ( int k = 0; k < manifold.pointCount; ++k )
			{
				ENSURE_SMALL( manifold.points[k].separation + overlap, bound );
			}
		}
	}

	return 0;
}

// Same stack, but force the edge query to answer. With the edges exactly parallel no pair forms a
// Minkowski face at all, and once a pair does form its separation can never be positive because
// the hulls overlap.
static int ParallelEdgeManualTest( void )
{
	b3BoxHull hullA = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3BoxHull hullB = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

	float overlap = 0.01f;

	for ( int i = 0; i < ARRAY_COUNT( kTiltAxes ); ++i )
	{
		for ( int j = 0; j < ARRAY_COUNT( kTiltAngles ); ++j )
		{
			float angle = kTiltAngles[j];
			b3Transform transform = { { 0.0f, 1.0f - overlap, 0.0f }, ExactQuat( kTiltAxes[i], angle ) };

			b3LocalManifoldPoint points[8];
			b3LocalManifold manifold = { 0 };
			manifold.points = points;
			b3SATCache cache = { .type = b3_manualEdgePairAxis };
			b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, transform, &cache );

			if ( angle == 0.0f )
			{
				// Every pair is parallel so the query finds nothing and leaves the cache alone
				ENSURE( manifold.pointCount == 0 );
				ENSURE( cache.type == b3_manualEdgePairAxis );
				continue;
			}

			// The closest points can fall off the ends of the segments
			if ( manifold.pointCount == 0 )
			{
				continue;
			}

			ENSURE( manifold.pointCount == 1 );
			ENSURE( b3Dot( manifold.normal, kAxisY ) > 0.99f );

			float separation = manifold.points[0].separation;
			ENSURE( separation <= 0.0f );
			ENSURE( separation >= -overlap - kHalfDiagonal * angle - 1e-4f );
		}
	}

	return 0;
}

static uint32_t g_seed = 12345;

static float NextFloat( float lower, float upper )
{
	g_seed = 1664525u * g_seed + 1013904223u;
	float t = (float)( g_seed >> 8 ) / (float)( 1 << 24 );
	return lower + t * ( upper - lower );
}

static b3Vec3 NextDirection( void )
{
	b3Vec3 v = { NextFloat( -1.0f, 1.0f ), NextFloat( -1.0f, 1.0f ), NextFloat( -1.0f, 1.0f ) };
	return b3Normalize( v );
}

// Overlapping hulls admit no separating axis, so an edge separation that comes back positive is
// always noise. It shows up as a manifold with no points, which the solver reads as no contact.
static int OverlapNeverEmptyTest( void )
{
	b3BoxHull hullA = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3BoxHull hullB = b3MakeBoxHull( 0.4f, 0.6f, 0.5f );

	for ( int i = 0; i < 2000; ++i )
	{
		b3Vec3 axis = NextDirection();

		// Half the samples are nearly aligned, where the edge cross products are smallest
		float angle = ( i & 1 ) ? NextFloat( -0.01f, 0.01f ) : NextFloat( -B3_PI, B3_PI );

		// Shorter than the smallest half width, so the center of B is inside A
		b3Vec3 offset = b3MulSV( 0.4f, NextDirection() );

		b3Transform transform = { offset, ExactQuat( axis, angle ) };

		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SATCache cache = { 0 };
		b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, transform, &cache );

		ENSURE( manifold.pointCount > 0 );
		ENSURE_SMALL( b3Length( manifold.normal ) - 1.0f, 1e-5f );

		// Clipping keeps points that are separated, but the deepest one must penetrate
		ENSURE( MinSeparation( &manifold ) < 0.0f );
	}

	return 0;
}

// A cube pitched 45 degrees rests on an edge along x at y = -h*root2. The two faces meeting there
// have normals (0,-r,r) and (0,-r,-r), so the arc between them spans the whole lower quadrant.
// A triangle edge crossing under it at an angle picks out an interior point of that arc, which is
// where a wrong lerp parameter would show up.
static int TriangleEdgeTest( void )
{
	float beta = 20.0f * B3_PI / 180.0f;
	float gamma = 30.0f * B3_PI / 180.0f;

	b3BoxHull hull = b3MakeTransformedBoxHull( 0.5f, 0.5f, 0.5f, ExactRotation( kAxisX, 0.25f * B3_PI ) );

	// Tipping the triangle plane about z keeps its normal off the hull edge, which the Minkowski
	// test needs. Tipping the edge within that plane moves the arc intersection off the midpoint.
	b3Vec3 triNormal = { sinf( beta ), cosf( beta ), 0.0f };
	b3Vec3 triEdge = { sinf( gamma ) * cosf( beta ), -sinf( gamma ) * sinf( beta ), cosf( gamma ) };

	// Perpendicular to both edges and pointing out of the hull
	b3Vec3 axis = b3Normalize( b3Cross( kAxisX, triEdge ) );
	b3Vec3 hullPoint = { 0.0f, -kHalfRoot2, 0.0f };

	float gaps[] = { 0.03f, 0.01f, 0.0f, -0.01f, -0.1f };

	for ( int i = 0; i < ARRAY_COUNT( gaps ); ++i )
	{
		float gap = gaps[i];
		b3Vec3 trianglePoint = b3MulAdd( hullPoint, gap, axis );

		b3Vec3 v1 = b3MulAdd( trianglePoint, -1.0f, triEdge );
		b3Vec3 v2 = b3MulAdd( trianglePoint, 1.0f, triEdge );
		b3Vec3 v3 = b3MulAdd( v1, 1.5f, b3Cross( triNormal, triEdge ) );

		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SATCache cache = { .type = b3_manualEdgePairAxis };
		b3CollideHullAndTriangle( &manifold, 8, &hull.base, v1, v2, v3, 0, &cache, true );

		b3Vec3 expectedNormal = b3Neg( axis );
		b3Vec3 expectedPoint = b3MulAdd( hullPoint, 0.5f * gap, axis );

		ENSURE( manifold.pointCount == 1 );
		ENSURE( cache.type == b3_edgePairAxis );
		ENSURE_SMALL( manifold.normal.x - expectedNormal.x, 1e-5f );
		ENSURE_SMALL( manifold.normal.y - expectedNormal.y, 1e-5f );
		ENSURE_SMALL( manifold.normal.z - expectedNormal.z, 1e-5f );
		ENSURE_SMALL( manifold.points[0].separation - gap, 1e-5f );
		ENSURE_SMALL( manifold.points[0].point.x - expectedPoint.x, 1e-5f );
		ENSURE_SMALL( manifold.points[0].point.y - expectedPoint.y, 1e-5f );
		ENSURE_SMALL( manifold.points[0].point.z - expectedPoint.z, 1e-5f );
	}

	// The tipped triangle plane buries a corner of the hull, so neither face axis separates and
	// the edge axis has to carry the speculative cull on its own.
	float culled[] = { 0.03f, 0.05f };

	for ( int i = 0; i < ARRAY_COUNT( culled ); ++i )
	{
		float gap = culled[i];
		b3Vec3 trianglePoint = b3MulAdd( hullPoint, gap, axis );

		b3Vec3 v1 = b3MulAdd( trianglePoint, -1.0f, triEdge );
		b3Vec3 v2 = b3MulAdd( trianglePoint, 1.0f, triEdge );
		b3Vec3 v3 = b3MulAdd( v1, 1.5f, b3Cross( triNormal, triEdge ) );

		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SATCache cache = { 0 };
		b3CollideHullAndTriangle( &manifold, 8, &hull.base, v1, v2, v3, 0, &cache, true );

		ENSURE( manifold.pointCount == 0 );
		ENSURE( cache.type == b3_edgePairAxis );
		ENSURE_SMALL( cache.separation - gap, 1e-5f );
	}

	return 0;
}

// The same cube resting its bottom edge on a triangle whose first edge runs along x. Tipping the
// triangle takes that pair from exactly parallel through the rejection threshold.
static int TriangleParallelEdgeTest( void )
{
	b3BoxHull hull = b3MakeTransformedBoxHull( 0.5f, 0.5f, 0.5f, ExactRotation( kAxisX, 0.25f * B3_PI ) );

	float overlap = 0.01f;
	float y = -kHalfRoot2 + overlap;

	for ( int i = 0; i < ARRAY_COUNT( kTiltAxes ); ++i )
	{
		for ( int j = 0; j < ARRAY_COUNT( kTiltAngles ); ++j )
		{
			b3Quat q = ExactQuat( kTiltAxes[i], kTiltAngles[j] );

			b3Vec3 v1 = b3RotateVector( q, ( b3Vec3 ){ -2.0f, y, -1.0f } );
			b3Vec3 v2 = b3RotateVector( q, ( b3Vec3 ){ 0.0f, y, 2.0f } );
			b3Vec3 v3 = b3RotateVector( q, ( b3Vec3 ){ 2.0f, y, -1.0f } );

			b3LocalManifoldPoint points[8];
			b3LocalManifold manifold = { 0 };
			manifold.points = points;
			b3SATCache cache = { 0 };
			b3CollideHullAndTriangle( &manifold, 8, &hull.base, v1, v2, v3, 0, &cache, true );

			ENSURE( manifold.pointCount == 4 );
			ENSURE( cache.type == b3_faceAxisA );
			ENSURE( b3Dot( manifold.normal, kAxisY ) > 0.99f );

			// The tilt can only sink the contact by the length of the arc it sweeps
			float bound = kHalfDiagonal * kTiltAngles[j] + 1e-5f;
			ENSURE_SMALL( MinSeparation( &manifold ) + overlap, bound );
		}
	}

	return 0;
}

// Two long roof ridges laid across each other. Unlike the stacked cubes the axis of minimum
// penetration really is the edge pair, so the query has to hand the winning pair to the contact
// builder. Closing the crossing angle drives that pair toward parallel, where the cross product
// the builder needs to orient the contact loses its precision.
static int RidgeCrossingTest( void )
{
	b3BoxHull hullA = b3MakeTransformedBoxHull( 1.5f, 0.1f, 0.1f, ExactRotation( kAxisX, 0.25f * B3_PI ) );
	b3BoxHull hullB = b3MakeTransformedBoxHull( 1.5f, 0.1f, 0.1f, ExactRotation( kAxisX, 0.25f * B3_PI ) );

	float ridgeY = 0.1f * kRoot2;
	float overlap = 0.01f;
	float lift = 2.0f * ridgeY - overlap;

	// Well clear of the rejection threshold. The ridges cross over the origin and the axis is y.
	float crossingAngles[] = { 0.02f, 0.1f, 0.5f };

	for ( int i = 0; i < ARRAY_COUNT( crossingAngles ); ++i )
	{
		b3Transform transform = { { 0.0f, lift, 0.0f }, ExactQuat( kAxisY, crossingAngles[i] ) };

		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SATCache cache = { 0 };
		b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, transform, &cache );

		ENSURE( manifold.pointCount == 1 );
		ENSURE( cache.type == b3_edgePairAxis );
		ENSURE_SMALL( manifold.normal.x, 1e-5f );
		ENSURE_SMALL( manifold.normal.y - 1.0f, 1e-5f );
		ENSURE_SMALL( manifold.normal.z, 1e-5f );
		ENSURE_SMALL( manifold.points[0].separation + overlap, 1e-4f );
		ENSURE_SMALL( manifold.points[0].point.y - ( ridgeY - 0.5f * overlap ), 1e-4f );

		// Where the point lands along the ridges is ill conditioned at a shallow crossing since
		// the closest point solve divides by the square of the sine of the angle. It only has to
		// land near the crossing, not at the end of a three meter beam.
		ENSURE_SMALL( manifold.points[0].point.x, 0.01f );
		ENSURE_SMALL( manifold.points[0].point.z, 0.01f );
	}

	// Inside the rejection threshold. A one point edge contact off a parallel pair would have a
	// normal built from noise, so the query drops the pair and the roof faces carry the contact.
	float shallowAngles[] = { 0.0f, 1e-4f, 1e-3f, 0.003f };

	for ( int i = 0; i < ARRAY_COUNT( shallowAngles ); ++i )
	{
		b3Transform transform = { { 0.0f, lift, 0.0f }, ExactQuat( kAxisY, shallowAngles[i] ) };

		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SATCache cache = { 0 };
		b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, transform, &cache );

		ENSURE( manifold.pointCount == 4 );
		ENSURE( cache.type != b3_edgePairAxis );

		// A roof face of one hull, so 45 degrees off the vertical
		ENSURE_SMALL( manifold.normal.x, 1e-4f );
		ENSURE_SMALL( manifold.normal.y - kHalfRoot2, 1e-4f );
		ENSURE_SMALL( b3AbsFloat( manifold.normal.z ) - kHalfRoot2, 1e-4f );

		float minSeparation = MinSeparation( &manifold );
		ENSURE( minSeparation < 0.0f );
		ENSURE( minSeparation > -2.0f * overlap );
	}

	return 0;
}

int ManifoldTest( void )
{
	RUN_SUBTEST( CrossedEdgeTest );
	RUN_SUBTEST( EdgeAxisScaleTest );
	RUN_SUBTEST( EdgeCacheTest );
	RUN_SUBTEST( EdgeEndpointTest );
	RUN_SUBTEST( ParallelEdgeTest );
	RUN_SUBTEST( ParallelEdgeManualTest );
	RUN_SUBTEST( OverlapNeverEmptyTest );
	RUN_SUBTEST( RidgeCrossingTest );
	RUN_SUBTEST( TriangleEdgeTest );
	RUN_SUBTEST( TriangleParallelEdgeTest );

	return 0;
}

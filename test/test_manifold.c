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
	// Cached edges are the even half of each twin pair
	ENSURE( ( cache.indexA & 1 ) == 0 && cache.indexA < hullA.base.edgeCount );
	ENSURE( ( cache.indexB & 1 ) == 0 && cache.indexB < hullB.base.edgeCount );

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

// The edge pair axis produced by the arc intersection must match the classic edge cross product.
// Rebuild the axis, the separation and the contact point from the two contributing edges and the
// convex radius, then compare against the manifold. orientRef fixes the sign of the axis to match
// the manifold normal convention for the shape pair. e1 belongs to the shape whose contact point is
// pulled in by the radius (the hull or triangle when it meets a capsule), e2 to the other edge.
static int CheckEdgeContact( const b3LocalManifold* manifold, b3Vec3 p1, b3Vec3 e1, b3Vec3 p2, b3Vec3 e2, b3Vec3 orientRef,
							 float radius, float normalTol, float sepTol, float pointTol )
{
	b3Vec3 axis = b3Normalize( b3Cross( e1, e2 ) );
	if ( b3Dot( axis, orientRef ) < 0.0f )
	{
		axis = b3Neg( axis );
	}

	// Normal matches the cross product and is perpendicular to both edges
	ENSURE_SMALL( manifold->normal.x - axis.x, normalTol );
	ENSURE_SMALL( manifold->normal.y - axis.y, normalTol );
	ENSURE_SMALL( manifold->normal.z - axis.z, normalTol );
	ENSURE_SMALL( b3Dot( manifold->normal, b3Normalize( e1 ) ), normalTol );
	ENSURE_SMALL( b3Dot( manifold->normal, b3Normalize( e2 ) ), normalTol );

	b3SegmentDistanceResult closest = b3LineDistance( p1, e1, p2, e2 );

	// Signed gap between the edge lines along the axis, less the capsule radius
	float expectedSeparation = b3Dot( axis, b3Sub( closest.point2, closest.point1 ) ) - radius;
	ENSURE_SMALL( manifold->points[0].separation - expectedSeparation, sepTol );

	// Midpoint of the closest approach, pulling the first point in by the radius
	b3Vec3 expectedPoint = b3MulSV( 0.5f, b3Add( b3MulSub( closest.point1, radius, axis ), closest.point2 ) );
	ENSURE_SMALL( manifold->points[0].point.x - expectedPoint.x, pointTol );
	ENSURE_SMALL( manifold->points[0].point.y - expectedPoint.y, pointTol );
	ENSURE_SMALL( manifold->points[0].point.z - expectedPoint.z, pointTol );

	return 0;
}

static void HullEdgeSegment( const b3HullData* hull, int edgeIndex, b3Transform transform, b3Vec3* point, b3Vec3* edge )
{
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3Vec3* points = b3GetHullPoints( hull );
	const b3HullHalfEdge* e = edges + edgeIndex;
	b3Vec3 tail = b3TransformPoint( transform, points[e->origin] );
	b3Vec3 head = b3TransformPoint( transform, points[edges[e->twin].origin] );
	*point = tail;
	*edge = b3Sub( head, tail );
}

// Two boxes crossing edge to edge. A holds a vertical edge, B is rolled to present a crossing edge
// and yawed so the arc intersection walks off the midpoint. For every configuration that resolves
// to an edge pair the recovered axis, separation and point must match the cross product oracle to
// tight tolerance. The oracle reads the edges the solver actually latched onto, so the check is
// exact regardless of which pair wins.
static int EdgeAxisOracleTest( void )
{
	b3BoxHull hullA = b3MakeTransformedBoxHull( 0.5f, 0.5f, 0.5f, ExactRotation( kAxisY, 0.25f * B3_PI ) );

	float rolls[] = { 0.18f * B3_PI, 0.25f * B3_PI, 0.32f * B3_PI };
	float yaws[] = { -0.35f, -0.15f, 0.0f, 0.15f, 0.35f };
	float distances[] = { 1.38f, 1.40f, kRoot2, 1.44f };

	int edgeContacts = 0;

	for ( int i = 0; i < ARRAY_COUNT( rolls ); ++i )
	{
		b3BoxHull hullB = b3MakeTransformedBoxHull( 0.5f, 0.5f, 0.5f, ExactRotation( kAxisZ, rolls[i] ) );

		for ( int j = 0; j < ARRAY_COUNT( yaws ); ++j )
		{
			for ( int k = 0; k < ARRAY_COUNT( distances ); ++k )
			{
				b3Transform transform = { { distances[k], 0.0f, 0.0f }, ExactQuat( kAxisY, yaws[j] ) };

				b3LocalManifoldPoint points[8];
				b3LocalManifold manifold = { 0 };
				manifold.points = points;
				b3SATCache cache = { 0 };
				b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, transform, &cache );

				if ( cache.type != b3_edgePairAxis || manifold.pointCount != 1 )
				{
					continue;
				}

				b3Vec3 p1, e1, p2, e2;
				HullEdgeSegment( &hullA.base, cache.indexA, b3Transform_identity, &p1, &e1 );
				HullEdgeSegment( &hullB.base, cache.indexB, transform, &p2, &e2 );

				b3Vec3 centerA = hullA.base.center;
				b3Vec3 centerB = b3TransformPoint( transform, hullB.base.center );
				b3Vec3 orientRef = b3Sub( centerB, centerA );

				if ( CheckEdgeContact( &manifold, p1, e1, p2, e2, orientRef, 0.0f, 2e-4f, 2e-4f, 2e-3f ) != 0 )
				{
					return 1;
				}

				++edgeContacts;
			}
		}
	}

	// The sweep is only meaningful if it actually drove the edge path
	ENSURE( edgeContacts >= 15 );

	return 0;
}

// The same oracle over randomly oriented box pairs. Whenever the solver reports an edge pair the
// recovered axis must be perpendicular to both edges and match the cross product. This casts a wide
// net over the arc that the structured sweep cannot reach.
static int EdgeAxisRandomOracleTest( void )
{
	g_seed = 246813579u;

	int edgeContacts = 0;

	for ( int i = 0; i < 2000; ++i )
	{
		float angleA = NextFloat( 0.2f, 0.5f ) * B3_PI;
		float angleB = NextFloat( 0.2f, 0.5f ) * B3_PI;
		b3BoxHull hullA = b3MakeTransformedBoxHull( 0.5f, 0.5f, 0.5f, ExactRotation( NextDirection(), angleA ) );
		b3BoxHull hullB = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

		float d = NextFloat( 1.2f, 1.55f );
		b3Transform transform = { b3MulSV( d, NextDirection() ), ExactQuat( NextDirection(), angleB ) };

		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SATCache cache = { 0 };
		b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, transform, &cache );

		if ( cache.type != b3_edgePairAxis || manifold.pointCount != 1 )
		{
			continue;
		}

		b3Vec3 p1, e1, p2, e2;
		HullEdgeSegment( &hullA.base, cache.indexA, b3Transform_identity, &p1, &e1 );
		HullEdgeSegment( &hullB.base, cache.indexB, transform, &p2, &e2 );

		// Skip crossings near parallel where the closest point solve is ill conditioned. The
		// parallel rejection itself is covered by ParallelEdgeTest.
		float sine = b3Length( b3Cross( b3Normalize( e1 ), b3Normalize( e2 ) ) );
		if ( sine < 0.1f )
		{
			continue;
		}

		b3Vec3 orientRef = b3Sub( b3TransformPoint( transform, hullB.base.center ), hullA.base.center );

		if ( CheckEdgeContact( &manifold, p1, e1, p2, e2, orientRef, 0.0f, 1e-3f, 1e-3f, 5e-3f ) != 0 )
		{
			return 1;
		}

		++edgeContacts;
	}

	ENSURE( edgeContacts >= 100 );

	return 0;
}

// A thin capsule stabbed through the +x +y edge of a box so the edge pair is the axis of minimum
// penetration. This drives the isolated edge axis (arc versus circle on the Gauss map) that a
// capsule presents. The edge is nearly parallel to a box face normal, exactly where the old center
// based orientation flickered, so the axis, the penetration and the point are all checked.
static int HullCapsuleEdgeDeepTest( void )
{
	b3BoxHull hull = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

	// The +x +y edge runs along z between the +x and +y faces
	b3Vec3 edgePoint = { 0.5f, 0.5f, 0.0f };
	b3Vec3 edgeDir = { 0.0f, 0.0f, 1.0f };
	b3Vec3 outward = b3Normalize( ( b3Vec3 ){ 1.0f, 1.0f, 0.0f } );

	// Penetrate far enough that the core segment clearly overlaps the box so the deep path runs,
	// but keep the radius small enough that the edge stays the axis of minimum penetration.
	float depths[] = { 0.12f, 0.18f, 0.25f };
	float radii[] = { 0.05f, 0.1f, 0.2f };
	float tilts[] = { 0.0f, 0.25f, -0.25f };

	int count = 0;

	for ( int i = 0; i < ARRAY_COUNT( depths ); ++i )
	{
		for ( int j = 0; j < ARRAY_COUNT( radii ); ++j )
		{
			for ( int k = 0; k < ARRAY_COUNT( tilts ); ++k )
			{
				b3Vec3 capsuleDir = b3Normalize( ( b3Vec3 ){ 1.0f, -1.0f, tilts[k] } );
				b3Vec3 mid = b3MulAdd( edgePoint, -depths[i], outward );
				b3Vec3 c1 = b3MulAdd( mid, -0.5f, capsuleDir );
				b3Vec3 c2 = b3MulAdd( mid, 0.5f, capsuleDir );
				b3Capsule capsule = { c1, c2, radii[j] };

				b3LocalManifoldPoint points[8];
				b3LocalManifold manifold = { 0 };
				manifold.points = points;
				b3SimplexCache cache = { 0 };
				b3CollideHullAndCapsule( &manifold, 8, &hull.base, &capsule, b3Transform_identity, &cache );

				ENSURE( manifold.pointCount == 1 );
				ENSURE( manifold.points[0].separation < 0.0f );

				// Hull edge is e1, capsule axis is e2, normal points out of the hull
				if ( CheckEdgeContact( &manifold, edgePoint, edgeDir, c1, b3Sub( c2, c1 ), outward, radii[j], 1e-4f, 1e-4f,
									   1e-4f ) != 0 )
				{
					return 1;
				}

				++count;
			}
		}
	}

	ENSURE( count == ARRAY_COUNT( depths ) * ARRAY_COUNT( radii ) * ARRAY_COUNT( tilts ) );

	return 0;
}

// Force the triangle versus hull edge query over a broad sweep of crossing geometries. A cube tipped
// 45 degrees rests an edge along x at y = -h*root2. A triangle edge is laid across it at a range of
// yaws, plane tips and gaps so the arc intersection lands all over the arc. The manual axis hands
// the winning pair to the builder, and the recovered axis must match the cross product of the chosen
// triangle and hull edges and point from the triangle into the hull.
static int TriangleHullEdgeSweepTest( void )
{
	b3BoxHull hull = b3MakeTransformedBoxHull( 0.5f, 0.5f, 0.5f, ExactRotation( kAxisX, 0.25f * B3_PI ) );
	b3Vec3 hullEdgePoint = { 0.0f, -kHalfRoot2, 0.0f };

	// Degrees: triangle plane tip about z, and triangle edge yaw
	float betas[] = { 8.0f, 20.0f, 32.0f };
	float gammas[] = { 20.0f, 35.0f, 50.0f, 70.0f };
	float gaps[] = { 0.02f, 0.0f, -0.03f, -0.08f };

	int edgeContacts = 0;

	for ( int a = 0; a < ARRAY_COUNT( betas ); ++a )
	{
		for ( int b = 0; b < ARRAY_COUNT( gammas ); ++b )
		{
			for ( int c = 0; c < ARRAY_COUNT( gaps ); ++c )
			{
				float beta = betas[a] * B3_PI / 180.0f;
				float gamma = gammas[b] * B3_PI / 180.0f;

				// Tip the plane off the hull edge so the Minkowski test holds, then yaw the edge
				b3Vec3 triNormal = { sinf( beta ), cosf( beta ), 0.0f };
				b3Vec3 triEdge = { sinf( gamma ) * cosf( beta ), -sinf( gamma ) * sinf( beta ), cosf( gamma ) };

				// Perpendicular to both edges and pointing out of the hull
				b3Vec3 axis = b3Normalize( b3Cross( kAxisX, triEdge ) );
				b3Vec3 trianglePoint = b3MulAdd( hullEdgePoint, gaps[c], axis );

				b3Vec3 v1 = b3MulAdd( trianglePoint, -1.0f, triEdge );
				b3Vec3 v2 = b3MulAdd( trianglePoint, 1.0f, triEdge );
				b3Vec3 v3 = b3MulAdd( v1, 1.5f, b3Cross( triNormal, triEdge ) );

				b3Vec3 triangleVerts[] = { v1, v2, v3 };
				b3Vec3 triangleEdges[] = { b3Sub( v2, v1 ), b3Sub( v3, v2 ), b3Sub( v1, v3 ) };
				b3Vec3 triangleCenter = b3MulSV( 1.0f / 3.0f, b3Add( v1, b3Add( v2, v3 ) ) );

				b3LocalManifoldPoint points[8];
				b3LocalManifold manifold = { 0 };
				manifold.points = points;
				b3SATCache cache = { .type = b3_manualEdgePairAxis };
				b3CollideHullAndTriangle( &manifold, 8, &hull.base, v1, v2, v3, 0, &cache, true );

				if ( cache.type != b3_edgePairAxis || manifold.pointCount != 1 )
				{
					continue;
				}

				b3Vec3 p1 = triangleVerts[cache.indexA];
				b3Vec3 e1 = triangleEdges[cache.indexA];

				b3Vec3 p2, e2;
				HullEdgeSegment( &hull.base, cache.indexB, b3Transform_identity, &p2, &e2 );

				// Normal points from the triangle into the hull
				b3Vec3 orientRef = b3Sub( hull.base.center, triangleCenter );

				if ( CheckEdgeContact( &manifold, p1, e1, p2, e2, orientRef, 0.0f, 1e-4f, 1e-4f, 1e-3f ) != 0 )
				{
					return 1;
				}

				++edgeContacts;
			}
		}
	}

	ENSURE( edgeContacts >= 30 );

	return 0;
}

// A capsule laid nearly in a triangle plane and pushed across one edge so the edge pair drives the
// deep contact. This exercises the two sided triangle edge, where the side normal trick chooses
// which half of the arc holds the axis. Validate every edge contact the sweep produces.
static int CapsuleTriangleEdgeDeepTest( void )
{
	// Triangle in the y = 0 plane. The v1 v2 edge runs along x at z = 0, the interior lies at z < 0.
	b3Vec3 v1 = { -2.0f, 0.0f, 0.0f };
	b3Vec3 v2 = { 2.0f, 0.0f, 0.0f };
	b3Vec3 v3 = { 0.0f, 0.0f, -2.0f };
	b3Vec3 triangle[] = { v1, v2, v3 };
	b3Vec3 triangleEdges[] = { b3Sub( v2, v1 ), b3Sub( v3, v2 ), b3Sub( v1, v3 ) };
	b3Vec3 triangleCenter = b3MulSV( 1.0f / 3.0f, b3Add( v1, b3Add( v2, v3 ) ) );

	// A nearly in plane core crossing the edge at (0,0,z0) with a small out of plane tilt. The core
	// pierces the triangle just inside the edge so the deep path runs and the tilted edge pair wins.
	float z0s[] = { -0.05f, -0.03f, -0.01f };
	float tilts[] = { 0.2f, 0.3f, 0.4f };
	float yaws[] = { 0.4f, 0.6f, 0.8f };
	float radii[] = { 0.05f, 0.1f };

	int edgeContacts = 0;

	for ( int i = 0; i < ARRAY_COUNT( z0s ); ++i )
	{
		for ( int j = 0; j < ARRAY_COUNT( tilts ); ++j )
		{
			for ( int y = 0; y < ARRAY_COUNT( yaws ); ++y )
			{
				for ( int r = 0; r < ARRAY_COUNT( radii ); ++r )
				{
					b3Vec3 capsuleDir = b3Normalize( ( b3Vec3 ){ sinf( yaws[y] ), tilts[j], cosf( yaws[y] ) } );
					b3Vec3 mid = { 0.0f, 0.0f, z0s[i] };
					b3Vec3 c1 = b3MulAdd( mid, -0.6f, capsuleDir );
					b3Vec3 c2 = b3MulAdd( mid, 0.6f, capsuleDir );
					b3Capsule capsule = { c1, c2, radii[r] };

					b3LocalManifoldPoint points[8];
					b3LocalManifold manifold = { 0 };
					manifold.points = points;
					b3SimplexCache cache = { 0 };
					b3CollideCapsuleAndTriangle( &manifold, 8, &capsule, triangle, &cache );

					// Only the edge contacts exercise the new axis. Face contacts are handled elsewhere.
					if ( manifold.pointCount != 1 || manifold.feature < b3_featureEdge1 || manifold.feature > b3_featureEdge3 )
					{
						continue;
					}

					int edgeIndex = manifold.feature - b3_featureEdge1;
					b3Vec3 p1 = triangle[edgeIndex];
					b3Vec3 e1 = triangleEdges[edgeIndex];

					// Normal points from the triangle toward the capsule
					b3Vec3 capsuleEdge = b3Sub( c2, c1 );
					b3Vec3 capsuleCenter = b3Lerp( c1, c2, 0.5f );
					b3Vec3 orientRef = b3Sub( capsuleCenter, triangleCenter );

					if ( CheckEdgeContact( &manifold, p1, e1, c1, capsuleEdge, orientRef, radii[r], 1e-4f, 1e-4f, 2e-4f ) != 0 )
					{
						return 1;
					}

					++edgeContacts;
				}
			}
		}
	}

	// The sweep must actually reach the edge path
	ENSURE( edgeContacts >= 15 );

	return 0;
}

// A sphere driven straight through a box face, from separated, across the surface where the collider
// switches from GJK closest points to the SAT face pick, and on into deep overlap. The separation
// must stay the analytic gap the whole way and the normal must not flip. A jump at the seam would
// read as a pop in the solver. The sweep is fine enough to land samples on both sides of the seam.
static int SphereHullSeamTest( void )
{
	b3BoxHull hull = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	float radius = 0.15f;

	float yStart = 0.5f + radius + 0.4f * B3_SPECULATIVE_DISTANCE;
	float yEnd = 0.1f;
	int steps = 400;
	float dy = ( yStart - yEnd ) / steps;

	float previous = 0.0f;
	int shallowSamples = 0;
	int deepSamples = 0;

	for ( int i = 0; i <= steps; ++i )
	{
		float y = yStart - i * dy;
		b3Sphere sphere = { { 0.0f, y, 0.0f }, radius };

		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SimplexCache cache = { 0 };
		b3CollideHullAndSphere( &manifold, 8, &hull.base, &sphere, b3Transform_identity, &cache );

		ENSURE( manifold.pointCount == 1 );

		float separation = manifold.points[0].separation;
		float expected = ( y - 0.5f ) - radius;

		// Separation is the analytic gap on both sides of the seam
		ENSURE_SMALL( separation - expected, 1e-5f );

		// Normal holds the face direction with no flip
		ENSURE_SMALL( manifold.normal.x, 1e-5f );
		ENSURE_SMALL( manifold.normal.y - 1.0f, 1e-5f );
		ENSURE_SMALL( manifold.normal.z, 1e-5f );

		// No jump across the seam: consecutive separations track the step
		if ( i > 0 )
		{
			ENSURE_SMALL( ( previous - separation ) - dy, 1e-5f );
		}
		previous = separation;

		if ( y > 0.5f )
		{
			shallowSamples += 1;
		}
		else
		{
			deepSamples += 1;
		}
	}

	// The sweep must straddle the surface so both the GJK and the SAT branch run
	ENSURE( shallowSamples > 0 && deepSamples > 0 );

	return 0;
}

// The same seam for a capsule laid parallel to the face. Above the surface the shallow path clips two
// points, in overlap the face path builds two, and every point must sit at the analytic gap through
// the transition with the normal fixed on the face.
static int CapsuleHullSeamTest( void )
{
	b3BoxHull hull = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	float radius = 0.15f;
	float halfLength = 0.3f;

	float yStart = 0.5f + radius + 0.4f * B3_SPECULATIVE_DISTANCE;
	float yEnd = 0.1f;
	int steps = 400;
	float dy = ( yStart - yEnd ) / steps;

	float previous = 0.0f;
	int shallowSamples = 0;
	int deepSamples = 0;

	for ( int i = 0; i <= steps; ++i )
	{
		float y = yStart - i * dy;
		b3Capsule capsule = { { -halfLength, y, 0.0f }, { halfLength, y, 0.0f }, radius };

		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SimplexCache cache = { 0 };
		b3CollideHullAndCapsule( &manifold, 8, &hull.base, &capsule, b3Transform_identity, &cache );

		ENSURE( manifold.pointCount >= 1 );

		float expected = ( y - 0.5f ) - radius;

		// Every point sits at the analytic gap
		for ( int k = 0; k < manifold.pointCount; ++k )
		{
			ENSURE_SMALL( manifold.points[k].separation - expected, 1e-5f );
		}

		// Normal holds the face direction with no flip
		ENSURE_SMALL( manifold.normal.x, 1e-5f );
		ENSURE_SMALL( manifold.normal.y - 1.0f, 1e-5f );
		ENSURE_SMALL( manifold.normal.z, 1e-5f );

		// No jump across the seam
		float minSeparation = MinSeparation( &manifold );
		if ( i > 0 )
		{
			ENSURE_SMALL( ( previous - minSeparation ) - dy, 1e-5f );
		}
		previous = minSeparation;

		if ( y > 0.5f )
		{
			shallowSamples += 1;
		}
		else
		{
			deepSamples += 1;
		}
	}

	ENSURE( shallowSamples > 0 && deepSamples > 0 );

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
	RUN_SUBTEST( EdgeAxisOracleTest );
	RUN_SUBTEST( EdgeAxisRandomOracleTest );
	RUN_SUBTEST( HullCapsuleEdgeDeepTest );
	RUN_SUBTEST( TriangleHullEdgeSweepTest );
	RUN_SUBTEST( CapsuleTriangleEdgeDeepTest );
	RUN_SUBTEST( SphereHullSeamTest );
	RUN_SUBTEST( CapsuleHullSeamTest );

	return 0;
}

// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "manifold.h"

#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_functions.h"

#include <float.h>
#include <math.h>

static const float kRoot2 = 1.41421356f;

static const b3Vec3 kAxisX = { 1.0f, 0.0f, 0.0f };
static const b3Vec3 kAxisY = { 0.0f, 1.0f, 0.0f };
static const b3Vec3 kAxisZ = { 0.0f, 0.0f, 1.0f };

// b3ComputeCosSin is a coarse rational approximation, so the fixtures build their rotations from
// libm to keep the analytic edge and face positions exact.
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

// Directed separation along a unit axis n pointing from A to B. Positive means B clears A along n.
// This is the projection gap min over B minus max over A, which is exactly what a valid separating
// axis measures, so the winning face or edge axis must reproduce it.
static float SepAlong( const b3HullData* hullA, const b3HullData* hullB, b3Transform xfB, b3Vec3 n )
{
	const b3Vec3* pointsA = b3GetHullPoints( hullA );
	float maxA = -FLT_MAX;
	for ( int i = 0; i < hullA->vertexCount; ++i )
	{
		maxA = b3MaxFloat( maxA, b3Dot( n, pointsA[i] ) );
	}

	const b3Vec3* pointsB = b3GetHullPoints( hullB );
	float minB = FLT_MAX;
	for ( int i = 0; i < hullB->vertexCount; ++i )
	{
		minB = b3MinFloat( minB, b3Dot( n, b3TransformPoint( xfB, pointsB[i] ) ) );
	}

	return minB - maxA;
}

static void TryAxis( const b3HullData* hullA, const b3HullData* hullB, b3Transform xfB, b3Vec3 n, float* best,
					 b3Vec3* bestNormal )
{
	float s = SepAlong( hullA, hullB, xfB, n );
	if ( s > *best )
	{
		*best = s;
		*bestNormal = n;
	}
}

// Brute force SAT. The maximum separation over every face normal and every edge cross product is
// the true answer the SIMD query has to match. Each undirected axis is tried both ways so the
// winner comes out oriented from A to B with no sign bookkeeping.
static float OracleSeparation( const b3HullData* hullA, const b3HullData* hullB, b3Transform xfB, b3Vec3* outNormal )
{
	float best = -FLT_MAX;
	b3Vec3 bestNormal = { 0.0f, 0.0f, 0.0f };

	const b3Plane* planesA = b3GetHullPlanes( hullA );
	for ( int i = 0; i < hullA->faceCount; ++i )
	{
		TryAxis( hullA, hullB, xfB, planesA[i].normal, &best, &bestNormal );
		TryAxis( hullA, hullB, xfB, b3Neg( planesA[i].normal ), &best, &bestNormal );
	}

	const b3Plane* planesB = b3GetHullPlanes( hullB );
	for ( int i = 0; i < hullB->faceCount; ++i )
	{
		b3Vec3 n = b3RotateVector( xfB.q, planesB[i].normal );
		TryAxis( hullA, hullB, xfB, n, &best, &bestNormal );
		TryAxis( hullA, hullB, xfB, b3Neg( n ), &best, &bestNormal );
	}

	const b3HullHalfEdge* edgesA = b3GetHullEdges( hullA );
	const b3Vec3* pointsA = b3GetHullPoints( hullA );
	const b3HullHalfEdge* edgesB = b3GetHullEdges( hullB );
	const b3Vec3* pointsB = b3GetHullPoints( hullB );

	// Half edges are stored as adjacent twin pairs, so the even index and its successor bound one edge.
	for ( int i = 0; i < hullA->edgeCount; i += 2 )
	{
		b3Vec3 dirA = b3Sub( pointsA[edgesA[i + 1].origin], pointsA[edgesA[i].origin] );
		for ( int j = 0; j < hullB->edgeCount; j += 2 )
		{
			b3Vec3 dirB = b3RotateVector( xfB.q, b3Sub( pointsB[edgesB[j + 1].origin], pointsB[edgesB[j].origin] ) );
			b3Vec3 cross = b3Cross( dirA, dirB );

			// Near parallel edges give no useful axis and cannot be the true maximum.
			if ( b3Dot( cross, cross ) < 1e-10f )
			{
				continue;
			}

			cross = b3Normalize( cross );
			TryAxis( hullA, hullB, xfB, cross, &best, &bestNormal );
			TryAxis( hullA, hullB, xfB, b3Neg( cross ), &best, &bestNormal );
		}
	}

	*outNormal = bestNormal;
	return best;
}

// Two axis aligned cubes pushed apart along x. A's plus x face is the separating axis, the gap is
// large enough to trip the speculative early out, and the reference vertex on B is its minus x side.
static int FaceAxisASeparatedTest( void )
{
	b3BoxHull hullA = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3BoxHull hullB = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

	b3Transform xfB = { { 1.2f, 0.0f, 0.0f }, b3Quat_identity };
	b3AxisQuery q = b3ComputeSeparatingAxis( &hullA.base, &hullB.base, xfB );

	ENSURE( q.feature == b3_faceAxisA );
	ENSURE_SMALL( q.separation - 0.2f, 1e-5f );
	ENSURE_SMALL( q.normal.x - 1.0f, 1e-6f );
	ENSURE_SMALL( q.normal.y, 1e-6f );
	ENSURE_SMALL( q.normal.z, 1e-6f );

	const b3Plane* planesA = b3GetHullPlanes( &hullA.base );
	ENSURE_SMALL( planesA[q.indexA].normal.x - 1.0f, 1e-6f );

	const b3Vec3* pointsB = b3GetHullPoints( &hullB.base );
	ENSURE_SMALL( pointsB[q.indexB].x + 0.5f, 1e-6f );

	ENSURE_SMALL( SepAlong( &hullA.base, &hullB.base, xfB, q.normal ) - q.separation, 1e-5f );
	return 0;
}

// A yawed 45 degrees about z so all of its faces sit oblique to x. Only B still has a face square to
// the gap, so the reference has to be B's minus x face. This is the case that ties when both hulls
// are aligned, and the tie is what a face A bias would hide.
static int FaceAxisBSeparatedTest( void )
{
	b3BoxHull hullA = b3MakeTransformedBoxHull( 0.5f, 0.5f, 0.5f, ExactRotation( kAxisZ, 0.25f * B3_PI ) );
	b3BoxHull hullB = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

	float aExtent = 0.5f * kRoot2;
	float gap = 0.2f;
	float d = aExtent + 0.5f + gap;
	b3Transform xfB = { { d, 0.0f, 0.0f }, b3Quat_identity };

	b3AxisQuery q = b3ComputeSeparatingAxis( &hullA.base, &hullB.base, xfB );

	ENSURE( q.feature == b3_faceAxisB );
	ENSURE_SMALL( q.separation - gap, 1e-5f );
	ENSURE_SMALL( q.normal.x - 1.0f, 1e-5f );
	ENSURE_SMALL( q.normal.y, 1e-5f );
	ENSURE_SMALL( q.normal.z, 1e-5f );

	const b3Plane* planesB = b3GetHullPlanes( &hullB.base );
	ENSURE_SMALL( planesB[q.indexB].normal.x + 1.0f, 1e-5f );

	const b3Vec3* pointsA = b3GetHullPoints( &hullA.base );
	ENSURE_SMALL( pointsA[q.indexA].x - aExtent, 1e-4f );

	ENSURE_SMALL( SepAlong( &hullA.base, &hullB.base, xfB, q.normal ) - q.separation, 1e-4f );
	return 0;
}

// Well beyond the speculative distance the face phase must report the axis and separation without
// running the edge phase. A sign slip in the early out would read as deep overlap.
static int FaceFarSeparatedTest( void )
{
	b3BoxHull hullA = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3BoxHull hullB = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

	b3Transform xfB = { { 3.0f, 0.0f, 0.0f }, b3Quat_identity };
	b3AxisQuery q = b3ComputeSeparatingAxis( &hullA.base, &hullB.base, xfB );

	ENSURE( q.feature == b3_faceAxisA );
	ENSURE_SMALL( q.separation - 2.0f, 1e-5f );
	ENSURE_SMALL( q.normal.x - 1.0f, 1e-6f );

	b3Vec3 oracleNormal;
	float oracleSep = OracleSeparation( &hullA.base, &hullB.base, xfB, &oracleNormal );
	ENSURE_SMALL( q.separation - oracleSep, 1e-5f );
	return 0;
}

// The support bias is derived from each hull's AABB, not from an origin assumed to sit inside it. A
// box built far from its own local origin is the case the old diagonal bias got wrong. A is yawed
// and pushed out to x = 5, so the face B query has to run getSupport over vertices clustered near
// x = 5 and still pick the right one.
static int OffsetFaceAxisBTest( void )
{
	float cx = 5.0f;
	b3Transform placeA = { { cx, 0.0f, 0.0f }, ExactQuat( kAxisZ, 0.25f * B3_PI ) };
	b3BoxHull hullA = b3MakeTransformedBoxHull( 0.5f, 0.5f, 0.5f, placeA );
	b3BoxHull hullB = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

	float aExtent = cx + 0.5f * kRoot2;
	float gap = 0.2f;
	float d = aExtent + 0.5f + gap;
	b3Transform xfB = { { d, 0.0f, 0.0f }, b3Quat_identity };

	b3AxisQuery q = b3ComputeSeparatingAxis( &hullA.base, &hullB.base, xfB );

	ENSURE( q.feature == b3_faceAxisB );
	ENSURE_SMALL( q.separation - gap, 1e-4f );
	ENSURE_SMALL( q.normal.x - 1.0f, 1e-5f );
	ENSURE_SMALL( q.normal.y, 1e-5f );
	ENSURE_SMALL( q.normal.z, 1e-5f );

	const b3Plane* planesB = b3GetHullPlanes( &hullB.base );
	ENSURE_SMALL( planesB[q.indexB].normal.x + 1.0f, 1e-5f );

	const b3Vec3* pointsA = b3GetHullPoints( &hullA.base );
	ENSURE_SMALL( pointsA[q.indexA].x - aExtent, 1e-4f );

	b3Vec3 oracleNormal;
	float oracleSep = OracleSeparation( &hullA.base, &hullB.base, xfB, &oracleNormal );
	ENSURE_SMALL( q.separation - oracleSep, 1e-4f );
	ENSURE_SMALL( SepAlong( &hullA.base, &hullB.base, xfB, q.normal ) - q.separation, 1e-4f );
	return 0;
}

// Cube A yawed 45 about y presents an edge along y at x = h*root2. Cube B rolled 45 about z presents
// an edge along z at x = -h*root2. Sliding B along x makes those edges the closest features, so the
// axis is x and the separation is d - root2. The sweep straddles contact into overlap while staying
// under the speculative distance, so no face axis can short circuit the edge phase. The overlap rows
// pin the maximum separation, which is where a premature edge early out shows up.
static int EdgePairSweepTest( void )
{
	b3BoxHull hullA = b3MakeTransformedBoxHull( 0.5f, 0.5f, 0.5f, ExactRotation( kAxisY, 0.25f * B3_PI ) );
	b3BoxHull hullB = b3MakeTransformedBoxHull( 0.5f, 0.5f, 0.5f, ExactRotation( kAxisZ, 0.25f * B3_PI ) );

	float distances[] = { 1.43f, 1.42f, kRoot2, 1.40f, 1.38f, 1.35f, 1.30f };

	for ( int i = 0; i < ARRAY_COUNT( distances ); ++i )
	{
		float d = distances[i];
		float expected = d - kRoot2;

		b3Transform xfB = { { d, 0.0f, 0.0f }, b3Quat_identity };
		b3AxisQuery q = b3ComputeSeparatingAxis( &hullA.base, &hullB.base, xfB );

		ENSURE( q.feature == b3_edgePairAxis );
		ENSURE_SMALL( q.separation - expected, 1e-4f );
		ENSURE_SMALL( q.normal.x - 1.0f, 1e-4f );
		ENSURE_SMALL( q.normal.y, 1e-4f );
		ENSURE_SMALL( q.normal.z, 1e-4f );

		// Half edge indices are the even twin of each pair.
		ENSURE( ( q.indexA & 1 ) == 0 );
		ENSURE( ( q.indexB & 1 ) == 0 );

		b3Vec3 oracleNormal;
		float oracleSep = OracleSeparation( &hullA.base, &hullB.base, xfB, &oracleNormal );
		ENSURE_SMALL( q.separation - oracleSep, 1e-4f );
		ENSURE_SMALL( SepAlong( &hullA.base, &hullB.base, xfB, q.normal ) - q.separation, 1e-4f );
	}

	return 0;
}

static uint32_t g_seed = 987654321u;

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

// A wide net over randomly oriented box pairs, from clearly separated to deep overlap. The brute
// force oracle is the source of truth. Whatever axis the query returns it must be a unit vector that
// reproduces its own separation, it must not exceed the true maximum, and it must not fall short of
// it. The short fall bound is what a suboptimal edge pick trips. Thin boxes are mixed in because a
// crossed pair of long edges stays the axis of minimum penetration even deep in overlap, which is
// the hardest case for the edge phase.
static int SeparatingAxisOracleTest( void )
{
	g_seed = 987654321u;

	int separated = 0;
	int penetrating = 0;
	int edgeWins = 0;
	float worstShortfall = 0.0f;
	float worstExcess = 0.0f;

	for ( int i = 0; i < 4000; ++i )
	{
		b3Vec3 halfA, halfB;
		float reach;

		if ( i & 1 )
		{
			// Thin crossed beams driven into deep overlap.
			halfA = (b3Vec3){ NextFloat( 1.0f, 1.6f ), NextFloat( 0.08f, 0.15f ), NextFloat( 0.08f, 0.15f ) };
			halfB = (b3Vec3){ NextFloat( 1.0f, 1.6f ), NextFloat( 0.08f, 0.15f ), NextFloat( 0.08f, 0.15f ) };
			reach = NextFloat( 0.0f, 0.4f );
		}
		else
		{
			// Chunky boxes across the whole separated to overlapping range.
			halfA = (b3Vec3){ NextFloat( 0.3f, 0.8f ), NextFloat( 0.3f, 0.8f ), NextFloat( 0.3f, 0.8f ) };
			halfB = (b3Vec3){ NextFloat( 0.3f, 0.8f ), NextFloat( 0.3f, 0.8f ), NextFloat( 0.3f, 0.8f ) };
			reach = NextFloat( 0.0f, 1.7f );
		}

		// Rotate A in place so it stays centered on the origin, which keeps the support bias valid.
		b3BoxHull hullA =
			b3MakeTransformedBoxHull( halfA.x, halfA.y, halfA.z, ExactRotation( NextDirection(), NextFloat( 0.0f, B3_PI ) ) );
		b3BoxHull hullB = b3MakeBoxHull( halfB.x, halfB.y, halfB.z );

		b3Transform xfB = { b3MulSV( reach, NextDirection() ), ExactQuat( NextDirection(), NextFloat( 0.0f, B3_PI ) ) };

		b3AxisQuery q = b3ComputeSeparatingAxis( &hullA.base, &hullB.base, xfB );

		b3Vec3 oracleNormal;
		float oracleSep = OracleSeparation( &hullA.base, &hullB.base, xfB, &oracleNormal );

		ENSURE_SMALL( b3Length( q.normal ) - 1.0f, 1e-3f );
		ENSURE_SMALL( SepAlong( &hullA.base, &hullB.base, xfB, q.normal ) - q.separation, 2e-3f );

		worstShortfall = b3MaxFloat( worstShortfall, oracleSep - q.separation );
		worstExcess = b3MaxFloat( worstExcess, q.separation - oracleSep );

		// A suboptimal axis pick lands a discrete step below the true maximum. The bound sits well
		// under that step yet clear of the edge normalization noise floor near 6e-4.
		ENSURE( q.separation <= oracleSep + 2e-3f );
		ENSURE( q.separation >= oracleSep - 4e-3f );

		if ( oracleSep > 0.0f )
		{
			separated += 1;
		}
		else
		{
			penetrating += 1;
		}

		if ( q.feature == b3_edgePairAxis )
		{
			edgeWins += 1;
		}
	}

	printf( "    oracle: separated=%d penetrating=%d edgeWins=%d worstShortfall=%.2e worstExcess=%.2e\n", separated, penetrating,
			edgeWins, worstShortfall, worstExcess );

	// The sweep has to cover both regimes and actually drive the edge path.
	ENSURE( separated > 100 );
	ENSURE( penetrating > 100 );
	ENSURE( edgeWins > 20 );
	return 0;
}

// The oracle over hulls whose vertices sit far from their own local origin, which the AABB based
// bias must handle. Half the pairs offset A in its own frame, half offset B, and B is placed so the
// two centers land near each other across the separated to overlapping range. Any bias that assumed
// the origin was inside the hull would pick the wrong support and blow the separation.
static int OffsetHullOracleTest( void )
{
	g_seed = 24681012u;

	int separated = 0;
	int penetrating = 0;
	int edgeWins = 0;
	float worstShortfall = 0.0f;
	float worstExcess = 0.0f;
	float worstConsistency = 0.0f;

	for ( int i = 0; i < 3000; ++i )
	{
		b3Vec3 halfA = { NextFloat( 0.3f, 0.8f ), NextFloat( 0.3f, 0.8f ), NextFloat( 0.3f, 0.8f ) };
		b3Vec3 halfB = { NextFloat( 0.3f, 0.8f ), NextFloat( 0.3f, 0.8f ), NextFloat( 0.3f, 0.8f ) };

		b3Vec3 offset = b3MulSV( NextFloat( 2.0f, 6.0f ), NextDirection() );
		float reach = NextFloat( 0.0f, 1.7f );
		b3Vec3 dir = NextDirection();

		b3BoxHull hullA;
		b3BoxHull hullB;
		b3Transform xfB;

		if ( i & 1 )
		{
			// A carries the offset in its own frame. B is centered and dropped near A's center.
			b3Transform placeA = { offset, ExactQuat( NextDirection(), NextFloat( 0.0f, B3_PI ) ) };
			hullA = b3MakeTransformedBoxHull( halfA.x, halfA.y, halfA.z, placeA );
			hullB = b3MakeBoxHull( halfB.x, halfB.y, halfB.z );
			xfB = (b3Transform){ b3MulAdd( offset, reach, dir ), ExactQuat( NextDirection(), NextFloat( 0.0f, B3_PI ) ) };
		}
		else
		{
			// B carries the offset in its own frame. Place it so its world center lands near the origin.
			b3Quat qB = ExactQuat( NextDirection(), NextFloat( 0.0f, B3_PI ) );
			hullA = b3MakeBoxHull( halfA.x, halfA.y, halfA.z );
			hullB = b3MakeOffsetBoxHull( halfB.x, halfB.y, halfB.z, offset );
			xfB = (b3Transform){ b3Sub( b3MulSV( reach, dir ), b3RotateVector( qB, offset ) ), qB };
		}

		b3AxisQuery q = b3ComputeSeparatingAxis( &hullA.base, &hullB.base, xfB );

		b3Vec3 oracleNormal;
		float oracleSep = OracleSeparation( &hullA.base, &hullB.base, xfB, &oracleNormal );

		float consistency = b3AbsFloat( SepAlong( &hullA.base, &hullB.base, xfB, q.normal ) - q.separation );
		worstConsistency = b3MaxFloat( worstConsistency, consistency );
		worstShortfall = b3MaxFloat( worstShortfall, oracleSep - q.separation );
		worstExcess = b3MaxFloat( worstExcess, q.separation - oracleSep );

		// Consistency is the sharp bias check: a wrong support pick on an offset hull would throw the
		// returned normal off its own separation by a vertex spacing, not a noise floor. The oracle
		// bounds are looser since differencing coordinates out at radius six costs a few more digits.
		ENSURE_SMALL( b3Length( q.normal ) - 1.0f, 1e-3f );
		ENSURE( consistency < 1e-3f );
		ENSURE( q.separation <= oracleSep + 3e-3f );
		ENSURE( q.separation >= oracleSep - 8e-3f );

		if ( oracleSep > 0.0f )
		{
			separated += 1;
		}
		else
		{
			penetrating += 1;
		}

		if ( q.feature == b3_edgePairAxis )
		{
			edgeWins += 1;
		}
	}

	printf( "    offset: separated=%d penetrating=%d edgeWins=%d worstShortfall=%.2e worstExcess=%.2e worstConsistency=%.2e\n",
			separated, penetrating, edgeWins, worstShortfall, worstExcess, worstConsistency );

	ENSURE( separated > 100 );
	ENSURE( penetrating > 100 );
	ENSURE( edgeWins > 20 );
	return 0;
}

int SeparatingAxisTest( void )
{
	RUN_SUBTEST( FaceAxisASeparatedTest );
	RUN_SUBTEST( FaceAxisBSeparatedTest );
	RUN_SUBTEST( FaceFarSeparatedTest );
	RUN_SUBTEST( OffsetFaceAxisBTest );
	RUN_SUBTEST( EdgePairSweepTest );
	RUN_SUBTEST( SeparatingAxisOracleTest );
	RUN_SUBTEST( OffsetHullOracleTest );

	return 0;
}

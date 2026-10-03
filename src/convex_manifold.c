// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "algorithm.h"
#include "manifold.h"
#include "shape.h"
#include "simd.h"

#include "box3d/base.h"
#include "box3d/collision.h"
#include "box3d/constants.h"

#include <float.h>
#include <stdbool.h>
#include <stddef.h>

static int b3ClipSegment( b3ClipVertex segment[2], b3Plane plane )
{
	int vertexCount = 0;
	b3ClipVertex vertex1 = segment[0];
	b3ClipVertex vertex2 = segment[1];

	float distance1 = b3PlaneSeparation( plane, vertex1.position );
	float distance2 = b3PlaneSeparation( plane, vertex2.position );

	// If the points are behind the plane
	if ( distance1 <= 0.0f )
	{
		segment[vertexCount++] = vertex1;
	}
	if ( distance2 <= 0.0f )
	{
		segment[vertexCount++] = vertex2;
	}

	// If the points are on different sides of the plane
	if ( distance1 * distance2 < 0.0f )
	{
		// Find intersection point of edge and plane
		float t = distance1 / ( distance1 - distance2 );
		segment[vertexCount].position = b3Add( b3MulSV( 1.0f - t, vertex1.position ), b3MulSV( t, vertex2.position ) );
		segment[vertexCount].pair = distance1 > 0.0f ? vertex1.pair : vertex2.pair;
		vertexCount++;
	}

	return vertexCount;
}

static int b3ClipSegmentToHullFace( b3ClipVertex segment[2], const b3HullData* hull, int refFace )
{
	const b3HullFace* faces = b3GetHullFaces( hull );
	const b3Plane* planes = b3GetHullPlanes( hull );
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3Vec3* points = b3GetHullPoints( hull );

	b3Plane refPlane = planes[refFace];

	const b3HullFace* face = faces + refFace;

	int edgeIndex = face->edge;

	do
	{
		const b3HullHalfEdge* edge = edges + edgeIndex;
		int nextEdgeIndex = edge->next;
		const b3HullHalfEdge* next = edges + nextEdgeIndex;

		b3Vec3 vertex1 = points[edge->origin];
		b3Vec3 vertex2 = points[next->origin];
		b3Vec3 tangent = b3Normalize( b3Sub( vertex2, vertex1 ) );
		b3Vec3 binormal = b3Cross( tangent, refPlane.normal );

		int pointCount = b3ClipSegment( segment, b3MakePlaneFromNormalAndPoint( binormal, vertex1 ) );
		if ( pointCount < 2 )
		{
			return 0;
		}

		edgeIndex = nextEdgeIndex;
	}
	while ( edgeIndex != face->edge );

	return 2;
}

static b3SeparatingAxis b3QueryFaceDirectionHullAndCapsule( const b3HullData* hull, const b3Capsule* capsule,
															b3Transform capsuleTransform )
{
	int maxFaceIndex = -1;
	int maxVertexIndex = -1;
	float maxFaceSeparation = -FLT_MAX;
	const b3Plane* planes = b3GetHullPlanes( hull );

	b3Vec3 capsulePoints[2] = {
		b3TransformPoint( capsuleTransform, capsule->center1 ),
		b3TransformPoint( capsuleTransform, capsule->center2 ),
	};

	for ( int faceIndex = 0; faceIndex < hull->faceCount; ++faceIndex )
	{
		b3Plane plane = planes[faceIndex];

		int vertexIndex = b3GetPointSupport( capsulePoints, 2, b3Neg( plane.normal ) );
		b3Vec3 support = capsulePoints[vertexIndex];
		float separation = b3PlaneSeparation( plane, support );
		if ( separation > maxFaceSeparation )
		{
			maxVertexIndex = vertexIndex;
			maxFaceIndex = faceIndex;
			maxFaceSeparation = separation;
		}
	}

	return (b3SeparatingAxis){
		.normal = planes[maxFaceIndex].normal,
		.separation = maxFaceSeparation,
		.indexA = (uint8_t)maxFaceIndex,
		.indexB = (uint8_t)maxVertexIndex,
	};
}

static b3SeparatingAxis b3QueryEdgeDirectionHullAndCapsule( const b3HullData* hull, const b3Capsule* capsule,
															b3Transform capsuleTransform )
{
	// Find axis of minimum penetration
	b3Vec3 maxNormal = b3Vec3_zero;
	float maxSeparation = -FLT_MAX;
	int maxIndexA = B3_NULL_INDEX;
	int maxIndexB = B3_NULL_INDEX;

	// We perform all computations in local space of the hull
	b3Vec3 pA = b3TransformPoint( capsuleTransform, capsule->center1 );
	b3Vec3 qA = b3TransformPoint( capsuleTransform, capsule->center2 );
	b3Vec3 eA = b3Sub( qA, pA );

	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3Vec3* points = b3GetHullPoints( hull );
	const b3Plane* planes = b3GetHullPlanes( hull );
	float squaredTolerance = B3_PARALLEL_EDGE_TOL * B3_PARALLEL_EDGE_TOL;

	for ( int index = 0; index < hull->edgeCount; index += 2 )
	{
		const b3HullHalfEdge* edge = edges + index;
		const b3HullHalfEdge* twin = edges + index + 1;
		B3_ASSERT( edge->twin == index + 1 && twin->twin == index );

		b3Vec3 qB = points[twin->origin];
		b3Vec3 uB = planes[edge->face].normal;
		b3Vec3 vB = planes[twin->face].normal;

		// An isolated edge (e.g. like in a capsule) defines a circle through the
		// origin on the Gauss map. So testing for overlap between this circle and
		// the arc AB simplifies to a plane test.
		float cba = b3Dot( uB, eA );
		float dba = b3Dot( vB, eA );

		if ( cba * dba < 0.0f )
		{
			// Avoid nearly parallel edges that may lead to invalid separation values at the noise floor.
			if ( b3MaxFloat( cba * cba, dba * dba ) < squaredTolerance * b3LengthSquared( eA ) )
			{
				continue;
			}

			// The intersection of the arcs on the Gauss map is the edge pair axis. Cast the
			// arc of hull B (from uB to vB) against the plane containing the arc of hull A:
			// dot(uB + t * (vB - uB), eA) == 0
			// then
			// t = cba / (cba - dba)
			//
			// The signs of cba and dba differ (Minkowski test), so the division is safe.
			//
			// The axis generated points from B to A by construction since it lands between
			// two face normals on B. This removes the need to orient the separation axis
			// using the hull centers.
			//
			// The axis is perpendicular to both edges so I can use qA and qB as arbitrary
			// points on edgeA and edgeB to measure the separation.

			float t = cba / ( cba - dba );
			b3Vec3 axis = b3Lerp( uB, vB, t );
			B3_VALIDATE( b3LengthSquared( axis ) > 1000.0f * FLT_MIN );
			axis = b3Normalize( axis );
			float separation = b3Dot( axis, b3Sub( qA, qB ) );

			if ( separation > maxSeparation )
			{
				// Note: We don't exit early if we find a separating axis here since we want to
				// find the best one for caching and account for the convex radius later.
				maxNormal = axis;
				maxSeparation = separation;
				maxIndexA = 0;
				maxIndexB = index;
			}
		}
	}

	// Save result
	return (b3SeparatingAxis){
		.normal = maxNormal,
		.separation = maxSeparation,
		.indexA = maxIndexA,
		.indexB = maxIndexB,
	};
}

_Static_assert( B3_MAX_MANIFOLD_POINTS >= 4, "must be 4 or more" );

#if B3_MAX_MANIFOLD_POINTS == 4

// Reduce the manifold points to a maximum of 4 points.
// Note: this modifies the input point array to improve performance
static void b3ReduceManifoldPoints( b3LocalManifold* manifold, int capacity, b3LocalManifoldPoint* points, int count )
{
	if ( capacity < 4 )
	{
		return;
	}

	if ( count <= 4 )
	{
		for ( int i = 0; i < count; ++i )
		{
			manifold->points[i] = points[i];
		}

		manifold->pointCount = count;
		return;
	}

	b3Vec3 normal = manifold->normal;
	// float linearSlop = B3_LINEAR_SLOP;
	float speculativeDistance = B3_SPECULATIVE_DISTANCE;
	float tolSqr = speculativeDistance * speculativeDistance;

	// This bias is very important for contact point consistency across time steps.
	// It creates a pecking order to avoid flickering between candidates with similar scores.
	float bias = 0.95f;

	// Step 1: find extreme point that is touching
	int bestIndex = B3_NULL_INDEX;
	float bestScore = -FLT_MAX;

	// Arbitrary tangent direction
	// b3Vec3 perp1 = b3Perp( normal );
	// b3Vec3 perp2 = b3Cross( perp1, normal );
	// b3Vec3 searchDirection = -0.4535961214255773f * perp1 + 0.8912073600614354f * perp2;
	b3Vec3 searchDirection = b3ArbitraryPerp( normal );
	for ( int index = 0; index < count; ++index )
	{
		b3LocalManifoldPoint* pt = points + index;

		if ( pt->separation > speculativeDistance )
		{
			continue;
		}

		// The deeper the better
		float score = -pt->separation + b3Dot( searchDirection, pt->point );
		if ( bias * score > bestScore )
		{
			bestIndex = index;
			bestScore = score;
		}
	}

	B3_VALIDATE( 0 <= bestIndex && bestIndex < count );
	if ( bestIndex == B3_NULL_INDEX )
	{
		manifold->pointCount = 0;
		return;
	}

	manifold->points[0] = points[bestIndex];
	manifold->pointCount = 1;

	// Remove best point from array
	points[bestIndex] = points[count - 1];
	count -= 1;

	b3Vec3 a = manifold->points[0].point;

	// Step 2: Find farthest point in 2D
	bestScore = 0.0f;
	bestIndex = B3_NULL_INDEX;

	for ( int index = 0; index < count; ++index )
	{
		b3Vec3 p = points[index].point;
		b3Vec3 d = b3Sub( p, a );
		b3Vec3 v = b3MulSub( d, b3Dot( d, normal ), normal );
		float distanceSquared = b3LengthSquared( v );
		float separation = b3MaxFloat( 0.0f, -points[index].separation );
		float score = distanceSquared + 4.0f * separation * separation;
		if ( bias * score > bestScore )
		{
			bestScore = score;
			bestIndex = index;
		}
	}

	if ( bestScore < tolSqr )
	{
		return;
	}

	B3_ASSERT( 0 <= bestIndex && bestIndex < count );
	manifold->points[1] = points[bestIndex];
	manifold->pointCount = 2;

	// Remove best point from array
	points[bestIndex] = points[count - 1];
	count -= 1;

	b3Vec3 b = manifold->points[1].point;

	// Step 3: Find the point with the maximum triangular area
	bestScore = tolSqr;
	bestIndex = B3_NULL_INDEX;
	float bestSignedArea = 0.0f;
	b3Vec3 ba = b3Sub( b, a );
	for ( int index = 0; index < count; ++index )
	{
		b3Vec3 p = points[index].point;
		float signedArea = b3Dot( normal, b3Cross( ba, b3Sub( p, a ) ) );
		float score = b3AbsFloat( signedArea );
		if ( bias * score >= bestScore )
		{
			bestScore = score;
			bestIndex = index;
			bestSignedArea = signedArea;
		}
	}

	if ( bestIndex == B3_NULL_INDEX )
	{
		return;
	}

	B3_ASSERT( bestIndex != B3_NULL_INDEX );

	manifold->points[2] = points[bestIndex];
	manifold->pointCount = 3;
	points[bestIndex] = points[count - 1];
	count -= 1;

	b3Vec3 c = manifold->points[2].point;

	// Step 4: get the point that adds the most area outside the current triangle
	bestScore = tolSqr;
	bestIndex = B3_NULL_INDEX;
	float sign = bestSignedArea < 0.0f ? -1.0f : 1.0f;
	for ( int index = 0; index < count; ++index )
	{
		b3Vec3 p = points[index].point;
		float u1 = sign * b3Dot( normal, b3Cross( b3Sub( p, a ), ba ) );
		float u2 = sign * b3Dot( normal, b3Cross( b3Sub( p, b ), b3Sub( c, b ) ) );
		float u3 = sign * b3Dot( normal, b3Cross( b3Sub( p, c ), b3Sub( a, c ) ) );
		float score = b3MaxFloat( u1, b3MaxFloat( u2, u3 ) );

		if ( bias * score > bestScore )
		{
			bestScore = score;
			bestIndex = index;
		}
	}

	if ( bestIndex != B3_NULL_INDEX )
	{
		manifold->points[manifold->pointCount] = points[bestIndex];
		manifold->pointCount += 1;
	}
}

#else

static void b3ReduceManifoldPoints( b3LocalManifold* manifold, int capacity, b3LocalManifoldPoint* points, int count )
{
	if ( capacity < 4 )
	{
		return;
	}

	B3_ASSERT( count <= B3_MAX_CLIP_POINTS );

	int target = b3MinInt( capacity, B3_MAX_MANIFOLD_POINTS );

	if ( count <= target )
	{
		for ( int i = 0; i < count; ++i )
		{
			manifold->points[i] = points[i];
		}

		manifold->pointCount = count;
		return;
	}

	b3Vec3 normal = manifold->normal;
	b3Vec3 u = b3Perp( normal );
	b3Vec3 v = b3Cross( normal, u );
	b3Vec3 origin = points[0].point;

	b3Point2D pts[B3_MAX_CLIP_POINTS];
	for ( int i = 0; i < count; ++i )
	{
		b3Vec3 d = b3Sub( points[i].point, origin );
		pts[i].p = (b3Vec2){ b3Dot( d, u ), b3Dot( d, v ) };
		pts[i].separation = points[i].separation;
		pts[i].originalIndex = i;
	}

	b3Point2D hull[2 * B3_MAX_CLIP_POINTS];
	int hullCount = b3Hull2D( pts, count, hull );
	int finalCount = b3SimplifyHull2D( hull, hullCount, target );
	B3_ASSERT( 0 < finalCount && finalCount <= target );

	for ( int i = 0; i < finalCount; ++i )
	{
		int index = hull[i].originalIndex;
		B3_ASSERT( 0 <= index && index < count );
		manifold->points[i] = points[index];
	}

	manifold->pointCount = finalCount;
}

#endif

void b3CollideSpheres( b3LocalManifold* manifold, int capacity, const b3Sphere* sphereA, const b3Sphere* sphereB,
					   b3Transform transformBtoA )
{
	if ( capacity == 0 )
	{
		return;
	}

	// Work in shapeB coordinates
	b3Vec3 center1 = sphereA->center;
	b3Vec3 center2 = b3TransformPoint( transformBtoA, sphereB->center );

	float totalRadius = sphereA->radius + sphereB->radius;
	b3Vec3 offset = b3Sub( center2, center1 );
	float distanceSq = b3LengthSquared( offset );

	if ( distanceSq > totalRadius * totalRadius )
	{
		// We found a separating axis
		return;
	}

	b3Vec3 normal = { 0.0f, 1.0f, 0.0f };
	float distance = sqrtf( distanceSq );
	if ( distance * distance > 1000.0f * FLT_MIN )
	{
		normal = b3MulSV( 1.0f / distance, offset );
	}

	// Contact at the midpoint
	// 0.5 * ( ((c1 + rA*n) + c2) - rB*n )
	b3Vec3 point =
		b3MulSV( 0.5f, b3MulSub( b3Add( b3MulAdd( center1, sphereA->radius, normal ), center2 ), sphereB->radius, normal ) );

	// Manifold in frame B
	manifold->normal = normal;
	manifold->pointCount = 1;

	b3LocalManifoldPoint* pt = manifold->points + 0;
	pt->point = point;
	pt->separation = distance - totalRadius;
	pt->pair = b3FeaturePair_single;
}

void b3CollideCapsuleAndSphere( b3LocalManifold* manifold, int capacity, const b3Capsule* capsuleA, const b3Sphere* sphereB,
								b3Transform transformBtoA )
{
	manifold->pointCount = 0;

	if ( capacity < 1 )
	{
		return;
	}

	// Work in shape B coordinates
	b3Vec3 center = b3TransformPoint( transformBtoA, sphereB->center );
	b3Vec3 center1 = capsuleA->center1;
	b3Vec3 center2 = capsuleA->center2;

	float totalRadius = sphereB->radius + capsuleA->radius;

	b3Vec3 closestPoint = b3PointToSegmentDistance( center1, center2, center );
	b3Vec3 offset = b3Sub( center, closestPoint );
	float distanceSq = b3LengthSquared( offset );

	if ( distanceSq > totalRadius * totalRadius )
	{
		// We found a separating axis
		return;
	}

	b3Vec3 normal = { 0.0f, 1.0f, 0.0f };
	float distance = sqrtf( distanceSq );
	if ( distance * distance > 1000.0f * FLT_MIN )
	{
		normal = b3MulSV( 1.0f / distance, offset );
	}

	// Contact at the midpoint
	// 0.5 * (((center - sB*n) + closestPoint) + cA*n)
	b3Vec3 point =
		b3MulSV( 0.5f, b3MulAdd( b3Add( b3MulSub( center, sphereB->radius, normal ), closestPoint ), capsuleA->radius, normal ) );

	// Manifold in frame B
	manifold->normal = normal;
	manifold->pointCount = 1;

	b3LocalManifoldPoint* pt = manifold->points + 0;
	pt->point = point;
	pt->separation = distance - totalRadius;
	pt->pair = b3FeaturePair_single;
}

void b3CollideHullAndSphere( b3LocalManifold* manifold, int capacity, const b3HullData* hullA, const b3Sphere* sphereB,
							 b3Transform transformBtoA, b3SimplexCache* cache )
{
	manifold->pointCount = 0;

	if ( capacity == 0 )
	{
		return;
	}

	b3Vec3 center = b3TransformPoint( transformBtoA, sphereB->center );

	const float speculativeDistance = B3_SPECULATIVE_DISTANCE;

	// Work in shapeA coordinates

	b3DistanceInput distanceInput;
	distanceInput.proxyA = (b3ShapeProxy){ b3GetHullPoints( hullA ), hullA->vertexCount, 0.0f };
	distanceInput.proxyB = (b3ShapeProxy){ &center, 1, 0.0f };
	distanceInput.transform = b3Transform_identity;
	distanceInput.useRadii = false;

	float radiusA = 0.0f;
	float radiusB = sphereB->radius;
	float radius = radiusA + radiusB;

	b3DistanceOutput distanceOutput = b3ShapeDistance( &distanceInput, cache, NULL, 0 );

	if ( distanceOutput.distance > radius + speculativeDistance )
	{
		// We found a separating axis
		*cache = (b3SimplexCache){ 0 };
		return;
	}

	if ( distanceOutput.distance > 100.0f * FLT_EPSILON )
	{
		// Shallow penetration
		b3Vec3 normal = b3Normalize( b3Sub( distanceOutput.pointB, distanceOutput.pointA ) );

		// cA is the projection of the sphere center onto to the hull (pointA if radiusA == 0).
		b3Vec3 cA = b3MulAdd( center, radiusA - b3Dot( b3Sub( center, distanceOutput.pointA ), normal ), normal );

		// cB is the deepest point on the sphere with respect to the reference f
		b3Vec3 cB = b3MulSub( center, radiusB, normal );

		b3Vec3 point = b3Lerp( cA, cB, 0.5f );

		// Manifold in frame A
		manifold->normal = normal;
		manifold->pointCount = 1;

		b3LocalManifoldPoint* pt = manifold->points + 0;
		pt->point = point;
		pt->separation = distanceOutput.distance - radius;
		pt->pair = b3FeaturePair_single;
	}
	else
	{
		// Deep penetration
		int bestIndex = -1;
		float bestDistance = -FLT_MAX;
		const b3Plane* planes = b3GetHullPlanes( hullA );

		for ( int index = 0; index < hullA->faceCount; ++index )
		{
			b3Plane plane = planes[index];

			float distance = b3PlaneSeparation( plane, center );
			if ( distance > bestDistance )
			{
				bestIndex = index;
				bestDistance = distance;
			}
		}
		B3_ASSERT( bestIndex >= 0 );

		b3Vec3 normal = planes[bestIndex].normal;

		// cA is the projection of the sphere center onto to the hull
		b3Vec3 cA = b3MulAdd( center, radiusA - b3Dot( b3Sub( center, distanceOutput.pointA ), normal ), normal );

		// cB is the deepest point on the sphere with respect to the reference f
		b3Vec3 cB = b3MulSub( center, radiusB, normal );

		b3Vec3 point = b3Lerp( cA, cB, 0.5f );

		// Manifold in frame A
		manifold->normal = normal;
		manifold->pointCount = 1;

		b3LocalManifoldPoint* pt = manifold->points + 0;
		pt->point = point;
		pt->separation = bestDistance - radius;
		pt->pair = b3FeaturePair_single;
	}
}

void b3CollideCapsules( b3LocalManifold* manifold, int capacity, const b3Capsule* capsuleA, const b3Capsule* capsuleB,
						b3Transform transformBtoA )
{
	manifold->pointCount = 0;

	if ( capacity < 2 )
	{
		return;
	}

	// Work in shapeA coordinates
	b3Vec3 centerA1 = capsuleA->center1;
	b3Vec3 centerA2 = capsuleA->center2;
	b3Vec3 centerB1 = b3TransformPoint( transformBtoA, capsuleB->center1 );
	b3Vec3 centerB2 = b3TransformPoint( transformBtoA, capsuleB->center2 );

	float radius = capsuleA->radius + capsuleB->radius;
	float maxDistance = radius + B3_SPECULATIVE_DISTANCE;

	b3SegmentDistanceResult result = b3SegmentDistance( centerA1, centerA2, centerB1, centerB2 );
	b3Vec3 offset = b3Sub( result.point2, result.point1 );
	float distanceSquared = b3LengthSquared( offset );
	float linearSlop = B3_LINEAR_SLOP;
	float minDistance = 0.01f * linearSlop;

	if ( distanceSquared > maxDistance * maxDistance || distanceSquared < minDistance * minDistance )
	{
		// We found a separating axis
		return;
	}

	float lengthA;
	b3Vec3 segmentA = b3Sub( centerA2, centerA1 );
	b3Vec3 edgeA = b3GetLengthAndNormalize( &lengthA, segmentA );
	if ( lengthA < B3_MIN_CAPSULE_LENGTH )
	{
		return;
	}

	float lengthB;
	b3Vec3 segmentB = b3Sub( centerB2, centerB1 );
	b3Vec3 edgeB = b3GetLengthAndNormalize( &lengthB, segmentB );
	if ( lengthB < B3_MIN_CAPSULE_LENGTH )
	{
		return;
	}

	// Parallel edges: |eA x eB| = sin(alpha)
	const float alphaTol = 0.05f;
	const float alphaTolSqr = alphaTol * alphaTol;
	b3Vec3 axis = b3Cross( edgeA, edgeB );

	// Try to create two contact points if the capsules are nearly parallel
	if ( b3LengthSquared( axis ) < alphaTolSqr )
	{
		// Clip segment B against side planes of segment A

		// Sides planes of A
		b3Plane planesA[2];
		planesA[0].normal = b3Neg( edgeA );
		planesA[0].offset = -b3Dot( edgeA, capsuleA->center1 );
		planesA[1].normal = edgeA;
		planesA[1].offset = b3Dot( edgeA, capsuleA->center2 );

		// Clip points for B
		b3ClipVertex verticesB[2];
		verticesB[0].position = centerB1;
		verticesB[0].separation = 0.0f;
		verticesB[0].pair = b3MakeFeaturePair( b3_featureShapeA, 0, b3_featureShapeA, 0 );
		verticesB[1].position = centerB2;
		verticesB[1].separation = 0.0f;
		verticesB[1].pair = b3MakeFeaturePair( b3_featureShapeA, 1, b3_featureShapeA, 1 );

		int pointCount = b3ClipSegment( verticesB, planesA[0] );
		if ( pointCount == 2 )
		{
			pointCount = b3ClipSegment( verticesB, planesA[1] );
		}

		if ( pointCount == 2 )
		{
			// Closest points on A to the clipped points on B.
			b3Vec3 closestPoint1 = b3PointToSegmentDistance( centerA1, centerA2, verticesB[0].position );
			b3Vec3 closestPoint2 = b3PointToSegmentDistance( centerA1, centerA2, verticesB[1].position );

			float distance1 = b3Distance( closestPoint1, verticesB[0].position );
			float distance2 = b3Distance( closestPoint2, verticesB[1].position );
			if ( distance1 <= radius && distance2 <= radius )
			{
				if ( distance1 < minDistance || distance2 < minDistance )
				{
					// Avoid divide by zero
					return;
				}

				b3Vec3 normal1 = b3MulSV( 1.0f / distance1, b3Sub( verticesB[0].position, closestPoint1 ) );
				b3Vec3 normal2 = b3MulSV( 1.0f / distance2, b3Sub( verticesB[1].position, closestPoint2 ) );
				b3Vec3 normal = b3Normalize( b3Add( normal1, normal2 ) );
				float radiusA = capsuleA->radius;
				float radiusB = capsuleB->radius;

				// Contact is at the midpoint: 0.5 * (((vB.pos + rA*nK) + cP) - rB*n)
				b3Vec3 point1 =
					b3MulSV( 0.5f, b3MulSub( b3Add( b3MulAdd( verticesB[0].position, radiusA, normal1 ), closestPoint1 ), radiusB,
											 normal ) );
				b3Vec3 point2 =
					b3MulSV( 0.5f, b3MulSub( b3Add( b3MulAdd( verticesB[1].position, radiusA, normal2 ), closestPoint2 ), radiusB,
											 normal ) );

				// Manifold in frame A
				manifold->normal = normal;
				manifold->pointCount = 2;

				b3LocalManifoldPoint* pt1 = manifold->points + 0;
				pt1->point = point1;
				pt1->separation = distance1 - radius;
				pt1->pair = verticesB[0].pair;

				b3LocalManifoldPoint* pt2 = manifold->points + 1;
				pt2->point = point2;
				pt2->separation = distance2 - radius;
				pt2->pair = verticesB[1].pair;

				return;
			}
		}
	}

	float distance;
	b3Vec3 normal = b3GetLengthAndNormalize( &distance, offset );
	// Contact at the midpoint 0.5 * (((p1 + rA*n) + p2) - rB*n)
	b3Vec3 point = b3MulSV(
		0.5f, b3MulSub( b3Add( b3MulAdd( result.point1, capsuleA->radius, normal ), result.point2 ), capsuleB->radius, normal ) );

	// Manifold in frame A
	manifold->normal = normal;
	manifold->pointCount = 1;

	b3LocalManifoldPoint* pt = manifold->points + 0;
	pt->point = point;
	pt->separation = distance - radius;
	pt->pair = b3FeaturePair_single;
}

static bool b3BuildHullFaceAndCapsuleContact( b3LocalManifold* manifold, const b3HullData* hullA, const b3Capsule* capsuleB,
											  b3Transform transformBtoA, b3SeparatingAxis query )
{
	// Work in shapeA coordinates
	const b3Plane* planes = b3GetHullPlanes( hullA );

	// Clip the capsule edge against the side planes of the reference face
	int refFace = query.indexA;
	b3Plane refPlane = planes[refFace];

	b3ClipVertex segmentB[2];
	segmentB[0].position = b3TransformPoint( transformBtoA, capsuleB->center1 );
	segmentB[0].separation = 0.0f;
	segmentB[0].pair = b3MakeFeaturePair( b3_featureShapeA, 0, b3_featureShapeA, 0 );
	segmentB[1].position = b3TransformPoint( transformBtoA, capsuleB->center2 );
	segmentB[1].separation = 0.0f;
	segmentB[1].pair = b3MakeFeaturePair( b3_featureShapeA, 1, b3_featureShapeA, 1 );

	int pointCount = b3ClipSegmentToHullFace( segmentB, hullA, refFace );
	if ( pointCount < 2 )
	{
		return false;
	}

	float distance1 = b3PlaneSeparation( refPlane, segmentB[0].position );
	float distance2 = b3PlaneSeparation( refPlane, segmentB[1].position );
	const float speculativeDistance = B3_SPECULATIVE_DISTANCE;

	if ( distance1 <= speculativeDistance || distance2 <= speculativeDistance )
	{
		b3Vec3 normal = refPlane.normal;
		b3Vec3 point1 = b3MulSub( segmentB[0].position, 0.5f * ( distance1 + capsuleB->radius ), normal );
		b3Vec3 point2 = b3MulSub( segmentB[1].position, 0.5f * ( distance2 + capsuleB->radius ), normal );

		// Manifold in frame A
		manifold->normal = normal;
		manifold->pointCount = 2;

		b3LocalManifoldPoint* pt1 = manifold->points + 0;
		pt1->point = point1;
		pt1->separation = distance1 - capsuleB->radius;
		pt1->pair = segmentB[0].pair;

		b3LocalManifoldPoint* pt2 = manifold->points + 1;
		pt2->point = point2;
		pt2->separation = distance2 - capsuleB->radius;
		pt2->pair = segmentB[1].pair;

		return true;
	}

	return false;
}

static bool b3BuildHullAndCapsuleEdgeContact( b3LocalManifold* manifold, int capacity, const b3HullData* hullA,
											  const b3Capsule* capsuleB, b3Transform transformBtoA, b3SeparatingAxis query )
{
	if ( capacity < 1 )
	{
		return false;
	}

	// Work in shapeA coordinates

	b3Vec3 pc = b3TransformPoint( transformBtoA, capsuleB->center1 );
	b3Vec3 qc = b3TransformPoint( transformBtoA, capsuleB->center2 );
	b3Vec3 ec = b3Sub( qc, pc );

	const b3HullHalfEdge* edges = b3GetHullEdges( hullA );
	const b3Vec3* points = b3GetHullPoints( hullA );

	const b3HullHalfEdge* edge2 = edges + query.indexB;
	const b3HullHalfEdge* twin2 = edges + edge2->twin;
	b3Vec3 ph = points[edge2->origin];
	b3Vec3 qh = points[twin2->origin];
	b3Vec3 eh = b3Sub( qh, ph );

	b3Vec3 normal = query.normal;

	b3SegmentDistanceResult result = b3LineDistance( ph, eh, pc, ec );

	if ( b3IsWithinSegments( &result ) == false )
	{
		// closest point beyond end points
		return false;
	}

	b3Vec3 point = b3MulSV( 0.5f, b3Add( b3MulSub( result.point1, capsuleB->radius, normal ), result.point2 ) );
	float separation = b3Dot( normal, b3Sub( result.point2, result.point1 ) );
	B3_VALIDATE( b3AbsFloat( separation - query.separation ) < B3_LINEAR_SLOP );

	// Manifold in frame A
	manifold->normal = normal;
	manifold->pointCount = 1;

	b3LocalManifoldPoint* pt = manifold->points + 0;
	pt->point = point;
	pt->separation = separation - capsuleB->radius;
	pt->pair = b3MakeFeaturePair( b3_featureShapeA, query.indexA, b3_featureShapeB, query.indexB );
	return true;
}

void b3CollideHullAndCapsule( b3LocalManifold* manifold, int capacity, const b3HullData* hullA, const b3Capsule* capsuleB,
							  b3Transform transformBtoA, b3SimplexCache* cache )
{
	manifold->pointCount = 0;

	if ( capacity < 2 )
	{
		return;
	}

	// Work in shapeA coordinates
	b3DistanceInput distanceInput;
	distanceInput.proxyA = (b3ShapeProxy){ b3GetHullPoints( hullA ), hullA->vertexCount, 0.0f };
	distanceInput.proxyB = (b3ShapeProxy){ &capsuleB->center1, 2, 0.0f };
	distanceInput.transform = transformBtoA;
	distanceInput.useRadii = false;

	b3DistanceOutput distanceOutput = b3ShapeDistance( &distanceInput, cache, NULL, 0 );
	const float speculativeDistance = B3_SPECULATIVE_DISTANCE;

	if ( distanceOutput.distance > capsuleB->radius + speculativeDistance )
	{
		// We found a separating axis
		*cache = (b3SimplexCache){ 0 };
		return;
	}

	if ( distanceOutput.distance > 100.0f * FLT_EPSILON )
	{
		const b3Plane* planes = b3GetHullPlanes( hullA );

		// Shallow penetration
		b3Vec3 delta = distanceOutput.normal;
		int refFace = b3FindHullSupportFace( hullA, delta );
		b3Plane refPlane = planes[refFace];

		// Try to create two contact points if closest
		// points difference is nearly parallel to face normal
		const float kTolerance = 0.998f;
		if ( b3AbsFloat( b3Dot( refPlane.normal, delta ) ) > kTolerance )
		{
			// Clip capsule segment against side planes of reference face
			b3ClipVertex verticesB[2];
			verticesB[0].position = b3TransformPoint( transformBtoA, capsuleB->center1 );
			verticesB[0].separation = 0.0f;
			verticesB[0].pair = b3MakeFeaturePair( b3_featureShapeA, 0, b3_featureShapeA, 0 );
			verticesB[1].position = b3TransformPoint( transformBtoA, capsuleB->center2 );
			verticesB[1].separation = 0.0f;
			verticesB[1].pair = b3MakeFeaturePair( b3_featureShapeA, 1, b3_featureShapeA, 1 );

			int pointCount = b3ClipSegmentToHullFace( verticesB, hullA, refFace );

			if ( pointCount == 2 )
			{
				float distance1 = b3PlaneSeparation( refPlane, verticesB[0].position );
				float distance2 = b3PlaneSeparation( refPlane, verticesB[1].position );
				if ( distance1 <= capsuleB->radius + speculativeDistance || distance2 <= capsuleB->radius + speculativeDistance )
				{
					b3Vec3 normal = refPlane.normal;
					b3Vec3 point1 = b3MulSub( verticesB[0].position, 0.5f * ( capsuleB->radius + distance1 ), normal );
					b3Vec3 point2 = b3MulSub( verticesB[1].position, 0.5f * ( capsuleB->radius + distance2 ), normal );

					// Manifold in frame A
					manifold->normal = normal;
					manifold->pointCount = 2;

					b3LocalManifoldPoint* pt1 = manifold->points + 0;
					pt1->point = point1;
					pt1->separation = distance1 - capsuleB->radius;
					pt1->pair = verticesB[0].pair;

					b3LocalManifoldPoint* pt2 = manifold->points + 1;
					pt2->point = point2;
					pt2->separation = distance2 - capsuleB->radius;
					pt2->pair = verticesB[1].pair;

					return;
				}
			}
		}

		// Create contact from closest points
		b3Vec3 point =
			b3MulSV( 0.5f, b3Add( b3MulSub( distanceOutput.pointA, capsuleB->radius, delta ), distanceOutput.pointB ) );

		// Manifold in frame A
		manifold->normal = delta;
		manifold->pointCount = 1;

		b3LocalManifoldPoint* pt = manifold->points + 0;
		pt->point = point;
		pt->separation = distanceOutput.distance - capsuleB->radius;
		pt->pair = b3FeaturePair_single;
		return;
	}

	// Deep penetration

	b3SeparatingAxis faceQuery = b3QueryFaceDirectionHullAndCapsule( hullA, capsuleB, transformBtoA );
	if ( faceQuery.separation > capsuleB->radius )
	{
		// We found a separating axis
		return;
	}

	b3SeparatingAxis edgeQuery = b3QueryEdgeDirectionHullAndCapsule( hullA, capsuleB, transformBtoA );
	if ( edgeQuery.separation > capsuleB->radius )
	{
		// We found a separating axis
		return;
	}

	// Create face contact
	float faceSeparation = faceQuery.separation - capsuleB->radius;
	b3BuildHullFaceAndCapsuleContact( manifold, hullA, capsuleB, transformBtoA, faceQuery );
	B3_VALIDATE( manifold->pointCount == 0 || manifold->pointCount == 2 );
	if ( manifold->pointCount == 2 )
	{
		// This becomes the clipped separation.
		faceSeparation = b3MinFloat( manifold->points[0].separation, manifold->points[1].separation );
	}

	// Is there a valid edge-edge axis?
	if ( edgeQuery.indexA == B3_NULL_INDEX )
	{
		return;
	}

	// Face contact can be empty if it does not realize the axis of minimum penetration.
	// Create edge contact if face contact fails or edge contact is significantly better!
	float linearSlop = B3_LINEAR_SLOP;
	float edgeSeparation = edgeQuery.separation - capsuleB->radius;
	if ( manifold->pointCount == 0 || edgeSeparation > faceSeparation + linearSlop )
	{
		// Edge contact
		b3BuildHullAndCapsuleEdgeContact( manifold, capacity, hullA, capsuleB, transformBtoA, edgeQuery );
	}
}

static int b3BuildPolygon( b3ClipVertex* out, b3Transform transform, const b3HullData* hull, int incFace, b3Plane refPlane )
{
	const b3HullFace* faces = b3GetHullFaces( hull );
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3Vec3* points = b3GetHullPoints( hull );

	const b3HullFace* face = faces + incFace;
	int edgeIndex = face->edge;
	B3_ASSERT( edges[edgeIndex].face == incFace );

	int outCount = 0;

	b3Matrix3 matrix = b3MakeMatrixFromQuat( transform.q );

	do
	{
		const b3HullHalfEdge* edge = edges + edgeIndex;

		int nextEdgeIndex = edge->next;
		const b3HullHalfEdge* next = edges + nextEdgeIndex;

		b3ClipVertex vertex;
		vertex.position = b3Add( b3MulMV( matrix, points[next->origin] ), transform.p );
		vertex.separation = b3PlaneSeparation( refPlane, vertex.position );
		vertex.pair = b3MakeFeaturePair( b3_featureShapeB, edgeIndex, b3_featureShapeB, nextEdgeIndex );

		out[outCount] = vertex;
		outCount += 1;

		edgeIndex = nextEdgeIndex;
	}
	while ( edgeIndex != face->edge && outCount < B3_MAX_CLIP_POINTS );

	B3_VALIDATE( b3ValidatePolygon( out, outCount ) );

	return outCount;
}

static bool b3BuildFaceAContact( b3LocalManifold* manifold, int capacity, const b3HullData* hullA, const b3HullData* hullB,
								 b3Transform transformBtoA, b3SeparatingAxis query, b3SATCache* cache )
{
	B3_ASSERT( capacity > 0 );
	B3_VALIDATE( query.type == b3_faceAxisA );
	B3_VALIDATE( 0 <= query.indexA && query.indexA < hullA->faceCount );
	B3_VALIDATE( 0 <= query.indexB && query.indexB < hullB->vertexCount );

	const b3HullFace* facesA = b3GetHullFaces( hullA );
	const b3HullHalfEdge* edgesA = b3GetHullEdges( hullA );
	const b3Plane* planesA = b3GetHullPlanes( hullA );
	const b3Vec3* pointsA = b3GetHullPoints( hullA );

	// Reference face
	int refFace = query.indexA;
	b3Plane refPlane = planesA[refFace];

	// Find incident face
	b3Vec3 refNormalInB = b3InvRotateVector( transformBtoA.q, refPlane.normal );
	int incFace = b3FindIncidentFace( hullB, refNormalInB, query.indexB );

	// Build clip polygon from incident face in frame A
	b3ClipVertex buffer1[B3_MAX_CLIP_POINTS], buffer2[B3_MAX_CLIP_POINTS];
	int pointCount = b3BuildPolygon( buffer1, transformBtoA, hullB, incFace, refPlane );

	// Clip incident face against side planes of reference face
	b3ClipVertex* input = buffer1;
	b3ClipVertex* output = buffer2;

	const b3HullFace* face = facesA + refFace;
	int edgeIndex = face->edge;

	do
	{
		const b3HullHalfEdge* edge = edgesA + edgeIndex;
		int nextEdgeIndex = edge->next;
		const b3HullHalfEdge* next = edgesA + nextEdgeIndex;
		b3Vec3 vertex1 = pointsA[edge->origin];
		b3Vec3 vertex2 = pointsA[next->origin];
		b3Vec3 tangent = b3Normalize( b3Sub( vertex2, vertex1 ) );
		b3Vec3 binormal = b3Cross( tangent, refPlane.normal );

		b3Plane clipPlane = b3MakePlaneFromNormalAndPoint( binormal, vertex1 );

		pointCount = b3ClipPolygon( output, input, pointCount, clipPlane, edgeIndex, refPlane );
		B3_ASSERT( pointCount <= B3_MAX_CLIP_POINTS );

		B3_SWAP( output, input );

		if ( pointCount < 3 )
		{
			*cache = (b3SATCache){ 0 };
			return false;
		}

		edgeIndex = nextEdgeIndex;
	}
	while ( edgeIndex != face->edge );

	pointCount = b3MinInt( pointCount, B3_MAX_CLIP_POINTS );

	b3LocalManifoldPoint points[B3_MAX_CLIP_POINTS];
	float minSeparation = FLT_MAX;

	manifold->normal = refPlane.normal;

	for ( int i = 0; i < pointCount; ++i )
	{
		b3ClipVertex* clipPoint = input + i;
		b3LocalManifoldPoint* pt = points + i;
		*pt = (b3LocalManifoldPoint){ 0 };

		// Using the half-way point keeps the points in the same position when swapping reference face from A to B.
		b3Vec3 point = b3MulSub( clipPoint->position, 0.5f * clipPoint->separation, refPlane.normal );

		// Old way of pushing onto the reference face.
		// b3Vec3 point = clipPoint->position - clipPoint->separation * refPlane.normal;

		pt->point = point;
		pt->separation = clipPoint->separation;
		pt->pair = clipPoint->pair;

		minSeparation = b3MinFloat( minSeparation, clipPoint->separation );
	}

	if ( minSeparation >= B3_SPECULATIVE_DISTANCE )
	{
		*cache = (b3SATCache){ 0 };
		return false;
	}

	b3ReduceManifoldPoints( manifold, capacity, points, pointCount );

	// Save cache
	cache->separation = minSeparation;
	cache->type = (uint8_t)b3_faceAxisA;
	cache->indexA = (uint8_t)query.indexA;
	cache->indexB = (uint8_t)query.indexB;

	return true;
}

static bool b3BuildFaceBContact( b3LocalManifold* manifold, int capacity, const b3HullData* hullA, const b3HullData* hullB,
								 b3Transform transformBtoA, b3SeparatingAxis query, b3SATCache* cache )
{
	B3_VALIDATE( query.type == b3_faceAxisB );

	b3Transform transformAtoB = b3InvertTransform( transformBtoA );
	b3SeparatingAxis flippedQuery = {
		.normal = b3Neg( query.normal ),
		.separation = query.separation,
		.indexA = query.indexB,
		.indexB = query.indexA,
		.type = b3_faceAxisA,
	};

	bool touching = b3BuildFaceAContact( manifold, capacity, hullB, hullA, transformAtoB, flippedQuery, cache );
	if ( touching == false )
	{
		*cache = (b3SATCache){ 0 };
		return false;
	}

	// Results are in frame B, need to transform them into frame A
	b3Matrix3 matrix = b3MakeMatrixFromQuat( transformBtoA.q );

	// Transform and flip normal so it points from A to B, even though the B has the reference face.
	manifold->normal = b3Neg( b3MulMV( matrix, manifold->normal ) );

	// Transform points from frame B to frame A.
	// Also flip the pairs to ensure correct matches.
	for ( int i = 0; i < manifold->pointCount; ++i )
	{
		b3LocalManifoldPoint* pt = manifold->points + i;
		pt->point = b3Add( b3MulMV( matrix, pt->point ), transformBtoA.p );
		pt->pair = b3FlipPair( pt->pair );
	}

	cache->type = (uint8_t)b3_faceAxisB;
	cache->indexA = (uint8_t)query.indexA;
	cache->indexB = (uint8_t)query.indexB;

	return true;
}

static bool b3BuildEdgeContact( b3LocalManifold* manifold, const b3HullData* hullA, const b3HullData* hullB,
								b3Transform transformBtoA, b3SeparatingAxis query, b3SATCache* cache )
{
	B3_VALIDATE( query.type == b3_edgePairAxis );
	B3_VALIDATE( 0 <= query.indexA && query.indexA < hullA->edgeCount );
	B3_VALIDATE( 0 <= query.indexB && query.indexB < hullB->edgeCount );

	// Work in shapeA coordinates
	const b3HullHalfEdge* edgesA = b3GetHullEdges( hullA );
	const b3Vec3* pointsA = b3GetHullPoints( hullA );

	const b3HullHalfEdge* edgesB = b3GetHullEdges( hullB );
	const b3Vec3* pointsB = b3GetHullPoints( hullB );

	// B3_VALIDATE( query.separation <= 2.0f * B3_SPECULATIVE_DISTANCE );

	const b3HullHalfEdge* edgeA = edgesA + query.indexA;
	const b3HullHalfEdge* twinA = edgesA + edgeA->twin;
	b3Vec3 pA = pointsA[edgeA->origin];
	b3Vec3 qA = pointsA[twinA->origin];
	b3Vec3 eA = b3Sub( qA, pA );

	const b3HullHalfEdge* edgeB = edgesB + query.indexB;
	const b3HullHalfEdge* twinB = edgesB + edgeB->twin;
	b3Vec3 pB = b3TransformPoint( transformBtoA, pointsB[edgeB->origin] );
	b3Vec3 qB = b3TransformPoint( transformBtoA, pointsB[twinB->origin] );
	b3Vec3 eB = b3Sub( qB, pB );

	b3Vec3 normal = query.normal;
	b3SegmentDistanceResult result = b3LineDistance( pA, eA, pB, eB );

	if ( b3IsWithinSegments( &result ) == false )
	{
		*cache = (b3SATCache){ 0 };
		return false;
	}

	// This can slide off the end from caching
	float separation = b3Dot( normal, b3Sub( result.point2, result.point1 ) );
	b3Vec3 point = b3MulSV( 0.5f, b3Add( result.point1, result.point2 ) );

	// Result in frame A
	manifold->normal = normal;
	manifold->pointCount = 1;

	b3LocalManifoldPoint* pt = manifold->points + 0;
	pt->point = point;
	pt->separation = separation;
	pt->pair = b3MakeFeaturePair( b3_featureShapeA, query.indexA, b3_featureShapeB, query.indexB );

	// Save cache
	cache->separation = separation;
	cache->type = (uint8_t)b3_edgePairAxis;
	cache->indexA = (uint8_t)query.indexA;
	cache->indexB = (uint8_t)query.indexB;

	return true;
}

// Transform a SoA point/normal stream (already split into X/Y/Z) by out = -(R*v (+t)).
// The inputs come straight from the hull's stored SoA arrays, so there's no transpose here.
static inline void b3NegativeTransformFromSoA( b3Matrix3 R, b3Vec3 p, const float* inX, const float* inY, const float* inZ,
											   int n, float* outX, float* outY, float* outZ, bool isPoint )
{
	// row-column
	b3Vec3W r0 = { b3SplatW( R.cx.x ), b3SplatW( R.cy.x ), b3SplatW( R.cz.x ) };
	b3Vec3W r1 = { b3SplatW( R.cx.y ), b3SplatW( R.cy.y ), b3SplatW( R.cz.y ) };
	b3Vec3W r2 = { b3SplatW( R.cx.z ), b3SplatW( R.cy.z ), b3SplatW( R.cz.z ) };

	b3Vec3W t = { b3ZeroW(), b3ZeroW(), b3ZeroW() };
	if ( isPoint )
	{
		t = b3SplatVW( p );
	}

	for ( int i = 0; i < n; i += 4 )
	{
		b3Vec3W v = b3LoadVW( inX + i, inY + i, inZ + i );

		// Rotate four vectors at a time
		b3Vec3W out = { b3DotW( r0, v ), b3DotW( r1, v ), b3DotW( r2, v ) };

		if ( isPoint )
		{
			out = b3AddVW( out, t );
		}

		b3StoreVW( outX + i, outY + i, outZ + i, b3NegVW( out ) );
	}
}

_Static_assert( B3_MAX_HULL_VERTICES == 128, "must be 128" );

#define B3_HULL_BIT_COUNT 7

// SIMD support point calculation using a SoA vertex array padded with repeats of the first vertex
// to a multiple of 4.
//
// This minimizes (bias - dot), where the caller is expected to provide a bias that makes this always positive.
// It can be direction dependent. The bias should be just big enough to ensure the value is positive because
// an excessive bias causes a precision loss in the support calculation.
//
// The vertex index is embedded in the low B3_HULL_BIT_COUNT mantissa bits of the value. By minimizing a value that
// is always positive, the minimum carries the smallest index so that padded SoA values will never win. This is the
// purpose of using the bias instead of maximizing the dot directly.
//
// The support is then recomputed exactly as dot(normal, vertex), without the embedded index.
// todo consider using this for GJK
static inline void b3GetSupportWide( b3Vec3 normal, const float* vx, const float* vy, const float* vz, int n, float bias,
									 float* support, int* vertexIndex )
{
	const b3Vec3W normalW = b3SplatVW( normal );
	const b3FloatW biasV = b3SplatW( bias );

	// Start the minimum at a large value.
	b3FloatW minValue = b3SplatW( INFINITY );

	// Tail lanes hold vertex 0 with index bits >= vertexCount, so they never become the min value.
	for ( int i = 0; i < n; i += 4 )
	{
		b3Vec3W v = b3LoadVW( vx + i, vy + i, vz + i );
		b3FloatW d = b3DotW( normalW, v );

		// This is always positive.
		b3FloatW value = b3SubW( biasV, d );
		b3FloatW augmentedValue = b3EmbedIndexW( value, i, B3_HULL_BIT_COUNT );
		minValue = b3MinW( minValue, augmentedValue );
	}

	// One horizontal min, the winning lane's value and index bits ride through.
	int vi = b3MinIndexW( minValue, B3_HULL_BIT_COUNT );

	// Exact support for the chosen vertex.
	*vertexIndex = vi;

	// Dot product
	*support = normal.x * vx[vi] + normal.y * vy[vi] + normal.z * vz[vi];
}

static inline float b3GetFaceSeparation( b3Vec3 direction, float planeSeparation, const float* vx, const float* vy,
										 const float* vz, int n, b3Vec3 center, b3Vec3 extents, int* vertexIndex )
{
	float bias = b3Dot( direction, center ) + 1.0625f * b3Dot( b3Abs( direction ), extents );
	float support;
	b3GetSupportWide( direction, vx, vy, vz, n, bias, &support, vertexIndex );
	return planeSeparation - support;
}

// Wide dot(n, d) for all face normals n of the hull, padded to the SIMD width.
static inline void b3GetFaceDots( const b3HullData* hull, b3Vec3 d, float* dots )
{
	int soaFaceCount = ( hull->faceCount + 3 ) & ~3;
	const float* nx = b3GetHullSoaNormals( hull );
	const float* ny = nx + soaFaceCount;
	const float* nz = ny + soaFaceCount;

	b3Vec3W dW = b3SplatVW( d );

	for ( int i = 0; i < soaFaceCount; i += 4 )
	{
		// dot product per lane
		b3FloatW m = b3DotW( b3LoadVW( nx + i, ny + i, nz + i ), dW );
		b3StoreW( dots + i, m );
	}
}

#define B3_PARALLEL_TOL 1e-4f

// Inscribed sphere edge test. https://box2d.org/posts/2026/09/inscribed-spheres/
// The implementation here is a mix of the versions from Cairn Overturf and Dirk Gregorius:
// https://gist.github.com/cairnc/dee7a2866da0709f2d9a77b6493b57b5
// https://gist.github.com/dgregorius/e6751b5c00937cd21af63cba3c53c861
// This benchmarks faster than the version from the blog post.
static inline int b3TestEdgeCandidateSorted( float a1, float a2, float c, float bound )
{
	// We want to maximize
	//
	//   f(t) = dot(d, nlerp(n1, n2, t))
	//
	// over t in [0, 1], where
	//
	//   a1 = dot(n1, d)
	//   a2 = dot(n2, d)
	//   c  = dot(n1, n2).
	//
	// The maximum can occur either at an endpoint or at an interior
	// critical point where f'(t) = 0.

	// Since the endpoint values are a1 and a2, only the larger endpoint
	// needs to be tested against the bound.
	float hi = b3MaxFloat( a1, a2 );
	float lo = b3MinFloat( a1, a2 );
	int exterior = hi >= bound;

	// Differentiating f(t) gives
	//
	//           a2 - c*a1 - (1 - c)*(a1 + a2)*t
	//   f'(t) = --------------------------------------
	//           ((1 - t)^2 + t^2 + 2*c*t*(1 - t))^(3/2)
	//
	// The denominator is positive and the numerator is linear in t.
	// An interior maximum therefore requires
	//
	//   f'(0) >= 0  ->  a2 >= c*a1
	//   f'(1) <= 0  ->  a1 >= c*a2.
	//
	// After sorting these become
	//
	//   hi >= c*lo
	//   lo >= c*hi.
	//
	// The second is always the stricter condition since
	//
	//   (hi - c*lo) - (lo - c*hi)
	//       = (1 + c)*(hi - lo) >= 0.
	//
	// Thus both conditions reduce to u >= 0.
	float u = lo - c * hi;
	int maxIsInterior = u >= 0.0f;

	// Solving f'(t) = 0 and evaluating f(t) at tmax gives
	//
	//   f(tmax)^2 =
	//       (a1^2 + a2^2 - 2*c*a1*a2) / (1 - c^2).
	//
	// After sorting,
	//
	//   hi^2 + lo^2 - 2*c*hi*lo
	//       = hi^2*(1 - c^2) + u^2.
	//
	// Let s = 1 - c^2 and rearrange f(tmax) >= bound:
	//
	//   u^2 >= (bound^2 - hi^2)*s.
	float s = 1.0f - c * c;
	float boundTerm = ( bound - hi ) * ( bound + hi );
	float lhs = u * u;
	float rhs = boundTerm * s;
	int maxBeatsBound = lhs >= rhs;

	// The interior expression contains 1 - c^2 in its denominator.
	// When this approaches zero the arc is degenerate or ill-conditioned,
	// so conservatively keep the edge as a candidate.
	int nearlyParallel = s < B3_PARALLEL_TOL;

	int interior = maxIsInterior & ( maxBeatsBound | nearlyParallel );

	return exterior | interior;
}

// dot(n, otherCenter) - offset for all faces of the hull, padded to the SIMD width.
static inline void b3GetFacePlaneSeparations( const b3HullData* hull, b3Vec3 otherCenter, float* separations )
{
	b3GetFaceDots( hull, otherCenter, separations );

	const b3Plane* planes = b3GetHullPlanes( hull );
	int faceCount = hull->faceCount;
	for ( int i = 0; i < faceCount; ++i )
	{
		separations[i] -= planes[i].offset;
	}
}

// The number of edge pair tests needed to make the extra culling pass worthwhile.
#define B3_EDGE_PROBE_MIN_TESTS 10

// Re-test edge candidates against the plane separations of a probe point on the other hull
static inline int b3FilterEdgeCandidates( const b3HullData* hull, const float* planeSeparations, float bound, int* edgeIndices,
										  int count )
{
	const b3HullHalfEdge* halfEdges = b3GetHullEdges( hull );
	const float* cosines = b3GetHullEdgeCosines( hull );
	int keptCount = 0;

	for ( int k = 0; k < count; ++k )
	{
		int i = edgeIndices[k];
		int i1 = halfEdges[i].face;
		int i2 = halfEdges[i + 1].face;
		float c = cosines[i >> 1];
		edgeIndices[keptCount] = i;
		keptCount += b3TestEdgeCandidateSorted( planeSeparations[i1], planeSeparations[i2], c, bound );
	}

	return keptCount;
}

// Temporary abbreviations for convenience.
#define NE ( B3_MAX_HULL_EDGES + B3_SIMD_WIDTH )
#define NF ( B3_MAX_HULL_FACES + B3_SIMD_WIDTH )
#define NV ( B3_MAX_HULL_VERTICES + B3_SIMD_WIDTH )

// SIMD separating axis test based on an implementation developed by Cairn Overturf.
// See his article: https://cairnc.github.io/posts/improvements-to-the-separating-axis/
b3AxisQuery b3ComputeSeparatingAxis( const b3HullData* hullA, const b3HullData* hullB, b3Transform xfB, bool earlyReturn )
{
	b3Matrix3 R = b3MakeMatrixFromQuat( xfB.q );
	b3Matrix3 invR = b3Transpose( R );

	float speculativeDistance = B3_SPECULATIVE_DISTANCE;

	b3AxisQuery res = {
		.faceA =
			{
				.normal = b3Vec3_zero,
				.separation = -INFINITY,
				.indexA = B3_NULL_INDEX,
				.indexB = B3_NULL_INDEX,
				.type = b3_faceAxisA,
			},
		.faceB =
			{
				.normal = b3Vec3_zero,
				.separation = -INFINITY,
				.indexA = B3_NULL_INDEX,
				.indexB = B3_NULL_INDEX,
				.type = b3_faceAxisB,
			},
		.edge =
			{
				.normal = b3Vec3_zero,
				.separation = -INFINITY,
				.indexA = B3_NULL_INDEX,
				.indexB = B3_NULL_INDEX,
				.type = b3_edgePairAxis,
			},
		.separatedFeature = b3_invalidAxis,
	};

	int faceCountA = hullA->faceCount;
	const b3Plane* planesA = b3GetHullPlanes( hullA );

	int soaVertexCountB = ( hullB->vertexCount + 3 ) & ~3;
	const float* vxB = b3GetHullSoaVertices( hullB );
	const float* vyB = vxB + soaVertexCountB;
	const float* vzB = vyB + soaVertexCountB;

	b3Vec3 cB = b3AABB_Center( hullB->aabb );
	b3Vec3 hB = b3AABB_Extents( hullB->aabb );

	// The hulls have a precomputed inner radius and centroid.
	// A given axis cannot achieve a separation larger than:
	// dot(axis, centerB - centerA) - innerRadiusA - innerRadiusB
	// So this is the upper bound for the separation of a candidate axis.
	// An axis can be skipped if it has an upper bound that is less than the current
	// best separation. This lets me skip many of the candidates without computing
	// support points.

	b3Vec3 deltaCenter = b3Sub( b3Add( b3MulMV( R, hullB->center ), xfB.p ), hullA->center );
	float centerDistance = b3Length( deltaCenter );
	float radius = hullA->innerRadius + hullB->innerRadius;

	// Adjust the radius to ensure the best axis isn't skipped.
	float radiusBound = radius - ( B3_LINEAR_SLOP + 0.001f * ( centerDistance + radius ) );

	// Compute dot(normalA, centerDelta) for all face normals of hullA.
	_Alignas( 16 ) float dotA[NF];
	b3GetFaceDots( hullA, deltaCenter, dotA );

	// Find the face of hullA that most aligns with centerDelta.
	int seedIndexA = 0;
	float maxDotA = dotA[0];
	for ( int i = 1; i < faceCountA; ++i )
	{
		if ( dotA[i] > maxDotA )
		{
			maxDotA = dotA[i];
			seedIndexA = i;
		}
	}

	// Use the seed to get a lower bound on the separation for the faces of hullA.
	float floorA = -INFINITY;
	float seedSeparationA = -INFINITY;
	int seedVertexB = 0;
	if ( earlyReturn )
	{
		b3Plane plane = planesA[seedIndexA];
		b3Vec3 direction = b3Neg( b3MulMV( invR, plane.normal ) );
		float planeSeparation = b3Dot( plane.normal, xfB.p ) - plane.offset;
		seedSeparationA =
			b3GetFaceSeparation( direction, planeSeparation, vxB, vyB, vzB, soaVertexCountB, cB, hB, &seedVertexB );
		floorA = b3MinFloat( seedSeparationA, speculativeDistance );
	}

	// Test A's face planes against B's vertices.
	for ( int i = 0; i < faceCountA; ++i )
	{
		// The bound offset ensures the seed will be evaluated.
		if ( dotA[i] - radiusBound < b3MaxFloat( floorA, res.faceA.separation ) )
		{
			continue;
		}

		b3Plane plane = planesA[i];
		int vertexIndex = seedVertexB;
		float separation = seedSeparationA;

		// Avoid recomputing the seed face separation.
		if ( earlyReturn == false || i != seedIndexA )
		{
			b3Vec3 direction = b3Neg( b3MulMV( invR, plane.normal ) );
			float planeSeparation = b3Dot( plane.normal, xfB.p ) - plane.offset;
			separation =
				b3GetFaceSeparation( direction, planeSeparation, vxB, vyB, vzB, soaVertexCountB, cB, hB, &vertexIndex );
		}

		if ( separation > res.faceA.separation )
		{
			res.faceA.normal = plane.normal;
			res.faceA.separation = separation;
			res.faceA.indexA = i;
			res.faceA.indexB = vertexIndex;
			if ( separation > speculativeDistance && earlyReturn )
			{
				res.separatedFeature = b3_faceAxisA;
				return res;
			}
		}
	}

	B3_VALIDATE( res.faceA.indexA != B3_NULL_INDEX );

	int faceCountB = hullB->faceCount;
	const b3Plane* planesB = b3GetHullPlanes( hullB );

	int soaVertexCountA = ( hullA->vertexCount + 3 ) & ~3;
	const float* vxA = b3GetHullSoaVertices( hullA );
	const float* vyA = vxA + soaVertexCountA;
	const float* vzA = vyA + soaVertexCountA;

	b3Vec3 cA = b3AABB_Center( hullA->aabb );
	b3Vec3 hA = b3AABB_Extents( hullA->aabb );

	// Similarly, find the face of hullB that most aligns with the vector pointing from centerB to centerA.
	_Alignas( 16 ) float dotB[NF];
	b3GetFaceDots( hullB, b3Neg( b3MulMV( invR, deltaCenter ) ), dotB );

	int seedIndexB = 0;
	float maxDotB = dotB[0];
	for ( int i = 1; i < faceCountB; ++i )
	{
		if ( dotB[i] > maxDotB )
		{
			maxDotB = dotB[i];
			seedIndexB = i;
		}
	}

	// Get a lower bound on the separation for the faces of hullB.
	float floorB = -INFINITY;
	float seedSeparationB = -INFINITY;
	int seedVertexA = 0;
	if ( earlyReturn )
	{
		b3Plane plane = planesB[seedIndexB];
		b3Vec3 direction = b3Neg( b3MulMV( R, plane.normal ) );
		float planeSeparation = b3Dot( direction, xfB.p ) - plane.offset;
		seedSeparationB =
			b3GetFaceSeparation( direction, planeSeparation, vxA, vyA, vzA, soaVertexCountA, cA, hA, &seedVertexA );

		// Include the floor set by hull A faces.
		floorB = b3MaxFloat( seedSeparationB, res.faceA.separation );
		floorB = b3MinFloat( floorB, speculativeDistance );
	}

	// Test B's face planes against A's vertices.
	for ( int i = 0; i < faceCountB; ++i )
	{
		if ( dotB[i] - radiusBound < b3MaxFloat( floorB, res.faceB.separation ) )
		{
			continue;
		}

		b3Plane plane = planesB[i];
		b3Vec3 direction = b3Neg( b3MulMV( R, plane.normal ) );
		int vertexIndex = seedVertexA;
		float separation = seedSeparationB;

		// Avoid recomputing the seed face separation.
		if ( earlyReturn == false || i != seedIndexB )
		{
			float planeSeparation = b3Dot( direction, xfB.p ) - plane.offset;
			separation =
				b3GetFaceSeparation( direction, planeSeparation, vxA, vyA, vzA, soaVertexCountA, cA, hA, &vertexIndex );
		}

		if ( separation > res.faceB.separation )
		{
			res.faceB.normal = direction;
			res.faceB.separation = separation;
			res.faceB.indexA = vertexIndex;
			res.faceB.indexB = i;
			if ( separation > speculativeDistance && earlyReturn )
			{
				res.separatedFeature = b3_faceAxisB;
				return res;
			}
		}
	}

	// Transform B into A's space once, into SoA arrays. Extra space so tail can be set to zero.
	_Static_assert( ( B3_MAX_HULL_EDGES & ( B3_SIMD_WIDTH - 1 ) ) == 0, "must be multiple of SIMD width" );
	_Static_assert( ( B3_MAX_HULL_FACES & ( B3_SIMD_WIDTH - 1 ) ) == 0, "must be multiple of SIMD width" );
	_Static_assert( ( B3_MAX_HULL_VERTICES & ( B3_SIMD_WIDTH - 1 ) ) == 0, "must be multiple of SIMD width" );

	B3_VALIDATE( earlyReturn == false ||
				 centerDistance >= b3MaxFloat( res.faceA.separation, res.faceB.separation ) + radiusBound );

	// Gather edges of A that can feasibly create a separating axis that beats the maximum face separation.
	int halfEdgeCountA = hullA->edgeCount;
	const b3HullHalfEdge* halfEdgesA = b3GetHullEdges( hullA );
	int edgeIndicesA[NE];
	int na = 0;

	int halfEdgeCountB = hullB->edgeCount;
	const b3HullHalfEdge* halfEdgesB = b3GetHullEdges( hullB );
	int edgeIndicesB[B3_MAX_HULL_EDGES];
	int nb = 0;

	float maxFaceSeparation = b3MaxFloat( res.faceA.separation, res.faceB.separation );
	float boundSlack = radius - radiusBound;
	float thresholdA = earlyReturn ? maxFaceSeparation + hullB->innerRadius - boundSlack : -INFINITY;
	float thresholdB = earlyReturn ? maxFaceSeparation + hullA->innerRadius - boundSlack : -INFINITY;

	b3Vec3 centerBinA = b3Add( b3MulMV( R, hullB->center ), xfB.p );
	b3Vec3 centerAinB = b3MulMV( invR, b3Sub( hullA->center, xfB.p ) );

	_Alignas( 16 ) float planeDotA[NF];
	_Alignas( 16 ) float planeDotB[NF];
	b3GetFacePlaneSeparations( hullA, centerBinA, planeDotA );
	b3GetFacePlaneSeparations( hullB, centerAinB, planeDotB );

	const float* cosinesA = b3GetHullEdgeCosines( hullA );
	const float* cosinesB = b3GetHullEdgeCosines( hullB );

	for ( int i = 0; i < halfEdgeCountA; i += 2 )
	{
		int i1 = halfEdgesA[i].face;
		int i2 = halfEdgesA[i + 1].face;
		float c = cosinesA[i >> 1];
		edgeIndicesA[na] = i;
		na += b3TestEdgeCandidateSorted( planeDotA[i1], planeDotA[i2], c, thresholdA );
	}

	for ( int i = 0; i < halfEdgeCountB; i += 2 )
	{
		int i1 = halfEdgesB[i].face;
		int i2 = halfEdgesB[i + 1].face;
		float c = cosinesB[i >> 1];
		edgeIndicesB[nb] = i;
		nb += b3TestEdgeCandidateSorted( planeDotB[i1], planeDotB[i2], c, thresholdB );
	}

	// Apply additional culling use the support points for the best face axes.
	// The support vertices of the best faces are points on the other hull, so an edge pair axis cannot
	// have a larger separation than the plane separation of these points over the arc of the edge.
	// This can slow down boxes so skip this if there are not enough edge candidates.
	if ( earlyReturn && nb * ( ( na + 3 ) >> 2 ) >= B3_EDGE_PROBE_MIN_TESTS )
	{
		float probeBound = maxFaceSeparation - ( 0.1f * B3_LINEAR_SLOP + 0.001f * b3AbsFloat( centerDistance + radius ) );

		int vertexB = res.faceA.indexB != B3_NULL_INDEX ? res.faceA.indexB : seedVertexB;
		b3Vec3 probeB = { vxB[vertexB], vyB[vertexB], vzB[vertexB] };
		b3GetFacePlaneSeparations( hullA, b3Add( b3MulMV( R, probeB ), xfB.p ), planeDotA );
		na = b3FilterEdgeCandidates( hullA, planeDotA, probeBound, edgeIndicesA, na );

		int vertexA = res.faceB.indexA != B3_NULL_INDEX ? res.faceB.indexA : seedVertexA;
		b3Vec3 probeA = { vxA[vertexA], vyA[vertexA], vzA[vertexA] };
		b3GetFacePlaneSeparations( hullB, b3MulMV( invR, b3Sub( probeA, xfB.p ) ), planeDotB );
		nb = b3FilterEdgeCandidates( hullB, planeDotB, probeBound, edgeIndicesB, nb );
	}

	if ( na == 0 || nb == 0 )
	{
		// No edge candidates found.
		return res;
	}

	// The alignments below are not necessary, but they don't hurt.

	// B face normals in A space, negated.
	_Alignas( 16 ) float bFNx[NF];
	_Alignas( 16 ) float bFNy[NF];
	_Alignas( 16 ) float bFNz[NF];

	// B vertices in A space, negated.
	_Alignas( 16 ) float bWx[NV];
	_Alignas( 16 ) float bWy[NV];
	_Alignas( 16 ) float bWz[NV];

	int soaFaceCountB = ( faceCountB + 3 ) & ~3;
	const float* nxB = b3GetHullSoaNormals( hullB );
	const float* nyB = nxB + soaFaceCountB;
	const float* nzB = nyB + soaFaceCountB;

	b3NegativeTransformFromSoA( R, xfB.p, nxB, nyB, nzB, soaFaceCountB, bFNx, bFNy, bFNz, false );
	b3NegativeTransformFromSoA( R, xfB.p, vxB, vyB, vzB, soaVertexCountB, bWx, bWy, bWz, true );

	// Per A edge data, already in A's space so just gathered. n0 and n1 are the two face
	// normals, d the edge vector av1-av0, v0 the first vertex. Tol is the
	// parallel edge tolerance, scaled by the edge length.
	_Alignas( 16 ) float aN0x[NE];
	_Alignas( 16 ) float aN0y[NE];
	_Alignas( 16 ) float aN0z[NE];
	_Alignas( 16 ) float aN1x[NE];
	_Alignas( 16 ) float aN1y[NE];
	_Alignas( 16 ) float aN1z[NE];
	// dir = av1 - av0
	_Alignas( 16 ) float aDx[NE];
	_Alignas( 16 ) float aDy[NE];
	_Alignas( 16 ) float aDz[NE];
	_Alignas( 16 ) float aV0x[NE];
	_Alignas( 16 ) float aV0y[NE];
	_Alignas( 16 ) float aV0z[NE];
	_Alignas( 16 ) float aTol[NE];

	float squaredTol = B3_PARALLEL_EDGE_TOL * B3_PARALLEL_EDGE_TOL;
	for ( int k = 0; k < na; ++k )
	{
		const b3HullHalfEdge* edge = halfEdgesA + edgeIndicesA[k];
		const b3HullHalfEdge* twin = edge + 1;

		b3Vec3 A = planesA[edge->face].normal;
		b3Vec3 B = planesA[twin->face].normal;
		aN0x[k] = A.x;
		aN0y[k] = A.y;
		aN0z[k] = A.z;
		aN1x[k] = B.x;
		aN1y[k] = B.y;
		aN1z[k] = B.z;

		int v0 = edge->origin;
		int v1 = twin->origin;

		aDx[k] = vxA[v1] - vxA[v0];
		aDy[k] = vyA[v1] - vyA[v0];
		aDz[k] = vzA[v1] - vzA[v0];
		aV0x[k] = vxA[v0];
		aV0y[k] = vyA[v0];
		aV0z[k] = vzA[v0];

		aTol[k] = squaredTol * ( aDx[k] * aDx[k] + aDy[k] * aDy[k] + aDz[k] * aDz[k] );
	}

	// Zero the tail lanes.
	b3FloatW zero = b3ZeroW();
	b3Vec3W zeroV = { zero, zero, zero };
	b3StoreVW( aN0x + na, aN0y + na, aN0z + na, zeroV );
	b3StoreVW( aN1x + na, aN1y + na, aN1z + na, zeroV );
	b3StoreVW( aDx + na, aDy + na, aDz + na, zeroV );
	b3StoreVW( aV0x + na, aV0y + na, aV0z + na, zeroV );
	b3StoreW( aTol + na, zero );

	float linearSlop = B3_LINEAR_SLOP;

#if defined( B3_SIMD_NONE )

	// The SIMD emulated version of this code is very slow. This is a purely scalar version
	// for platforms that don't have SIMD capability. It is much faster than SIMD emulation.
	// WARNING: this math needs to match the SIMD version for cross platform determinism.

	const float EPS = -linearSlop * linearSlop;

	for ( int j = 0; j < nb; ++j )
	{
		const b3HullHalfEdge* edge = halfEdgesB + edgeIndicesB[j];
		const b3HullHalfEdge* twin = edge + 1;
		int f0 = edge->face;
		int f1 = twin->face;
		int v0 = edge->origin;
		int v1 = twin->origin;

		b3Vec3 C = { bFNx[f0], bFNy[f0], bFNz[f0] };
		b3Vec3 D = { bFNx[f1], bFNy[f1], bFNz[f1] };
		b3Vec3 bv0 = { bWx[v0], bWy[v0], bWz[v0] };
		b3Vec3 bv1 = { bWx[v1], bWy[v1], bWz[v1] };
		b3Vec3 DC = b3Sub( bv1, bv0 );

		for ( int i = 0; i < na; ++i )
		{
			b3Vec3 d = { aDx[i], aDy[i], aDz[i] };

			// CBA = C.dir, DBA = D.dir, where dir = B_x_A
			float CBA = b3Dot( C, d );
			float DBA = b3Dot( D, d );
			if ( CBA * DBA >= EPS )
			{
				continue;
			}

			b3Vec3 n0 = { aN0x[i], aN0y[i], aN0z[i] };
			b3Vec3 n1 = { aN1x[i], aN1y[i], aN1z[i] };

			// ADC = n0.DC, BDC = n1.DC, where DC = D_x_C
			float ADC = b3Dot( n0, DC );
			float BDC = b3Dot( n1, DC );
			if ( ADC * BDC >= EPS || CBA * BDC >= EPS )
			{
				continue;
			}

			// Reject near parallel edges
			float maxCD = b3MaxFloat( CBA * CBA, DBA * DBA );
			if ( maxCD <= aTol[i] )
			{
				continue;
			}

			// t = -CBA / (DBA - CBA)
			float t = -CBA / ( DBA - CBA );

			// normal = lerp(t, C, D) = C + (D-C)*t
			b3Vec3 n = b3MulAdd( C, t, b3Sub( D, C ) );
			float len2 = b3Dot( n, n );
			float inv = 1.0f / sqrtf( len2 );
			n = b3MulSV( inv, n );

			// separation = -dot(normal, av0 + bv0)
			b3Vec3 av0 = { aV0x[i], aV0y[i], aV0z[i] };
			float separation = -b3Dot( b3Add( av0, bv0 ), n );
			if ( separation > res.edge.separation )
			{
				res.edge.normal = n;
				res.edge.separation = separation;

				// Half edge index
				res.edge.indexA = edgeIndicesA[i];
				res.edge.indexB = edgeIndicesB[j];

				if ( separation > speculativeDistance && earlyReturn )
				{
					res.separatedFeature = b3_edgePairAxis;
					return res;
				}
			}
		}
	}

#else

	// Edge phase, one B edge against four A edges at a time, no transforms in the loop.

	// This tolerance can skip edges shorter than 1cm.
	const b3FloatW EPS = b3SplatW( -linearSlop * linearSlop );
	const b3FloatW INF = b3SplatW( INFINITY );

	for ( int j = 0; j < nb; ++j )
	{
		const b3HullHalfEdge* edge = halfEdgesB + edgeIndicesB[j];
		const b3HullHalfEdge* twin = edge + 1;
		int f0 = edge->face;
		int f1 = twin->face;
		int v0 = edge->origin;
		int v1 = twin->origin;

		b3Vec3 nC = { bFNx[f0], bFNy[f0], bFNz[f0] };
		b3Vec3 nD = { bFNx[f1], bFNy[f1], bFNz[f1] };
		b3Vec3 pB = { bWx[v0], bWy[v0], bWz[v0] };
		b3Vec3 qB = { bWx[v1], bWy[v1], bWz[v1] };

		const b3Vec3W C = b3SplatVW( nC );
		const b3Vec3W D = b3SplatVW( nD );
		const b3Vec3W DC = b3SplatVW( b3Sub( qB, pB ) );
		const b3Vec3W bv0 = b3SplatVW( pB );

		for ( int i = 0; i < na; i += 4 )
		{
			b3Vec3W n0 = b3LoadVW( aN0x + i, aN0y + i, aN0z + i );
			b3Vec3W n1 = b3LoadVW( aN1x + i, aN1y + i, aN1z + i );
			b3Vec3W d = b3LoadVW( aDx + i, aDy + i, aDz + i );
			b3Vec3W av0 = b3LoadVW( aV0x + i, aV0y + i, aV0z + i );
			b3FloatW tol = b3LoadW( aTol + i );

			// CBA = C.dir, DBA = D.dir, where dir = B_x_A
			b3FloatW CBA = b3DotW( C, d );
			b3FloatW DBA = b3DotW( D, d );
			// ADC = n0.DC, BDC = n1.DC, where DC = D_x_C
			b3FloatW ADC = b3DotW( n0, DC );
			b3FloatW BDC = b3DotW( n1, DC );

			// Gauss map arc crossing test, CBA*DBA<eps and ADC*BDC<eps and CBA*BDC<eps
			b3FloatW m1 = b3LessThanW( b3MulW( CBA, DBA ), EPS );
			b3FloatW m2 = b3LessThanW( b3MulW( ADC, BDC ), EPS );
			b3FloatW m3 = b3LessThanW( b3MulW( CBA, BDC ), EPS );

			// Reject near parallel edges. The arc lerp is ill conditioned when both of B's normals are nearly
			// perpendicular to edge A, a scale invariant sine threshold relative to the edge length.
			b3FloatW maxCD = b3MaxW( b3MulW( CBA, CBA ), b3MulW( DBA, DBA ) );
			b3FloatW notParallel = b3GreaterThanW( maxCD, tol );
			b3FloatW mask = b3AndW( b3AndW( m1, m2 ), b3AndW( m3, notParallel ) );

			// Most A-edges fail the Gauss test, so skip the divide, sqrt and support work when no
			// lane passed.
			if ( b3AnyTrueW( mask ) == false )
			{
				continue;
			}

			// t = -CBA / (DBA - CBA)
			b3FloatW t = b3DivW( b3NegW( CBA ), b3SubW( DBA, CBA ) );

			// normal = lerp(t, C, D) = C + (D-C)*t
			b3Vec3W n = b3MulAddSVW( C, t, b3SubVW( D, C ) );

			// normalize
			b3FloatW len2 = b3DotW( n, n );
			b3FloatW inv = b3DivW( b3SplatW( 1.0f ), b3SqrtW( len2 ) );
			n = b3MulSVW( inv, n );

			// support = dot(normal, av0 + bv0)
			b3FloatW support = b3DotW( b3AddVW( av0, bv0 ), n );

			// Lanes that fail the Gauss test can never win.
			support = b3BlendW( INF, support, mask );
			b3FloatW separation = b3NegW( support );

			// Test all 4 supports against the running best at once. If none beats it, skip the
			// store and scalar reduction.
			b3FloatW improves = b3GreaterThanW( separation, b3SplatW( res.edge.separation ) );
			if ( b3AnyTrueW( improves ) == false )
			{
				continue;
			}

			_Alignas( 16 ) float sA[4];
			_Alignas( 16 ) float nxA[4];
			_Alignas( 16 ) float nyA[4];
			_Alignas( 16 ) float nzA[4];
			b3StoreW( sA, separation );
			b3StoreVW( nxA, nyA, nzA, n );

			// Reduce in lane order so ties keep the first edge and the early out takes the first
			// improving support below zero. Padded tail lanes carry +INF support, so they never
			// update or index edges out of range.
			for ( int lane = 0; lane < 4; lane++ )
			{
				int ei = i + lane;
				float s = sA[lane];
				if ( s > res.edge.separation )
				{
					res.edge.normal = (b3Vec3){ nxA[lane], nyA[lane], nzA[lane] };
					res.edge.separation = s;

					// Half edge index
					res.edge.indexA = edgeIndicesA[ei];
					res.edge.indexB = edgeIndicesB[j];

					if ( s > speculativeDistance && earlyReturn )
					{
						res.separatedFeature = b3_edgePairAxis;
						return res;
					}
				}
			}
		}
	}
#endif

	return res;
}

#undef NE
#undef NF
#undef NV

void b3CollideHulls( b3LocalManifold* manifold, int capacity, const b3HullData* hullA, const b3HullData* hullB,
					 b3Transform transformBtoA, b3SATCache* cache )
{
	manifold->pointCount = 0;

	if ( capacity < 4 )
	{
		return;
	}

	// Work in shapeA coordinates
	float speculativeDistance = B3_SPECULATIVE_DISTANCE;

	float linearSlop = B3_LINEAR_SLOP;
	const b3HullHalfEdge* edgesA = b3GetHullEdges( hullA );
	const b3Plane* planesA = b3GetHullPlanes( hullA );
	const b3Vec3* pointsA = b3GetHullPoints( hullA );

	const b3HullHalfEdge* edgesB = b3GetHullEdges( hullB );
	const b3Plane* planesB = b3GetHullPlanes( hullB );
	const b3Vec3* pointsB = b3GetHullPoints( hullB );

	cache->hit = 0;

	// Attempt to use the cache to speed up collision
	switch ( cache->type )
	{
		case b3_invalidAxis:
			break;

		case b3_faceAxisA:
		{
			B3_ASSERT( cache->indexA < hullA->faceCount );

			// Check for separation using cached face
			b3Plane plane = planesA[cache->indexA];
			b3Vec3 searchDirectionInB = b3Neg( b3InvRotateVector( transformBtoA.q, plane.normal ) );

			int vertexIndex = b3FindHullSupportVertex( hullB, searchDirectionInB );
			b3Vec3 support = b3TransformPoint( transformBtoA, pointsB[vertexIndex] );
			float separation = b3PlaneSeparation( plane, support );

			if ( separation >= speculativeDistance )
			{
				// Cache hit, shapes are separated
				cache->hit = 1;
				return;
			}

			// Attempt face contact using cached feature
			b3SeparatingAxis faceQuery;
			faceQuery.normal = plane.normal;
			faceQuery.separation = 0.0f;
			faceQuery.indexA = cache->indexA;
			faceQuery.indexB = vertexIndex;
			faceQuery.type = b3_faceAxisA;

			b3SATCache localCache = { 0 };
			bool touching = b3BuildFaceAContact( manifold, capacity, hullA, hullB, transformBtoA, faceQuery, &localCache );
			if ( touching == true && b3AbsFloat( cache->separation - localCache.separation ) < linearSlop )
			{
				// Cache hit, contact points generated
				cache->hit = 1;
				return;
			}
		}
		break;

		case b3_faceAxisB:
		{
			B3_ASSERT( cache->indexB < hullB->faceCount );

			// Check for separation using cached face
			b3Plane plane = planesB[cache->indexB];
			b3Vec3 searchDirectionInA = b3Neg( b3RotateVector( transformBtoA.q, plane.normal ) );

			// todo use b3GetSupportWide
			int vertexIndex = b3FindHullSupportVertex( hullA, searchDirectionInA );
			b3Vec3 support = b3InvTransformPoint( transformBtoA, pointsA[vertexIndex] );
			float separation = b3PlaneSeparation( plane, support );

			if ( separation >= speculativeDistance )
			{
				// Cache hit, shapes are separated
				cache->hit = 1;
				return;
			}

			// Attempt face contact using cached feature
			b3SeparatingAxis faceQuery;
			faceQuery.normal = b3Neg( plane.normal );
			faceQuery.separation = 0.0f;
			faceQuery.indexA = vertexIndex;
			faceQuery.indexB = cache->indexB;
			faceQuery.type = b3_faceAxisB;

			b3SATCache localCache = { 0 };
			bool touching = b3BuildFaceBContact( manifold, capacity, hullA, hullB, transformBtoA, faceQuery, &localCache );
			if ( touching == true && b3AbsFloat( cache->separation - localCache.separation ) < linearSlop )
			{
				// Cache hit, contact points generated
				cache->hit = 1;
				return;
			}
		}
		break;

		case b3_edgePairAxis:
		{
			int indexA = cache->indexA;
			const b3HullHalfEdge* edge1 = edgesA + indexA;
			const b3HullHalfEdge* twin1 = edgesA + indexA + 1;
			B3_ASSERT( edge1->twin == indexA + 1 && twin1->twin == indexA );

			b3Vec3 pA = pointsA[edge1->origin];
			b3Vec3 qA = pointsA[twin1->origin];
			b3Vec3 eA = b3Sub( qA, pA );

			b3Vec3 uA = planesA[edge1->face].normal;
			b3Vec3 vA = planesA[twin1->face].normal;

			int indexB = cache->indexB;
			const b3HullHalfEdge* edge2 = edgesB + indexB;
			const b3HullHalfEdge* twin2 = edgesB + indexB + 1;
			B3_ASSERT( edge2->twin == indexB + 1 && twin2->twin == indexB );

			b3Vec3 pB = b3TransformPoint( transformBtoA, pointsB[edge2->origin] );
			b3Vec3 qB = b3TransformPoint( transformBtoA, pointsB[twin2->origin] );
			b3Vec3 eB = b3Sub( qB, pB );

			b3Vec3 uB = b3RotateVector( transformBtoA.q, planesB[edge2->face].normal );
			b3Vec3 vB = b3RotateVector( transformBtoA.q, planesB[twin2->face].normal );

			// flipping the signs of u2 and v2
			// cross(v2, u2) == cross(-v2, -u2)
			// so we still use -e2
			// but we can also use e1 = cross(u1, v1) and e2 = cross(u2, v2)
			float cba = b3Dot( uB, eA );
			float dba = b3Dot( vB, eA );
			float adc = -b3Dot( uA, eB );
			float bdc = -b3Dot( vA, eB );

			if ( cba * dba < 0.0f && adc * bdc < 0.0f && cba * bdc > 0.0f )
			{
				// Avoid nearly parallel edges that may lead to invalid separation values at the noise floor.
				float squaredTolerance = B3_PARALLEL_EDGE_TOL * B3_PARALLEL_EDGE_TOL;
				if ( b3MaxFloat( cba * cba, dba * dba ) >= squaredTolerance * b3LengthSquared( eA ) )
				{
					// Transform reference center of the first hull into local space of the second hull
					float t = cba / ( cba - dba );
					b3Vec3 axis = b3Lerp( uB, vB, t );
					B3_VALIDATE( b3LengthSquared( axis ) > 1000.0f * FLT_MIN );
					axis = b3Normalize( axis );
					float separation = b3Dot( axis, b3Sub( qA, qB ) );

					if ( separation > speculativeDistance )
					{
						// Cache hit, shapes are separated
						cache->hit = 1;
						return;
					}

					// Try to rebuild contact from last features
					b3SeparatingAxis edgeQuery = { 0 };
					edgeQuery.normal = b3Neg( axis );
					edgeQuery.separation = 0.0f;
					edgeQuery.indexA = cache->indexA;
					edgeQuery.indexB = cache->indexB;
					edgeQuery.type = b3_edgePairAxis;

					b3SATCache localCache = { 0 };
					bool touching = b3BuildEdgeContact( manifold, hullA, hullB, transformBtoA, edgeQuery, &localCache );

					// This separation tolerance may have a big impact on performance in some benchmarks.
					if ( touching && b3AbsFloat( cache->separation - localCache.separation ) < linearSlop )
					{
						// Cache hit, contact point generated
						cache->hit = 1;
						return;
					}
				}
			}
		}
		break;

			// This case is for testing
		case b3_manualFaceAxisA:
		{
			b3AxisQuery axisQuery = b3ComputeSeparatingAxis( hullA, hullB, transformBtoA, false );
			b3SeparatingAxis faceQuery = axisQuery.faceA;
			b3BuildFaceAContact( manifold, capacity, hullA, hullB, transformBtoA, faceQuery, cache );
			return;
		}

			// This case is for testing
		case b3_manualFaceAxisB:
		{
			b3AxisQuery axisQuery = b3ComputeSeparatingAxis( hullA, hullB, transformBtoA, false );
			b3SeparatingAxis faceQuery = axisQuery.faceB;
			b3BuildFaceBContact( manifold, capacity, hullA, hullB, transformBtoA, faceQuery, cache );
			return;
		}

			// This case is for testing
		case b3_manualEdgePairAxis:
		{
			b3AxisQuery axisQuery = b3ComputeSeparatingAxis( hullA, hullB, transformBtoA, false );
			b3SeparatingAxis edgeQuery = axisQuery.edge;
			if ( edgeQuery.indexA != B3_NULL_INDEX )
			{
				b3BuildEdgeContact( manifold, hullA, hullB, transformBtoA, edgeQuery, cache );
			}
			return;
		}

		default:
			B3_ASSERT( false );
			break;
	}

	manifold->pointCount = 0;
	*cache = (b3SATCache){ 0 };

	b3AxisQuery axisQuery = b3ComputeSeparatingAxis( hullA, hullB, transformBtoA, true );

	if ( axisQuery.separatedFeature != b3_invalidAxis )
	{
		// We found a separating axis
		cache->type = axisQuery.separatedFeature;

		if ( axisQuery.separatedFeature == b3_faceAxisA )
		{
			B3_VALIDATE( axisQuery.faceA.separation > speculativeDistance );
			cache->separation = axisQuery.faceA.separation;
			cache->indexA = (uint8_t)axisQuery.faceA.indexA;
			cache->indexB = (uint8_t)axisQuery.faceA.indexB;
		}
		else if ( axisQuery.separatedFeature == b3_faceAxisB )
		{
			B3_VALIDATE( axisQuery.faceB.separation > speculativeDistance );
			cache->separation = axisQuery.faceB.separation;
			cache->indexA = (uint8_t)axisQuery.faceB.indexA;
			cache->indexB = (uint8_t)axisQuery.faceB.indexB;
		}
		else
		{
			B3_ASSERT( axisQuery.separatedFeature == b3_edgePairAxis );
			B3_VALIDATE( axisQuery.edge.separation > speculativeDistance );
			cache->separation = axisQuery.edge.separation;
			cache->indexA = (uint8_t)axisQuery.edge.indexA;
			cache->indexB = (uint8_t)axisQuery.edge.indexB;
		}
		return;
	}

	B3_VALIDATE( axisQuery.faceA.separation <= speculativeDistance || axisQuery.faceB.separation <= speculativeDistance ||
				 axisQuery.edge.separation <= speculativeDistance );

	if ( axisQuery.faceA.separation > axisQuery.faceB.separation )
	{
		b3SeparatingAxis faceQuery = axisQuery.faceA;
		B3_VALIDATE( 0 <= faceQuery.indexA && faceQuery.indexA < hullA->faceCount );
		B3_VALIDATE( 0 <= faceQuery.indexB && faceQuery.indexB < hullB->vertexCount );

		// Face contact A
		b3BuildFaceAContact( manifold, capacity, hullA, hullB, transformBtoA, faceQuery, cache );

		B3_VALIDATE( cache->indexA < hullA->faceCount );
		B3_VALIDATE( cache->indexB < hullB->vertexCount );
	}
	else
	{
		b3SeparatingAxis faceQuery = axisQuery.faceB;
		B3_VALIDATE( 0 <= faceQuery.indexA && faceQuery.indexA < hullA->vertexCount );
		B3_VALIDATE( 0 <= faceQuery.indexB && faceQuery.indexB < hullB->faceCount );

		// Face contact B
		b3BuildFaceBContact( manifold, capacity, hullA, hullB, transformBtoA, faceQuery, cache );

		B3_VALIDATE( cache->indexA < hullA->vertexCount );
		B3_VALIDATE( cache->indexB < hullB->faceCount );
	}

	b3SeparatingAxis edgeQuery = axisQuery.edge;

	if ( edgeQuery.indexA == B3_NULL_INDEX )
	{
		// There are no valid edge pairs (all edges parallel)
		return;
	}

	float faceSeparation = b3MaxFloat( axisQuery.faceA.separation, axisQuery.faceB.separation );
	float clipSeparation = cache->separation;
	float edgeTol = linearSlop;

	// Face contact can be empty if it is not the axis of maximum separation. It can also
	// be empty in narrow cases in the speculative region. If that case was important then
	// a GJK fallback would be used. So far it doesn't seem important.
	// Create edge contact if face contact fails or edge contact is significantly better.
	if ( ( manifold->pointCount == 0 && edgeQuery.separation > faceSeparation ) ||
		 edgeQuery.separation > clipSeparation + edgeTol )
	{
		B3_ASSERT( 0 <= edgeQuery.indexA && edgeQuery.indexA < hullA->edgeCount );
		B3_ASSERT( 0 <= edgeQuery.indexB && edgeQuery.indexB < hullB->edgeCount );

		// Edge contact
		b3LocalManifold edgeManifold = { 0 };
		b3LocalManifoldPoint edgePoint = { 0 };
		edgeManifold.points = &edgePoint;

		b3SATCache edgeCache = { 0 };
		b3BuildEdgeContact( &edgeManifold, hullA, hullB, transformBtoA, edgeQuery, &edgeCache );

		// It is possible with speculation to have vertex-vertex collision that is missed by SAT,
		// so edge contact yields no points. In that case perhaps the face contact has some points.
		if ( edgeManifold.pointCount == 1 )
		{
			// Copy edge manifold out, being careful to preserve manifold point buffer.
			b3LocalManifoldPoint* points = manifold->points;
			*manifold = edgeManifold;
			manifold->points = points;
			manifold->points[0] = edgePoint;
			*cache = edgeCache;
		}
	}
}

// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "contact.h"
#include "ctz.h"
#include "manifold.h"
#include "physics_world.h"
#include "shape.h"
#include "simd.h"

#include "box3d/types.h"

#include <stdio.h>
#include <string.h>

#define B3_MAX_POINTS_PER_TRIANGLE 32

#if B3_ENABLE_VALIDATION
static bool b3IsSorted( const int* array, int count )
{
	for ( int i = 0; i < count - 1; ++i )
	{
		if ( array[i] >= array[i + 1] )
		{
			return false;
		}
	}

	return true;
}
#endif

#define B3_INITIAL_TRIANGLE_CAPACITY 512

typedef struct b3TriangleQueryContext
{
	b3Arena* arena;
	int* indices;
	int capacity;
	int count;
} b3TriangleQueryContext;

static bool b3CollectTriangleIndicesCallback( b3Vec3 a, b3Vec3 b, b3Vec3 c, int triangleIndex, void* context )
{
	B3_UNUSED( a, b, c );
	b3TriangleQueryContext* triangleContext = context;
	if ( triangleContext->count == triangleContext->capacity )
	{
		int capacity = 2 * triangleContext->capacity;
		int* indices = b3Bump( triangleContext->arena, capacity * (int)sizeof( int ) );
		memcpy( indices, triangleContext->indices, triangleContext->count * sizeof( int ) );
		triangleContext->indices = indices;
		triangleContext->capacity = capacity;
	}

	triangleContext->indices[triangleContext->count] = triangleIndex;
	triangleContext->count += 1;
	return true;
}

static void b3RefreshCache( b3Contact* contact, const b3Shape* shapeA, b3WorldTransform xfA, const b3AABB* bounds,
							b3Arena arena )
{
	B3_ASSERT( shapeA->type == b3_meshShape || shapeA->type == b3_heightShape );

	b3MeshContact* meshContact = &contact->meshContact;

	// If the dynamic body didn't move out of the cached query bounds we are done!
	if ( b3AABB_Contains( meshContact->queryBounds, *bounds ) )
	{
		if ( shapeA->type == b3_meshShape )
		{
			for ( int i = 0; i < contact->meshContact.triangleCache.count; ++i )
			{
				B3_ASSERT( 0 <= contact->meshContact.triangleCache.data[i].triangleIndex &&
						   contact->meshContact.triangleCache.data[i].triangleIndex <
							   shapeA->mesh.data->triangleCount );
			}
		}

		return;
	}

	// Enlarge to the query bounds to absorb small movement
	float radius = B3_MAX_AABB_MARGIN + B3_SPECULATIVE_DISTANCE;
	b3Vec3 extension = { radius, radius, radius };
	meshContact->queryBounds.lowerBound = b3Sub( bounds->lowerBound, extension );
	meshContact->queryBounds.upperBound = b3Add( bounds->upperBound, extension );

	// Query triangles
	b3TriangleQueryContext queryContext = {
		.arena = &arena,
		.indices = b3Bump( &arena, B3_INITIAL_TRIANGLE_CAPACITY * (int)sizeof( int ) ),
		.capacity = B3_INITIAL_TRIANGLE_CAPACITY,
		.count = 0,
	};

	// Bounds are in world space. Convert to the local mesh frame. The broadphase bounds are float,
	// so the demoted mesh transform is the matching float world frame (exact in float mode).
	b3Transform meshTransform = b3ToRelativeTransform( xfA, b3Pos_zero );
	b3AABB localBounds = b3AABB_Transform( b3InvertTransform( meshTransform ), meshContact->queryBounds );
	if ( shapeA->type == b3_meshShape )
	{
		b3QueryMesh( &shapeA->mesh, localBounds, b3CollectTriangleIndicesCallback, &queryContext );
	}
	else
	{
		B3_ASSERT( shapeA->type == b3_heightShape );
		b3QueryHeightField( shapeA->heightField, localBounds, b3CollectTriangleIndicesCallback, &queryContext );
	}

	int* triangleIndices = queryContext.indices;
	int triangleCount = queryContext.count;

	// Triangle indices must be sorted to match caches.
	B3_VALIDATE( b3IsSorted( triangleIndices, triangleCount ) );

	// Create new contact cache and match with old one
	b3ContactCache* contactCache = b3Bump( &arena, triangleCount * sizeof( b3ContactCache ) );

	int index2 = 0;
	for ( int index1 = 0; index1 < triangleCount; ++index1 )
	{
		contactCache[index1] = (b3ContactCache){ 0 };

		while ( index2 < contact->meshContact.triangleCache.count &&
				contact->meshContact.triangleCache.data[index2].triangleIndex < triangleIndices[index1] )
		{
			index2 += 1;
		}

		if ( index2 < contact->meshContact.triangleCache.count &&
			 contact->meshContact.triangleCache.data[index2].triangleIndex == triangleIndices[index1] )
		{
			contactCache[index1] = contact->meshContact.triangleCache.data[index2].cache;
		}
	}

	// Save new cache
	b3Array_Resize( contact->meshContact.triangleCache, triangleCount );
	for ( int i = 0; i < triangleCount; ++i )
	{
		contact->meshContact.triangleCache.data[i] = (b3TriangleCache){ triangleIndices[i], contactCache[i] };

		if ( shapeA->type == b3_meshShape )
		{
			B3_ASSERT( 0 <= contact->meshContact.triangleCache.data[i].triangleIndex &&
					   contact->meshContact.triangleCache.data[i].triangleIndex < shapeA->mesh.data->triangleCount );
		}
	}
}

#if B3_MAX_MANIFOLD_POINTS == 4

// Returns true if (score, separation) should replace (bestScore, bestSeparation).
static inline bool b3IsBetterCullCandidate( float score, float separation, float bestScore, float bestSeparation,
											float scoreTol, float separationTol )
{
	if ( score > bestScore + scoreTol )
	{
		return true;
	}
	if ( score < bestScore - scoreTol )
	{
		return false;
	}

	// Break the tie using separation
	return separation < bestSeparation - separationTol;
}

static int b3CullPoints( b3Point2D* points, int count )
{
	if ( count <= 1 )
	{
		return count;
	}

	float tol = 0.25f * B3_LINEAR_SLOP;
	float tolSqr = tol * tol;
	float separationTol = B3_LINEAR_SLOP;

	b3Point2D finalPoints[4];
	int count1 = count;

	// Step 1: the two points with the largest distance, ties broken by deepest combined separation
	float bestScore = 0.0f;
	float bestSeparation = FLT_MAX;
	int bestIndex1 = B3_NULL_INDEX;
	int bestIndex2 = B3_NULL_INDEX;

	for ( int i = 0; i < count1; ++i )
	{
		b3Vec2 p1 = points[i].p;
		for ( int j = i + 1; j < count1; ++j )
		{
			float score = b3DistanceSquared2( p1, points[j].p );
			// Separation sum heuristic
			float separation = points[i].separation + points[j].separation;

			if ( b3IsBetterCullCandidate( score, separation, bestScore, bestSeparation, tolSqr, separationTol ) )
			{
				bestIndex1 = i;
				bestIndex2 = j;
				bestScore = score;
				bestSeparation = separation;
			}
		}
	}

	if ( bestScore < tolSqr )
	{
		// Choose deepest point
		int deepestIndex = 0;
		for ( int i = 1; i < count1; ++i )
		{
			if ( points[i].separation < points[deepestIndex].separation )
			{
				deepestIndex = i;
			}
		}

		if ( deepestIndex != 0 )
		{
			points[0] = points[deepestIndex];
		}
		return 1;
	}

	finalPoints[0] = points[bestIndex1];
	finalPoints[1] = points[bestIndex2];

	// Cull
	points[bestIndex2] = points[count1 - 1];
	points[bestIndex1] = points[count1 - 2];
	count1 -= 2;

	if ( count1 == 0 )
	{
		points[0] = finalPoints[0];
		points[1] = finalPoints[1];
		return 2;
	}

	// First anchor point
	b3Vec2 a = finalPoints[0].p;

	// Second anchor point
	b3Vec2 b = finalPoints[1].p;
	b3Vec2 ba = b3Sub2( b, a );
	// float length = b3Length2( ba );
	// float areaTol = tol * length;

	// Step 2: find the point with the maximum triangular area, ties broken by deepest separation
	bestScore = 0.0f;
	bestSeparation = FLT_MAX;
	int bestIndex = B3_NULL_INDEX;
	float bestSignedArea = 0.0f;
	for ( int i = 0; i < count1; ++i )
	{
		b3Vec2 p = points[i].p;
		float signedArea = b3Cross2( ba, b3Sub2( p, a ) );
		float score = b3AbsFloat( signedArea );

		if ( b3IsBetterCullCandidate( score, points[i].separation, bestScore, bestSeparation, tolSqr, separationTol ) )
		{
			bestSignedArea = signedArea;
			bestScore = score;
			bestSeparation = points[i].separation;
			bestIndex = i;
		}
	}

	if ( bestIndex == B3_NULL_INDEX )
	{
		// All points collinear
		points[0] = finalPoints[0];
		points[1] = finalPoints[1];
		return 2;
	}

	// Store best point
	finalPoints[2] = points[bestIndex];

	if ( count1 == 1 )
	{
		points[0] = finalPoints[0];
		points[1] = finalPoints[1];
		points[2] = finalPoints[2];
		return 3;
	}

	// Cull
	points[bestIndex] = points[count1 - 1];
	count1 -= 1;

	// Step 4: get the point that adds the most area outside the current triangle

	// Third anchor
	b3Vec2 c = finalPoints[2].p;

	// Ensure CCW ordering
	if ( bestSignedArea < 0.0f )
	{
		B3_SWAP( b, c );
		ba = b3Sub2( b, a );
	}

	b3Vec2 cb = b3Sub2( c, b );
	b3Vec2 ac = b3Sub2( a, c );

	bestScore = 0.0f;
	bestSeparation = FLT_MAX;
	bestIndex = B3_NULL_INDEX;
	for ( int i = 0; i < count1; ++i )
	{
		b3Vec2 p = points[i].p;
		float u1 = b3Cross2( b3Sub2( p, a ), ba );
		float u2 = b3Cross2( b3Sub2( p, b ), cb );
		float u3 = b3Cross2( b3Sub2( p, c ), ac );
		float score = b3MaxFloat( u1, b3MaxFloat( u2, u3 ) );

		// Use the area tolerance for collinear points and hysteresis
		if ( b3IsBetterCullCandidate( score, points[i].separation, bestScore, bestSeparation, tolSqr, separationTol ) )
		{
			bestScore = score;
			bestSeparation = points[i].separation;
			bestIndex = i;
		}
	}

	if ( bestIndex == B3_NULL_INDEX )
	{
		// No additional area
		points[0] = finalPoints[0];
		points[1] = finalPoints[1];
		points[2] = finalPoints[2];
		return 3;
	}

	// Store best point
	finalPoints[3] = points[bestIndex];

	// Full quad
	points[0] = finalPoints[0];
	points[1] = finalPoints[1];
	points[2] = finalPoints[2];
	points[3] = finalPoints[3];
	return 4;
}

#endif

static int b3ReduceCluster( b3LocalManifoldPoint* points, int count1, b3Vec3 normal, b3Arena arena )
{
	int targetCount = 1;
	if ( count1 <= targetCount )
	{
		return count1;
	}

	b3Point2D* pts = b3Bump( &arena, count1 * sizeof( b3Point2D ) );
	b3Vec3 u = b3Perp( normal );
	b3Vec3 v = b3Cross( normal, u );
	b3Vec3 origin = points[0].point;

	for ( int i = 0; i < count1; ++i )
	{
		b3Vec3 d = b3Sub( points[i].point, origin );
		pts[i].p = (b3Vec2){ b3Dot( d, u ), b3Dot( d, v ) };
		pts[i].separation = points[i].separation;
		pts[i].originalIndex = i;
	}

#if B3_MAX_MANIFOLD_POINTS == 4
	b3Point2D* hull = pts;
	int count2 = b3CullPoints( pts, count1 );
#else
	b3Point2D* hull = b3Bump( &arena, 2 * count1 * sizeof( b3Point2D ) );
	int hullCount = b3Hull2D( pts, count1, hull );
	int count2 = b3SimplifyHull2D( hull, hullCount, B3_MAX_MANIFOLD_POINTS );
#endif
	B3_ASSERT( count2 <= B3_MAX_MANIFOLD_POINTS );

	b3LocalManifoldPoint finalPoints[B3_MAX_MANIFOLD_POINTS];
	for ( int i = 0; i < count2; ++i )
	{
		int index = hull[i].originalIndex;
		B3_ASSERT( 0 <= index && index < count1 );
		finalPoints[i] = points[index];
	}

	memcpy( points, finalPoints, count2 * sizeof( b3LocalManifoldPoint ) );
	return count2;
}

typedef struct b3Cluster
{
	b3Vec3 manifoldNormal;
	b3Vec3 triangleNormal;
	b3LocalManifoldPoint* points;
	int pointCapacity;
	int pointCount;
} b3Cluster;

typedef enum b3NeighborResult
{
	// Admit for flat edges
	b3_neighborAdmit,
	// Skip for convex edges
	b3_neighborSkip,
	// Snap for concave edges
	b3_neighborSnap,

	// todo comment
	b3_neighborDrop,
} b3NeighborResult;

// 1.0027 degrees
#define B3_NEIGHBOR_TOLERANCE 0.0175f

static bool b3GetNeighborDirection( const b3Shape* shape, int triangleIndex, const b3Vec3* vertices,
									b3Matrix3 meshToLocal, int edgeIndex, b3Vec3* direction )
{
	b3Vec3 neighborNormal;

	bool haveNeighbor = b3GetNeighborNormal( shape, triangleIndex, edgeIndex, &neighborNormal );
	if ( haveNeighbor == false )
	{
		// todo comment
		*direction = b3Neg( b3MakeNormalFromPoints( vertices[0], vertices[1], vertices[2] ) );
		return false;
	}

	neighborNormal = b3MulMV( meshToLocal, neighborNormal );
	b3Vec3 edge = b3Normalize( b3Sub( vertices[( edgeIndex + 1 ) % 3], vertices[edgeIndex] ) );
	*direction = b3Cross( edge, neighborNormal );
	return true;
}

// Is the normal in the vertex Voronoi region?
static bool b3IsInVertexRegion( const b3Vec3* vertices, int vertexIndex, b3Vec3 normal )
{
	b3Vec3 vertex = vertices[vertexIndex];
	b3Vec3 next = vertices[vertexIndex < 2 ? vertexIndex + 1 : 0];
	b3Vec3 previous = vertices[vertexIndex > 0 ? vertexIndex - 1 : 2];
	b3Vec3 e1 = b3Sub( next, vertex );
	b3Vec3 e2 = b3Sub( previous, vertex );
	return b3Dot( normal, e1 ) <= 0.0f && b3Dot( normal, e2 ) <= 0.0f;
}

// This is used to avoid ghost collisions at vertices. It is called rarely.
// trees25 : 2.6%
// mesh_drop : 0.04%
static bool b3IsInVertexCone( const b3Shape* shape, int triangleIndex, int vertexIndex, b3Matrix3 meshToLocal,
							  b3Vec3 normal )
{
	b3Vec3 meshNormal = b3MulMV( b3Transpose( meshToLocal ), normal );
	float tolerance = B3_NEIGHBOR_TOLERANCE;

	if ( shape->type == b3_meshShape )
	{
		return b3IsInMeshVertexRing( &shape->mesh, triangleIndex, vertexIndex, meshNormal, tolerance );
	}

	B3_ASSERT( shape->type == b3_heightShape );
	float toleranceSquared = tolerance * tolerance;

	b3Triangle triangle = b3GetHeightFieldTriangle( shape->heightField, triangleIndex );
	int indices[3] = { triangle.i1, triangle.i2, triangle.i3 };
	int vertex = indices[vertexIndex];

	int fanTriangles[6];
	int fanCount = b3GetHeightFieldVertexTriangles( shape->heightField, vertex, fanTriangles );

	for ( int i = 0; i < fanCount; ++i )
	{
		int fanIndex = fanTriangles[i];
		b3Triangle fan = b3GetHeightFieldTriangle( shape->heightField, fanIndex );
		int fanIndices[3] = { fan.i1, fan.i2, fan.i3 };
		int k = fanIndices[0] == vertex ? 0 : ( fanIndices[1] == vertex ? 1 : 2 );
		B3_ASSERT( fanIndices[k] == vertex );

		int next = k < 2 ? k + 1 : 0;
		int previous = k > 0 ? k - 1 : 2;
		b3Vec3 edges[2] = { b3Sub( fan.vertices[next], fan.vertices[k] ),
							b3Sub( fan.vertices[previous], fan.vertices[k] ) };
		int edgeIndices[2] = { k, previous };

		for ( int j = 0; j < 2; ++j )
		{
			float lean = b3Dot( meshNormal, edges[j] );
			if ( lean > 0.0f && lean * lean > toleranceSquared * b3LengthSquared( edges[j] ) )
			{
				return false;
			}

			b3Vec3 neighborNormal;
			if ( b3GetNeighborNormal( shape, fanIndex, edgeIndices[j], &neighborNormal ) == false )
			{
				// todo comment
				b3Vec3 fanNormal = b3MakeNormalFromPoints( fan.vertices[0], fan.vertices[1], fan.vertices[2] );
				if ( b3Dot( meshNormal, fanNormal ) < -tolerance )
				{
					return false;
				}
			}
		}
	}

	return true;
}

// Check the vertex flags to determined if this normal is admissible.
static bool b3AdmitsVertexNormal( const b3Shape* shape, int triangleIndex, const b3Vec3* vertices, int vertexIndex,
								  b3Matrix3 meshToLocal, b3Vec3 normal )
{
	if ( b3IsInVertexRegion( vertices, vertexIndex, normal ) == false )
	{
		// Not in the vertex region so this vertex cannot admit the normal.
		return false;
	}

	if ( b3IsConvexVertex( shape, triangleIndex, vertexIndex ) == false )
	{
		return false;
	}

	return b3IsInVertexCone( shape, triangleIndex, vertexIndex, meshToLocal, normal );
}

// Voronoi test of an edge or vertex normal against the neighbor normals.
static b3NeighborResult b3TestNeighbors( const b3Shape* shape, int triangleIndex, const b3Vec3* vertices,
										 b3Matrix3 meshToLocal, b3TriangleFeature feature, b3Vec3 normal,
										 int* ownerEdge )
{
	*ownerEdge = B3_NULL_INDEX;

	int edges[2] = { 0, 0 };
	int edgeCount = 1;
	int vertexIndex = B3_NULL_INDEX;

	switch ( feature )
	{
		case b3_featureEdge1:
			edges[0] = 0;
			break;

		case b3_featureEdge2:
			edges[0] = 1;
			break;

		case b3_featureEdge3:
			edges[0] = 2;
			break;

		case b3_featureVertex1:
			edges[0] = 0;
			edges[1] = 2;
			edgeCount = 2;
			vertexIndex = 0;
			break;

		case b3_featureVertex2:
			edges[0] = 0;
			edges[1] = 1;
			edgeCount = 2;
			vertexIndex = 1;
			break;

		case b3_featureVertex3:
			edges[0] = 1;
			edges[1] = 2;
			edgeCount = 2;
			vertexIndex = 2;
			break;

		default:
			return b3_neighborAdmit;
	}

	b3Vec3 triangleNormal = b3MakeNormalFromPoints( vertices[0], vertices[1], vertices[2] );
	float tolerance = B3_NEIGHBOR_TOLERANCE;
	bool admit = true;
	bool convex = false;

	for ( int i = 0; i < edgeCount; ++i )
	{
		b3Vec3 direction;
		bool haveNeighbor = b3GetNeighborDirection( shape, triangleIndex, vertices, meshToLocal, edges[i], &direction );

		bool admitted = b3Dot( normal, direction ) <= tolerance;
		if ( admitted == false && haveNeighbor && *ownerEdge == B3_NULL_INDEX )
		{
			*ownerEdge = edges[i];
		}

		admit = admit && admitted;
		convex = convex || b3Dot( triangleNormal, direction ) < -tolerance;
	}

	if ( admit )
	{
		if ( vertexIndex == B3_NULL_INDEX ||
			 b3IsInVertexCone( shape, triangleIndex, vertexIndex, meshToLocal, normal ) )
		{
			return b3_neighborAdmit;
		}

		// todo comment
		return b3IsInVertexRegion( vertices, vertexIndex, normal ) ? b3_neighborSkip : b3_neighborSnap;
	}

	if ( convex )
	{
		// todo comment
		return *ownerEdge == B3_NULL_INDEX ? b3_neighborDrop : b3_neighborSkip;
	}

	// Check vertex.
	if ( vertexIndex != B3_NULL_INDEX &&
		 b3AdmitsVertexNormal( shape, triangleIndex, vertices, vertexIndex, meshToLocal, normal ) )
	{
		return b3_neighborAdmit;
	}

	return b3_neighborSnap;
}

// Voronoi test of neighbors based on deepest point.
static b3NeighborResult b3TestHullFaceNeighbors( const b3Shape* shape, int triangleIndex, const b3Vec3* vertices,
												 b3Matrix3 meshToLocal, const b3LocalManifold* manifold,
												 b3TriangleFeature* nearFeature )
{
	*nearFeature = b3_featureNone;
	b3Vec3 normal = manifold->normal;
	b3Vec3 triangleNormal = b3MakeNormalFromPoints( vertices[0], vertices[1], vertices[2] );
	float tolerance = B3_NEIGHBOR_TOLERANCE;
	float cosAngle = b3Dot( normal, triangleNormal );
	if ( cosAngle > 0.0f && cosAngle * cosAngle >= 1.0f - tolerance * tolerance )
	{
		// Face normal (flipped to point from A to B) is parallel to triangle normal.
		return b3_neighborAdmit;
	}

	int deepest = 0;
	for ( int i = 1; i < manifold->pointCount; ++i )
	{
		if ( manifold->points[i].separation < manifold->points[deepest].separation )
		{
			deepest = i;
		}
	}

	// Clip points were moved onto the hull face, step back along the normal to the triangle.
	const b3LocalManifoldPoint* deepestPoint = manifold->points + deepest;
	b3Vec3 point = b3MulSub( deepestPoint->point, deepestPoint->separation, normal );

	float linearSlop = B3_LINEAR_SLOP;

	// If the hull overlaps this triangle then determine if the deepest point
	// is close to a vertex.
	if ( deepestPoint->separation < 0.0f )
	{
		for ( int i = 0; i < 3; ++i )
		{
			if ( b3DistanceSquared( point, vertices[i] ) > linearSlop * linearSlop )
			{
				continue;
			}

			bool admit = b3AdmitsVertexNormal( shape, triangleIndex, vertices, i, meshToLocal, normal );
			if ( admit )
			{
				// This vertex tentatively accepts the normal.
				b3TriangleFeature vertexFeatures[3] = { b3_featureVertex1, b3_featureVertex2, b3_featureVertex3 };
				*nearFeature = vertexFeatures[i];
				return b3_neighborAdmit;
			}
		}
	}

	int nearEdges[3];
	int nearCount = 0;
	bool inwardTilt = false;
	bool alongEdge = false;
	for ( int i = 0; i < 3; ++i )
	{
		b3Vec3 vertex = vertices[i];
		b3Vec3 edge = b3Normalize( b3Sub( vertices[i < 2 ? i + 1 : 0], vertex ) );
		b3Vec3 inward = b3Cross( triangleNormal, edge );
		if ( b3Dot( inward, b3Sub( point, vertex ) ) <= linearSlop )
		{
			nearEdges[nearCount] = i;
			nearCount += 1;

			// An edge or vertex normal cone tilts outward from the triangle and an edge cone is perpendicular to the
			// edge, a hull face normal need be neither.
			inwardTilt = inwardTilt || b3Dot( normal, inward ) > tolerance;
			alongEdge = alongEdge || b3AbsFloat( b3Dot( normal, edge ) ) > tolerance;
		}
	}

	b3TriangleFeature feature = b3_featureNone;
	if ( nearCount == 2 )
	{
		if ( nearEdges[0] == 0 && nearEdges[1] == 1 )
		{
			feature = b3_featureVertex2;
		}
		else if ( nearEdges[0] == 1 )
		{
			feature = b3_featureVertex3;
		}
		else
		{
			feature = b3_featureVertex1;
		}
	}

	if ( nearCount == 0 || inwardTilt )
	{
		// A tilted hull face over the triangle interior, the triangle face owns the normal.
		return b3_neighborSnap;
	}
	else if ( nearCount == 1 )
	{
		if ( alongEdge )
		{
			return b3_neighborSnap;
		}

		b3TriangleFeature edgeFeatures[3] = { b3_featureEdge1, b3_featureEdge2, b3_featureEdge3 };
		feature = edgeFeatures[nearEdges[0]];
	}
	else if ( nearCount == 3 )
	{
		return b3_neighborSnap;
	}

	*nearFeature = feature;

	// Never skip a hull face, its normal can miss the whole vertex cone so every triangle would
	// skip, the face contact is empty when the hull is off this triangle.
	int ownerEdge;
	b3NeighborResult result =
		b3TestNeighbors( shape, triangleIndex, vertices, meshToLocal, feature, normal, &ownerEdge );
	return result == b3_neighborSkip || result == b3_neighborDrop ? b3_neighborSnap : result;
}

static int b3CollideMeshTriangles( b3Cluster** clustersOut, const b3Shape* shapeA, const b3Shape* shapeB,
								   b3Transform transformAtoB, b3TriangleCache* triangleCaches, int triangleCount,
								   bool enableSpeculative, bool handleGhosts, bool isFast, b3TaskContext* context,
								   b3Arena* arena )
{
	*clustersOut = NULL;

	// Collide with triangles and build manifolds
	b3LocalManifold** acceptedManifolds = b3Bump( arena, triangleCount * sizeof( b3LocalManifold* ) );
	int acceptedManifoldCount = 0;

	b3Matrix3 relativeMatrix = b3MakeMatrixFromQuat( transformAtoB.q );
	float linearSlop = B3_LINEAR_SLOP;

	// This should push apart shapes after a time of impact event.
	// In the past I've called this `polygon skin`, but PhysX and Unreal
	// call it `rest offset` which seems appropriate in this case.
	// It leads to a small visual gap but seems to improve the quality of mesh
	// collision, especially for hull versus mesh.
	float restOffset = B3_MESH_REST_OFFSET;

	const b3HullData* hullB = shapeB->type == b3_hullShape ? shapeB->hull : NULL;
	float hullSpeculativeDistance = enableSpeculative ? B3_SPECULATIVE_DISTANCE : restOffset;
	b3Vec3 hullMargin = { hullSpeculativeDistance, hullSpeculativeDistance, hullSpeculativeDistance };

	int* survivorIndices = b3Bump( arena, triangleCount * sizeof( int ) );
	b3Vec3* survivorVertices = b3Bump( arena, 3 * triangleCount * sizeof( b3Vec3 ) );
	int survivorCount = 0;

	const b3MeshTriangle* meshTriangles = NULL;
	const b3Vec3* meshVertices = NULL;
	b3Vec3 meshScale = b3Vec3_one;
	bool meshFlipped = false;
	bool unitScale = true;
	if ( shapeA->type == b3_meshShape )
	{
		meshTriangles = b3GetMeshTriangles( shapeA->mesh.data );
		meshVertices = b3GetMeshVertices( shapeA->mesh.data );
		meshScale = shapeA->mesh.scale;
		meshFlipped = meshScale.x * meshScale.y * meshScale.z < 0.0f;
		unitScale = meshScale.x == 1.0f && meshScale.y == 1.0f && meshScale.z == 1.0f;
	}

	b3FloatW4 cxx = b3SplatW4( relativeMatrix.cx.x );
	b3FloatW4 cxy = b3SplatW4( relativeMatrix.cx.y );
	b3FloatW4 cxz = b3SplatW4( relativeMatrix.cx.z );
	b3FloatW4 cyx = b3SplatW4( relativeMatrix.cy.x );
	b3FloatW4 cyy = b3SplatW4( relativeMatrix.cy.y );
	b3FloatW4 cyz = b3SplatW4( relativeMatrix.cy.z );
	b3FloatW4 czx = b3SplatW4( relativeMatrix.cz.x );
	b3FloatW4 czy = b3SplatW4( relativeMatrix.cz.y );
	b3FloatW4 czz = b3SplatW4( relativeMatrix.cz.z );
	b3FloatW4 px = b3SplatW4( transformAtoB.p.x );
	b3FloatW4 py = b3SplatW4( transformAtoB.p.y );
	b3FloatW4 pz = b3SplatW4( transformAtoB.p.z );

	b3FloatW4 marginW = b3SplatW4( hullMargin.x );
	b3AABB hullBounds = hullB != NULL ? hullB->aabb : (b3AABB){ 0 };
	b3FloatW4 hullLowerX = b3SplatW4( hullBounds.lowerBound.x );
	b3FloatW4 hullLowerY = b3SplatW4( hullBounds.lowerBound.y );
	b3FloatW4 hullLowerZ = b3SplatW4( hullBounds.lowerBound.z );
	b3FloatW4 hullUpperX = b3SplatW4( hullBounds.upperBound.x );
	b3FloatW4 hullUpperY = b3SplatW4( hullBounds.upperBound.y );
	b3FloatW4 hullUpperZ = b3SplatW4( hullBounds.upperBound.z );

	for ( int baseIndex = 0; baseIndex < triangleCount; baseIndex += 4 )
	{
		int laneCount = b3MinInt( 4, triangleCount - baseIndex );

		b3Vec3 lanes[3][4];
		for ( int lane = 0; lane < 4; ++lane )
		{
			int index = baseIndex + b3MinInt( lane, laneCount - 1 );
			int triangleIndex = triangleCaches[index].triangleIndex;

			if ( meshVertices != NULL )
			{
				b3MeshTriangle meshTriangle = meshTriangles[triangleIndex];
				b3Vec3 v1 = meshVertices[meshTriangle.index1];
				b3Vec3 v2 = meshVertices[meshFlipped ? meshTriangle.index3 : meshTriangle.index2];
				b3Vec3 v3 = meshVertices[meshFlipped ? meshTriangle.index2 : meshTriangle.index3];

				// This bool gets predicted, so worth it.
				if ( unitScale == false )
				{
					v1 = b3Mul( meshScale, v1 );
					v2 = b3Mul( meshScale, v2 );
					v3 = b3Mul( meshScale, v3 );
				}

				lanes[0][lane] = v1;
				lanes[1][lane] = v2;
				lanes[2][lane] = v3;
			}
			else
			{
				B3_ASSERT( shapeA->type == b3_heightShape );
				b3Triangle triangle = b3GetHeightFieldTriangle( shapeA->heightField, triangleIndex );
				lanes[0][lane] = triangle.vertices[0];
				lanes[1][lane] = triangle.vertices[1];
				lanes[2][lane] = triangle.vertices[2];
			}
		}

		// Transform triangle into the shape frame
		b3FloatW4 vx[3], vy[3], vz[3];
		for ( int k = 0; k < 3; ++k )
		{
			b3FloatW4 ax = b3SetW4( lanes[k][0].x, lanes[k][1].x, lanes[k][2].x, lanes[k][3].x );
			b3FloatW4 ay = b3SetW4( lanes[k][0].y, lanes[k][1].y, lanes[k][2].y, lanes[k][3].y );
			b3FloatW4 az = b3SetW4( lanes[k][0].z, lanes[k][1].z, lanes[k][2].z, lanes[k][3].z );

			vx[k] = b3AddW4( b3AddW4( b3AddW4( b3MulW4( cxx, ax ), b3MulW4( cyx, ay ) ), b3MulW4( czx, az ) ), px );
			vy[k] = b3AddW4( b3AddW4( b3AddW4( b3MulW4( cxy, ax ), b3MulW4( cyy, ay ) ), b3MulW4( czy, az ) ), py );
			vz[k] = b3AddW4( b3AddW4( b3AddW4( b3MulW4( cxz, ax ), b3MulW4( cyz, ay ) ), b3MulW4( czz, az ) ), pz );
		}

		uint32_t keepMask = ( 1u << laneCount ) - 1u;
		if ( hullB != NULL )
		{
			b3FloatW4 lowerX = b3SubW4( b3MinW4( vx[0], b3MinW4( vx[1], vx[2] ) ), marginW );
			b3FloatW4 lowerY = b3SubW4( b3MinW4( vy[0], b3MinW4( vy[1], vy[2] ) ), marginW );
			b3FloatW4 lowerZ = b3SubW4( b3MinW4( vz[0], b3MinW4( vz[1], vz[2] ) ), marginW );
			b3FloatW4 upperX = b3AddW4( b3MaxW4( vx[0], b3MaxW4( vx[1], vx[2] ) ), marginW );
			b3FloatW4 upperY = b3AddW4( b3MaxW4( vy[0], b3MaxW4( vy[1], vy[2] ) ), marginW );
			b3FloatW4 upperZ = b3AddW4( b3MaxW4( vz[0], b3MaxW4( vz[1], vz[2] ) ), marginW );

			b3FloatW4 separated = b3OrW4( b3LessThanW4( upperX, hullLowerX ), b3GreaterThanW4( lowerX, hullUpperX ) );
			separated = b3OrW4( separated,
								b3OrW4( b3LessThanW4( upperY, hullLowerY ), b3GreaterThanW4( lowerY, hullUpperY ) ) );
			separated = b3OrW4( separated,
								b3OrW4( b3LessThanW4( upperZ, hullLowerZ ), b3GreaterThanW4( lowerZ, hullUpperZ ) ) );

			uint32_t cullMask = keepMask & (uint32_t)b3MoveMaskW4( separated );
			keepMask &= ~cullMask;

			while ( cullMask != 0 )
			{
				uint32_t lane = b3CTZ32( cullMask );
				cullMask &= cullMask - 1u;
				triangleCaches[baseIndex + (int)lane].cache.satCache = (b3SATCache){ 0 };
			}
		}

		if ( keepMask == 0 )
		{
			continue;
		}

		float x[3][4], y[3][4], z[3][4];
		for ( int k = 0; k < 3; ++k )
		{
			b3StoreW4( x[k], vx[k] );
			b3StoreW4( y[k], vy[k] );
			b3StoreW4( z[k], vz[k] );
		}

		while ( keepMask != 0 )
		{
			uint32_t lane = b3CTZ32( keepMask );
			keepMask &= keepMask - 1u;

			b3Vec3* survivor = survivorVertices + 3 * survivorCount;
			survivor[0] = (b3Vec3){ x[0][lane], y[0][lane], z[0][lane] };
			survivor[1] = (b3Vec3){ x[1][lane], y[1][lane], z[1][lane] };
			survivor[2] = (b3Vec3){ x[2][lane], y[2][lane], z[2][lane] };
			survivorIndices[survivorCount] = baseIndex + (int)lane;
			survivorCount += 1;
		}
	}

	// Make room for clip points. Additional capacity for a heavy clip.
	int pointBufferCapacity = B3_MAX_POINTS_PER_TRIANGLE * survivorCount + 2 * B3_MAX_CLIP_POINTS;

	b3LocalManifoldPoint* pointBuffer = b3Bump( arena, pointBufferCapacity * sizeof( b3LocalManifoldPoint ) );
	int totalPointCount = 0;

	b3LocalManifold* manifoldBuffer = b3Bump( arena, survivorCount * sizeof( b3LocalManifold ) );
	int manifoldCount = 0;

	bool validateNeighbors = handleGhosts;

	int* survivorManifolds = b3Bump( arena, survivorCount * sizeof( int ) );
	memset( survivorManifolds, 0xFF, survivorCount * sizeof( int ) );

	for ( int survivorIndex = 0; survivorIndex < survivorCount && totalPointCount + 3 < pointBufferCapacity;
		  ++survivorIndex )
	{
		int index = survivorIndices[survivorIndex];
		int triangleIndex = triangleCaches[index].triangleIndex;
		const b3Vec3* vertices = survivorVertices + 3 * survivorIndex;

		b3Triangle triangle;
		if ( shapeA->type == b3_meshShape )
		{
			triangle = b3GetMeshTriangle( &shapeA->mesh, triangleIndex );
		}
		else
		{
			B3_ASSERT( shapeA->type == b3_heightShape );
			triangle = b3GetHeightFieldTriangle( shapeA->heightField, triangleIndex );
		}

		b3ContactCache* cache = &triangleCaches[index].cache;
		int pointCapacity = pointBufferCapacity - totalPointCount;
		b3LocalManifold* manifold = manifoldBuffer + manifoldCount;
		manifold->points = pointBuffer + totalPointCount;
		manifold->pointCount = 0;
		manifold->feature = b3_featureNone;

		switch ( shapeB->type )
		{
			case b3_capsuleShape:
				b3CollideTriangleAndCapsule( manifold, pointCapacity, vertices, &shapeB->capsule,
											 &cache->simplexCache );
				break;

			case b3_hullShape:
				// Cached edge contact is dangerous at high speed because the hull can rotate around
				// the edge and tunnel through the triangle.
				if ( isFast && cache->satCache.type == b3_edgePairAxis )
				{
					cache->satCache = (b3SATCache){ 0 };
				}

				b3CollideTriangleAndHull( manifold, pointCapacity, vertices[0], vertices[1], vertices[2],
										  triangle.flags, hullB, &cache->satCache, enableSpeculative );
				if ( context != NULL )
				{
					context->satCallCount += 1;
					context->satCacheHitCount += cache->satCache.hit;
				}
				break;

			case b3_sphereShape:
				b3CollideTriangleAndSphere( manifold, pointCapacity, vertices, &shapeB->sphere );
				break;

			default:
				B3_ASSERT( false );
				return 0;
		}

		b3TriangleFeature nearFeature = manifold->feature;
		b3NeighborResult result = b3_neighborAdmit;
		int ownerEdge = B3_NULL_INDEX;
		uint8_t meshFlags = manifold->feature == b3_featureHullFace ? b3_meshContactHullFace : 0;
		if ( validateNeighbors && manifold->pointCount > 0 && manifold->feature != b3_featureTriangleFace )
		{
			if ( manifold->feature == b3_featureHullFace )
			{
				result =
					b3TestHullFaceNeighbors( shapeA, triangleIndex, vertices, relativeMatrix, manifold, &nearFeature );
			}
			else
			{
				result = b3TestNeighbors( shapeA, triangleIndex, vertices, relativeMatrix, manifold->feature,
										  manifold->normal, &ownerEdge );
			}

			bool deep = false;
			if ( result == b3_neighborSkip || result == b3_neighborSnap )
			{
				float minSeparation = manifold->points[0].separation;
				for ( int i = 1; i < manifold->pointCount; ++i )
				{
					minSeparation = b3MinFloat( minSeparation, manifold->points[i].separation );
				}

				deep = minSeparation < -2.0f * linearSlop;
			}

			if ( result == b3_neighborSkip && deep )
			{
				// In deep overlap the owning neighbor may apply backside culling. Experiments
				// show the that in this case the neighbor should snap the normal.
				if ( shapeB->type != b3_sphereShape )
				{
					result = b3_neighborSnap;
				}
				else
				{
					// todo comment
					b3Vec3 neighborNormal;
					if ( ownerEdge != B3_NULL_INDEX &&
						 b3GetNeighborNormal( shapeA, triangleIndex, ownerEdge, &neighborNormal ) )
					{
						neighborNormal = b3MulMV( relativeMatrix, neighborNormal );
						float offset = b3Dot( neighborNormal, b3Sub( shapeB->sphere.center, vertices[ownerEdge] ) );
						if ( offset < 0.0f )
						{
							result = b3_neighborAdmit;
						}
					}
				}
			}

			if ( result == b3_neighborSnap )
			{
				meshFlags = b3_meshContactSnapped;
			}

			if ( result == b3_neighborSkip || result == b3_neighborSnap || result == b3_neighborDrop )
			{
				// Skip leaves the normal to the neighbor. A sphere off the face cannot snap.
				manifold->pointCount = 0;
			}

			if ( result == b3_neighborSnap && shapeB->type == b3_capsuleShape )
			{
				// Snap clips the segment to the triangle for a face contact. May clip all points.
				b3Plane plane = b3MakePlaneFromPoints( vertices[0], vertices[1], vertices[2] );
				b3BuildTriangleAndCapsuleFaceContact( manifold, vertices, plane, &shapeB->capsule );
			}
			else if ( result == b3_neighborSnap && shapeB->type == b3_hullShape )
			{
				b3BuildTriangleAndHullFaceContact( manifold, pointCapacity, vertices, triangle.flags, hullB,
												   &cache->satCache, enableSpeculative );
			}
		}

		int manifoldPointCount = manifold->pointCount;

		if ( manifoldPointCount > 0 )
		{
			B3_ASSERT( manifold->feature != b3_featureNone );

			b3TriangleFeature pointFeature = manifold->feature;
			if ( pointFeature == b3_featureHullFace && nearFeature != b3_featureHullFace &&
				 nearFeature != b3_featureNone )
			{
				pointFeature = nearFeature;
			}
			else if ( pointFeature == b3_featureHullFace && nearFeature == b3_featureNone &&
					  result == b3_neighborAdmit )
			{
				pointFeature = b3_featureTriangleFace;
			}

			for ( int i = 0; i < manifoldPointCount; ++i )
			{
				manifold->points[i].triangleFeature = (uint8_t)pointFeature;
				manifold->points[i].meshFlags = meshFlags;
			}

			manifold->triangleIndex = triangleIndex;
			manifold->triangleNormal = b3MakeNormalFromPoints( vertices[0], vertices[1], vertices[2] );

			survivorManifolds[survivorIndex] = manifoldCount;
			manifoldCount += 1;
			totalPointCount += manifoldPointCount;
		}
	}

	if ( hullB != NULL && validateNeighbors )
	{
		// Cull triangles fully below the hull face when it rests on a triangle and has
		// a tilted manifold normal. This helps when a box slides across a chamfered
		// channel.
		float tolerance = B3_NEIGHBOR_TOLERANCE;
		float cosSquaredTolerance = 1.0f - tolerance * tolerance;
		const b3LocalManifold* resting = NULL;
		for ( int i = 0; i < manifoldCount && resting == NULL; ++i )
		{
			const b3LocalManifold* manifold = manifoldBuffer + i;
			float cosAngle = b3Dot( manifold->normal, manifold->triangleNormal );
			if ( manifold->pointCount > 0 && manifold->feature == b3_featureHullFace && cosAngle > 0.0f &&
				 cosAngle * cosAngle >= cosSquaredTolerance )
			{
				resting = manifold;
			}
		}

		if ( resting != NULL )
		{
			b3Vec3 normal = resting->triangleNormal;
			const b3Vec3* hullPoints = b3GetHullPoints( hullB );
			float hullBottom = FLT_MAX;
			for ( int i = 0; i < hullB->vertexCount; ++i )
			{
				hullBottom = b3MinFloat( hullBottom, b3Dot( normal, hullPoints[i] ) );
			}

			for ( int i = 0; i < survivorCount; ++i )
			{
				if ( survivorManifolds[i] == B3_NULL_INDEX )
				{
					continue;
				}

				b3LocalManifold* manifold = manifoldBuffer + survivorManifolds[i];
				const b3Vec3* vertices = survivorVertices + 3 * i;
				float cosAngle = b3Dot( manifold->normal, normal );
				float height = b3MaxFloat( b3Dot( normal, vertices[0] ),
										   b3MaxFloat( b3Dot( normal, vertices[1] ), b3Dot( normal, vertices[2] ) ) );
				if ( ( cosAngle <= 0.0f || cosAngle * cosAngle < cosSquaredTolerance ) && height < hullBottom )
				{
					manifold->pointCount = 0;
				}
			}
		}
	}

	for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
	{
		b3LocalManifold* manifold = manifoldBuffer + manifoldIndex;
		if ( manifold->pointCount > 0 )
		{
			acceptedManifolds[acceptedManifoldCount++] = manifold;
		}
	}

	B3_ASSERT( acceptedManifoldCount <= triangleCount );

	if ( acceptedManifoldCount == 0 )
	{
		return 0;
	}

	b3Cluster* clusters = b3Bump( arena, acceptedManifoldCount * sizeof( b3Cluster ) );
	int* clusterMemberships = b3Bump( arena, acceptedManifoldCount * sizeof( int ) );

	// Cluster tolerance is tighter than the warm starting manifold matching tolerance. These
	// serve different purposes.
	const float clusterThreshold = 0.996f;
	int clusterCount = 0;
	int clusterPointCount = 0;
	for ( int i = 0; i < acceptedManifoldCount; ++i )
	{
		clusterMemberships[i] = B3_NULL_INDEX;

		const b3LocalManifold* manifold = acceptedManifolds[i];
		clusterPointCount += manifold->pointCount;

		// Cluster based on the triangle normal and contact normal.
		// The first cluster found is accepted because the tolerance is tight.
		// todo consider requiring the triangles to be connect by an edge.
		// todo consider looking for the best cluster instead of the first one within tolerance
		// This bool is here to allow quick testing with and without clustering.
		bool allowClustering = true;
		b3Vec3 manifoldNormal = manifold->normal;
		b3Vec3 triangleNormal = manifold->triangleNormal;
		int clusterIndex = B3_NULL_INDEX;
		for ( int j = 0; j < clusterCount && allowClustering; ++j )
		{
			float cosManifoldAngle = b3Dot( clusters[j].manifoldNormal, manifoldNormal );
			float cosTriangleAngle = b3Dot( clusters[j].triangleNormal, triangleNormal );
			if ( cosManifoldAngle <= clusterThreshold || cosTriangleAngle <= clusterThreshold )
			{
				continue;
			}

			// Found a cluster
			clusterIndex = j;
			break;
		}

		if ( clusterIndex != B3_NULL_INDEX )
		{
			clusterMemberships[i] = clusterIndex;
			clusters[clusterIndex].pointCapacity += manifold->pointCount;
		}
		else
		{
			clusters[clusterCount].manifoldNormal = manifoldNormal;
			clusters[clusterCount].triangleNormal = triangleNormal;
			clusters[clusterCount].pointCapacity = manifold->pointCount;
			clusterMemberships[i] = clusterCount;
			clusterCount += 1;
		}
	}

	if ( clusterPointCount == 0 )
	{
		return 0;
	}

	// Setup clusters
	b3LocalManifoldPoint* clusterPoints = b3Bump( arena, clusterPointCount * sizeof( b3LocalManifoldPoint ) );
	int pointOffset = 0;

	for ( int i = 0; i < clusterCount; ++i )
	{
		b3Cluster* cluster = clusters + i;
		cluster->points = clusterPoints + pointOffset;
		cluster->pointCount = 0;
		pointOffset += cluster->pointCapacity;
	}

	// Populate clusters
	for ( int i = 0; i < acceptedManifoldCount; ++i )
	{
		int clusterIndex = clusterMemberships[i];
		if ( clusterIndex == B3_NULL_INDEX )
		{
			continue;
		}

		B3_ASSERT( 0 <= clusterIndex && clusterIndex < clusterCount );

		b3LocalManifold* am = acceptedManifolds[i];
		b3Cluster* cm = clusters + clusterIndex;
		for ( int j = 0; j < am->pointCount; ++j )
		{
			B3_ASSERT( cm->pointCount < cm->pointCapacity );
			b3LocalManifoldPoint* ap = am->points + j;
			b3LocalManifoldPoint* cp = cm->points + cm->pointCount;

			cp->triangleIndex = am->triangleIndex;
			cp->point = ap->point;
			cp->separation = ap->separation;
			cp->pair = ap->pair;
			cp->triangleFeature = ap->triangleFeature;
			cp->meshFlags = ap->meshFlags;
			cm->pointCount += 1;
		}
	}

	// Simplify clusters
	for ( int i = 0; i < clusterCount; ++i )
	{
		b3Cluster* cm = clusters + i;
		B3_ASSERT( cm->pointCount == cm->pointCapacity );
		int reducedCount = b3ReduceCluster( cm->points, cm->pointCount, cm->triangleNormal, *arena );
		cm->pointCount = reducedCount;
	}

	*clustersOut = clusters;
	return clusterCount;
}

bool b3ComputeMeshManifolds( b3World* world, int workerIndex, b3Contact* contact, const b3Shape* shapeA,
							 const uint16_t* materialMap, b3WorldTransform xfA, const b3Shape* shapeB,
							 b3WorldTransform xfB, bool isFast, b3Arena arena )
{
	B3_ASSERT( shapeA->type == b3_meshShape || shapeA->type == b3_heightShape );
	b3TaskContext* context = b3Array_Get( world->taskContexts, workerIndex );

	b3RefreshCache( contact, shapeA, xfA, &shapeB->aabb, arena );

	b3MeshContact* meshContact = &contact->meshContact;
	b3Transform transformAtoB = b3InvMulWorldTransforms( xfB, xfA );
	bool enableSpeculative = contact->flags & b3_enableSpeculativePoints;
	bool handleGhosts = B3_FORCE_GHOST_COLLISIONS == 0;

	b3Cluster* clusters;
	int clusterCount = b3CollideMeshTriangles( &clusters, shapeA, shapeB, transformAtoB,
											   meshContact->triangleCache.data, meshContact->triangleCache.count,
											   enableSpeculative, handleGhosts, isFast, context, &arena );

	if ( clusterCount == 0 )
	{
		if ( contact->manifoldCount > 0 )
		{
			b3FreeManifolds( world, contact->manifolds, contact->manifoldCount );
			contact->manifolds = NULL;
			contact->manifoldCount = 0;
		}
		return false;
	}

	float restOffset = B3_MESH_REST_OFFSET;

	// Make a temporary copy of previous manifolds
	int oldManifoldCount = contact->manifoldCount;
	b3Manifold* oldManifolds = NULL;
	if ( oldManifoldCount > 0 )
	{
		oldManifolds = b3Bump( &arena, oldManifoldCount * sizeof( b3Manifold ) );
		memcpy( oldManifolds, contact->manifolds, oldManifoldCount * sizeof( b3Manifold ) );
	}

	// Resize manifolds if needed
	if ( oldManifoldCount != clusterCount )
	{
		b3FreeManifolds( world, contact->manifolds, contact->manifoldCount );
		contact->manifolds = b3AllocateManifolds( world, clusterCount );
		contact->manifoldCount = (uint16_t)clusterCount;
	}
	else
	{
		// Mem zero manifolds
		memset( contact->manifolds, 0, contact->manifoldCount * sizeof( b3Manifold ) );
	}

	bool* consumed = NULL;
	if ( oldManifoldCount > 0 )
	{
		consumed = b3Bump( &arena, oldManifoldCount * sizeof( bool ) );
		memset( consumed, 0, oldManifoldCount * sizeof( bool ) );
	}

	b3Matrix3 matrixB = b3MakeMatrixFromQuat( xfB.q );
	b3Vec3 offsetA = b3SubPos( xfB.p, xfA.p );

	const float normalMatchTolerance = 0.995f;
	for ( int i = 0; i < clusterCount; ++i )
	{
		b3Cluster* cm = clusters + i;
		int pointCount = cm->pointCount;
		B3_ASSERT( 0 < pointCount && pointCount <= B3_MAX_MANIFOLD_POINTS );

		b3Manifold* manifold = contact->manifolds + i;
		manifold->pointCount = pointCount;
		manifold->normal = b3MulMV( matrixB, cm->manifoldNormal );

		b3Vec3 clusterNormal = b3MulMV( matrixB, cm->manifoldNormal );
		float bestDot = normalMatchTolerance;
		int bestIndex = B3_NULL_INDEX;

		for ( int j = 0; j < oldManifoldCount; ++j )
		{
			if ( consumed[j] == true )
			{
				continue;
			}

			float dot = b3Dot( oldManifolds[j].normal, clusterNormal );
			if ( dot > bestDot )
			{
				bestIndex = j;
				bestDot = dot;
			}
		}

		b3Manifold* matchedManifold = NULL;
		if ( bestIndex != B3_NULL_INDEX )
		{
			matchedManifold = oldManifolds + bestIndex;
			manifold->frictionImpulse = matchedManifold->frictionImpulse;
			manifold->rollingImpulse = matchedManifold->rollingImpulse;
			manifold->twistImpulse = matchedManifold->twistImpulse;
			consumed[bestIndex] = true;
		}

		for ( int j = 0; j < pointCount; ++j )
		{
			const b3LocalManifoldPoint* source = cm->points + j;
			b3ManifoldPoint* target = manifold->points + j;

			// Contact points are computed in frame B
			target->anchorB = b3MulMV( matrixB, source->point );
			target->anchorA = b3Add( target->anchorB, offsetA );
			target->separation = source->separation - restOffset;
			target->featureId = b3MakeFeatureId( source->pair );
			target->triangleIndex = source->triangleIndex;
			target->triangleFeature = source->triangleFeature;
			target->meshFlags = source->meshFlags;

			// Preserve normal impulse if possible
			if ( matchedManifold != NULL )
			{
				int oldPointCount = matchedManifold->pointCount;
				for ( int k = 0; k < oldPointCount; ++k )
				{
					b3ManifoldPoint* oldPt = matchedManifold->points + k;

					if ( target->featureId == oldPt->featureId && target->triangleIndex == oldPt->triangleIndex )
					{
						target->normalImpulse = oldPt->normalImpulse;
						target->persisted = true;

						// claimed
						oldPt->triangleIndex = B3_NULL_INDEX;
						break;
					}
				}
			}
		}
	}

	const b3SurfaceMaterial* materialsA = b3GetShapeMaterials( shapeA );
	const b3SurfaceMaterial* materialB = b3GetShapeMaterials( shapeB );
	b3Vec3 tangentVelocityA = b3Vec3_zero;

	// Update friction and restitution if the mesh has per triangle material
	if ( shapeA->materialCount > 0 )
	{
		float friction = 0.0f;
		float restitution = 0.0f;
		float sampleCount = 0.0f;

		const uint8_t* materialIndices;
		if ( shapeA->type == b3_meshShape )
		{
			materialIndices = b3GetMeshMaterialIndices( shapeA->mesh.data );
		}
		else
		{
			materialIndices = b3GetHeightFieldMaterialIndices( shapeA->heightField );
		}

		for ( int i = 0; i < clusterCount; ++i )
		{
			b3Manifold* manifold = contact->manifolds + i;
			int pointCount = manifold->pointCount;
			for ( int j = 0; j < pointCount; ++j )
			{
				int triangleIndex = manifold->points[j].triangleIndex;
				int materialIndex;
				if ( shapeA->type == b3_meshShape )
				{
					materialIndex = materialIndices[triangleIndex];

					if ( materialMap != NULL )
					{
						materialIndex = materialMap[materialIndex];
					}
				}
				else
				{
					materialIndex = materialIndices[triangleIndex >> 1];
				}

				materialIndex = b3ClampInt( materialIndex, 0, shapeA->materialCount - 1 );
				b3SurfaceMaterial material = materialsA[materialIndex];
				friction += world->frictionCallback( material.friction, material.userMaterialId, materialB->friction,
													 materialB->userMaterialId );
				restitution += world->restitutionCallback( material.restitution, material.userMaterialId,
														   materialB->restitution, materialB->userMaterialId );

				tangentVelocityA = b3Add( tangentVelocityA, material.tangentVelocity );

				sampleCount += 1.0f;
			}
		}

		if ( sampleCount > 0.0f )
		{
			float invCount = 1.0f / sampleCount;
			contact->friction = invCount * friction;
			contact->restitution = invCount * restitution;
			tangentVelocityA = b3MulSV( invCount, tangentVelocityA );
		}

		B3_ASSERT( b3IsValidFloat( contact->friction ) && contact->friction >= 0.0f );
		B3_ASSERT( b3IsValidFloat( contact->restitution ) && contact->restitution >= 0.0f );
	}
	else
	{
		// Keep these updated in case the values on the shapes are modified
		contact->friction = world->frictionCallback( materialsA[0].friction, materialsA[0].userMaterialId,
													 materialB->friction, materialB->userMaterialId );
		contact->restitution = world->restitutionCallback( materialsA[0].restitution, materialsA[0].userMaterialId,
														   materialB->restitution, materialB->userMaterialId );
		tangentVelocityA = materialsA[0].tangentVelocity;
	}

	tangentVelocityA = b3RotateVector( xfA.q, tangentVelocityA );

	float radiusB = 0.0f;
	if ( shapeB->type == b3_sphereShape )
	{
		radiusB = shapeB->sphere.radius;
	}
	else if ( shapeB->type == b3_capsuleShape )
	{
		radiusB = shapeB->capsule.radius;
	}
	else if ( shapeB->type == b3_hullShape )
	{
		radiusB = shapeB->hull->innerRadius;
	}

	contact->rollingResistance = materialB->rollingResistance * radiusB;

	b3Vec3 tangentVelocityB = b3RotateVector( xfB.q, materialB->tangentVelocity );
	contact->tangentVelocity = b3Sub( tangentVelocityA, tangentVelocityB );
	return true;
}

int b3CollideMeshAndHull( b3Manifold* manifolds, int capacity, const b3Mesh* mesh, const b3HullData* hull,
						  b3Transform transformBtoA, bool enableSpeculative, bool handleGhosts )
{
	if ( capacity <= 0 )
	{
		return 0;
	}

	b3Shape meshShape = {
		.type = b3_meshShape,
		.mesh = *mesh,
	};

	b3Shape hullShape = {
		.type = b3_hullShape,
		.hull = hull,
	};

	b3Arena arena = b3CreateArena( 64 * 1024 );

	float margin = B3_SPECULATIVE_DISTANCE;
	b3Vec3 extension = { margin, margin, margin };
	b3AABB hullBounds = b3AABB_Transform( transformBtoA, hull->aabb );
	b3AABB queryBounds = {
		.lowerBound = b3Sub( hullBounds.lowerBound, extension ),
		.upperBound = b3Add( hullBounds.upperBound, extension ),
	};

	b3TriangleQueryContext queryContext = {
		.arena = &arena,
		.indices = b3Bump( &arena, B3_INITIAL_TRIANGLE_CAPACITY * (int)sizeof( int ) ),
		.capacity = B3_INITIAL_TRIANGLE_CAPACITY,
		.count = 0,
	};

	b3QueryMesh( mesh, queryBounds, b3CollectTriangleIndicesCallback, &queryContext );

	int triangleCount = queryContext.count;
	b3TriangleCache* triangleCaches = b3Bump( &arena, triangleCount * (int)sizeof( b3TriangleCache ) );
	for ( int i = 0; i < triangleCount; ++i )
	{
		triangleCaches[i] = (b3TriangleCache){ .triangleIndex = queryContext.indices[i] };
	}

	b3Transform transformAtoB = b3InvertTransform( transformBtoA );

	b3Cluster* clusters;
	int clusterCount = b3CollideMeshTriangles( &clusters, &meshShape, &hullShape, transformAtoB, triangleCaches,
											   triangleCount, enableSpeculative, handleGhosts, false, NULL, &arena );

	int count = b3MinInt( clusterCount, capacity );
	b3Matrix3 matrixB = b3MakeMatrixFromQuat( transformBtoA.q );
	float restOffset = B3_MESH_REST_OFFSET;

	for ( int i = 0; i < count; ++i )
	{
		const b3Cluster* cluster = clusters + i;
		b3Manifold* manifold = manifolds + i;
		*manifold = (b3Manifold){ 0 };
		manifold->normal = b3MulMV( matrixB, cluster->manifoldNormal );
		manifold->pointCount = cluster->pointCount;

		for ( int j = 0; j < cluster->pointCount; ++j )
		{
			const b3LocalManifoldPoint* source = cluster->points + j;
			b3ManifoldPoint* target = manifold->points + j;
			target->anchorB = b3MulMV( matrixB, source->point );
			target->anchorA = b3Add( target->anchorB, transformBtoA.p );
			target->separation = source->separation - restOffset;
			target->featureId = b3MakeFeatureId( source->pair );
			target->triangleIndex = source->triangleIndex;
			target->triangleFeature = source->triangleFeature;
			target->meshFlags = source->meshFlags;
		}
	}

	b3DestroyArena( &arena );
	return count;
}

// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "contact.h"
#include "manifold.h"
#include "physics_world.h"
#include "qsort.h"
#include "shape.h"

#include "box3d/types.h"

#include <stdio.h>

// This guards against excessive memory usage and complex collision
#define B3_MAX_MESH_CONTACT_TRIANGLES 256
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

typedef struct b3TriangleQueryContext
{
	int* indices;
	int capacity;
	int count;
} b3TriangleQueryContext;

static bool b3CollectTriangleIndicesCallback( b3Vec3 a, b3Vec3 b, b3Vec3 c, int triangleIndex, void* context )
{
	B3_UNUSED( a, b, c );
	b3TriangleQueryContext* triangleContext = (b3TriangleQueryContext*)context;
	if ( triangleContext->count == triangleContext->capacity )
	{
		return false;
	}
	triangleContext->indices[triangleContext->count] = triangleIndex;
	triangleContext->count += 1;
	return triangleContext->count < triangleContext->capacity;
}

static int b3QueryMeshTriangles( int* indices, int capacity, const b3Mesh* mesh, b3AABB bounds )
{
	b3TriangleQueryContext context = {
		.indices = indices,
		.capacity = capacity,
		.count = 0,
	};

	b3QueryMesh( mesh, bounds, b3CollectTriangleIndicesCallback, &context );
	return context.count;
}

static int b3QueryHeightFieldTriangles( int* indices, int capacity, const b3HeightFieldData* heightField, b3AABB bounds )
{
	b3TriangleQueryContext context = {
		.indices = indices,
		.capacity = capacity,
		.count = 0,
	};

	b3QueryHeightField( heightField, bounds, b3CollectTriangleIndicesCallback, &context );
	return context.count;
}

static void b3RefreshCache( b3MeshContact* meshContact, const b3Shape* shapeA, b3WorldTransform xfA, const b3AABB* bounds )
{
	B3_ASSERT( shapeA->type == b3_meshShape || shapeA->type == b3_heightShape );

	// If the dynamic body didn't move out of the cached query bounds we are done!
	if ( b3AABB_Contains( meshContact->queryBounds, *bounds ) )
	{
		if ( shapeA->type == b3_meshShape )
		{
			for ( int i = 0; i < meshContact->triangleCache.count; ++i )
			{
				B3_ASSERT( 0 <= meshContact->triangleCache.data[i].triangleIndex &&
						   meshContact->triangleCache.data[i].triangleIndex < shapeA->mesh.data->triangleCount );
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
	int triangleCapacity = B3_MAX_MESH_CONTACT_TRIANGLES;

	int triangleIndices[B3_MAX_MESH_CONTACT_TRIANGLES];

	// Bounds are in world space. Convert to the local mesh frame. The broadphase bounds are float,
	// so the demoted mesh transform is the matching float world frame (exact in float mode).
	b3Transform meshTransform = b3ToRelativeTransform( xfA, b3Pos_zero );
	b3AABB localBounds = b3AABB_Transform( b3InvertTransform( meshTransform ), meshContact->queryBounds );
	int triangleCount;
	if ( shapeA->type == b3_meshShape )
	{
		triangleCount = b3QueryMeshTriangles( triangleIndices, triangleCapacity, &shapeA->mesh, localBounds );
	}
	else
	{
		B3_ASSERT( shapeA->type == b3_heightShape );
		triangleCount = b3QueryHeightFieldTriangles( triangleIndices, triangleCapacity, shapeA->heightField, localBounds );
	}

	if ( triangleCount == triangleCapacity )
	{
		static bool s_once = false;
		if ( s_once == false )
		{
			b3Log( "WARNING: complex mesh detected, triangle buffer capacity of %d reached", triangleCapacity );
			s_once = true;
		}
	}

	// Triangle indices must be sorted to match caches.
	B3_VALIDATE( b3IsSorted( triangleIndices, triangleCount ) );

	// Create new contact cache and match with old one
	b3ContactCache contactCache[B3_MAX_MESH_CONTACT_TRIANGLES];

	int index2 = 0;
	for ( int index1 = 0; index1 < triangleCount; ++index1 )
	{
		contactCache[index1] = (b3ContactCache){ 0 };

		while ( index2 < meshContact->triangleCache.count &&
				meshContact->triangleCache.data[index2].triangleIndex < triangleIndices[index1] )
		{
			index2 += 1;
		}

		if ( index2 < meshContact->triangleCache.count &&
			 meshContact->triangleCache.data[index2].triangleIndex == triangleIndices[index1] )
		{
			contactCache[index1] = meshContact->triangleCache.data[index2].cache;
		}
	}

	// Save new cache
	b3Array_Resize( meshContact->triangleCache, triangleCount );
	for ( int i = 0; i < triangleCount; ++i )
	{
		meshContact->triangleCache.data[i] = (b3TriangleCache){ triangleIndices[i], contactCache[i] };

		if ( shapeA->type == b3_meshShape )
		{
			B3_ASSERT( 0 <= meshContact->triangleCache.data[i].triangleIndex &&
					   meshContact->triangleCache.data[i].triangleIndex < shapeA->mesh.data->triangleCount );
		}
	}
}

typedef struct b3TentativeTriangle
{
	float squaredDistance;
	int index;
} b3TentativeTriangle;

#define B3_MAX_EDGE_COUNT 64

typedef struct b3FoundEdges
{
	uint64_t keys[B3_MAX_EDGE_COUNT];
	int count;
} b3FoundEdges;

static inline bool b3AddEdge( b3FoundEdges* edges, int vertex1, int vertex2 )
{
	uint64_t i1 = (uint64_t)b3MinInt( vertex1, vertex2 );
	uint64_t i2 = (uint64_t)b3MaxInt( vertex1, vertex2 );
	uint64_t key = i1 << 32 | i2;

	int count = edges->count;
	for ( int i = 0; i < count; ++i )
	{
		if ( edges->keys[i] == key )
		{
			return false;
		}
	}

	if ( count == B3_MAX_EDGE_COUNT )
	{
		// This will lead to a potential ghost collision
		return true;
	}

	edges->keys[count] = key;
	edges->count += 1;

	return true;
}

static inline bool b3FindEdge( b3FoundEdges* edges, int vertex1, int vertex2 )
{
	uint64_t i1 = (uint64_t)b3MinInt( vertex1, vertex2 );
	uint64_t i2 = (uint64_t)b3MaxInt( vertex1, vertex2 );
	uint64_t key = i1 << 32 | i2;

	int count = edges->count;
	for ( int i = 0; i < count; ++i )
	{
		if ( edges->keys[i] == key )
		{
			return true;
		}
	}

	return false;
}

#if 0
// Two triangles share an edge iff they share at least two vertex indices.
static inline bool b3TrianglesShareEdge( int a1, int a2, int a3, int b1, int b2, int b3 )
{
	int matches = 0;
	matches += ( a1 == b1 || a1 == b2 || a1 == b3 );
	matches += ( a2 == b1 || a2 == b2 || a2 == b3 );
	matches += ( a3 == b1 || a3 == b2 || a3 == b3 );
	return matches >= 2;
}
#endif

#define B3_MAX_VERTEX_COUNT 64

typedef struct b3FoundVertices
{
	int keys[B3_MAX_VERTEX_COUNT];
	int count;
} b3FoundVertices;

static inline bool b3AddVertex( b3FoundVertices* vertices, int vertex )
{
	int key = vertex;

	int count = vertices->count;
	for ( int i = 0; i < count; ++i )
	{
		if ( vertices->keys[i] == key )
		{
			return false;
		}
	}

	if ( count == B3_MAX_VERTEX_COUNT )
	{
		// This will lead to a potential ghost collision
		return true;
	}

	vertices->keys[count] = key;
	vertices->count += 1;

	return true;
}

// Returns true if (score, separation) should replace (bestScore, bestSeparation).
static inline bool b3IsBetterCullCandidate( float score, float separation, float bestScore, float bestSeparation, float scoreTol,
											float separationTol )
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

typedef struct b3Point2D
{
	b3Vec2 p;
	float separation;
	int originalIndex;
} b3Point2D;

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

	int count2 = b3CullPoints( pts, count1 );
	B3_ASSERT( count2 <= B3_MAX_MANIFOLD_POINTS );

	b3LocalManifoldPoint finalPoints[B3_MAX_MANIFOLD_POINTS];
	for ( int i = 0; i < count2; ++i )
	{
		int index = pts[i].originalIndex;
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

// Collide a mesh or height-field sub against shapeB. Produces accepted local manifolds with points in the
// shapeB frame. All buffers are caller owned so the results persist. The cache must already be refreshed.
static int b3GatherMeshSubManifolds( b3World* world, int workerIndex, b3MeshContact* meshContact, bool enableSpeculative,
									 const b3Shape* shapeA, b3WorldTransform xfA, const b3Shape* shapeB, b3WorldTransform xfB,
									 bool isFast, b3LocalManifold** acceptedManifolds, b3LocalManifold** tentativeManifolds,
									 b3TentativeTriangle* tentativeTriangles, b3LocalManifold* manifoldBuffer,
									 b3LocalManifoldPoint* pointBuffer, int pointBufferCapacity, int triangleCount )
{
	b3TaskContext* context = b3Array_Get( world->taskContexts, workerIndex );

	int acceptedManifoldCount = 0;
	int tentativeManifoldCount = 0;
	int tentativeTriangleCount = 0;

	b3FoundEdges foundEdges;
	b3FoundVertices foundVertices;
	foundEdges.count = 0;
	foundVertices.count = 0;

	// This transform converts from mesh frame into the shapeB frame
	b3Transform transformAtoB = b3InvMulWorldTransforms( xfB, xfA );
	b3Matrix3 relativeMatrix = b3MakeMatrixFromQuat( transformAtoB.q );
	float linearSlop = B3_LINEAR_SLOP;

	int totalPointCount = 0;
	int manifoldCount = 0;

	b3TriangleCache* triangleCaches = meshContact->triangleCache.data;

	const b3HullData* hullB = shapeB->type == b3_hullShape ? shapeB->hull : NULL;

	for ( int index = 0; index < triangleCount && totalPointCount + 3 < pointBufferCapacity; ++index )
	{
		int triangleIndex = triangleCaches[index].triangleIndex;

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

		// Transform triangle into the shape frame
		b3Vec3 vertices[3];
		vertices[0] = b3Add( b3MulMV( relativeMatrix, triangle.vertices[0] ), transformAtoB.p );
		vertices[1] = b3Add( b3MulMV( relativeMatrix, triangle.vertices[1] ), transformAtoB.p );
		vertices[2] = b3Add( b3MulMV( relativeMatrix, triangle.vertices[2] ), transformAtoB.p );

		b3ContactCache* cache = &triangleCaches[index].cache;
		int pointCapacity = pointBufferCapacity - totalPointCount;
		b3LocalManifold* manifold = manifoldBuffer + manifoldCount;
		manifold->points = pointBuffer + totalPointCount;
		manifold->pointCount = 0;
		manifold->triangleFlags = triangle.flags;
		manifold->feature = b3_featureNone;

		switch ( shapeB->type )
		{
			case b3_capsuleShape:
				b3CollideCapsuleAndTriangle( manifold, pointCapacity, &shapeB->capsule, vertices, &cache->simplexCache );
				break;

			case b3_hullShape:
				// Cached edge contact is dangerous at high speed because the hull can rotate around the edge and tunnel
				// through the triangle.
				if ( isFast && cache->satCache.type == b3_edgePairAxis )
				{
					cache->satCache = (b3SATCache){ 0 };
				}

				b3CollideHullAndTriangle( manifold, pointCapacity, hullB, vertices[0], vertices[1], vertices[2], triangle.flags,
										  &cache->satCache, enableSpeculative );
				context->satCallCount += 1;
				context->satCacheHitCount += cache->satCache.hit;
				break;

			case b3_sphereShape:
				b3CollideSphereAndTriangle( manifold, pointCapacity, &shapeB->sphere, vertices );
				break;

			default:
				B3_ASSERT( false );
				return false;
		}

		int manifoldPointCount = manifold->pointCount;

		if ( manifoldPointCount > 0 )
		{
			B3_ASSERT( manifold->feature != b3_featureNone );

			manifoldCount += 1;
			totalPointCount += manifoldPointCount;
			manifold->triangleIndex = triangleIndex;
			manifold->triangleNormal = b3MakeNormalFromPoints( vertices[0], vertices[1], vertices[2] );
			manifold->i1 = triangle.i1;
			manifold->i2 = triangle.i2;
			manifold->i3 = triangle.i3;

			if ( manifold->feature == b3_featureTriangleFace || B3_FORCE_GHOST_COLLISIONS )
			{
				(void)b3AddEdge( &foundEdges, triangle.i1, triangle.i2 );
				(void)b3AddEdge( &foundEdges, triangle.i2, triangle.i3 );
				(void)b3AddEdge( &foundEdges, triangle.i3, triangle.i1 );
				(void)b3AddVertex( &foundVertices, triangle.i1 );
				(void)b3AddVertex( &foundVertices, triangle.i2 );
				(void)b3AddVertex( &foundVertices, triangle.i3 );

				acceptedManifolds[acceptedManifoldCount++] = manifold;
			}
			else if ( manifold->feature == b3_featureHullFace )
			{
				float cosNormalAngle = b3Dot( manifold->triangleNormal, manifold->normal );
				if ( cosNormalAngle > 0.5f )
				{
					(void)b3AddEdge( &foundEdges, triangle.i1, triangle.i2 );
					(void)b3AddEdge( &foundEdges, triangle.i2, triangle.i3 );
					(void)b3AddEdge( &foundEdges, triangle.i3, triangle.i1 );
					(void)b3AddVertex( &foundVertices, triangle.i1 );
					(void)b3AddVertex( &foundVertices, triangle.i2 );
					(void)b3AddVertex( &foundVertices, triangle.i3 );

					acceptedManifolds[acceptedManifoldCount++] = manifold;
				}
				else
				{
					float minSeparation = manifold->points[0].separation;
					for ( int i = 1; i < manifoldPointCount; ++i )
					{
						minSeparation = b3MinFloat( minSeparation, manifold->points[i].separation );
					}

					if ( minSeparation < -2.0f * linearSlop )
					{
						// Deep overlap
						(void)b3AddEdge( &foundEdges, triangle.i1, triangle.i2 );
						(void)b3AddEdge( &foundEdges, triangle.i2, triangle.i3 );
						(void)b3AddEdge( &foundEdges, triangle.i3, triangle.i1 );
						(void)b3AddVertex( &foundVertices, triangle.i1 );
						(void)b3AddVertex( &foundVertices, triangle.i2 );
						(void)b3AddVertex( &foundVertices, triangle.i3 );
						acceptedManifolds[acceptedManifoldCount++] = manifold;
					}
					else
					{
						b3TentativeTriangle tentativeTriangle = { .squaredDistance = manifold->squaredDistance,
																  .index = tentativeManifoldCount };
						tentativeTriangles[tentativeTriangleCount++] = tentativeTriangle;
						tentativeManifolds[tentativeManifoldCount++] = manifold;
					}
				}
			}
			else
			{
				b3TentativeTriangle tentativeTriangle = { .squaredDistance = manifold->squaredDistance,
														  .index = tentativeManifoldCount };
				tentativeTriangles[tentativeTriangleCount++] = tentativeTriangle;
				tentativeManifolds[tentativeManifoldCount++] = manifold;
			}
		}
	}

	B3_ASSERT( acceptedManifoldCount <= triangleCount );
	B3_ASSERT( tentativeManifoldCount <= triangleCount );
	B3_ASSERT( tentativeTriangleCount <= triangleCount );

	if ( shapeB->type == b3_sphereShape )
	{
		// Sort triangles so the closest triangles are processed first
		{
#define LESS( i, j ) tentativeTriangles[(int)i].squaredDistance < tentativeTriangles[(int)j].squaredDistance
#define SWAP( i, j )                                                                                                             \
	do                                                                                                                           \
	{                                                                                                                            \
		b3TentativeTriangle tmp = tentativeTriangles[(int)i];                                                                    \
		tentativeTriangles[(int)i] = tentativeTriangles[(int)j];                                                                 \
		tentativeTriangles[(int)j] = tmp;                                                                                        \
	}                                                                                                                            \
	while ( 0 )
			QSORT( tentativeTriangleCount, LESS, SWAP );
#undef LESS
#undef SWAP
		}

		// Add tentative manifolds in sorted order. Avoid adding manifolds that generate ghost collisions.
		for ( int i = 0; i < tentativeTriangleCount; ++i )
		{
			b3LocalManifold* m = tentativeManifolds[tentativeTriangles[i].index];

			bool addedEdge1 = b3AddEdge( &foundEdges, m->i1, m->i2 );
			bool addedEdge2 = b3AddEdge( &foundEdges, m->i2, m->i3 );
			bool addedEdge3 = b3AddEdge( &foundEdges, m->i3, m->i1 );
			bool addedVertex1 = b3AddVertex( &foundVertices, m->i1 );
			bool addedVertex2 = b3AddVertex( &foundVertices, m->i2 );
			bool addedVertex3 = b3AddVertex( &foundVertices, m->i3 );

			b3TriangleFeature feature = m->feature;
			bool shouldCollide = false;
			switch ( feature )
			{
				case b3_featureNone:
				case b3_featureTriangleFace:
					B3_ASSERT( false );
					break;

				case b3_featureEdge1:
					shouldCollide = addedEdge1;
					break;

				case b3_featureEdge2:
					shouldCollide = addedEdge2;
					break;

				case b3_featureEdge3:
					shouldCollide = addedEdge3;
					break;

				case b3_featureVertex1:
					shouldCollide = addedVertex1;
					break;

				case b3_featureVertex2:
					shouldCollide = addedVertex2;
					break;

				case b3_featureVertex3:
					shouldCollide = addedVertex3;
					break;

				default:
					B3_ASSERT( false );
					break;
			}

			if ( shouldCollide == true )
			{
				acceptedManifolds[acceptedManifoldCount++] = m;
			}
		}
	}
	else
	{
		// Problem: hull can tunnel if time of impact is at concave edge
		// Example: flat box sliding down a ramp to a flat bottom
		// Solution: only ignore flat edges
		for ( int i = 0; i < tentativeManifoldCount; ++i )
		{
			b3LocalManifold* m = tentativeManifolds[i];
			int triangleFlags = m->triangleFlags;

			if ( ( triangleFlags & b3_allFlatEdges ) == b3_allFlatEdges )
			{
				continue;
			}

			if ( ( triangleFlags & b3_flatEdge1 ) == b3_flatEdge1 )
			{
				if ( b3FindEdge( &foundEdges, m->i1, m->i2 ) )
				{
					continue;
				}
			}

			if ( ( triangleFlags & b3_flatEdge2 ) == b3_flatEdge2 )
			{
				if ( b3FindEdge( &foundEdges, m->i2, m->i3 ) )
				{
					continue;
				}
			}

			if ( ( triangleFlags & b3_flatEdge3 ) == b3_flatEdge3 )
			{
				if ( b3FindEdge( &foundEdges, m->i3, m->i1 ) )
				{
					continue;
				}
			}

			acceptedManifolds[acceptedManifoldCount++] = m;
		}
	}

	B3_ASSERT( acceptedManifoldCount <= triangleCount );

	return acceptedManifoldCount;
}

bool b3ComputeMeshManifolds( b3World* world, int workerIndex, b3Contact* contact, const b3Shape* shapeA, const int* materialMap,
							 b3WorldTransform xfA, const b3Shape* shapeB, b3WorldTransform xfB, bool isFast, b3Arena arena )
{
	B3_ASSERT( shapeA->type == b3_meshShape || shapeA->type == b3_heightShape );

	b3MeshContact* meshContact = &contact->sub0.meshContact;
	b3RefreshCache( meshContact, shapeA, xfA, &shapeB->aabb );

	int triangleCount = meshContact->triangleCache.count;

	b3LocalManifold** acceptedManifolds = b3Bump( &arena, triangleCount * sizeof( b3LocalManifold* ) );
	b3LocalManifold** tentativeManifolds = b3Bump( &arena, triangleCount * sizeof( b3LocalManifold* ) );
	b3TentativeTriangle* tentativeTriangles = b3Bump( &arena, triangleCount * sizeof( b3TentativeTriangle ) );

	int pointBufferCapacity = B3_MAX_POINTS_PER_TRIANGLE * triangleCount;
	b3LocalManifoldPoint* pointBuffer = b3Bump( &arena, pointBufferCapacity * sizeof( b3LocalManifoldPoint ) );
	b3LocalManifold* manifoldBuffer = b3Bump( &arena, triangleCount * sizeof( b3LocalManifold ) );

	bool enableSpeculative = ( contact->flags & b3_enableSpeculativePoints ) != 0;

	int acceptedManifoldCount = b3GatherMeshSubManifolds( world, workerIndex, meshContact, enableSpeculative, shapeA, xfA, shapeB,
														  xfB, isFast, acceptedManifolds, tentativeManifolds, tentativeTriangles,
														  manifoldBuffer, pointBuffer, pointBufferCapacity, triangleCount );

	float restOffset = B3_MESH_REST_OFFSET;

	if ( acceptedManifoldCount == 0 )
	{
		if ( contact->manifoldCount > 0 )
		{
			b3FreeManifolds( world, contact->manifolds, contact->manifoldCount );
			contact->manifolds = NULL;
			contact->manifoldCount = 0;
		}
		return false;
	}

	b3Cluster* clusters = b3Bump( &arena, acceptedManifoldCount * sizeof( b3Cluster ) );
	int* clusterMemberships = b3Bump( &arena, acceptedManifoldCount * sizeof( int ) );

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

#if 0
			// todo there could be later triangles that create the connection
			// then failure to cluster breaks greedy impulse warm starting
			bool edgeConnected = false;

			for ( int k = 0; k < i; ++k )
			{
				if ( clusterMemberships[k] != j )
				{
					continue;
				}

				const b3LocalManifold* other = acceptedManifolds[k];
				if ( b3TrianglesShareEdge( manifold->i1, manifold->i2, manifold->i3, other->i1, other->i2, other->i3 ) )
				{
					edgeConnected = true;
					break;
				}
			}

			if ( edgeConnected )
			{
				clusterIndex = j;
				break;
			}
#else

			// Found a cluster
			clusterIndex = j;
			break;
#endif
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
		return false;
	}

	// Setup clusters
	b3LocalManifoldPoint* clusterPoints = b3Bump( &arena, clusterPointCount * sizeof( b3LocalManifoldPoint ) );
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
			cm->pointCount += 1;
		}
	}

	// Simplify clusters
	for ( int i = 0; i < clusterCount; ++i )
	{
		b3Cluster* cm = clusters + i;
		B3_ASSERT( cm->pointCount == cm->pointCapacity );
		int reducedCount = b3ReduceCluster( cm->points, cm->pointCount, cm->triangleNormal, arena );
		cm->pointCount = reducedCount;
	}

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
				restitution += world->restitutionCallback( material.restitution, material.userMaterialId, materialB->restitution,
														   materialB->userMaterialId );

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
		contact->friction = world->frictionCallback( materialsA[0].friction, materialsA[0].userMaterialId, materialB->friction,
													 materialB->userMaterialId );
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

// Per sub material data cached during gathering, consumed in the final averaging pass.
typedef struct b3SubMaterial
{
	bool isMesh;
	bool isHeightField;

	// Convex mixed values
	float friction;
	float restitution;

	// Mesh material lookup
	const b3SurfaceMaterial* materialsA;
	const b3SurfaceMaterial* materialB;
	const uint8_t* triMaterialIndices;
	const int* materialMap;
	int materialCount;

	b3Vec3 tangentVelocity;
	float rollingResistance;
} b3SubMaterial;

static inline float b3SubShapeRadius( b3ShapeType type, const b3Sphere* sphere, const b3Capsule* capsule, const b3HullData* hull )
{
	if ( type == b3_sphereShape )
	{
		return sphere->radius;
	}
	if ( type == b3_capsuleShape )
	{
		return capsule->radius;
	}
	return 0.25f * hull->innerRadius;
}

// Narrowphase for a body pair with more than one shape pair. Each sub is collided in its own frame, then
// all local manifolds are expressed in a single canonical frame (world axes relative to body A origin,
// normal from body A to body B), clustered by normal, reduced, and emitted like the mesh path.
bool b3ComputeMultiSubManifolds( b3World* world, int workerIndex, b3Contact* contact, b3WorldTransform xfA, b3WorldTransform xfB,
								 bool isFast, b3Arena arena )
{
	b3Shape* shapes = world->shapes.data;
	b3TaskContext* taskContext = b3Array_Get( world->taskContexts, workerIndex );
	int edges0BodyId = contact->edges[0].bodyId;
	int subCount = contact->subCount;
	bool enableSpeculative = ( contact->flags & b3_enableSpeculativePoints ) != 0;
	float restOffset = B3_MESH_REST_OFFSET;

	// Size the shared buffers. Refresh mesh caches now so triangle counts are known.
	int maxLocalManifolds = 0;
	int maxLocalPoints = 0;
	for ( int s = 0; s < subCount; ++s )
	{
		b3SubContact* sub = b3GetSubContact( contact, s );
		b3Shape* primShape = shapes + sub->shapeIdA;
		b3Shape* secShape = shapes + sub->shapeIdB;

		bool meshSub = primShape->type == b3_meshShape || primShape->type == b3_heightShape;
		b3ChildShape child = { 0 };
		bool isCompound = primShape->type == b3_compoundShape;
		if ( isCompound )
		{
			child = b3GetCompoundChild( primShape->compound, sub->childIndex );
			meshSub = child.type == b3_meshShape;
		}

		if ( meshSub )
		{
			int primBodyId = primShape->bodyId;
			b3WorldTransform xfPrimBody = ( primBodyId == edges0BodyId ) ? xfA : xfB;

			b3Shape meshShapeStorage;
			const b3Shape* meshShapeP = primShape;
			b3WorldTransform xfMesh = xfPrimBody;
			if ( isCompound )
			{
				memcpy( &meshShapeStorage, primShape, sizeof( b3Shape ) );
				meshShapeStorage.type = b3_meshShape;
				meshShapeStorage.mesh = child.mesh;
				meshShapeP = &meshShapeStorage;
				xfMesh = b3MulWorldTransforms( xfPrimBody, child.transform );
			}

			b3RefreshCache( &sub->meshContact, meshShapeP, xfMesh, &secShape->aabb );
			int triangleCount = sub->meshContact.triangleCache.count;
			maxLocalManifolds += triangleCount;
			maxLocalPoints += triangleCount * B3_MAX_POINTS_PER_TRIANGLE;
		}
		else
		{
			maxLocalManifolds += 1;
			maxLocalPoints += B3_MAX_POINTS_PER_TRIANGLE;
		}
	}

	b3LocalManifold* localManifolds = b3Bump( &arena, maxLocalManifolds * sizeof( b3LocalManifold ) );
	b3LocalManifoldPoint* localPoints = b3Bump( &arena, maxLocalPoints * sizeof( b3LocalManifoldPoint ) );
	b3SubMaterial* subInfo = b3Bump( &arena, subCount * sizeof( b3SubMaterial ) );
	int localCount = 0;
	int localPointCount = 0;

	uint32_t hitEventFlag = 0;

	// Gather local manifolds from every sub into the canonical frame.
	for ( int s = 0; s < subCount; ++s )
	{
		b3SubContact* sub = b3GetSubContact( contact, s );
		b3Shape* primShape = shapes + sub->shapeIdA;
		b3Shape* secShape = shapes + sub->shapeIdB;
		int subStartPoints = localPointCount;

		if ( ( primShape->flags & b3_enableHitEvents ) || ( secShape->flags & b3_enableHitEvents ) )
		{
			hitEventFlag = b3_simEnableHitEvent;
		}

		int primBodyId = primShape->bodyId;
		bool bodyFlip = primBodyId != edges0BodyId;
		b3WorldTransform xfPrimBody = bodyFlip ? xfB : xfA;
		b3WorldTransform xfSecBody = bodyFlip ? xfA : xfB;

		bool meshSub = primShape->type == b3_meshShape || primShape->type == b3_heightShape;
		b3ChildShape child = { 0 };
		bool isCompound = primShape->type == b3_compoundShape;
		if ( isCompound )
		{
			child = b3GetCompoundChild( primShape->compound, sub->childIndex );
			meshSub = child.type == b3_meshShape;
		}

		const b3SurfaceMaterial* materialB = b3GetShapeMaterials( secShape );

		if ( meshSub )
		{
			b3Shape meshShapeStorage;
			const b3Shape* meshShapeP = primShape;
			b3WorldTransform xfMesh = xfPrimBody;
			const int* materialMap = NULL;
			if ( isCompound )
			{
				memcpy( &meshShapeStorage, primShape, sizeof( b3Shape ) );
				meshShapeStorage.type = b3_meshShape;
				meshShapeStorage.mesh = child.mesh;
				meshShapeP = &meshShapeStorage;
				xfMesh = b3MulWorldTransforms( xfPrimBody, child.transform );
				materialMap = child.materialIndices;
			}

			int triangleCount = sub->meshContact.triangleCache.count;
			if ( triangleCount > 0 )
			{
				b3Arena scratch = arena;
				b3LocalManifold** accepted = b3Bump( &scratch, triangleCount * sizeof( b3LocalManifold* ) );
				b3LocalManifold** tentative = b3Bump( &scratch, triangleCount * sizeof( b3LocalManifold* ) );
				b3TentativeTriangle* tentTri = b3Bump( &scratch, triangleCount * sizeof( b3TentativeTriangle ) );
				int pcap = B3_MAX_POINTS_PER_TRIANGLE * triangleCount;
				b3LocalManifoldPoint* pbuf = b3Bump( &scratch, pcap * sizeof( b3LocalManifoldPoint ) );
				b3LocalManifold* mbuf = b3Bump( &scratch, triangleCount * sizeof( b3LocalManifold ) );

				int accCount = b3GatherMeshSubManifolds( world, workerIndex, &sub->meshContact, enableSpeculative, meshShapeP,
														 xfMesh, secShape, xfSecBody, isFast, accepted, tentative, tentTri, mbuf,
														 pbuf, pcap, triangleCount );

				b3Matrix3 matSec = b3MakeMatrixFromQuat( xfSecBody.q );
				b3Vec3 secOriginRelA = b3SubPos( xfSecBody.p, xfA.p );

				for ( int a = 0; a < accCount; ++a )
				{
					b3LocalManifold* am = accepted[a];
					b3Vec3 normalWorld = b3MulMV( matSec, am->normal );
					b3Vec3 triNormalWorld = b3MulMV( matSec, am->triangleNormal );
					int tag = ( am->triangleIndex & 0x003FFFFF ) | ( s << 22 );

					b3LocalManifold* lm = localManifolds + localCount;
					lm->normal = bodyFlip ? b3Neg( normalWorld ) : normalWorld;
					lm->triangleNormal = bodyFlip ? b3Neg( triNormalWorld ) : triNormalWorld;
					lm->points = localPoints + localPointCount;
					lm->pointCount = am->pointCount;
					lm->triangleIndex = tag;

					for ( int j = 0; j < am->pointCount; ++j )
					{
						b3LocalManifoldPoint* src = am->points + j;
						b3LocalManifoldPoint* dst = localPoints + localPointCount;
						dst->point = b3Add( secOriginRelA, b3MulMV( matSec, src->point ) );
						dst->separation = src->separation - restOffset;
						dst->pair = src->pair;
						dst->triangleIndex = tag;
						localPointCount += 1;
					}
					localCount += 1;
				}
			}

			const b3SurfaceMaterial* materialsA = b3GetShapeMaterials( meshShapeP );
			subInfo[s].isMesh = true;
			subInfo[s].isHeightField = meshShapeP->type == b3_heightShape;
			subInfo[s].materialsA = materialsA;
			subInfo[s].materialB = materialB;
			subInfo[s].materialMap = materialMap;
			subInfo[s].materialCount = meshShapeP->materialCount;
			if ( meshShapeP->type == b3_meshShape )
			{
				subInfo[s].triMaterialIndices = b3GetMeshMaterialIndices( meshShapeP->mesh.data );
			}
			else
			{
				subInfo[s].triMaterialIndices = b3GetHeightFieldMaterialIndices( meshShapeP->heightField );
			}

			b3Vec3 tvMesh = b3RotateVector( xfMesh.q, materialsA[0].tangentVelocity );
			b3Vec3 tvSec = b3RotateVector( xfSecBody.q, materialB->tangentVelocity );
			subInfo[s].tangentVelocity = bodyFlip ? b3Sub( tvSec, tvMesh ) : b3Sub( tvMesh, tvSec );

			// Mesh path convention: full hull inner radius for the secondary.
			float radiusB = 0.0f;
			if ( secShape->type == b3_sphereShape )
			{
				radiusB = secShape->sphere.radius;
			}
			else if ( secShape->type == b3_capsuleShape )
			{
				radiusB = secShape->capsule.radius;
			}
			else if ( secShape->type == b3_hullShape )
			{
				radiusB = secShape->hull->innerRadius;
			}
			subInfo[s].rollingResistance = materialB->rollingResistance * radiusB;
		}
		else
		{
			// Effective primary geometry (possibly a compound child)
			b3ShapeType primType;
			b3Sphere primSphere = { 0 };
			b3Capsule primCap = { 0 };
			const b3HullData* primHull = NULL;
			b3WorldTransform xfPrim;
			if ( isCompound )
			{
				primType = child.type;
				xfPrim = b3MulWorldTransforms( xfPrimBody, child.transform );
				if ( child.type == b3_capsuleShape )
				{
					primCap = child.capsule;
				}
				else if ( child.type == b3_hullShape )
				{
					primHull = child.hull;
				}
				else
				{
					primSphere = child.sphere;
				}
			}
			else
			{
				primType = primShape->type;
				xfPrim = xfPrimBody;
				if ( primType == b3_capsuleShape )
				{
					primCap = primShape->capsule;
				}
				else if ( primType == b3_hullShape )
				{
					primHull = primShape->hull;
				}
				else
				{
					primSphere = primShape->sphere;
				}
			}

			b3ShapeType secType = secShape->type;
			b3Sphere secSphere = secType == b3_sphereShape ? secShape->sphere : (b3Sphere){ 0 };
			b3Capsule secCap = secType == b3_capsuleShape ? secShape->capsule : (b3Capsule){ 0 };
			const b3HullData* secHull = secType == b3_hullShape ? secShape->hull : NULL;

			int rankPrim = primType == b3_hullShape ? 2 : ( primType == b3_capsuleShape ? 1 : 0 );
			int rankSec = secType == b3_hullShape ? 2 : ( secType == b3_capsuleShape ? 1 : 0 );
			bool collideFlip = rankSec > rankPrim;

			b3ShapeType aType, bType;
			const b3Sphere *aSphere, *bSphere;
			const b3Capsule *aCap, *bCap;
			const b3HullData *aHull, *bHull;
			b3WorldTransform xfCA, xfCB;
			int collideABodyId;
			if ( collideFlip == false )
			{
				aType = primType;
				bType = secType;
				aSphere = &primSphere;
				aCap = &primCap;
				aHull = primHull;
				bSphere = &secSphere;
				bCap = &secCap;
				bHull = secHull;
				xfCA = xfPrim;
				xfCB = xfSecBody;
				collideABodyId = primBodyId;
			}
			else
			{
				aType = secType;
				bType = primType;
				aSphere = &secSphere;
				aCap = &secCap;
				aHull = secHull;
				bSphere = &primSphere;
				bCap = &primCap;
				bHull = primHull;
				xfCA = xfSecBody;
				xfCB = xfPrim;
				collideABodyId = secShape->bodyId;
			}

			b3ContactCache* cache = &sub->convexContact.cache;
			b3Transform btoa = b3InvMulWorldTransforms( xfCA, xfCB );

			b3LocalManifoldPoint geomPoints[B3_MAX_POINTS_PER_TRIANGLE];
			b3LocalManifold geom = { 0 };
			geom.points = geomPoints;

			if ( aType == b3_hullShape )
			{
				if ( bType == b3_hullShape )
				{
					b3CollideHulls( &geom, B3_MAX_POINTS_PER_TRIANGLE, aHull, bHull, btoa, &cache->satCache );
					taskContext->satCallCount += 1;
					taskContext->satCacheHitCount += cache->satCache.hit;
				}
				else if ( bType == b3_capsuleShape )
				{
					b3CollideHullAndCapsule( &geom, B3_MAX_POINTS_PER_TRIANGLE, aHull, bCap, btoa, &cache->simplexCache );
				}
				else
				{
					b3CollideHullAndSphere( &geom, B3_MAX_POINTS_PER_TRIANGLE, aHull, bSphere, btoa, &cache->simplexCache );
				}
			}
			else if ( aType == b3_capsuleShape )
			{
				if ( bType == b3_capsuleShape )
				{
					b3CollideCapsules( &geom, B3_MAX_POINTS_PER_TRIANGLE, aCap, bCap, btoa );
				}
				else
				{
					b3CollideCapsuleAndSphere( &geom, B3_MAX_POINTS_PER_TRIANGLE, aCap, bSphere, btoa );
				}
			}
			else
			{
				b3CollideSpheres( &geom, B3_MAX_POINTS_PER_TRIANGLE, aSphere, bSphere, btoa );
			}

			if ( geom.pointCount > 0 )
			{
				b3Matrix3 matCA = b3MakeMatrixFromQuat( xfCA.q );
				b3Vec3 normalWorld = b3MulMV( matCA, geom.normal );
				bool normalFlip = collideABodyId != edges0BodyId;
				b3Vec3 caOriginRelA = b3SubPos( xfCA.p, xfA.p );
				int tag = -( s + 2 );

				b3LocalManifold* lm = localManifolds + localCount;
				lm->normal = normalFlip ? b3Neg( normalWorld ) : normalWorld;
				lm->triangleNormal = lm->normal;
				lm->points = localPoints + localPointCount;
				lm->pointCount = geom.pointCount;
				lm->triangleIndex = tag;

				for ( int j = 0; j < geom.pointCount; ++j )
				{
					b3LocalManifoldPoint* src = geom.points + j;
					b3LocalManifoldPoint* dst = localPoints + localPointCount;
					dst->point = b3Add( caOriginRelA, b3MulMV( matCA, src->point ) );
					dst->separation = src->separation;
					dst->pair = src->pair;
					dst->triangleIndex = tag;
					localPointCount += 1;
				}
				localCount += 1;
			}

			const b3SurfaceMaterial* materialA = b3GetShapeMaterials( primShape );
			subInfo[s].isMesh = false;
			subInfo[s].friction = world->frictionCallback( materialA[0].friction, materialA[0].userMaterialId,
														   materialB[0].friction, materialB[0].userMaterialId );
			subInfo[s].restitution = world->restitutionCallback( materialA[0].restitution, materialA[0].userMaterialId,
																 materialB[0].restitution, materialB[0].userMaterialId );

			b3Vec3 tvPrim = b3RotateVector( xfPrimBody.q, materialA[0].tangentVelocity );
			b3Vec3 tvSec = b3RotateVector( xfSecBody.q, materialB[0].tangentVelocity );
			subInfo[s].tangentVelocity = bodyFlip ? b3Sub( tvSec, tvPrim ) : b3Sub( tvPrim, tvSec );

			if ( materialA[0].rollingResistance > 0.0f || materialB[0].rollingResistance > 0.0f )
			{
				float radiusPrim = b3SubShapeRadius( primType, &primSphere, &primCap, primHull );
				float radiusSec = b3SubShapeRadius( secType, &secSphere, &secCap, secHull );
				float maxRadius = b3MaxFloat( radiusPrim, radiusSec );
				subInfo[s].rollingResistance = b3MaxFloat( materialA[0].rollingResistance, materialB[0].rollingResistance ) * maxRadius;
			}
			else
			{
				subInfo[s].rollingResistance = 0.0f;
			}
		}

		sub->touching = localPointCount > subStartPoints;
	}

	if ( localCount == 0 || localPointCount == 0 )
	{
		if ( contact->manifoldCount > 0 )
		{
			b3FreeManifolds( world, contact->manifolds, contact->manifoldCount );
			contact->manifolds = NULL;
			contact->manifoldCount = 0;
		}
		return false;
	}

	// Cluster all local manifolds together by normal.
	b3Cluster* clusters = b3Bump( &arena, localCount * sizeof( b3Cluster ) );
	int* clusterMemberships = b3Bump( &arena, localCount * sizeof( int ) );

	const float clusterThreshold = 0.996f;
	int clusterCount = 0;
	int clusterPointCount = 0;
	for ( int i = 0; i < localCount; ++i )
	{
		clusterMemberships[i] = B3_NULL_INDEX;
		const b3LocalManifold* manifold = localManifolds + i;
		clusterPointCount += manifold->pointCount;

		b3Vec3 manifoldNormal = manifold->normal;
		b3Vec3 triangleNormal = manifold->triangleNormal;
		int clusterIndex = B3_NULL_INDEX;
		for ( int j = 0; j < clusterCount; ++j )
		{
			float cosManifoldAngle = b3Dot( clusters[j].manifoldNormal, manifoldNormal );
			float cosTriangleAngle = b3Dot( clusters[j].triangleNormal, triangleNormal );
			if ( cosManifoldAngle <= clusterThreshold || cosTriangleAngle <= clusterThreshold )
			{
				continue;
			}
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

	b3LocalManifoldPoint* clusterPoints = b3Bump( &arena, clusterPointCount * sizeof( b3LocalManifoldPoint ) );
	int pointOffset = 0;
	for ( int i = 0; i < clusterCount; ++i )
	{
		clusters[i].points = clusterPoints + pointOffset;
		clusters[i].pointCount = 0;
		pointOffset += clusters[i].pointCapacity;
	}

	for ( int i = 0; i < localCount; ++i )
	{
		int clusterIndex = clusterMemberships[i];
		B3_ASSERT( 0 <= clusterIndex && clusterIndex < clusterCount );

		b3LocalManifold* am = localManifolds + i;
		b3Cluster* cm = clusters + clusterIndex;
		for ( int j = 0; j < am->pointCount; ++j )
		{
			B3_ASSERT( cm->pointCount < cm->pointCapacity );
			b3LocalManifoldPoint* ap = am->points + j;
			b3LocalManifoldPoint* cp = cm->points + cm->pointCount;
			cp->triangleIndex = ap->triangleIndex;
			cp->point = ap->point;
			cp->separation = ap->separation;
			cp->pair = ap->pair;
			cm->pointCount += 1;
		}
	}

	for ( int i = 0; i < clusterCount; ++i )
	{
		b3Cluster* cm = clusters + i;
		int reducedCount = b3ReduceCluster( cm->points, cm->pointCount, cm->triangleNormal, arena );
		cm->pointCount = reducedCount;
	}

	// Match to previous manifolds for warm starting
	int oldManifoldCount = contact->manifoldCount;
	b3Manifold* oldManifolds = NULL;
	if ( oldManifoldCount > 0 )
	{
		oldManifolds = b3Bump( &arena, oldManifoldCount * sizeof( b3Manifold ) );
		memcpy( oldManifolds, contact->manifolds, oldManifoldCount * sizeof( b3Manifold ) );
	}

	if ( oldManifoldCount != clusterCount )
	{
		b3FreeManifolds( world, contact->manifolds, contact->manifoldCount );
		contact->manifolds = b3AllocateManifolds( world, clusterCount );
		contact->manifoldCount = (uint16_t)clusterCount;
	}
	else
	{
		memset( contact->manifolds, 0, contact->manifoldCount * sizeof( b3Manifold ) );
	}

	bool* consumed = NULL;
	if ( oldManifoldCount > 0 )
	{
		consumed = b3Bump( &arena, oldManifoldCount * sizeof( bool ) );
		memset( consumed, 0, oldManifoldCount * sizeof( bool ) );
	}

	b3Vec3 offsetBA = b3SubPos( xfA.p, xfB.p );
	const float normalMatchTolerance = 0.995f;

	for ( int i = 0; i < clusterCount; ++i )
	{
		b3Cluster* cm = clusters + i;
		int pointCount = cm->pointCount;
		B3_ASSERT( 0 < pointCount && pointCount <= B3_MAX_MANIFOLD_POINTS );

		b3Manifold* manifold = contact->manifolds + i;
		manifold->pointCount = pointCount;
		manifold->normal = cm->manifoldNormal;

		float bestDot = normalMatchTolerance;
		int bestIndex = B3_NULL_INDEX;
		for ( int j = 0; j < oldManifoldCount; ++j )
		{
			if ( consumed[j] )
			{
				continue;
			}
			float dot = b3Dot( oldManifolds[j].normal, cm->manifoldNormal );
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

			target->anchorA = source->point;
			target->anchorB = b3Add( target->anchorA, offsetBA );
			target->separation = source->separation;
			target->featureId = b3MakeFeatureId( source->pair );
			target->triangleIndex = source->triangleIndex;

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
						oldPt->triangleIndex = B3_NULL_INDEX;
						break;
					}
				}
			}
		}
	}

	// Average friction and restitution across all emitted points.
	float frictionSum = 0.0f;
	float restitutionSum = 0.0f;
	b3Vec3 tangentVelocitySum = b3Vec3_zero;
	float sampleCount = 0.0f;
	float rollingResistance = 0.0f;
	for ( int s = 0; s < subCount; ++s )
	{
		rollingResistance = b3MaxFloat( rollingResistance, subInfo[s].rollingResistance );
	}

	for ( int i = 0; i < clusterCount; ++i )
	{
		b3Manifold* manifold = contact->manifolds + i;
		for ( int j = 0; j < manifold->pointCount; ++j )
		{
			int tag = manifold->points[j].triangleIndex;
			int subIndex;
			float friction;
			float restitution;
			if ( tag < 0 )
			{
				subIndex = -tag - 2;
				friction = subInfo[subIndex].friction;
				restitution = subInfo[subIndex].restitution;
			}
			else
			{
				subIndex = tag >> 22;
				int triangleIndex = tag & 0x003FFFFF;
				b3SubMaterial* si = subInfo + subIndex;
				int materialIndex = 0;
				if ( si->materialCount > 0 )
				{
					if ( si->isHeightField )
					{
						materialIndex = si->triMaterialIndices[triangleIndex >> 1];
					}
					else
					{
						materialIndex = si->triMaterialIndices[triangleIndex];
						if ( si->materialMap != NULL )
						{
							materialIndex = si->materialMap[materialIndex];
						}
					}
					materialIndex = b3ClampInt( materialIndex, 0, si->materialCount - 1 );
				}
				b3SurfaceMaterial material = si->materialsA[materialIndex];
				friction = world->frictionCallback( material.friction, material.userMaterialId, si->materialB->friction,
													si->materialB->userMaterialId );
				restitution = world->restitutionCallback( material.restitution, material.userMaterialId,
														  si->materialB->restitution, si->materialB->userMaterialId );
			}

			frictionSum += friction;
			restitutionSum += restitution;
			tangentVelocitySum = b3Add( tangentVelocitySum, subInfo[subIndex].tangentVelocity );
			sampleCount += 1.0f;
		}
	}

	if ( sampleCount > 0.0f )
	{
		float invCount = 1.0f / sampleCount;
		contact->friction = invCount * frictionSum;
		contact->restitution = invCount * restitutionSum;
		contact->tangentVelocity = b3MulSV( invCount, tangentVelocitySum );
	}
	contact->rollingResistance = rollingResistance;

	B3_ASSERT( b3IsValidFloat( contact->friction ) && contact->friction >= 0.0f );
	B3_ASSERT( b3IsValidFloat( contact->restitution ) && contact->restitution >= 0.0f );

	if ( hitEventFlag )
	{
		contact->flags |= b3_simEnableHitEvent;
	}
	else
	{
		contact->flags &= ~b3_simEnableHitEvent;
	}

	return true;
}

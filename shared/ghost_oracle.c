// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "ghost_oracle.h"

#include "ghost_collisions.h"

#include "box3d/box3d.h"
#include "box3d/constants.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

const char* g_oracleSceneNames[e_oracleSceneCount] = {
	"flat", "fan", "crest", "valley", "ridge", "saddle ridge", "rail", "wave",
};

// The wave scene is the largest, 169 vertices and 288 triangles
#define ORACLE_MAX_VERTICES 256
#define ORACLE_MAX_INDICES ( 3 * 512 )
#define ORACLE_MAX_MANIFOLDS 128

typedef struct OracleBuilder
{
	b3Vec3 vertices[ORACLE_MAX_VERTICES];
	int32_t indices[ORACLE_MAX_INDICES];
	int vertexCount;
	int indexCount;
} OracleBuilder;

static int AddVertex( OracleBuilder* builder, b3Vec3 vertex )
{
	B3_ASSERT( builder->vertexCount < ORACLE_MAX_VERTICES );
	if ( builder->vertexCount == ORACLE_MAX_VERTICES )
	{
		return 0;
	}

	builder->vertices[builder->vertexCount] = vertex;
	return builder->vertexCount++;
}

// Winds the triangle to face the given direction
static void AddTriangle( OracleBuilder* builder, int a, int b, int c, b3Vec3 facing )
{
	B3_ASSERT( builder->indexCount + 3 <= ORACLE_MAX_INDICES );
	if ( builder->indexCount + 3 > ORACLE_MAX_INDICES )
	{
		return;
	}

	b3Vec3 va = builder->vertices[a];
	b3Vec3 normal = b3Cross( b3Sub( builder->vertices[b], va ), b3Sub( builder->vertices[c], va ) );
	if ( b3Dot( normal, facing ) < 0.0f )
	{
		int temp = b;
		b = c;
		c = temp;
	}

	builder->indices[builder->indexCount++] = a;
	builder->indices[builder->indexCount++] = b;
	builder->indices[builder->indexCount++] = c;
}

typedef float OracleHeightFcn( float x, float z );

// Up facing grid with alternating diagonals
static void BuildGrid( OracleBuilder* builder, int countX, int countZ, float cellSize, OracleHeightFcn* height )
{
	float x0 = -0.5f * cellSize * countX;
	float z0 = -0.5f * cellSize * countZ;
	int base = builder->vertexCount;
	for ( int k = 0; k <= countZ; ++k )
	{
		for ( int i = 0; i <= countX; ++i )
		{
			float x = x0 + cellSize * i;
			float z = z0 + cellSize * k;
			AddVertex( builder, (b3Vec3){ x, height( x, z ), z } );
		}
	}

	int stride = countX + 1;
	for ( int k = 0; k < countZ; ++k )
	{
		for ( int i = 0; i < countX; ++i )
		{
			int a = base + k * stride + i;
			int b = a + 1;
			int c = a + stride + 1;
			int d = a + stride;
			if ( ( i + k ) & 1 )
			{
				AddTriangle( builder, a, d, b, b3Vec3_axisY );
				AddTriangle( builder, b, d, c, b3Vec3_axisY );
			}
			else
			{
				AddTriangle( builder, a, d, c, b3Vec3_axisY );
				AddTriangle( builder, a, c, b, b3Vec3_axisY );
			}
		}
	}
}

static float FlatHeight( float x, float z )
{
	(void)x;
	(void)z;
	return 0.0f;
}

static float CrestHeight( float x, float z )
{
	(void)z;
	return -0.26795f * fabsf( x );
}

static float ValleyHeight( float x, float z )
{
	(void)z;
	return 0.26795f * fabsf( x );
}

static float RidgeHeight( float x, float z )
{
	(void)z;
	return -fabsf( x );
}

// libm trig differs in the last bit between platforms, which would make each a different mesh
static float WaveHeight( float x, float z )
{
	float s1 = b3ComputeCosSin( 2.3f * x + 0.4f ).sine;
	float c1 = b3ComputeCosSin( 1.7f * z ).cosine;
	float s2 = b3ComputeCosSin( 4.1f * z - 1.3f * x ).sine;
	return 0.12f * s1 * c1 + 0.05f * s2;
}

static void BuildFan( OracleBuilder* builder )
{
	int center = AddVertex( builder, b3Vec3_zero );
	for ( int i = 0; i < 6; ++i )
	{
		b3CosSin cs = b3ComputeCosSin( ( i + 0.5f ) * B3_PI / 3.0f );
		AddVertex( builder, (b3Vec3){ 2.0f * cs.cosine, 0.0f, 2.0f * cs.sine } );
	}

	for ( int i = 0; i < 6; ++i )
	{
		AddTriangle( builder, center, 1 + i, 1 + ( i + 1 ) % 6, b3Vec3_axisY );
	}
}

// A ridge along x running into a rising wall, so the ridge end is a saddle vertex
static void BuildSaddleRidge( OracleBuilder* builder )
{
	float xs[3] = { -2.0f, 0.0f, 1.0f };
	float crests[3] = { -0.004f, 0.0f, 0.3f };
	float sides[3] = { -0.7f, -0.7f, -0.4f };
	for ( int i = 0; i < 3; ++i )
	{
		AddVertex( builder, (b3Vec3){ xs[i], sides[i], -1.0f } );
		AddVertex( builder, (b3Vec3){ xs[i], crests[i], 0.0f } );
		AddVertex( builder, (b3Vec3){ xs[i], sides[i], 1.0f } );
	}

	for ( int i = 0; i < 2; ++i )
	{
		for ( int k = 0; k < 2; ++k )
		{
			int a = 3 * i + k;
			AddTriangle( builder, a, a + 1, a + 3, b3Vec3_axisY );
			AddTriangle( builder, a + 3, a + 1, a + 4, b3Vec3_axisY );
		}
	}
}

// A wall facing -z meeting a narrow top at a convex edge, like a conveyor side rail
static void BuildRail( OracleBuilder* builder )
{
	float sectionZ[4] = { 0.0f, 0.0f, GHOST_RAIL_THICKNESS, GHOST_RAIL_THICKNESS };
	float sectionY[4] = { 0.0f, GHOST_RAIL_HEIGHT, GHOST_RAIL_HEIGHT, 0.0f };
	int columns = 5;
	for ( int row = 0; row < 4; ++row )
	{
		for ( int column = 0; column < columns; ++column )
		{
			AddVertex( builder, (b3Vec3){ -0.5f + 0.25f * column, sectionY[row], sectionZ[row] } );
		}
	}

	b3Vec3 facings[3] = { { 0.0f, 0.0f, -1.0f }, b3Vec3_axisY, b3Vec3_axisZ };
	for ( int row = 0; row < 3; ++row )
	{
		for ( int column = 0; column < columns - 1; ++column )
		{
			int a = columns * row + column;
			int b = a + 1;
			int c = a + columns + 1;
			int d = a + columns;
			if ( ( row + column ) & 1 )
			{
				AddTriangle( builder, a, d, b, facings[row] );
				AddTriangle( builder, d, c, b, facings[row] );
			}
			else
			{
				AddTriangle( builder, a, d, c, facings[row] );
				AddTriangle( builder, c, b, a, facings[row] );
			}
		}
	}
}

b3MeshData* CreateOracleMesh( OracleScene scene )
{
	static OracleBuilder builder;
	builder.vertexCount = 0;
	builder.indexCount = 0;

	switch ( scene )
	{
		case e_oracleFlat:
			BuildGrid( &builder, 6, 6, 0.5f, FlatHeight );
			break;
		case e_oracleFan:
			BuildFan( &builder );
			break;
		case e_oracleCrest:
			BuildGrid( &builder, 8, 4, 0.5f, CrestHeight );
			break;
		case e_oracleValley:
			BuildGrid( &builder, 8, 4, 0.5f, ValleyHeight );
			break;
		case e_oracleRidge:
			BuildGrid( &builder, 6, 4, 0.5f, RidgeHeight );
			break;
		case e_oracleSaddleRidge:
			BuildSaddleRidge( &builder );
			break;
		case e_oracleRail:
			BuildRail( &builder );
			break;
		case e_oracleWave:
			BuildGrid( &builder, 12, 12, 0.25f, WaveHeight );
			break;
		default:
			return NULL;
	}

	b3MeshDef def = { 0 };
	def.vertices = builder.vertices;
	def.stride = sizeof( b3Vec3 );
	def.vertexCount = builder.vertexCount;
	def.indices = builder.indices;
	def.triangleCount = builder.indexCount / 3;
	def.weldVertices = true;
	def.weldTolerance = 0.001f;
	def.identifyEdges = true;
	return b3CreateMesh( &def, NULL, 0 );
}

// Local state and 24 bits per draw, which the global 15 bit generator in utils.h does not give
typedef struct OracleRandom
{
	uint32_t state;
} OracleRandom;

static float RandomUnit( OracleRandom* random )
{
	uint32_t x = random->state;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	random->state = x;
	return (float)( x >> 8 ) / 16777216.0f;
}

static float RandomRange( OracleRandom* random, float lower, float upper )
{
	return lower + ( upper - lower ) * RandomUnit( random );
}

static b3Vec3 RandomDirection( OracleRandom* random )
{
	for ( int i = 0; i < 16; ++i )
	{
		b3Vec3 v = { RandomRange( random, -1.0f, 1.0f ), RandomRange( random, -1.0f, 1.0f ),
					 RandomRange( random, -1.0f, 1.0f ) };
		float lengthSquared = b3LengthSquared( v );
		if ( 0.01f < lengthSquared && lengthSquared <= 1.0f )
		{
			return b3MulSV( 1.0f / sqrtf( lengthSquared ), v );
		}
	}

	return b3Vec3_axisY;
}

// Direction within the given angle of the axis
static b3Vec3 RandomTilt( OracleRandom* random, b3Vec3 axis, float maxAngle )
{
	b3Vec3 perp = b3Cross( axis, RandomDirection( random ) );
	float length = b3Length( perp );
	if ( length < 1.0e-4f )
	{
		return axis;
	}

	perp = b3MulSV( 1.0f / length, perp );
	b3CosSin cs = b3ComputeCosSin( RandomRange( random, 0.0f, maxAngle ) );
	return b3Add( b3MulSV( cs.cosine, axis ), b3MulSV( cs.sine, perp ) );
}

static bool CollectPair( b3Vec3 a, b3Vec3 b, b3Vec3 c, int triangleIndex, void* context )
{
	OracleReference* reference = context;

	// A truncated union would misjudge the pose
	B3_ASSERT( reference->pairCount < ORACLE_MAX_TRIANGLES );
	if ( reference->pairCount == ORACLE_MAX_TRIANGLES )
	{
		return false;
	}

	OraclePair* pair = reference->pairs + reference->pairCount;
	*pair = (OraclePair){ 0 };
	pair->triangleIndex = triangleIndex;
	pair->vertices[0] = a;
	pair->vertices[1] = b;
	pair->vertices[2] = c;
	pair->faceNormal = b3Normalize( b3Cross( b3Sub( b, a ), b3Sub( c, a ) ) );
	pair->owned = true;
	pair->dominatedBy = -1;
	reference->pairCount += 1;
	return true;
}

// From Real-time Collision Detection, p141
static b3Vec3 ClosestPointOnTriangle( b3Vec3 p, b3Vec3 a, b3Vec3 b, b3Vec3 c )
{
	b3Vec3 ab = b3Sub( b, a );
	b3Vec3 ac = b3Sub( c, a );
	b3Vec3 ap = b3Sub( p, a );
	float d1 = b3Dot( ab, ap );
	float d2 = b3Dot( ac, ap );
	if ( d1 <= 0.0f && d2 <= 0.0f )
	{
		return a;
	}

	b3Vec3 bp = b3Sub( p, b );
	float d3 = b3Dot( ab, bp );
	float d4 = b3Dot( ac, bp );
	if ( d3 >= 0.0f && d4 <= d3 )
	{
		return b;
	}

	float vc = d1 * d4 - d3 * d2;
	if ( vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f )
	{
		return b3MulAdd( a, d1 / ( d1 - d3 ), ab );
	}

	b3Vec3 cp = b3Sub( p, c );
	float d5 = b3Dot( ab, cp );
	float d6 = b3Dot( ac, cp );
	if ( d6 >= 0.0f && d5 <= d6 )
	{
		return c;
	}

	float vb = d5 * d2 - d1 * d6;
	if ( vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f )
	{
		return b3MulAdd( a, d2 / ( d2 - d6 ), ac );
	}

	float va = d3 * d6 - d5 * d4;
	if ( va <= 0.0f && ( d4 - d3 ) >= 0.0f && ( d5 - d6 ) >= 0.0f )
	{
		float w = ( d4 - d3 ) / ( ( d4 - d3 ) + ( d5 - d6 ) );
		return b3MulAdd( b, w, b3Sub( c, b ) );
	}

	float denom = 1.0f / ( va + vb + vc );
	float v = vb * denom;
	float w = vc * denom;
	return b3Add( a, b3Add( b3MulSV( v, ab ), b3MulSV( w, ac ) ) );
}

// The closest pair of each nearby triangle and the union distance
static float ComputePairs( OracleReference* reference, const b3MeshData* mesh, const b3HullData* hull, b3Transform transform )
{
	reference->pairCount = 0;

	b3AABB bounds = b3ComputeHullAABB( hull, transform );
	// Collided points can be well past the speculative distance and the proximal test looks around them
	float margin = 0.25f;
	bounds.lowerBound = b3Sub( bounds.lowerBound, (b3Vec3){ margin, margin, margin } );
	bounds.upperBound = b3Add( bounds.upperBound, (b3Vec3){ margin, margin, margin } );

	b3Mesh meshShape = { .data = (b3MeshData*)mesh, .scale = b3Vec3_one };
	b3QueryMesh( &meshShape, bounds, CollectPair, reference );

	float minDistance = FLT_MAX;
	for ( int i = 0; i < reference->pairCount; ++i )
	{
		OraclePair* pair = reference->pairs + i;
		b3DistanceInput input = { 0 };
		input.proxyA = (b3ShapeProxy){ pair->vertices, 3, 0.0f };
		input.proxyB = (b3ShapeProxy){ b3GetHullPoints( hull ), hull->vertexCount, 0.0f };
		input.transform = transform;
		input.useRadii = false;
		b3SimplexCache cache = { 0 };
		b3DistanceOutput output = b3ShapeDistance( &input, &cache, NULL, 0 );
		pair->meshPoint = output.pointA;
		pair->hullPoint = output.pointB;
		pair->normal = output.normal;
		pair->distance = output.distance;

		if ( output.distance < minDistance )
		{
			minDistance = output.distance;
			reference->closestMeshPoint = output.pointA;
			reference->closestHullPoint = output.pointB;
		}
	}

	reference->distance = minDistance;
	return minDistance;
}

#define ORACLE_CONTAIN_TOLERANCE 1.0e-4f

static bool TriangleContains( const OraclePair* pair, b3Vec3 point )
{
	b3Vec3 closest = ClosestPointOnTriangle( point, pair->vertices[0], pair->vertices[1], pair->vertices[2] );
	return b3DistanceSquared( closest, point ) < ORACLE_CONTAIN_TOLERANCE * ORACLE_CONTAIN_TOLERANCE;
}

// A closest pair is a contact of the union unless a closer triangle also contains its mesh point
static void ComputeOwnership( OracleReference* reference )
{
	// Ties along shared features differ only by GJK round off
	float tieTolerance = 0.02f * B3_LINEAR_SLOP;

	for ( int i = 0; i < reference->pairCount; ++i )
	{
		OraclePair* pair = reference->pairs + i;
		pair->owned = true;
		pair->dominatedBy = -1;
		for ( int j = 0; j < reference->pairCount; ++j )
		{
			const OraclePair* other = reference->pairs + j;
			if ( j == i || other->distance >= pair->distance - tieTolerance )
			{
				continue;
			}

			if ( TriangleContains( other, pair->meshPoint ) )
			{
				pair->owned = false;
				pair->dominatedBy = other->triangleIndex;
				break;
			}
		}
	}
}

// On success the reference holds the closest pairs of the returned pose
static bool MakeOraclePose( OracleReference* reference, const b3MeshData* mesh, OracleScene scene, int index, b3BoxHull* box,
							b3Transform* transform )
{
	OracleRandom random = { 0x9E3779B9u ^ ( 2654435761u * (uint32_t)( index + 1 ) ) ^ ( 40503u * (uint32_t)scene ) };
	for ( int i = 0; i < 4; ++i )
	{
		RandomUnit( &random );
	}

	const b3Vec3* vertices = b3GetMeshVertices( mesh );
	const b3MeshTriangle* triangles = b3GetMeshTriangles( mesh );
	int triangleIndex = (int)( RandomUnit( &random ) * mesh->triangleCount );
	triangleIndex = triangleIndex < mesh->triangleCount ? triangleIndex : mesh->triangleCount - 1;
	b3Vec3 a = vertices[triangles[triangleIndex].index1];
	b3Vec3 b = vertices[triangles[triangleIndex].index2];
	b3Vec3 c = vertices[triangles[triangleIndex].index3];
	b3Vec3 faceNormal = b3Normalize( b3Cross( b3Sub( b, a ), b3Sub( c, a ) ) );

	float u = RandomUnit( &random );
	float v = RandomUnit( &random );
	if ( u + v > 1.0f )
	{
		u = 1.0f - u;
		v = 1.0f - v;
	}

	b3Vec3 target = b3Add( a, b3Add( b3MulSV( u, b3Sub( b, a ) ), b3MulSV( v, b3Sub( c, a ) ) ) );

	float hx = RandomRange( &random, 0.05f, 0.4f );
	float hy = RandomRange( &random, 0.05f, 0.4f );
	float hz = RandomRange( &random, 0.05f, 0.4f );
	*box = b3MakeBoxHull( hx, hy, hz );

	// Half the boxes rest nearly flat on the face, where parallel faces make ties
	b3Quat rotation;
	if ( RandomUnit( &random ) < 0.5f )
	{
		b3Vec3 up = RandomTilt( &random, faceNormal, 0.2f );
		b3Quat align = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisY, up );
		b3Quat yaw = b3MakeQuatFromAxisAngle( b3Vec3_axisY, RandomRange( &random, -B3_PI, B3_PI ) );
		rotation = b3MulQuat( align, yaw );
	}
	else
	{
		b3Vec3 axis = RandomDirection( &random );
		rotation = b3MakeQuatFromAxisAngle( axis, RandomRange( &random, -B3_PI, B3_PI ) );
	}

	b3Vec3 approach = RandomTilt( &random, faceNormal, 1.0f );

	// Put the hull support point against the target
	const b3Vec3* hullPoints = b3GetHullPoints( &box->base );
	b3Vec3 localDirection = b3InvRotateVector( rotation, b3Neg( approach ) );
	b3Vec3 support = hullPoints[0];
	for ( int i = 1; i < box->base.vertexCount; ++i )
	{
		if ( b3Dot( hullPoints[i], localDirection ) > b3Dot( support, localDirection ) )
		{
			support = hullPoints[i];
		}
	}

	float linearSlop = B3_LINEAR_SLOP;
	float gap = linearSlop * ( 0.2f + 3.4f * RandomUnit( &random ) * RandomUnit( &random ) );
	b3Transform pose = { b3Sub( target, b3RotateVector( rotation, support ) ), rotation };
	pose.p = b3MulAdd( pose.p, gap, approach );

	for ( int iteration = 0; iteration < 20; ++iteration )
	{
		*transform = pose;

		float distance = ComputePairs( reference, mesh, &box->base, pose );
		if ( reference->pairCount == 0 )
		{
			return false;
		}

		if ( distance <= 0.0f )
		{
			pose.p = b3MulAdd( pose.p, 2.0f * linearSlop, approach );
			continue;
		}

		if ( b3AbsFloat( distance - gap ) < 0.05f * linearSlop )
		{
			return true;
		}

		b3Vec3 direction = b3Normalize( b3Sub( reference->closestHullPoint, reference->closestMeshPoint ) );
		pose.p = b3MulAdd( pose.p, gap - distance, direction );
	}

	*transform = pose;
	float distance = ComputePairs( reference, mesh, &box->base, pose );
	return reference->pairCount > 0 && ORACLE_MIN_DISTANCE <= distance && distance <= B3_SPECULATIVE_DISTANCE;
}

// Smallest squared sine between the normal and the face normal of a triangle that contains the mesh
// point and faces the normal. FLT_MAX if there is no such triangle.
static float FacingSineSquared( const OracleReference* reference, b3Vec3 meshPoint, b3Vec3 normal )
{
	float best = FLT_MAX;
	for ( int i = 0; i < reference->pairCount; ++i )
	{
		const OraclePair* pair = reference->pairs + i;
		if ( b3Dot( pair->faceNormal, normal ) <= 0.0f || TriangleContains( pair, meshPoint ) == false )
		{
			continue;
		}

		best = b3MinFloat( best, b3LengthSquared( b3Cross( pair->faceNormal, normal ) ) );
	}

	return best;
}

// The mesh point is the closest union point to a point a little way out along the normal
static bool IsProximalNormal( const OracleReference* reference, b3Vec3 meshPoint, b3Vec3 normal, float sinTolerance )
{
	float probeDistance = 0.01f;
	b3Vec3 probe = b3MulAdd( meshPoint, probeDistance, normal );
	float bestDistanceSquared = FLT_MAX;
	b3Vec3 best = meshPoint;
	for ( int i = 0; i < reference->pairCount; ++i )
	{
		const OraclePair* pair = reference->pairs + i;
		b3Vec3 closest = ClosestPointOnTriangle( probe, pair->vertices[0], pair->vertices[1], pair->vertices[2] );
		float distanceSquared = b3DistanceSquared( closest, probe );
		if ( distanceSquared < bestDistanceSquared )
		{
			bestDistanceSquared = distanceSquared;
			best = closest;
		}
	}

	float offset = sinTolerance * probeDistance;
	return b3DistanceSquared( best, meshPoint ) < offset * offset;
}

// Collides the hull and judges every manifold against the reference
static void JudgeCollision( OracleResult* result, const OracleReference* reference, const b3MeshData* mesh,
							const b3HullData* hull, b3Transform transform, bool handleGhosts )
{
	result->pointCount = 0;
	result->ghostCount = 0;
	result->missed = false;

	b3Manifold manifolds[ORACLE_MAX_MANIFOLDS];
	b3Mesh meshShape = { .data = (b3MeshData*)mesh, .scale = b3Vec3_one };
	int manifoldCount =
		b3CollideMeshAndHull( manifolds, ORACLE_MAX_MANIFOLDS, &meshShape, hull, transform, true, handleGhosts );
	B3_ASSERT( manifoldCount < ORACLE_MAX_MANIFOLDS );

	// Clustering merges manifolds whose normals are within about 5 degrees
	float sinTolerance = 0.1f;
	float minSeparation = FLT_MAX;
	for ( int i = 0; i < manifoldCount; ++i )
	{
		const b3Manifold* manifold = manifolds + i;
		int firstPoint = result->pointCount;
		for ( int j = 0; j < manifold->pointCount; ++j )
		{
			B3_ASSERT( result->pointCount < ORACLE_MAX_POINTS );
			if ( result->pointCount == ORACLE_MAX_POINTS )
			{
				break;
			}

			const b3ManifoldPoint* manifoldPoint = manifold->points + j;
			OraclePoint* point = result->points + result->pointCount;
			point->point = manifoldPoint->anchorA;
			point->meshPoint = manifoldPoint->anchorA;
			point->normal = manifold->normal;
			point->separation = manifoldPoint->separation + B3_MESH_REST_OFFSET;
			point->triangleIndex = manifoldPoint->triangleIndex;
			point->verdict = e_oracleGhost;
			result->pointCount += 1;

			minSeparation = b3MinFloat( minSeparation, point->separation );

			const OraclePair* pair = NULL;
			for ( int k = 0; k < reference->pairCount; ++k )
			{
				if ( reference->pairs[k].triangleIndex == point->triangleIndex )
				{
					pair = reference->pairs + k;
					break;
				}
			}

			// The reference bounds contain the collider bounds, so every collided triangle has a pair
			B3_ASSERT( pair != NULL );
			if ( pair == NULL )
			{
				continue;
			}

			b3Vec3 meshPoint =
				ClosestPointOnTriangle( point->point, pair->vertices[0], pair->vertices[1], pair->vertices[2] );
			point->meshPoint = meshPoint;

			// A proximal normal also has to be in front of a face here, or an inverted face normal would pass
			float sineSquared = FacingSineSquared( reference, meshPoint, point->normal );
			if ( sineSquared < sinTolerance * sinTolerance )
			{
				point->verdict = e_oracleFace;
			}
			else if ( sineSquared < FLT_MAX && IsProximalNormal( reference, meshPoint, point->normal, sinTolerance ) )
			{
				point->verdict = e_oracleFeature;
			}
		}

		float deepest = FLT_MAX;
		for ( int j = firstPoint; j < result->pointCount; ++j )
		{
			deepest = b3MinFloat( deepest, result->points[j].separation );
		}

		bool valid = false;
		for ( int j = firstPoint; j < result->pointCount; ++j )
		{
			const OraclePoint* point = result->points + j;
			if ( point->separation <= deepest + B3_LINEAR_SLOP && point->verdict != e_oracleGhost )
			{
				valid = true;
				break;
			}
		}

		for ( int j = firstPoint; j < result->pointCount && valid; ++j )
		{
			OraclePoint* point = result->points + j;
			point->verdict = point->verdict == e_oracleGhost ? e_oracleShared : point->verdict;
		}

		if ( valid == false && result->pointCount > firstPoint )
		{
			result->ghostCount += 1;
		}
	}

	// The union closest pair has to be represented. SAT separation is a lower bound on distance.
	float linearSlop = B3_LINEAR_SLOP;
	float distance = reference->distance;
	if ( distance < B3_SPECULATIVE_DISTANCE - linearSlop && minSeparation > distance + linearSlop )
	{
		result->missed = true;
	}
}

bool EvaluateOraclePose( OraclePose* pose, const b3MeshData* mesh, OracleScene scene, int index )
{
	if ( MakeOraclePose( &pose->reference, mesh, scene, index, &pose->box, &pose->transform ) == false )
	{
		return false;
	}

	B3_ASSERT( pose->reference.distance >= ORACLE_MIN_DISTANCE );
	ComputeOwnership( &pose->reference );

	const b3HullData* hull = &pose->box.base;
	JudgeCollision( &pose->raw, &pose->reference, mesh, hull, pose->transform, false );
	JudgeCollision( &pose->handled, &pose->reference, mesh, hull, pose->transform, true );
	return true;
}

bool IsOracleCullingMiss( const OraclePose* pose )
{
	return pose->handled.missed && pose->raw.missed == false;
}

OracleScan ScanGhostOracle( const b3MeshData* mesh, OracleScene scene, int poseCount )
{
	static OraclePose pose;
	OracleScan scan = { 0 };
	scan.firstGhost = -1;
	scan.firstMissed = -1;

	for ( int i = 0; i < poseCount; ++i )
	{
		if ( EvaluateOraclePose( &pose, mesh, scene, i ) == false )
		{
			continue;
		}

		scan.validCount += 1;
		scan.rawGhostPoses += pose.raw.ghostCount > 0 ? 1 : 0;

		if ( pose.handled.ghostCount > 0 )
		{
			scan.ghostPoses += 1;
			scan.firstGhost = scan.firstGhost < 0 ? i : scan.firstGhost;
		}

		if ( IsOracleCullingMiss( &pose ) )
		{
			scan.missedPoses += 1;
			scan.firstMissed = scan.firstMissed < 0 ? i : scan.firstMissed;
		}
	}

	return scan;
}

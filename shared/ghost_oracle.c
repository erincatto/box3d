// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "ghost_oracle.h"

#include "box3d/box3d.h"
#include "box3d/constants.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

const char* g_oracleSceneNames[e_oracleSceneCount] = {
	"flat", "fan", "crest", "valley", "ridge", "saddle ridge", "rail", "wave",
};

#define ORACLE_MAX_VERTICES 256
#define ORACLE_MAX_INDICES ( 3 * 256 )

typedef struct OracleBuilder
{
	b3Vec3 vertices[ORACLE_MAX_VERTICES];
	int32_t indices[ORACLE_MAX_INDICES];
	int vertexCount;
	int indexCount;
} OracleBuilder;

static int AddVertex( OracleBuilder* builder, b3Vec3 vertex )
{
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

static float WaveHeight( float x, float z )
{
	return 0.12f * sinf( 2.3f * x + 0.4f ) * cosf( 1.7f * z ) + 0.05f * sinf( 4.1f * z - 1.3f * x );
}

static void BuildFan( OracleBuilder* builder )
{
	int center = AddVertex( builder, b3Vec3_zero );
	for ( int i = 0; i < 6; ++i )
	{
		float angle = ( i + 0.5f ) * B3_PI / 3.0f;
		AddVertex( builder, (b3Vec3){ 2.0f * cosf( angle ), 0.0f, 2.0f * sinf( angle ) } );
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
	float sectionZ[4] = { 0.0f, 0.0f, 0.08f, 0.08f };
	float sectionY[4] = { 0.0f, 0.35f, 0.35f, 0.0f };
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
	float angle = RandomRange( random, 0.0f, maxAngle );
	return b3Add( b3MulSV( cosf( angle ), axis ), b3MulSV( sinf( angle ), perp ) );
}

static bool CollectPair( b3Vec3 a, b3Vec3 b, b3Vec3 c, int triangleIndex, void* context )
{
	OracleResult* result = context;
	if ( result->pairCount == ORACLE_MAX_TRIANGLES )
	{
		return false;
	}

	OraclePair* pair = result->pairs + result->pairCount;
	*pair = (OraclePair){ 0 };
	pair->triangleIndex = triangleIndex;
	pair->vertices[0] = a;
	pair->vertices[1] = b;
	pair->vertices[2] = c;
	pair->dominatedBy = -1;
	result->pairCount += 1;
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

static float ComputePairs( OracleResult* result, const b3MeshData* mesh, const b3HullData* hull, b3Transform transform )
{
	result->pairCount = 0;

	b3AABB bounds = b3ComputeHullAABB( hull, transform );
	// Collided points can be well past the speculative distance and the proximal test looks around them
	float margin = 0.25f;
	bounds.lowerBound = b3Sub( bounds.lowerBound, (b3Vec3){ margin, margin, margin } );
	bounds.upperBound = b3Add( bounds.upperBound, (b3Vec3){ margin, margin, margin } );

	b3Mesh meshShape = { .data = (b3MeshData*)mesh, .scale = b3Vec3_one };
	b3QueryMesh( &meshShape, bounds, CollectPair, result );

	float minDistance = FLT_MAX;
	for ( int i = 0; i < result->pairCount; ++i )
	{
		OraclePair* pair = result->pairs + i;
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
			result->closestMeshPoint = output.pointA;
			result->closestHullPoint = output.pointB;
		}
	}

	result->distance = minDistance;
	return minDistance;
}

bool MakeOraclePose( const b3MeshData* mesh, OracleScene scene, int index, b3BoxHull* box, b3Transform* transform )
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

	static OracleResult scratch;
	for ( int iteration = 0; iteration < 20; ++iteration )
	{
		float distance = ComputePairs( &scratch, mesh, &box->base, pose );
		if ( scratch.pairCount == 0 )
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
			*transform = pose;
			return true;
		}

		b3Vec3 direction = b3Normalize( b3Sub( scratch.closestHullPoint, scratch.closestMeshPoint ) );
		pose.p = b3MulAdd( pose.p, gap - distance, direction );
	}

	float distance = ComputePairs( &scratch, mesh, &box->base, pose );
	*transform = pose;
	return ORACLE_MIN_DISTANCE <= distance && distance <= B3_SPECULATIVE_DISTANCE;
}

bool EvaluateGhostOracle( OracleResult* result, const b3MeshData* mesh, const b3HullData* hull, b3Transform transform,
						  bool handleGhosts )
{
	result->pointCount = 0;
	result->ghostCount = 0;
	result->missed = false;

	float distance = ComputePairs( result, mesh, hull, transform );
	if ( result->pairCount == 0 || distance < ORACLE_MIN_DISTANCE )
	{
		result->pairCount = 0;
		return false;
	}

	// Ties along shared features differ only by GJK round off
	float tieTolerance = 0.02f * B3_LINEAR_SLOP;
	float containTolerance = 1.0e-4f;

	for ( int i = 0; i < result->pairCount; ++i )
	{
		OraclePair* pair = result->pairs + i;
		pair->owned = true;
		for ( int j = 0; j < result->pairCount; ++j )
		{
			const OraclePair* other = result->pairs + j;
			if ( j == i || other->distance >= pair->distance - tieTolerance )
			{
				continue;
			}

			b3Vec3 closest =
				ClosestPointOnTriangle( pair->meshPoint, other->vertices[0], other->vertices[1], other->vertices[2] );
			if ( b3DistanceSquared( closest, pair->meshPoint ) < containTolerance * containTolerance )
			{
				pair->owned = false;
				pair->dominatedBy = other->triangleIndex;
				break;
			}
		}
	}

	b3Manifold manifolds[ORACLE_MAX_TRIANGLES];
	b3Mesh meshShape = { .data = (b3MeshData*)mesh, .scale = b3Vec3_one };
	int manifoldCount =
		b3CollideMeshAndHull( manifolds, ORACLE_MAX_TRIANGLES, &meshShape, hull, transform, true, handleGhosts );

	// Clustering merges manifolds whose normals are within about 5 degrees
	float sinTolerance = 0.1f;
	float probeDistance = 0.01f;
	float minSeparation = FLT_MAX;
	for ( int i = 0; i < manifoldCount; ++i )
	{
		const b3Manifold* manifold = manifolds + i;
		int firstPoint = result->pointCount;
		for ( int j = 0; j < manifold->pointCount && result->pointCount < ORACLE_MAX_POINTS; ++j )
		{
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
			for ( int k = 0; k < result->pairCount; ++k )
			{
				if ( result->pairs[k].triangleIndex == point->triangleIndex )
				{
					pair = result->pairs + k;
					break;
				}
			}

			if ( pair == NULL )
			{
				continue;
			}

			b3Vec3 meshPoint =
				ClosestPointOnTriangle( point->point, pair->vertices[0], pair->vertices[1], pair->vertices[2] );
			point->meshPoint = meshPoint;

			for ( int k = 0; k < result->pairCount; ++k )
			{
				const OraclePair* other = result->pairs + k;
				b3Vec3 closest =
					ClosestPointOnTriangle( meshPoint, other->vertices[0], other->vertices[1], other->vertices[2] );
				if ( b3DistanceSquared( closest, meshPoint ) > containTolerance * containTolerance )
				{
					continue;
				}

				b3Vec3 faceNormal = b3Normalize( b3Cross( b3Sub( other->vertices[1], other->vertices[0] ),
														  b3Sub( other->vertices[2], other->vertices[0] ) ) );
				b3Vec3 cross = b3Cross( faceNormal, point->normal );
				if ( b3Dot( faceNormal, point->normal ) > 0.0f &&
					 b3LengthSquared( cross ) < sinTolerance * sinTolerance )
				{
					point->verdict = e_oracleFace;
					break;
				}
			}

			if ( point->verdict == e_oracleGhost )
			{
				b3Vec3 probe = b3MulAdd( meshPoint, probeDistance, point->normal );
				float bestDistanceSquared = FLT_MAX;
				b3Vec3 best = meshPoint;
				for ( int k = 0; k < result->pairCount; ++k )
				{
					const OraclePair* other = result->pairs + k;
					b3Vec3 closest =
						ClosestPointOnTriangle( probe, other->vertices[0], other->vertices[1], other->vertices[2] );
					float distanceSquared = b3DistanceSquared( closest, probe );
					if ( distanceSquared < bestDistanceSquared )
					{
						bestDistanceSquared = distanceSquared;
						best = closest;
					}
				}

				// The probe must also be in front of the surface, or an inverted face normal would pass
				float offset = sinTolerance * probeDistance;
				if ( b3DistanceSquared( best, meshPoint ) < offset * offset )
				{
					for ( int k = 0; k < result->pairCount; ++k )
					{
						const OraclePair* other = result->pairs + k;
						b3Vec3 closest = ClosestPointOnTriangle( meshPoint, other->vertices[0], other->vertices[1],
																 other->vertices[2] );
						if ( b3DistanceSquared( closest, meshPoint ) > containTolerance * containTolerance )
						{
							continue;
						}

						b3Vec3 faceNormal = b3Cross( b3Sub( other->vertices[1], other->vertices[0] ),
													 b3Sub( other->vertices[2], other->vertices[0] ) );
						if ( b3Dot( faceNormal, point->normal ) > 0.0f )
						{
							point->verdict = e_oracleFeature;
							break;
						}
					}
				}
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
	if ( distance < B3_SPECULATIVE_DISTANCE - linearSlop && minSeparation > distance + linearSlop )
	{
		result->missed = true;
	}

	return true;
}

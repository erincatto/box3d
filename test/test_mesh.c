// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "ghost_collisions.h"
#include "ghost_oracle.h"
#include "shape.h"
#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Two quads meeting at a concave crease along the shared edge 1-4. The crease exercises
// edge identification, which reads the baked winding rather than the input winding.
#define VALLEY_VERTEX_COUNT 6
#define VALLEY_TRIANGLE_COUNT 4

static void MakeValley( b3Vec3* vertices, int32_t* indices )
{
	b3Vec3 v[VALLEY_VERTEX_COUNT] = {
		{ -1.0f, 1.0f, -1.0f }, { 0.0f, 0.0f, -1.0f }, { 1.0f, 1.0f, -1.0f },
		{ -1.0f, 1.0f, 1.0f },	{ 0.0f, 0.0f, 1.0f },  { 1.0f, 1.0f, 1.0f },
	};

	int32_t i[3 * VALLEY_TRIANGLE_COUNT] = {
		0, 3, 1, // left quad
		3, 4, 1,
		1, 4, 2, // right quad
		4, 5, 2,
	};

	memcpy( vertices, v, sizeof( v ) );
	memcpy( indices, i, sizeof( i ) );
}

// A vertex laid out the way a renderer would hand it over. The position sits off the
// front of the struct so a non-zero base offset gets exercised too.
typedef struct FatVertex
{
	float weight;
	b3Vec3 position;
	float uv[2];
} FatVertex;

static void MakeFatVertices( FatVertex* fat, const b3Vec3* vertices, int count )
{
	for ( int i = 0; i < count; ++i )
	{
		// Poison the padding so a stride mistake bakes obvious garbage instead of near misses
		fat[i].weight = 1000.0f + (float)i;
		fat[i].position = vertices[i];
		fat[i].uv[0] = -1000.0f;
		fat[i].uv[1] = -2000.0f;
	}
}

static void ReverseWinding( int32_t* indices, int triangleCount )
{
	for ( int i = 0; i < triangleCount; ++i )
	{
		int32_t temp = indices[3 * i + 1];
		indices[3 * i + 1] = indices[3 * i + 2];
		indices[3 * i + 2] = temp;
	}
}

// The hash covers every byte of the mesh block, so this compares the tree, the vertices,
// the baked triangles and the edge flags in one shot.
static bool MeshesMatch( const b3MeshData* mesh1, const b3MeshData* mesh2 )
{
	return mesh1->byteCount == mesh2->byteCount && mesh1->hash == mesh2->hash;
}

static bool VerticesMatch( const b3MeshData* mesh, const b3Vec3* expected, int count )
{
	if ( mesh->vertexCount != count )
	{
		return false;
	}

	const b3Vec3* vertices = b3GetMeshVertices( mesh );
	for ( int i = 0; i < count; ++i )
	{
		if ( b3DistanceSquared( vertices[i], expected[i] ) > FLT_EPSILON )
		{
			return false;
		}
	}

	return true;
}

// The valley faces up, so every baked triangle should wind counter clockwise about +Y
static bool FacesUp( const b3MeshData* mesh )
{
	const b3Vec3* vertices = b3GetMeshVertices( mesh );
	const b3MeshTriangle* triangles = b3GetMeshTriangles( mesh );

	for ( int i = 0; i < mesh->triangleCount; ++i )
	{
		b3Vec3 v1 = vertices[triangles[i].index1];
		b3Vec3 v2 = vertices[triangles[i].index2];
		b3Vec3 v3 = vertices[triangles[i].index3];

		b3Vec3 normal = b3Cross( b3Sub( v2, v1 ), b3Sub( v3, v1 ) );
		if ( normal.y <= 0.0f )
		{
			return false;
		}
	}

	return true;
}

static bool HasConcaveEdge( const b3MeshData* mesh )
{
	const uint8_t* flags = b3GetMeshFlags( mesh );

	for ( int i = 0; i < mesh->triangleCount; ++i )
	{
		if ( ( flags[i] & b3_allConcaveEdges ) != 0 )
		{
			return true;
		}
	}

	return false;
}

static b3MeshDef MakeValleyDef( b3Vec3* vertices, int32_t* indices )
{
	b3MeshDef def = { 0 };
	def.vertices = vertices;
	def.stride = sizeof( b3Vec3 );
	def.indices = indices;
	def.vertexCount = VALLEY_VERTEX_COUNT;
	def.triangleCount = VALLEY_TRIANGLE_COUNT;
	def.identifyEdges = true;
	return def;
}

// Dense input is the reference the other cases are measured against
static int MeshDenseStride( void )
{
	b3Vec3 vertices[VALLEY_VERTEX_COUNT];
	int32_t indices[3 * VALLEY_TRIANGLE_COUNT];
	MakeValley( vertices, indices );

	b3MeshDef def = MakeValleyDef( vertices, indices );
	b3MeshData* mesh = b3CreateMesh( &def, NULL, 0 );
	ENSURE( mesh != NULL );

	ENSURE( mesh->vertexCount == VALLEY_VERTEX_COUNT );
	ENSURE( mesh->triangleCount == VALLEY_TRIANGLE_COUNT );
	ENSURE( mesh->degenerateCount == 0 );
	ENSURE( VerticesMatch( mesh, vertices, VALLEY_VERTEX_COUNT ) );
	ENSURE( FacesUp( mesh ) );
	ENSURE( HasConcaveEdge( mesh ) );

	b3DestroyMesh( mesh );
	return 0;
}

// Interleaved input must bake to exactly the same mesh as dense input
static int MeshFatStride( void )
{
	b3Vec3 vertices[VALLEY_VERTEX_COUNT];
	int32_t indices[3 * VALLEY_TRIANGLE_COUNT];
	MakeValley( vertices, indices );

	b3MeshDef denseDef = MakeValleyDef( vertices, indices );
	b3MeshData* denseMesh = b3CreateMesh( &denseDef, NULL, 0 );
	ENSURE( denseMesh != NULL );

	FatVertex fat[VALLEY_VERTEX_COUNT];
	MakeFatVertices( fat, vertices, VALLEY_VERTEX_COUNT );

	b3MeshDef fatDef = MakeValleyDef( &fat[0].position, indices );
	fatDef.stride = sizeof( FatVertex );
	b3MeshData* fatMesh = b3CreateMesh( &fatDef, NULL, 0 );
	ENSURE( fatMesh != NULL );

	ENSURE( VerticesMatch( fatMesh, vertices, VALLEY_VERTEX_COUNT ) );
	ENSURE( FacesUp( fatMesh ) );
	ENSURE( MeshesMatch( denseMesh, fatMesh ) );

	b3DestroyMesh( fatMesh );
	b3DestroyMesh( denseMesh );
	return 0;
}

// Welding reads the source vertices through the stride as well
static int MeshStrideWeld( void )
{
	b3Vec3 vertices[VALLEY_VERTEX_COUNT];
	int32_t indices[3 * VALLEY_TRIANGLE_COUNT];
	MakeValley( vertices, indices );

	// Split the crease so each quad owns a copy of the shared edge
	b3Vec3 splitVertices[8];
	memcpy( splitVertices, vertices, sizeof( vertices ) );
	splitVertices[6] = vertices[1];
	splitVertices[7] = vertices[4];

	// Point the right quad at the copies
	int32_t splitIndices[3 * VALLEY_TRIANGLE_COUNT];
	memcpy( splitIndices, indices, sizeof( indices ) );
	splitIndices[6] = 6;
	splitIndices[7] = 7;
	splitIndices[9] = 7;

	FatVertex fat[8];
	MakeFatVertices( fat, splitVertices, 8 );

	b3MeshDef def = MakeValleyDef( &fat[0].position, splitIndices );
	def.stride = sizeof( FatVertex );
	def.vertexCount = 8;
	def.weldVertices = true;
	def.weldTolerance = 0.01f;

	b3MeshData* mesh = b3CreateMesh( &def, NULL, 0 );
	ENSURE( mesh != NULL );

	ENSURE( mesh->vertexCount == VALLEY_VERTEX_COUNT );
	ENSURE( mesh->triangleCount == VALLEY_TRIANGLE_COUNT );
	ENSURE( VerticesMatch( mesh, vertices, VALLEY_VERTEX_COUNT ) );
	ENSURE( FacesUp( mesh ) );
	ENSURE( HasConcaveEdge( mesh ) );

	b3DestroyMesh( mesh );
	return 0;
}

// Clockwise input plus the flag must bake to the same mesh as counter clockwise input
static int MeshClockWise( void )
{
	b3Vec3 vertices[VALLEY_VERTEX_COUNT];
	int32_t indices[3 * VALLEY_TRIANGLE_COUNT];
	MakeValley( vertices, indices );

	b3MeshDef ccwDef = MakeValleyDef( vertices, indices );
	b3MeshData* ccwMesh = b3CreateMesh( &ccwDef, NULL, 0 );
	ENSURE( ccwMesh != NULL );

	int32_t reversed[3 * VALLEY_TRIANGLE_COUNT];
	memcpy( reversed, indices, sizeof( indices ) );
	ReverseWinding( reversed, VALLEY_TRIANGLE_COUNT );

	b3MeshDef cwDef = MakeValleyDef( vertices, reversed );
	cwDef.clockWiseWinding = true;
	b3MeshData* cwMesh = b3CreateMesh( &cwDef, NULL, 0 );
	ENSURE( cwMesh != NULL );

	ENSURE( FacesUp( cwMesh ) );
	ENSURE( HasConcaveEdge( cwMesh ) );
	ENSURE( MeshesMatch( ccwMesh, cwMesh ) );

	b3DestroyMesh( cwMesh );
	b3DestroyMesh( ccwMesh );
	return 0;
}

// Without the flag the same clockwise input must bake inside out
static int MeshClockWiseIgnored( void )
{
	b3Vec3 vertices[VALLEY_VERTEX_COUNT];
	int32_t indices[3 * VALLEY_TRIANGLE_COUNT];
	MakeValley( vertices, indices );
	ReverseWinding( indices, VALLEY_TRIANGLE_COUNT );

	b3MeshDef def = MakeValleyDef( vertices, indices );
	b3MeshData* mesh = b3CreateMesh( &def, NULL, 0 );
	ENSURE( mesh != NULL );

	ENSURE( FacesUp( mesh ) == false );

	b3DestroyMesh( mesh );
	return 0;
}

// Winding, stride and welding have to compose
static int MeshClockWiseStrideWeld( void )
{
	b3Vec3 vertices[VALLEY_VERTEX_COUNT];
	int32_t indices[3 * VALLEY_TRIANGLE_COUNT];
	MakeValley( vertices, indices );

	b3MeshDef ccwDef = MakeValleyDef( vertices, indices );
	ccwDef.weldVertices = true;
	ccwDef.weldTolerance = 0.01f;
	b3MeshData* ccwMesh = b3CreateMesh( &ccwDef, NULL, 0 );
	ENSURE( ccwMesh != NULL );

	int32_t reversed[3 * VALLEY_TRIANGLE_COUNT];
	memcpy( reversed, indices, sizeof( indices ) );
	ReverseWinding( reversed, VALLEY_TRIANGLE_COUNT );

	FatVertex fat[VALLEY_VERTEX_COUNT];
	MakeFatVertices( fat, vertices, VALLEY_VERTEX_COUNT );

	b3MeshDef cwDef = MakeValleyDef( &fat[0].position, reversed );
	cwDef.stride = sizeof( FatVertex );
	cwDef.clockWiseWinding = true;
	cwDef.weldVertices = true;
	cwDef.weldTolerance = 0.01f;
	b3MeshData* cwMesh = b3CreateMesh( &cwDef, NULL, 0 );
	ENSURE( cwMesh != NULL );

	ENSURE( FacesUp( cwMesh ) );
	ENSURE( MeshesMatch( ccwMesh, cwMesh ) );

	b3DestroyMesh( cwMesh );
	b3DestroyMesh( ccwMesh );
	return 0;
}

// The built in creators fill their own def, so a missed stride shows up here
static int MeshCreators( void )
{
	b3Vec3 center = { 1.0f, 2.0f, 3.0f };
	b3Vec3 extent = { 0.5f, 1.0f, 1.5f };

	b3MeshData* meshes[] = {
		b3CreateGridMesh( 4, 4, 1.0f, 2, true ),
		b3CreateWaveMesh( 4, 4, 1.0f, 0.5f, 1.0f, 1.0f ),
		b3CreateTorusMesh( 8, 6, 2.0f, 0.5f ),
		b3CreateBoxMesh( center, extent, true ),
		b3CreateHollowBoxMesh( center, extent ),
		b3CreatePlatformMesh( center, 2.0f, 1.0f, 2.0f ),
	};

	for ( int i = 0; i < ARRAY_COUNT( meshes ); ++i )
	{
		ENSURE( meshes[i] != NULL );
		ENSURE( meshes[i]->vertexCount >= 3 );
		ENSURE( meshes[i]->triangleCount >= 1 );
		ENSURE( b3IsSaneAABB( meshes[i]->bounds ) );

		b3DestroyMesh( meshes[i] );
	}

	return 0;
}

// Brute force search for the triangle across edge a-b, walked b to a with consistent winding
static int FindNeighbor( const b3Mesh* mesh, int triangleIndex, int a, int b, int* edgeIndex )
{
	for ( int i = 0; i < mesh->data->triangleCount; ++i )
	{
		if ( i == triangleIndex )
		{
			continue;
		}

		b3Triangle triangle = b3GetMeshTriangle( mesh, i );
		int indices[3] = { triangle.i1, triangle.i2, triangle.i3 };
		for ( int j = 0; j < 3; ++j )
		{
			if ( indices[j] == b && indices[( j + 1 ) % 3] == a )
			{
				*edgeIndex = j;
				return i;
			}
		}
	}

	return B3_NULL_INDEX;
}

// Both sides of an edge store the same angle
static bool EdgeAnglesAreSymmetric( const b3MeshData* data, int* boundaryCount )
{
	b3Mesh mesh = { .data = data, .scale = b3Vec3_one };
	const int8_t* edgeAngles = b3GetMeshEdgeAngles( data );
	*boundaryCount = 0;

	for ( int i = 0; i < data->triangleCount; ++i )
	{
		b3Triangle triangle = b3GetMeshTriangle( &mesh, i );
		int indices[3] = { triangle.i1, triangle.i2, triangle.i3 };

		for ( int k = 0; k < 3; ++k )
		{
			int8_t angle = edgeAngles[3 * i + k];
			if ( angle == B3_BOUNDARY_EDGE_ANGLE )
			{
				*boundaryCount += 1;
				continue;
			}

			int edgeIndex = -1;
			int neighbor = FindNeighbor( &mesh, i, indices[k], indices[( k + 1 ) % 3], &edgeIndex );
			if ( neighbor == B3_NULL_INDEX || edgeAngles[3 * neighbor + edgeIndex] != angle )
			{
				return false;
			}
		}
	}

	return true;
}

// The rebuilt neighbor normal must match the real one under any scale. A non-uniform scale
// stretches the quantization error by up to the ratio of its largest and smallest components.
static bool NeighborNormalsMatch( const b3MeshData* data, b3Vec3 scale )
{
	b3Mesh mesh = { .data = data, .scale = scale };

	b3Vec3 s = b3Abs( scale );
	float spread = b3MaxFloat( s.x, b3MaxFloat( s.y, s.z ) ) / b3MinFloat( s.x, b3MinFloat( s.y, s.z ) );
	float tolerance = 0.8f * spread * B3_PI / 180.0f;
	float cosTolerance = cosf( tolerance );

	for ( int i = 0; i < data->triangleCount; ++i )
	{
		b3Triangle triangle = b3GetMeshTriangle( &mesh, i );
		int indices[3] = { triangle.i1, triangle.i2, triangle.i3 };

		for ( int k = 0; k < 3; ++k )
		{
			int edgeIndex = -1;
			int neighbor = FindNeighbor( &mesh, i, indices[k], indices[( k + 1 ) % 3], &edgeIndex );

			b3Vec3 rebuilt;
			bool found = b3GetMeshNeighborNormal( &mesh, i, k, &rebuilt );
			if ( found != ( neighbor != B3_NULL_INDEX ) )
			{
				return false;
			}

			if ( found == false )
			{
				continue;
			}

			b3Triangle other = b3GetMeshTriangle( &mesh, neighbor );
			b3Vec3 e1 = b3Sub( other.vertices[1], other.vertices[0] );
			b3Vec3 e2 = b3Sub( other.vertices[2], other.vertices[0] );
			b3Vec3 actual = b3Normalize( b3Cross( e1, e2 ) );
			if ( b3Dot( rebuilt, actual ) < cosTolerance )
			{
				return false;
			}
		}
	}

	return true;
}

// Positive angles are convex, negative are concave. A right angle sits on a rounding boundary.
static bool HasRightAngle( const b3MeshData* data, int sign )
{
	const int8_t* edgeAngles = b3GetMeshEdgeAngles( data );
	for ( int i = 0; i < 3 * data->triangleCount; ++i )
	{
		int angle = sign * edgeAngles[i];
		if ( angle == 63 || angle == 64 )
		{
			return true;
		}
	}

	return false;
}

static int MeshEdgeAngles( void )
{
	{
		b3MeshData* grid = b3CreateGridMesh( 4, 4, 1.0f, 0, true );
		ENSURE( b3GetMeshEdgeAngles( grid ) != NULL );

		int boundaryCount = 0;
		ENSURE( EdgeAnglesAreSymmetric( grid, &boundaryCount ) );
		ENSURE( boundaryCount == 16 );

		const int8_t* edgeAngles = b3GetMeshEdgeAngles( grid );
		for ( int i = 0; i < 3 * grid->triangleCount; ++i )
		{
			ENSURE( edgeAngles[i] == 0 || edgeAngles[i] == B3_BOUNDARY_EDGE_ANGLE );
		}

		b3DestroyMesh( grid );
	}

	{
		b3MeshData* grid = b3CreateGridMesh( 4, 4, 1.0f, 0, false );
		ENSURE( b3GetMeshEdgeAngles( grid ) == NULL );
		b3DestroyMesh( grid );
	}

	b3Vec3 vertices[VALLEY_VERTEX_COUNT];
	int32_t indices[3 * VALLEY_TRIANGLE_COUNT];
	MakeValley( vertices, indices );
	b3MeshDef def = MakeValleyDef( vertices, indices );

	b3MeshData* meshes[] = {
		b3CreateMesh( &def, NULL, 0 ),
		b3CreateBoxMesh( b3Vec3_zero, (b3Vec3){ 1.0f, 2.0f, 3.0f }, true ),
		b3CreateWaveMesh( 8, 8, 1.0f, 0.5f, 0.2f, 0.15f ),
		b3CreateTorusMesh( 8, 6, 2.0f, 0.5f ),
	};

	b3MeshData* valley = meshes[0];
	b3MeshData* box = meshes[1];
	ENSURE( HasRightAngle( valley, -1 ) );
	ENSURE( HasRightAngle( box, 1 ) );

	int boundaryCount = 0;
	ENSURE( EdgeAnglesAreSymmetric( box, &boundaryCount ) );
	ENSURE( boundaryCount == 0 );

	b3Vec3 scales[] = {
		{ 1.0f, 1.0f, 1.0f },	 { 3.0f, 3.0f, 3.0f },	{ -1.0f, 1.0f, 1.0f },
		{ 2.0f, 0.5f, 1.5f },	 { 0.5f, -2.0f, 1.0f }, { -1.0f, -1.0f, -1.0f },
	};

	for ( int i = 0; i < ARRAY_COUNT( meshes ); ++i )
	{
		ENSURE( meshes[i] != NULL );
		ENSURE( EdgeAnglesAreSymmetric( meshes[i], &boundaryCount ) );

		for ( int j = 0; j < ARRAY_COUNT( scales ); ++j )
		{
			ENSURE( NeighborNormalsMatch( meshes[i], scales[j] ) );
		}

		b3DestroyMesh( meshes[i] );
	}

	{
		// Three triangles on edge 0-1 are non-manifold. The last two share edge 5-6 walked the
		// same way, which is inconsistent winding. Neither gets an angle.
		b3Vec3 finVertices[] = {
			{ 0.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, -1.0f },
			{ 0.0f, 1.0f, 0.0f }, { 5.0f, 0.0f, 0.0f }, { 5.0f, 0.0f, 1.0f }, { 5.0f, 1.0f, 0.0f },
		};

		int32_t finIndices[] = {
			0, 1, 2, 1, 0, 3, 0, 1, 4, 5, 6, 7, 5, 6, 2,
		};

		b3MeshDef finDef = { 0 };
		finDef.vertices = finVertices;
		finDef.vertexCount = ARRAY_COUNT( finVertices );
		finDef.indices = finIndices;
		finDef.triangleCount = ARRAY_COUNT( finIndices ) / 3;
		finDef.identifyEdges = true;
		b3MeshData* fin = b3CreateMesh( &finDef, NULL, 0 );
		ENSURE( fin != NULL );

		ENSURE( EdgeAnglesAreSymmetric( fin, &boundaryCount ) );
		ENSURE( boundaryCount == 15 );
		b3DestroyMesh( fin );
	}

	return 0;
}

// A wide box sliding over a fine grid sits on hundreds of flat internal edges. With no friction
// any manifold off the face normal is a ghost that slows or lifts the box.
static int WideHullSlide( b3Vec3 meshScale )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3MeshData* mesh = b3CreateGridMesh( 120, 40, 0.25f, 0, true );
	ENSURE( mesh != NULL );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateMeshShape( groundId, &shapeDef, mesh, meshScale );

	float speed = 4.0f;
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ -10.0f, 0.5f, 0.0f };
	bodyDef.linearVelocity = (b3Vec3){ speed, 0.0f, 0.0f };
	b3BodyId boxId = b3CreateBody( worldId, &bodyDef );

	b3BoxHull box = b3MakeBoxHull( 1.25f, 0.5f, 1.25f );
	shapeDef.baseMaterial.friction = 0.0f;
	b3CreateHullShape( boxId, &shapeDef, &box.base );

	int ghostCount = 0;
	for ( int step = 0; step < 120; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );

		b3ContactData contacts[4];
		int contactCount = b3Body_GetContactData( boxId, contacts, ARRAY_COUNT( contacts ) );
		for ( int i = 0; i < contactCount; ++i )
		{
			for ( int j = 0; j < contacts[i].manifoldCount; ++j )
			{
				if ( b3AbsFloat( contacts[i].manifolds[j].normal.y ) < 0.99f )
				{
					ghostCount += 1;
				}
			}
		}
	}

	b3Vec3 velocity = b3Body_GetLinearVelocity( boxId );

	ENSURE( ghostCount == 0 );
	ENSURE( velocity.x > 0.99f * speed );
	ENSURE( b3AbsFloat( velocity.y ) < 0.01f );

	b3DestroyWorld( worldId );
	b3DestroyMesh( mesh );
	return 0;
}

// The box overlaps more than a thousand grid triangles. All of them must reach the narrow
// phase, so the reduced manifold reaches all four sides of the footprint.
static int MeshManyTriangles( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3MeshData* mesh = b3CreateGridMesh( 40, 40, 0.25f, 0, true );
	ENSURE( mesh != NULL );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateMeshShape( groundId, &shapeDef, mesh, b3Vec3_one );

	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 0.5f, 0.0f };
	b3BodyId boxId = b3CreateBody( worldId, &bodyDef );

	b3BoxHull box = b3MakeBoxHull( 3.0f, 0.5f, 3.0f );
	b3ShapeId boxShapeId = b3CreateHullShape( boxId, &shapeDef, &box.base );

	for ( int step = 0; step < 30; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3Pos position = b3Body_GetPosition( boxId );
	b3Quat rotation = b3Body_GetRotation( boxId );

	ENSURE( b3AbsFloat( (float)position.y - 0.5f ) < 0.02f );
	ENSURE( b3AbsFloat( rotation.v.x ) < 0.001f );
	ENSURE( b3AbsFloat( rotation.v.z ) < 0.001f );

	b3Vec3 lower = { FLT_MAX, FLT_MAX, FLT_MAX };
	b3Vec3 upper = { -FLT_MAX, -FLT_MAX, -FLT_MAX };

	b3ContactData contacts[4];
	int contactCount = b3Body_GetContactData( boxId, contacts, ARRAY_COUNT( contacts ) );
	for ( int i = 0; i < contactCount; ++i )
	{
		bool boxIsA = B3_ID_EQUALS( contacts[i].shapeIdA, boxShapeId );
		for ( int j = 0; j < contacts[i].manifoldCount; ++j )
		{
			const b3Manifold* manifold = contacts[i].manifolds + j;
			for ( int k = 0; k < manifold->pointCount; ++k )
			{
				b3Vec3 anchor = boxIsA ? manifold->points[k].anchorA : manifold->points[k].anchorB;
				lower = b3Min( lower, anchor );
				upper = b3Max( upper, anchor );
			}
		}
	}

	ENSURE( lower.x < -2.5f && upper.x > 2.5f );
	ENSURE( lower.z < -2.5f && upper.z > 2.5f );

	b3DestroyWorld( worldId );
	b3DestroyMesh( mesh );
	return 0;
}

// The grid is symmetric in x, so the mirror only changes the winding
static int MeshWideHullSlide( void )
{
	ENSURE( WideHullSlide( b3Vec3_one ) == 0 );
	ENSURE( WideHullSlide( (b3Vec3){ -1.0f, 1.0f, 1.0f } ) == 0 );
	return 0;
}

static bool HasVertex( const b3MeshData* mesh, int triangleIndex, b3Vec3 point )
{
	const b3MeshTriangle* triangle = b3GetMeshTriangles( mesh ) + triangleIndex;
	const b3Vec3* vertices = b3GetMeshVertices( mesh );
	int indices[3] = { triangle->index1, triangle->index2, triangle->index3 };
	for ( int i = 0; i < 3; ++i )
	{
		if ( b3DistanceSquared( vertices[indices[i]], point ) < 1.0e-8f )
		{
			return true;
		}
	}

	return false;
}

// A box face rests on a ridge that runs into a rising wall, so the ridge end is a saddle vertex. The
// ridge sags slightly away from the saddle, leaving the saddle the deepest point by less than linear
// slop. The ridge edge admits the up normal and the saddle cone does not, so the deepest points have
// to be taken as the edge they span, not as the single deepest vertex, or the contact snaps to a slope.
static int MeshHullOnSaddleRidge( void )
{
	b3Vec3 ridgeStart = { -2.0f, -0.004f, 0.0f };
	b3Vec3 saddle = { 0.0f, 0.0f, 0.0f };

	b3Vec3 vertices[] = {
		{ -2.0f, -0.7f, -1.0f },
		ridgeStart,
		{ -2.0f, -0.7f, 1.0f },
		{ 0.0f, -0.7f, -1.0f },
		saddle,
		{ 0.0f, -0.7f, 1.0f },
		{ 1.0f, -0.4f, -1.0f },
		{ 1.0f, 0.3f, 0.0f },
		{ 1.0f, -0.4f, 1.0f },
	};

	int32_t indices[] = {
		0, 1, 3, 3, 1, 4, 1, 2, 4, 4, 2, 5, 3, 4, 6, 6, 4, 7, 4, 5, 7, 7, 5, 8,
	};

	b3MeshDef def = { 0 };
	def.vertices = vertices;
	def.vertexCount = ARRAY_COUNT( vertices );
	def.indices = indices;
	def.triangleCount = ARRAY_COUNT( indices ) / 3;
	def.identifyEdges = true;
	b3MeshData* meshData = b3CreateMesh( &def, NULL, 0 );
	ENSURE( meshData != NULL );

	// The footprint covers the saddle and a stretch of the ridge, slightly overlapping
	float hy = 0.1f;
	b3BoxHull box = b3MakeBoxHull( 0.46f, hy, 0.5f );
	b3Transform transform = { { -0.44f, hy - 0.004f, 0.0f }, b3Quat_identity };

	b3Mesh mesh = { .data = meshData, .scale = b3Vec3_one };
	b3Manifold manifolds[16];
	int manifoldCount =
		b3CollideMeshAndHull( manifolds, ARRAY_COUNT( manifolds ), &mesh, &box.base, transform, true, true );

	int ridgePointCount = 0;
	for ( int i = 0; i < manifoldCount; ++i )
	{
		const b3Manifold* manifold = manifolds + i;
		for ( int j = 0; j < manifold->pointCount; ++j )
		{
			int triangleIndex = manifold->points[j].triangleIndex;
			if ( HasVertex( meshData, triangleIndex, ridgeStart ) && HasVertex( meshData, triangleIndex, saddle ) )
			{
				ridgePointCount += 1;
				ENSURE( manifold->normal.y > 0.999f );
			}
		}
	}

	ENSURE( ridgePointCount > 0 );

	b3DestroyMesh( meshData );
	return 0;
}

// A sphere balanced on a convex ridge is held up by edge contacts alone. Their normals lie between
// the two face normals, so neighbor validation must keep them.
static int MeshSphereOnRidge( void )
{
	b3Vec3 vertices[] = {
		{ -2.0f, -0.7f, -1.0f }, { 0.0f, 0.0f, -1.0f }, { 2.0f, -0.7f, -1.0f },
		{ -2.0f, -0.7f, 1.0f },	 { 0.0f, 0.0f, 1.0f },	{ 2.0f, -0.7f, 1.0f },
	};

	int32_t indices[] = {
		0, 3, 1, 3, 4, 1, 1, 4, 2, 4, 5, 2,
	};

	b3MeshDef def = { 0 };
	def.vertices = vertices;
	def.vertexCount = ARRAY_COUNT( vertices );
	def.indices = indices;
	def.triangleCount = ARRAY_COUNT( indices ) / 3;
	def.identifyEdges = true;
	b3MeshData* mesh = b3CreateMesh( &def, NULL, 0 );
	ENSURE( mesh != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateMeshShape( groundId, &shapeDef, mesh, b3Vec3_one );

	float radius = 0.25f;
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, radius, 0.0f };
	b3BodyId sphereId = b3CreateBody( worldId, &bodyDef );
	b3Sphere sphere = { b3Vec3_zero, radius };
	b3CreateSphereShape( sphereId, &shapeDef, &sphere );

	for ( int step = 0; step < 30; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );

		b3ContactData contacts[4];
		int contactCount = b3Body_GetContactData( sphereId, contacts, ARRAY_COUNT( contacts ) );
		ENSURE( contactCount == 1 );
		ENSURE( contacts[0].manifoldCount >= 1 );

		for ( int j = 0; j < contacts[0].manifoldCount; ++j )
		{
			ENSURE( b3AbsFloat( contacts[0].manifolds[j].normal.y ) > 0.99f );
		}
	}

	b3Pos position = b3Body_GetPosition( sphereId );
	ENSURE( position.y > 0.9f * radius );

	b3DestroyWorld( worldId );
	b3DestroyMesh( mesh );
	return 0;
}

// The normal points from the mesh to the body. Any normal in the span of the face normals under
// the body is real, anything else is a ghost.
static bool IsProfileGhost( const b3Vec2* profile, int count, b3Vec3 normal, float xMin, float xMax )
{
	float tolerance = 5.0f * B3_PI / 180.0f;
	if ( b3AbsFloat( normal.z ) > sinf( tolerance ) )
	{
		return true;
	}

	float lower = B3_PI;
	float upper = -B3_PI;
	for ( int k = 0; k + 1 < count; ++k )
	{
		if ( profile[k + 1].x < xMin || xMax < profile[k].x )
		{
			continue;
		}

		b3Vec2 a = profile[k];
		b3Vec2 b = profile[k + 1];
		float angle = atan2f( b.x - a.x, a.y - b.y );
		lower = b3MinFloat( lower, angle );
		upper = b3MaxFloat( upper, angle );
	}

	float angle = atan2f( normal.y, normal.x );
	return angle < lower - tolerance || upper + tolerance < angle;
}

// A capsule lying across the slide direction crosses a 5.7 degree crest at speed, like the crest
// scene in the ghost collision sample. The edges are flat, convex and concave, and a capsule edge
// contact on the wrong side of a fold kicks the capsule.
static int CapsuleOverCrest( bool mirror )
{
	b3Vec2 profile[] = { { -20.0f, 0.0f }, { -10.0f, 0.0f }, { 0.0f, 1.0f }, { 10.0f, 0.0f }, { 20.0f, 0.0f } };
	int profileCount = ARRAY_COUNT( profile );

	// Quarter meter cells with alternating diagonals, like cooked level geometry
	float cellSize = 0.25f;
	b3Vec2 points[256];
	int pointCount = 0;
	points[pointCount++] = profile[0];
	for ( int k = 0; k + 1 < profileCount; ++k )
	{
		b3Vec2 a = profile[k];
		b3Vec2 b = profile[k + 1];
		float length = sqrtf( ( b.x - a.x ) * ( b.x - a.x ) + ( b.y - a.y ) * ( b.y - a.y ) );
		int n = (int)ceilf( length / cellSize - 0.01f );
		for ( int s = 1; s <= n; ++s )
		{
			ENSURE( pointCount < ARRAY_COUNT( points ) );
			float t = (float)s / (float)n;
			points[pointCount++] = (b3Vec2){ a.x + t * ( b.x - a.x ), a.y + t * ( b.y - a.y ) };
		}
	}

	// Mirror the data and flip it back with the mesh scale, so the world geometry is the same
	float halfWidth = 3.0f;
	int zCount = 24;
	int stride = zCount + 1;
	float sign = mirror ? -1.0f : 1.0f;

	int vertexCount = pointCount * stride;
	b3Vec3* vertices = malloc( vertexCount * sizeof( b3Vec3 ) );
	for ( int i = 0; i < pointCount; ++i )
	{
		for ( int j = 0; j <= zCount; ++j )
		{
			float z = -halfWidth + 2.0f * halfWidth * (float)j / (float)zCount;
			vertices[i * stride + j] = (b3Vec3){ sign * points[i].x, points[i].y, z };
		}
	}

	int triangleCount = 2 * ( pointCount - 1 ) * zCount;
	int32_t* indices = malloc( 3 * triangleCount * sizeof( int32_t ) );
	int indexCount = 0;
	for ( int i = 0; i + 1 < pointCount; ++i )
	{
		for ( int j = 0; j < zCount; ++j )
		{
			int a = i * stride + j;
			int b = a + 1;
			int c = a + stride + 1;
			int d = a + stride;

			int corners[6] = { a, b, c, c, d, a };
			if ( ( i + j ) & 1 )
			{
				int alternate[6] = { a, b, d, b, c, d };
				memcpy( corners, alternate, sizeof( corners ) );
			}

			for ( int k = 0; k < 6; k += 3 )
			{
				indices[indexCount++] = corners[k];
				indices[indexCount++] = mirror ? corners[k + 2] : corners[k + 1];
				indices[indexCount++] = mirror ? corners[k + 1] : corners[k + 2];
			}
		}
	}

	b3MeshDef def = { 0 };
	def.vertices = vertices;
	def.vertexCount = vertexCount;
	def.indices = indices;
	def.triangleCount = triangleCount;
	def.identifyEdges = true;
	b3MeshData* mesh = b3CreateMesh( &def, NULL, 0 );
	free( vertices );
	free( indices );
	ENSURE( mesh != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Vec3 scale = mirror ? (b3Vec3){ -1.0f, 1.0f, 1.0f } : b3Vec3_one;
	b3CreateMeshShape( groundId, &shapeDef, mesh, scale );

	float radius = 0.2f;
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ -16.0f, radius + 0.01f, 0.0f };
	bodyDef.linearVelocity = (b3Vec3){ 6.0f, 0.0f, 0.0f };
	bodyDef.enableSleep = false;
	b3BodyId capsuleId = b3CreateBody( worldId, &bodyDef );

	b3Capsule capsule = { { 0.0f, 0.0f, -0.4f }, { 0.0f, 0.0f, 0.4f }, radius };
	shapeDef.baseMaterial.friction = 0.0f;
	b3ShapeId capsuleShapeId = b3CreateCapsuleShape( capsuleId, &shapeDef, &capsule );

	int ghostCount = 0;
	for ( int step = 0; step < 600 && b3Body_GetPosition( capsuleId ).x < 15.0f; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );

		b3AABB bounds = b3Shape_GetAABB( capsuleShapeId );
		float xMin = (float)bounds.lowerBound.x - 0.1f;
		float xMax = (float)bounds.upperBound.x + 0.1f;

		b3ContactData contacts[4];
		int contactCount = b3Body_GetContactData( capsuleId, contacts, ARRAY_COUNT( contacts ) );
		for ( int i = 0; i < contactCount; ++i )
		{
			bool capsuleIsA = B3_ID_EQUALS( contacts[i].shapeIdA, capsuleShapeId );
			for ( int j = 0; j < contacts[i].manifoldCount; ++j )
			{
				const b3Manifold* manifold = contacts[i].manifolds + j;

				// Speculative points that never pushed are harmless
				float impulse = 0.0f;
				for ( int k = 0; k < manifold->pointCount; ++k )
				{
					impulse += manifold->points[k].totalNormalImpulse;
				}

				b3Vec3 normal = capsuleIsA ? b3Neg( manifold->normal ) : manifold->normal;
				if ( impulse > 0.0f && IsProfileGhost( profile, profileCount, normal, xMin, xMax ) )
				{
					ghostCount += 1;
				}
			}
		}
	}

	ENSURE( ghostCount == 0 );
	ENSURE( b3Body_GetPosition( capsuleId ).x >= 15.0f );

	b3DestroyWorld( worldId );
	b3DestroyMesh( mesh );
	return 0;
}

static int MeshCapsuleOverCrest( void )
{
	ENSURE( CapsuleOverCrest( false ) == 0 );
	ENSURE( CapsuleOverCrest( true ) == 0 );
	return 0;
}

// Fires a sphere or the end of a capsule at the apex along the given direction, without gravity,
// and returns the deepest separation from the apex.
static float ApexImpact( b3MeshData* mesh, b3Vec3 direction, bool useCapsule )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateMeshShape( groundId, &shapeDef, mesh, b3Vec3_one );

	float radius = 0.25f;
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = b3ToPos( b3MulSV( radius + 0.1f, direction ) );
	bodyDef.linearVelocity = b3MulSV( -2.0f, direction );
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	// The capsule points away from the apex, so its lower end is the closest point
	b3Vec3 tip = b3MulSV( 0.5f, direction );
	if ( useCapsule )
	{
		b3Capsule capsule = { b3Vec3_zero, tip, radius };
		b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
	}
	else
	{
		b3Sphere sphere = { b3Vec3_zero, radius };
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );
	}

	float minSeparation = FLT_MAX;
	for ( int step = 0; step < 60; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );

		b3Pos p1 = b3Body_GetWorldPoint( bodyId, b3Vec3_zero );
		b3Pos p2 = b3Body_GetWorldPoint( bodyId, useCapsule ? tip : b3Vec3_zero );
		b3Vec3 a = b3ToVec3( p1 );
		b3Vec3 segment = b3Sub( b3ToVec3( p2 ), a );
		float lengthSquared = b3LengthSquared( segment );
		float t = lengthSquared > 0.0f ? b3ClampFloat( -b3Dot( a, segment ) / lengthSquared, 0.0f, 1.0f ) : 0.0f;
		float distance = b3Length( b3MulAdd( a, t, segment ) );
		minSeparation = b3MinFloat( minSeparation, distance - radius );
	}

	b3DestroyWorld( worldId );
	return minSeparation;
}

// A vertex manifold that fails a neighbor test is dropped. That relies on some triangle around a
// convex vertex admitting every normal in the vertex normal cone. On this irregular valence 6 apex
// each direction is inside the cone and exactly one of the six triangles admits it.
static int MeshApexContact( void )
{
	b3Vec3 vertices[] = {
		{ 0.0f, 0.0f, 0.0f },	   { 1.08f, -1.9f, -0.45f }, { -1.1f, -0.89f, -0.35f }, { -1.73f, -1.91f, -0.06f },
		{ -0.8f, -1.45f, 0.27f }, { 0.35f, -1.63f, 0.35f }, { 0.79f, -1.96f, 0.07f },
	};

	int32_t indices[] = {
		0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 5, 0, 5, 6, 0, 6, 1,
	};

	b3MeshDef def = { 0 };
	def.vertices = vertices;
	def.vertexCount = ARRAY_COUNT( vertices );
	def.indices = indices;
	def.triangleCount = ARRAY_COUNT( indices ) / 3;
	def.identifyEdges = true;
	b3MeshData* mesh = b3CreateMesh( &def, NULL, 0 );
	ENSURE( mesh != NULL );

	b3Vec3 directions[] = {
		{ -0.5703f, 0.7369f, 0.3629f },
		{ 0.8019f, 0.4956f, 0.3337f },
	};

	// Dropping every vertex manifold here lets the body sink a full step before the deep overlap
	// fallback catches it
	for ( int i = 0; i < ARRAY_COUNT( directions ); ++i )
	{
		b3Vec3 direction = b3Normalize( directions[i] );
		ENSURE( ApexImpact( mesh, direction, false ) > -B3_LINEAR_SLOP );
		ENSURE( ApexImpact( mesh, direction, true ) > -B3_LINEAR_SLOP );
	}

	b3DestroyMesh( mesh );
	return 0;
}

// Runs a Mesh / Ghost Collisions scene with the cylinder and returns the ghost count
static int CountCylinderGhosts( GhostScene scene, int peelPreset, bool mirror, int* runCount )
{
	GhostSettings settings = GetDefaultGhostSettings();
	settings.scene = scene;
	settings.peelPreset = peelPreset;
	settings.body = e_ghostCylinder;
	settings.mirror = mirror;

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	GhostCollisionsData data = CreateGhostCollisions( worldId, settings );

	for ( int step = 0; step < 900; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		UpdateGhostCollisions( worldId, &data, NULL, 0 );
	}

	*runCount = data.runCount;
	int ghostCount = data.ghostCount;

	b3DestroyWorld( worldId );
	DestroyGhostCollisions( &data );
	return ghostCount;
}

// The PEEL cylinder slides along its axis on a polygon edge that runs along an internal edge of the
// strip. Deep edge contacts on the far side fail the neighbor test and their face snap is empty, so
// the triangles under the cylinder own the contact.
static int MeshPeelCylinder( void )
{
	int runCount;
	ENSURE( CountCylinderGhosts( e_ghostInternal, 2, false, &runCount ) == 0 );
	ENSURE( runCount >= 1 );
	return 0;
}

// Exact vertex cone from every triangle at the vertex. Returns the largest lean of the normal past
// an edge at the vertex or past the cliff below a boundary edge.
static float ExactVertexLean( const b3Mesh* mesh, int vertex, b3Vec3 normal )
{
	float lean = -FLT_MAX;
	for ( int i = 0; i < mesh->data->triangleCount; ++i )
	{
		b3Triangle triangle = b3GetMeshTriangle( mesh, i );
		int indices[3] = { triangle.i1, triangle.i2, triangle.i3 };
		int k = indices[0] == vertex ? 0 : ( indices[1] == vertex ? 1 : ( indices[2] == vertex ? 2 : -1 ) );
		if ( k < 0 )
		{
			continue;
		}

		int next = ( k + 1 ) % 3;
		int previous = ( k + 2 ) % 3;
		b3Vec3 edges[2] = { b3Sub( triangle.vertices[next], triangle.vertices[k] ),
							b3Sub( triangle.vertices[previous], triangle.vertices[k] ) };
		int edgeIndices[2] = { k, previous };
		b3Vec3 faceNormal = b3MakeNormalFromPoints( triangle.vertices[0], triangle.vertices[1], triangle.vertices[2] );

		for ( int j = 0; j < 2; ++j )
		{
			lean = b3MaxFloat( lean, b3Dot( normal, b3Normalize( edges[j] ) ) );

			b3Vec3 neighborNormal;
			if ( b3GetMeshNeighborNormal( mesh, i, edgeIndices[j], &neighborNormal ) == false )
			{
				lean = b3MaxFloat( lean, -b3Dot( normal, faceNormal ) );
			}
		}
	}

	return lean;
}

static uint32_t NextRandom( uint32_t* state )
{
	*state = *state * 1664525u + 1013904223u;
	return *state;
}

static float RandomFloat( uint32_t* state )
{
	return (float)( NextRandom( state ) >> 8 ) / (float)( 1 << 24 ) * 2.0f - 1.0f;
}

// The quantized ring must admit normals well inside the exact cone and reject normals well outside.
// The band between is quantization noise. A non-uniform scale stretches it.
static bool VertexRingMatches( const b3MeshData* data, b3Vec3 scale, int* insideCount, int* outsideCount )
{
	b3Mesh mesh = { .data = data, .scale = scale };

	b3Vec3 s = b3Abs( scale );
	float spread = b3MaxFloat( s.x, b3MaxFloat( s.y, s.z ) ) / b3MinFloat( s.x, b3MinFloat( s.y, s.z ) );
	float tolerance = 0.0175f;
	float margin = 0.025f * spread;
	uint32_t state = 12345;

	for ( int i = 0; i < data->triangleCount; ++i )
	{
		b3Triangle triangle = b3GetMeshTriangle( &mesh, i );
		int indices[3] = { triangle.i1, triangle.i2, triangle.i3 };
		b3Vec3 faceNormal = b3MakeNormalFromPoints( triangle.vertices[0], triangle.vertices[1], triangle.vertices[2] );

		for ( int k = 0; k < 3; ++k )
		{
			b3Vec3 samples[24];
			int sampleCount = 0;
			samples[sampleCount++] = faceNormal;

			// Fan normals and their blends land near the cone
			for ( int j = 0; j < data->triangleCount && sampleCount < 12; ++j )
			{
				b3Triangle other = b3GetMeshTriangle( &mesh, j );
				if ( j == i || ( other.i1 != indices[k] && other.i2 != indices[k] && other.i3 != indices[k] ) )
				{
					continue;
				}

				b3Vec3 otherNormal = b3MakeNormalFromPoints( other.vertices[0], other.vertices[1], other.vertices[2] );
				samples[sampleCount++] = otherNormal;
				samples[sampleCount++] = b3Normalize( b3Add( faceNormal, otherNormal ) );
			}

			while ( sampleCount < 24 )
			{
				b3Vec3 tilt = { RandomFloat( &state ), RandomFloat( &state ), RandomFloat( &state ) };
				float amount = sampleCount % 2 == 0 ? 0.1f : 1.0f;
				samples[sampleCount++] = b3Normalize( b3MulAdd( faceNormal, amount, tilt ) );
			}

			for ( int j = 0; j < sampleCount; ++j )
			{
				b3Vec3 normal = samples[j];
				float lean = ExactVertexLean( &mesh, indices[k], normal );
				bool admit = b3IsInMeshVertexRing( &mesh, i, k, normal, tolerance );

				if ( lean <= tolerance - margin )
				{
					*insideCount += 1;
					if ( admit == false )
					{
						return false;
					}
				}
				else if ( lean > tolerance + margin )
				{
					*outsideCount += 1;
					if ( admit )
					{
						return false;
					}
				}
			}
		}
	}

	return true;
}

static int MeshVertexRing( void )
{
	b3Vec3 vertices[VALLEY_VERTEX_COUNT];
	int32_t indices[3 * VALLEY_TRIANGLE_COUNT];
	MakeValley( vertices, indices );
	b3MeshDef def = MakeValleyDef( vertices, indices );

	b3MeshData* meshes[] = {
		b3CreateMesh( &def, NULL, 0 ),
		b3CreateBoxMesh( b3Vec3_zero, (b3Vec3){ 1.0f, 2.0f, 3.0f }, true ),
		b3CreateWaveMesh( 8, 8, 1.0f, 0.5f, 0.2f, 0.15f ),
		b3CreateTorusMesh( 8, 6, 2.0f, 0.5f ),
		b3CreateGridMesh( 4, 4, 1.0f, 0, true ),
	};

	b3Vec3 scales[] = {
		{ 1.0f, 1.0f, 1.0f },    { 3.0f, 3.0f, 3.0f },  { -1.0f, 1.0f, 1.0f },
		{ 2.0f, 0.5f, 1.5f },    { 0.5f, -2.0f, 1.0f }, { -1.0f, -1.0f, -1.0f },
	};

	for ( int i = 0; i < ARRAY_COUNT( meshes ); ++i )
	{
		ENSURE( meshes[i] != NULL );

		for ( int j = 0; j < ARRAY_COUNT( scales ); ++j )
		{
			int insideCount = 0;
			int outsideCount = 0;
			ENSURE( VertexRingMatches( meshes[i], scales[j], &insideCount, &outsideCount ) );
			ENSURE( insideCount > 0 && outsideCount > 0 );
		}

		b3DestroyMesh( meshes[i] );
	}

	// Without edge data every normal passes
	b3MeshData* bare = b3CreateGridMesh( 4, 4, 1.0f, 0, false );
	b3Mesh mesh = { .data = bare, .scale = b3Vec3_one };
	ENSURE( b3IsInMeshVertexRing( &mesh, 0, 0, (b3Vec3){ 1.0f, 0.0f, 0.0f }, 0.0175f ) );
	b3DestroyMesh( bare );

	return 0;
}

/// Separated poses have an exact reference, the union of the triangles, see ghost_oracle.h. Without ghost
// handling the oracle has to find ghosts in every scene, which shows it has teeth.
//
// With ghost handling a pose fails as a ghost when a manifold carries a normal the union lacks at its
// deep points, and as a miss when the union closest pair goes unrepresented with ghost handling but not
// without it. Both are rare rather than absent, so each has a budget holding the current count. The
// counts belong to the first ORACLE_SCAN_POSES poses, so a change to a scene or to the pose stream moves
// them without the collider changing. Lower a budget when a case is fixed.
//
// The remaining ghost, wave pose 291, is a hull face normal admitted by a vertex cone with the deepest
// point half a linear slop from the vertex, where the oracle only accepts the normal at the vertex.
static int MeshGhostOracle( void )
{
	int ghostBudget[e_oracleSceneCount] = { 0, 0, 0, 0, 0, 0, 0, 1 };
	int missedBudget[e_oracleSceneCount] = { 0, 0, 0, 0, 0, 0, 0, 0 };

	for ( int scene = 0; scene < e_oracleSceneCount; ++scene )
	{
		b3MeshData* mesh = CreateOracleMesh( (OracleScene)scene );
		ENSURE( mesh != NULL );

		OracleScan scan = ScanGhostOracle( mesh, (OracleScene)scene, ORACLE_SCAN_POSES );
		b3DestroyMesh( mesh );

		if ( scan.validCount <= ORACLE_SCAN_POSES / 2 || scan.rawGhostPoses == 0 || scan.ghostPoses != ghostBudget[scene] ||
			 scan.missedPoses != missedBudget[scene] )
		{
			printf( "  oracle %s: valid %d, unhandled ghosts %d, ghosts %d of %d (first pose %d), missed %d of %d "
					"(first pose %d)\n",
					g_oracleSceneNames[scene], scan.validCount, scan.rawGhostPoses, scan.ghostPoses, ghostBudget[scene],
					scan.firstGhost, scan.missedPoses, missedBudget[scene], scan.firstMissed );
		}

		ENSURE( scan.validCount > ORACLE_SCAN_POSES / 2 );
		ENSURE( scan.rawGhostPoses > 0 );
		ENSURE( scan.ghostPoses <= ghostBudget[scene] );
		ENSURE( scan.missedPoses <= missedBudget[scene] );
	}

	return 0;
}

int MeshTest( void )
{
	RUN_SUBTEST( MeshDenseStride );
	RUN_SUBTEST( MeshFatStride );
	RUN_SUBTEST( MeshStrideWeld );
	RUN_SUBTEST( MeshClockWise );
	RUN_SUBTEST( MeshClockWiseIgnored );
	RUN_SUBTEST( MeshClockWiseStrideWeld );
	RUN_SUBTEST( MeshCreators );
	RUN_SUBTEST( MeshWideHullSlide );
	RUN_SUBTEST( MeshManyTriangles );
	RUN_SUBTEST( MeshEdgeAngles );
	RUN_SUBTEST( MeshVertexRing );
	RUN_SUBTEST( MeshSphereOnRidge );
	RUN_SUBTEST( MeshHullOnSaddleRidge );
	RUN_SUBTEST( MeshCapsuleOverCrest );
	RUN_SUBTEST( MeshApexContact );
	RUN_SUBTEST( MeshPeelCylinder );
	RUN_SUBTEST( MeshGhostOracle );

	return 0;
}

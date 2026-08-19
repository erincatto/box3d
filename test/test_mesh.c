// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <float.h>
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

int MeshTest( void )
{
	RUN_SUBTEST( MeshDenseStride );
	RUN_SUBTEST( MeshFatStride );
	RUN_SUBTEST( MeshStrideWeld );
	RUN_SUBTEST( MeshClockWise );
	RUN_SUBTEST( MeshClockWiseIgnored );
	RUN_SUBTEST( MeshClockWiseStrideWeld );
	RUN_SUBTEST( MeshCreators );

	return 0;
}

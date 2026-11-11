// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "mesh_benchmark.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum MeshShape
{
	meshShape_wave,
	meshShape_torus,
	meshShape_soup,
} MeshShape;

typedef struct MeshCase
{
	const char* name;
	MeshShape shape;
	int countA;
	int countB;
	bool useMedianSplit;
	bool identifyEdges;
	bool weldVertices;
} MeshCase;

typedef struct MeshSource
{
	b3Vec3* vertices;
	int32_t* indices;
	int vertexCount;
	int triangleCount;
} MeshSource;

static float WaveHeight( int i, int j )
{
	return 0.5f * sinf( 0.31f * (float)i ) * cosf( 0.23f * (float)j ) + 0.25f * sinf( 0.07f * (float)( i + 2 * j ) );
}

// Shared-vertex grid, or a triangle soup with three vertices per triangle when unshared is set
static MeshSource MakeWave( int xCount, int zCount, bool unshared )
{
	float cellWidth = 0.5f;
	MeshSource source = { 0 };
	source.triangleCount = 2 * xCount * zCount;
	source.indices = malloc( 3 * source.triangleCount * sizeof( int32_t ) );

	if ( unshared == false )
	{
		source.vertexCount = ( xCount + 1 ) * ( zCount + 1 );
		source.vertices = malloc( source.vertexCount * sizeof( b3Vec3 ) );
		for ( int i = 0; i <= xCount; ++i )
		{
			for ( int j = 0; j <= zCount; ++j )
			{
				source.vertices[i * ( zCount + 1 ) + j] = (b3Vec3){ cellWidth * i, WaveHeight( i, j ), cellWidth * j };
			}
		}

		int index = 0;
		for ( int i = 0; i < xCount; ++i )
		{
			for ( int j = 0; j < zCount; ++j )
			{
				int a = i * ( zCount + 1 ) + j;
				int b = a + 1;
				int c = a + ( zCount + 1 ) + 1;
				int d = a + ( zCount + 1 );
				source.indices[index++] = a;
				source.indices[index++] = b;
				source.indices[index++] = c;
				source.indices[index++] = a;
				source.indices[index++] = c;
				source.indices[index++] = d;
			}
		}
	}
	else
	{
		source.vertexCount = 3 * source.triangleCount;
		source.vertices = malloc( source.vertexCount * sizeof( b3Vec3 ) );
		int index = 0;
		for ( int i = 0; i < xCount; ++i )
		{
			for ( int j = 0; j < zCount; ++j )
			{
				b3Vec3 a = { cellWidth * i, WaveHeight( i, j ), cellWidth * j };
				b3Vec3 b = { cellWidth * i, WaveHeight( i, j + 1 ), cellWidth * ( j + 1 ) };
				b3Vec3 c = { cellWidth * ( i + 1 ), WaveHeight( i + 1, j + 1 ), cellWidth * ( j + 1 ) };
				b3Vec3 d = { cellWidth * ( i + 1 ), WaveHeight( i + 1, j ), cellWidth * j };
				b3Vec3 tris[6] = { a, b, c, a, c, d };
				for ( int k = 0; k < 6; ++k )
				{
					source.vertices[index] = tris[k];
					source.indices[index] = index;
					index += 1;
				}
			}
		}
	}

	return source;
}

static MeshSource MakeTorus( int radialCount, int tubularCount )
{
	float radius = 8.0f;
	float thickness = 2.0f;
	MeshSource source = { 0 };
	source.vertexCount = radialCount * tubularCount;
	source.triangleCount = 2 * radialCount * tubularCount;
	source.vertices = malloc( source.vertexCount * sizeof( b3Vec3 ) );
	source.indices = malloc( 3 * source.triangleCount * sizeof( int32_t ) );

	for ( int i = 0; i < radialCount; ++i )
	{
		float u = 2.0f * B3_PI * i / radialCount;
		for ( int j = 0; j < tubularCount; ++j )
		{
			float v = 2.0f * B3_PI * j / tubularCount;
			float r = radius + thickness * cosf( v );
			source.vertices[i * tubularCount + j] = (b3Vec3){ r * cosf( u ), thickness * sinf( v ), r * sinf( u ) };
		}
	}

	int index = 0;
	for ( int i = 0; i < radialCount; ++i )
	{
		int i2 = ( i + 1 ) % radialCount;
		for ( int j = 0; j < tubularCount; ++j )
		{
			int j2 = ( j + 1 ) % tubularCount;
			int a = i * tubularCount + j;
			int b = i2 * tubularCount + j;
			int c = i2 * tubularCount + j2;
			int d = i * tubularCount + j2;
			source.indices[index++] = a;
			source.indices[index++] = c;
			source.indices[index++] = b;
			source.indices[index++] = a;
			source.indices[index++] = d;
			source.indices[index++] = c;
		}
	}

	return source;
}

void RunMeshBenchmarks( int runCount, const char* caseName )
{
	MeshCase cases[] = {
		{ "wave_64", meshShape_wave, 64, 64, true, true, false },
		{ "wave_128", meshShape_wave, 128, 128, true, true, false },
		{ "wave_256", meshShape_wave, 256, 256, true, true, false },
		{ "wave_128_no_edges", meshShape_wave, 128, 128, true, false, false },
		{ "wave_128_sah", meshShape_wave, 128, 128, false, true, false },
		{ "torus_128x64", meshShape_torus, 128, 64, false, true, false },
		{ "soup_128_weld", meshShape_soup, 128, 128, true, true, true },
	};
	int caseCount = sizeof( cases ) / sizeof( cases[0] );

#ifdef NDEBUG
	int targetTriangles = 1 << 20;
#else
	int targetTriangles = 1 << 14;
#endif

	printf( "benchmark: mesh creation, runs = %d\n", runCount );
	printf( "%-20s %10s %8s %12s %12s\n", "case", "triangles", "builds", "ms / build", "ns / tri" );

	for ( int caseIndex = 0; caseIndex < caseCount; ++caseIndex )
	{
		const MeshCase* mc = cases + caseIndex;
		if ( caseName != NULL && strcmp( caseName, mc->name ) != 0 )
		{
			continue;
		}

		MeshSource source;
		if ( mc->shape == meshShape_torus )
		{
			source = MakeTorus( mc->countA, mc->countB );
		}
		else
		{
			source = MakeWave( mc->countA, mc->countB, mc->shape == meshShape_soup );
		}

		b3MeshDef def = { 0 };
		def.vertices = source.vertices;
		def.vertexCount = source.vertexCount;
		def.indices = source.indices;
		def.triangleCount = source.triangleCount;
		def.useMedianSplit = mc->useMedianSplit;
		def.identifyEdges = mc->identifyEdges;
		def.weldVertices = mc->weldVertices;
		def.weldTolerance = mc->weldVertices ? 0.001f : 0.0f;

		int buildCount = targetTriangles / source.triangleCount;
		if ( buildCount < 1 )
		{
			buildCount = 1;
		}

		float bestMs = FLT_MAX;
		for ( int runIndex = 0; runIndex < runCount; ++runIndex )
		{
			uint64_t ticks = b3GetTicks();
			for ( int buildIndex = 0; buildIndex < buildCount; ++buildIndex )
			{
				b3MeshData* mesh = b3CreateMesh( &def, NULL, 0 );
				b3DestroyMesh( mesh );
			}
			float ms = b3GetMilliseconds( ticks ) / buildCount;
			bestMs = b3MinFloat( bestMs, ms );
		}

		float nsPerTriangle = 1.0e6f * bestMs / source.triangleCount;
		printf( "%-20s %10d %8d %12.3f %12.1f\n", mc->name, source.triangleCount, buildCount, bestMs, nsPerTriangle );

		free( source.vertices );
		free( source.indices );
	}

	printf( "\n" );
}

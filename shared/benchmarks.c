// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "benchmarks.h"

#include "human.h"
#include "utils.h"

#include "box3d/box3d.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

#ifdef NDEBUG
#define BENCHMARK_DEBUG 0
#else
#define BENCHMARK_DEBUG 1
#endif

static b3ShapeId g_groundShapeId = { 0 };

b3ShapeId GetGroundShapeId( void )
{
	return g_groundShapeId;
}

void ResetGroundShapeId( void )
{
	g_groundShapeId = b3_nullShapeId;
}

void CreateJointGrid( b3WorldId worldId )
{
	b3World_EnableSleeping( worldId, false );

	int n = BENCHMARK_DEBUG ? 10 : 100;

	// Allocate to avoid huge stack usage
	b3BodyId* bodies = malloc( n * n * sizeof( b3BodyId ) );
	int index = 0;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.filter.categoryBits = 2;
	shapeDef.filter.maskBits = ~2u;

	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.4f };

	b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.enableSleep = false;

	for ( int k = 0; k < n; ++k )
	{
		for ( int i = 0; i < n; ++i )
		{
			float fk = (float)k;
			float fi = (float)i;

			if ( i == 0 )
			{
				bodyDef.type = b3_staticBody;
			}
			else
			{
				bodyDef.type = b3_dynamicBody;
			}

			bodyDef.position = (b3Pos){ fk, -fi, 0.0f };

			b3BodyId body = b3CreateBody( worldId, &bodyDef );

			b3CreateSphereShape( body, &shapeDef, &sphere );

			if ( i > 0 )
			{
				jointDef.base.bodyIdA = bodies[index - 1];
				jointDef.base.bodyIdB = body;
				jointDef.base.localFrameA.p = (b3Vec3){ 0.0f, -0.5f, 0.0f };
				jointDef.base.localFrameB.p = (b3Vec3){ 0.0f, 0.5f, 0.0f };
				b3CreateSphericalJoint( worldId, &jointDef );
			}

			if ( k > 0 )
			{
				jointDef.base.bodyIdA = bodies[index - n];
				jointDef.base.bodyIdB = body;
				jointDef.base.localFrameA.p = (b3Vec3){ 0.5f, 0.0f, 0.0f };
				jointDef.base.localFrameB.p = (b3Vec3){ -0.5f, 0.0f, 0.0f };
				b3CreateSphericalJoint( worldId, &jointDef );
			}

			bodies[index++] = body;
		}
	}

	free( bodies );
}

// The 80 block version falls over after 1000 steps.
void CreateLargePyramid( b3WorldId worldId )
{
	b3World_EnableSleeping( worldId, false );

	int baseCount = BENCHMARK_DEBUG ? 20 : 100;

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Pos){ 0.0f, -1.0f, 0.0f };
		b3BodyId groundId = b3CreateBody( worldId, &bodyDef );

		b3BoxHull box = b3MakeBoxHull( 400.0f, 1.0f, 400.0f );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		g_groundShapeId = b3CreateHullShape( groundId, &shapeDef, &box.base );
	}

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 100.0f;

	float h = 0.5f;
	b3BoxHull box = b3MakeBoxHull( h, h, h );

	float shift = 1.0f * h;

	for ( int i = 0; i < baseCount; ++i )
	{
		float y = ( 2.0f * i + 1.0f ) * shift;

		for ( int j = i; j < baseCount; ++j )
		{
			float x = ( i + 1.0f ) * shift + 2.0f * ( j - i ) * shift - h * baseCount;

			bodyDef.position = (b3Pos){ x, y, 0.0f };

			b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );
		}
	}
}

void CreateWidePyramid( b3WorldId worldId )
{
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Pos){ 0.0f, -1.0f, 0.0f };
		b3BodyId groundId = b3CreateBody( worldId, &bodyDef );

		b3BoxHull box = b3MakeBoxHull( 100.0f, 1.0f, 100.0f );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		g_groundShapeId = b3CreateHullShape( groundId, &shapeDef, &box.base );
	}

	const float boxSize = 2.0f;
	const float boxSeparation = 0.5f;
	const float halfBoxSize = 0.5f * boxSize;
	const int pyramidHeight = BENCHMARK_DEBUG ? 5 : 15;

	float h = halfBoxSize - 0.025;
	b3BoxHull box = b3MakeBoxHull( h, h, h );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	b3ShapeDef shapeDef = b3DefaultShapeDef();

	for ( int i = 0; i < pyramidHeight; ++i )
	{
		for ( int j = i / 2; j < pyramidHeight - ( i + 1 ) / 2; ++j )
		{
			for ( int k = i / 2; k < pyramidHeight - ( i + 1 ) / 2; ++k )
			{
				float x = -pyramidHeight + boxSize * j + ( i & 1 ? halfBoxSize : 0.0f );
				float y = 1.0f + ( boxSize + boxSeparation ) * i;
				float z = -pyramidHeight + boxSize * k + ( i & 1 ? halfBoxSize : 0.0f );
				bodyDef.position = (b3Pos){ x, y, z };

				b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
				b3CreateHullShape( bodyId, &shapeDef, &box.base );
			}
		}
	}
}

static void CreateSmallPyramid( b3WorldId worldId, int baseCount, float extent, float centerX, float baseZ )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.enableSleep = false;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 100.0f;

	b3BoxHull box = b3MakeBoxHull( extent, extent, extent );

	for ( int i = 0; i < baseCount; ++i )
	{
		float y = ( 2.0f * i + 1.0f ) * extent;

		for ( int j = i; j < baseCount; ++j )
		{
			float x = ( i + 1.0f ) * extent + 2.0f * ( j - i ) * extent + centerX - 0.5f;
			bodyDef.position = (b3Pos){ x, y, baseZ };

			b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );
		}
	}
}

void CreateManyPyramids( b3WorldId worldId )
{
	int baseCount = 10;
	float extent = 0.5f;
	int rowCount = BENCHMARK_DEBUG ? 3 : 14;
	int columnCount = BENCHMARK_DEBUG ? 3 : 14;
	float groundExtent = extent * columnCount * ( baseCount + 1.0f );

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Pos){ 0.0f, -1.0f, 0.0f };
		b3BodyId groundId = b3CreateBody( worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull box = b3MakeBoxHull( groundExtent, 1.0f, groundExtent );
		g_groundShapeId = b3CreateHullShape( groundId, &shapeDef, &box.base );
	}

	float baseWidth = 2.0f * extent * baseCount;
	float baseZ = -groundExtent + 2.0f * extent;
	float deltaZ = 2.0f * ( groundExtent - 2.0f * extent ) / ( rowCount - 1.0f );

	for ( int i = 0; i < rowCount; ++i )
	{
		for ( int j = 0; j < columnCount; ++j )
		{
			float centerX = -groundExtent + j * ( baseWidth + 2.0f * extent ) + 2.0f * extent;
			CreateSmallPyramid( worldId, baseCount, extent, centerX, baseZ );
		}

		baseZ += deltaZ;
	}
}

#define RAIN_GRID_SIZE 15.0f
#define RAIN_LARGE_WORLD 0
#ifdef NDEBUG
#if RAIN_LARGE_WORLD == 1
#define RAIN_GRID_COUNT 700
#define RAIN_GROUP_SIZE 1
#else
#define RAIN_GRID_COUNT 10
#define RAIN_GROUP_SIZE 3
#endif

#else

#if RAIN_LARGE_WORLD == 1
#define RAIN_GRID_COUNT 12
#define RAIN_GROUP_SIZE 1
#else
#define RAIN_GRID_COUNT 3
#define RAIN_GROUP_SIZE 2
#endif
#endif

typedef struct Group
{
	Human humans[RAIN_GROUP_SIZE];
} Group;

typedef struct RainData
{
	Group* groups;
	b3MeshData* gridMesh;
	b3MeshData* torusMesh;
	int columnCount;
	int columnIndex;
} RainData;

RainData g_rainData;

void GetRainCapacity( b3Capacity* capacity )
{
#if RAIN_LARGE_WORLD == 1
#ifdef NDEBUG
	capacity->staticShapeCount = 1000 * 1024;
	capacity->dynamicShapeCount = 2 * 1024;
	capacity->staticBodyCount = 500 * 1024;
	capacity->dynamicBodyCount = 2 * 1024;
#else
	capacity->staticShapeCount = 512;
	capacity->dynamicShapeCount = 128;
	capacity->staticBodyCount = 256;
	capacity->dynamicBodyCount = 128;
#endif
#endif
}

void CreateRain( b3WorldId worldId )
{
	memset( &g_rainData, 0, sizeof( g_rainData ) );

	g_rainData.groups = malloc( RAIN_GRID_COUNT * RAIN_GRID_COUNT * sizeof( Group ) );
	memset( g_rainData.groups, 0, RAIN_GRID_COUNT * RAIN_GRID_COUNT * sizeof( Group ) );

	int halfMeshGridRows = 4;
	float meshGridCellWidth = RAIN_GRID_SIZE / ( 2.0f * halfMeshGridRows );
	g_rainData.gridMesh = b3CreateGridMesh( 2 * halfMeshGridRows, 2 * halfMeshGridRows, meshGridCellWidth, 1, true );
	g_rainData.torusMesh = b3CreateTorusMesh( 16, 16, 0.25f * RAIN_GRID_SIZE, 1.0f );

	float span = RAIN_GRID_SIZE * RAIN_GRID_COUNT;
	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3ShapeDef shapeDef = b3DefaultShapeDef();

	bodyDef.position.x = -0.5f * span + 0.5f * RAIN_GRID_SIZE;
	for ( int i = 0; i < RAIN_GRID_COUNT; ++i )
	{
		bodyDef.position.z = -0.5f * span + 0.5f * RAIN_GRID_SIZE;
		for ( int j = 0; j < RAIN_GRID_COUNT; ++j )
		{
			b3BodyId body = b3CreateBody( worldId, &bodyDef );
			b3CreateMeshShape( body, &shapeDef, g_rainData.gridMesh, b3Vec3_one );
			b3CreateMeshShape( body, &shapeDef, g_rainData.torusMesh, b3Vec3_one );

			bodyDef.position.z += RAIN_GRID_SIZE;
		}

		bodyDef.position.x += RAIN_GRID_SIZE;
	}

	// b3World_SetJointTuning( worldId, 60.0f, 1.0f );
}

void DestroyRain( void )
{
	b3DestroyMesh( g_rainData.gridMesh );
	b3DestroyMesh( g_rainData.torusMesh );

	free( g_rainData.groups );
	g_rainData.groups = NULL;
}

void CreateGroup( b3WorldId worldId, int rowIndex, int columnIndex )
{
	assert( rowIndex < RAIN_GRID_COUNT && columnIndex < RAIN_GRID_COUNT );

	int groupIndex = rowIndex * RAIN_GRID_COUNT + columnIndex;

	float span = RAIN_GRID_COUNT * RAIN_GRID_SIZE;
	float groupDistance = 1.0f * span / RAIN_GRID_COUNT;

	b3Pos position;
	position.x = -0.5f * span + groupDistance * ( columnIndex + 0.5f );
	position.y = 20.0f;
	position.z = -0.5f * span + groupDistance * ( rowIndex + 0.5f );

	float frictionTorque = 5.0f;
	float hertz = 1.0f;
	float dampingRatio = 0.7f;
	bool colorize = false;

	for ( int i = 0; i < RAIN_GROUP_SIZE; ++i )
	{
		Human* human = g_rainData.groups[groupIndex].humans + i;
		CreateHuman( human, worldId, position, frictionTorque, hertz, dampingRatio, groupIndex, NULL, colorize );
		position.x += 0.75f;
	}
}

void DestroyGroup( int rowIndex, int columnIndex )
{
	assert( rowIndex < RAIN_GRID_COUNT && columnIndex < RAIN_GRID_COUNT );

	int groupIndex = rowIndex * RAIN_GRID_COUNT + columnIndex;

	for ( int i = 0; i < RAIN_GROUP_SIZE; ++i )
	{
		DestroyHuman( g_rainData.groups[groupIndex].humans + i );
	}
}

void StepRain( b3WorldId worldId, int stepCount )
{
	int delay = BENCHMARK_DEBUG ? 0x7F : 0x2F;
	int increment = RAIN_LARGE_WORLD ? 100 : 1;

	if ( ( stepCount & delay ) == 0 )
	{
		if ( g_rainData.columnCount < RAIN_GRID_COUNT )
		{
			for ( int i = 0; i < RAIN_GRID_COUNT; i += increment )
			{
				CreateGroup( worldId, i, g_rainData.columnCount );
			}

			g_rainData.columnCount = b3MinInt( g_rainData.columnCount + increment, RAIN_GRID_COUNT );
		}
		else
		{
			for ( int i = 0; i < RAIN_GRID_COUNT; i += increment )
			{
				DestroyGroup( i, g_rainData.columnIndex );
				CreateGroup( worldId, i, g_rainData.columnIndex );
			}

			g_rainData.columnIndex = g_rainData.columnIndex + increment;
			if ( g_rainData.columnIndex >= RAIN_GRID_COUNT )
			{
				g_rainData.columnIndex = 0;
			}
		}
	}
}

// Static Floor: a huge grid of static box bodies with b3ShapeDef::invokeContactCreation = true,
// plus a small number of dynamic spheres that drop onto it staggered over time. The point of this
// benchmark is to exercise the b3BroadPhase move buffer at steady state when it was once populated
// with ~staticShapeCount entries and now sees only a handful of dynamic moves per step.
#define STATIC_FLOOR_CELL_SIZE 10.0f
#if BENCHMARK_DEBUG
#define STATIC_FLOOR_GRID 32
#define STATIC_FLOOR_SPHERES 16
#define STATIC_FLOOR_DROP_INTERVAL 8
#else
#define STATIC_FLOOR_GRID 1000
#define STATIC_FLOOR_SPHERES 100
#define STATIC_FLOOR_DROP_INTERVAL 5
#endif

typedef struct StaticFloorData
{
	int spheresDropped;
} StaticFloorData;

static StaticFloorData g_staticFloorData;

void GetLargeWorldCapacity( b3Capacity* capacity )
{
	int floorCount = STATIC_FLOOR_GRID * STATIC_FLOOR_GRID;
	capacity->staticShapeCount = floorCount;
	capacity->staticBodyCount = floorCount;
	capacity->dynamicShapeCount = STATIC_FLOOR_SPHERES;
	capacity->dynamicBodyCount = STATIC_FLOOR_SPHERES;
	capacity->contactCount = b3MaxInt( 1024, 8 * STATIC_FLOOR_SPHERES );
}

void CreateLargeWorld( b3WorldId worldId )
{
	memset( &g_staticFloorData, 0, sizeof( g_staticFloorData ) );

	float cell = STATIC_FLOOR_CELL_SIZE;
	int gridCount = STATIC_FLOOR_GRID;
	float halfSpan = 0.5f * cell * gridCount;

	b3BoxHull box = b3MakeBoxHull( 0.5f * cell, 0.25f, 0.5f * cell );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3ShapeDef shapeDef = b3DefaultShapeDef();

	// The trigger: every static shape gets buffered into the move set on creation.
	shapeDef.invokeContactCreation = true;

	for ( int i = 0; i < gridCount; ++i )
	{
		float x = -halfSpan + ( i + 0.5f ) * cell;
		for ( int j = 0; j < gridCount; ++j )
		{
			float z = -halfSpan + ( j + 0.5f ) * cell;
			bodyDef.position = (b3Pos){ x, 0.0f, z };
			b3BodyId body = b3CreateBody( worldId, &bodyDef );
			b3CreateHullShape( body, &shapeDef, &box.base );
		}
	}
}

void StepLargeWorld( b3WorldId worldId, int stepCount )
{
	if ( g_staticFloorData.spheresDropped >= STATIC_FLOOR_SPHERES )
	{
		return;
	}

	if ( stepCount == 0 )
	{
		return;
	}

	if ( ( stepCount % STATIC_FLOOR_DROP_INTERVAL ) != 0 )
	{
		return;
	}

	// Spread spheres in a coarse grid across the floor so they don't all pile on one box.
	int side = 1;
	while ( side * side < STATIC_FLOOR_SPHERES )
	{
		side += 1;
	}

	int idx = g_staticFloorData.spheresDropped;
	int gi = idx % side;
	int gj = idx / side;

	float halfSpan = 0.5f * STATIC_FLOOR_CELL_SIZE * STATIC_FLOOR_GRID;
	// Confine drops to the inner 80% of the floor so spheres can't roll off the edge.
	float inset = 0.1f * 2.0f * halfSpan;
	float usable = 2.0f * halfSpan - 2.0f * inset;
	float x = -halfSpan + inset + ( gi + 0.5f ) * ( usable / side );
	float z = -halfSpan + inset + ( gj + 0.5f ) * ( usable / side );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ x, 1.5f, z };

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };

	b3BodyId body = b3CreateBody( worldId, &bodyDef );
	b3CreateSphereShape( body, &shapeDef, &sphere );

	g_staticFloorData.spheresDropped += 1;
}

void GetWasherCapacity( b3Capacity* capacity )
{
	capacity->staticShapeCount = 16;
	capacity->dynamicShapeCount = 10000;
	capacity->staticBodyCount = 16;
	capacity->dynamicBodyCount = 10000;
	capacity->contactCount = 60000;
}

void CreateWasher( b3WorldId worldId )
{
	bool kinematic = true;

	b3BodyId groundId;
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position.y = -1.0f;
		groundId = b3CreateBody( worldId, &bodyDef );

		b3BoxHull box = b3MakeBoxHull( 60.0f, 1.0f, 60.0f );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		g_groundShapeId = b3CreateHullShape( groundId, &shapeDef, &box.base );
	}

	{
		float motorSpeed = 25.0f;

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Pos){ 0.0f, 21.0f, 0.0f };

		if ( kinematic == true )
		{
			bodyDef.type = b3_kinematicBody;
			bodyDef.angularVelocity = (b3Vec3){ 0.0f, 0.0f, ( B3_PI / 180.0f ) * motorSpeed };
			bodyDef.linearVelocity = (b3Vec3){ 0.001f, -0.002f, 0.0f };
		}
		else
		{
			bodyDef.type = b3_dynamicBody;
		}

		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();

		float r0 = 14.0f;
		float r1 = 16.0f;
		float r2 = 18.0f;
		b3Vec3 nd = { 0.0f, 0.0f, -10.0f };
		b3Vec3 pd = { 0.0f, 0.0f, 10.0f };

		float angle = B3_PI / 18.0f;
		b3Quat q = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, angle );
		b3Quat qo = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.1f * angle );
		b3Vec3 u1 = { 1.0f, 0.0f, 0.0f };
		for ( int i = 0; i < 36; ++i )
		{
			b3Vec3 u2;
			if ( i == 35 )
			{
				u2 = (b3Vec3){ 1.0f, 0.0f, 0.0f };
			}
			else
			{
				u2 = b3RotateVector( q, u1 );
			}

			{
				b3Vec3 a1 = b3InvRotateVector( qo, u1 );
				b3Vec3 a2 = b3RotateVector( qo, u2 );
				b3Vec3 p1 = b3MulAdd( nd, r1, a1 );
				b3Vec3 p2 = b3MulAdd( nd, r2, a1 );
				b3Vec3 p3 = b3MulAdd( nd, r1, a2 );
				b3Vec3 p4 = b3MulAdd( nd, r2, a2 );
				b3Vec3 p5 = b3MulAdd( pd, r1, a1 );
				b3Vec3 p6 = b3MulAdd( pd, r2, a1 );
				b3Vec3 p7 = b3MulAdd( pd, r1, a2 );
				b3Vec3 p8 = b3MulAdd( pd, r2, a2 );

				b3Vec3 points[8] = { p1, p2, p3, p4, p5, p6, p7, p8 };
				b3HullData* hull = b3CreateHull( points, 8, 8 );
				b3CreateHullShape( bodyId, &shapeDef, hull );
				b3DestroyHull( hull );
			}

			if ( i % 9 == 0 )
			{
				b3Vec3 p1 = b3MulAdd( nd, r0, u1 );
				b3Vec3 p2 = b3MulAdd( nd, r1, u1 );
				b3Vec3 p3 = b3MulAdd( nd, r0, u2 );
				b3Vec3 p4 = b3MulAdd( nd, r1, u2 );
				b3Vec3 p5 = b3MulAdd( pd, r0, u1 );
				b3Vec3 p6 = b3MulAdd( pd, r1, u1 );
				b3Vec3 p7 = b3MulAdd( pd, r0, u2 );
				b3Vec3 p8 = b3MulAdd( pd, r1, u2 );

				b3Vec3 points[8] = { p1, p2, p3, p4, p5, p6, p7, p8 };
				b3HullData* hull = b3CreateHull( points, 8, 8 );
				b3CreateHullShape( bodyId, &shapeDef, hull );
				b3DestroyHull( hull );
			}

			u1 = u2;
		}

		if ( kinematic == false )
		{
			b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
			jointDef.base.bodyIdA = groundId;
			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p.y = 10.0f;
			jointDef.motorSpeed = ( B3_PI / 180.0f ) * motorSpeed;
			jointDef.maxMotorTorque = 1e8f;
			jointDef.enableMotor = true;

			b3CreateRevoluteJoint( worldId, &jointDef );
		}
	}

	int gridCount = BENCHMARK_DEBUG ? 8 : 20;
	float a = 0.2f;

	b3BoxHull cube = b3MakeBoxHull( a, a, a );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3ShapeDef shapeDef = b3DefaultShapeDef();

	float x = -2.0f * a * gridCount;
	for ( int i = 0; i < gridCount; ++i )
	{
		float y = -2.0f * a * gridCount + 21.0f;
		for ( int j = 0; j < gridCount; ++j )
		{
			float z = -2.0f * a * gridCount;
			for ( int k = 0; k < gridCount; ++k )
			{
				bodyDef.position = (b3Pos){ x, y, z };
				b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

				b3CreateHullShape( bodyId, &shapeDef, &cube.base );
				z += 4.0f * a;
			}

			y += 4.0f * a;
		}

		x += 4.0f * a;
	}
}

struct
{
	b3MeshData* meshData;
} g_treeData;

static void CreateTrees( b3WorldId worldId, int scale )
{
	memset( &g_treeData, 0, sizeof( g_treeData ) );

	// float tilt = 0.15f * B3_PI;
	float tilt = 0.0f * B3_PI;
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.position = (b3Pos){ 0.0f, 0.0f, 0.0f };
	bodyDef.rotation = b3MakeQuatFromAxisAngle( (b3Vec3){ 1.0f, 0.0f, 0.0 }, tilt );
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );

	int xCount = scale * 150;
	// int zCount = BENCHMARK_DEBUG ? 100 : 400;
	int zCount = scale * 200;

	float cellWidth = 1.0f / scale;
	float amplitude = 0.4f;
	float rowHz = 0.05f;
	float columnHz = 0.1f;

	g_treeData.meshData = b3CreateWaveMesh( xCount, zCount, cellWidth, amplitude, rowHz, columnHz );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateMeshShape( groundId, &shapeDef, g_treeData.meshData, b3Vec3_one );

	bodyDef.type = b3_dynamicBody;
	bodyDef.sleepThreshold = 0.2f;
	bodyDef.rotation = b3Quat_identity;

	int bodyCount = BENCHMARK_DEBUG ? 10 : 50;

	shapeDef.baseMaterial.friction = 0.9f;
	shapeDef.baseMaterial.rollingResistance = 0.05f;
	shapeDef.updateBodyMass = false;
	shapeDef.density = 1.0f;

	int hullCount = 22;
	b3HullData* hulls[22] = { 0 };

	float y = 1.0f;
	float r = 0.75f;
	float l = 1.5f;
	for ( int i = 0; i < hullCount; ++i )
	{
		hulls[i] = b3CreateCylinder( l + 2.0f * r, r, y - r, 6 );
		y += l + 2.0f * r;
		r = 0.95f * r;
	}

	float angularVelocity = -0.5f;
	float z = BENCHMARK_DEBUG ? -15.0f : -70.0f;
	b3CosSin cs = b3ComputeCosSin( tilt );
	float yTilt = cs.sine / cs.cosine;
	for ( int bodyIndex = 0; bodyIndex < bodyCount; ++bodyIndex )
	{
		bodyDef.position = (b3Pos){ 0.0, 1.0f - z * yTilt, z };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

		for ( int shapeIndex = 0; shapeIndex < 22; ++shapeIndex )
		{
			// Random rotation provided no benefit to behavior.
			// float angle = B3_PI * RandomFloat();
			// b3Transform xf;
			// xf.p = b3Vec3_zero;
			// xf.q = b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 1.0f, 0.0f }, angle );
			// b3Vec3 scale = {1.0f, 1.0f, 1.0f};
			// b3CreateTransformedHullShape( bodyId, &shapeDef, hulls[shapeIndex], xf, scale );
			b3CreateHullShape( bodyId, &shapeDef, hulls[shapeIndex] );
		}

		float velocityScale = 0.5f + ( 0.5f * bodyIndex ) / bodyCount;
		b3Body_ApplyMassFromShapes( bodyId );
		b3Pos center = b3Body_GetWorldCenter( bodyId );
		b3Vec3 omega = { 0.0f, 0.0f, velocityScale * angularVelocity };
		b3Vec3 v = b3Cross( omega, b3SubPos( center, bodyDef.position ) );
		b3Body_SetAngularVelocity( bodyId, omega );
		b3Body_SetLinearVelocity( bodyId, v );

		z += 3.0f;
		angularVelocity = -angularVelocity;
	}

	for ( int i = 0; i < hullCount; ++i )
	{
		b3DestroyHull( hulls[i] );
	}
}

void CreateTrees25( b3WorldId worldId )
{
	CreateTrees( worldId, 4 );
}

void CreateTrees50( b3WorldId worldId )
{
	CreateTrees( worldId, 2 );
}

void CreateTrees100( b3WorldId worldId )
{
	CreateTrees( worldId, 1 );
}

void DestroyTrees( void )
{
	b3DestroyMesh( g_treeData.meshData );
	memset( &g_treeData, 0, sizeof( g_treeData ) );
}

struct JunkyardData
{
	b3BodyId pusherId;
	float degrees;
	float radius;
} g_junkyardData;

void CreateJunkyard( b3WorldId worldId )
{
	b3BodyId groundId;
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position.y = -1.0f;
		groundId = b3CreateBody( worldId, &bodyDef );
	}

	{
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		{
			b3BoxHull box = b3MakeBoxHull( 120.0f, 1.0f, 120.0f );
			g_groundShapeId = b3CreateHullShape( groundId, &shapeDef, &box.base );
		}
		{
			b3Vec3 offset = { -50.0f, 8.0f, 0.0f };
			b3BoxHull box = b3MakeOffsetBoxHull( 1.0f, 8.0f, 50.0f, offset );
			b3CreateHullShape( groundId, &shapeDef, &box.base );
		}
		{
			b3Vec3 offset = { 50.0f, 8.0f, 0.0f };
			b3BoxHull box = b3MakeOffsetBoxHull( 1.0f, 8.0f, 50.0f, offset );
			b3CreateHullShape( groundId, &shapeDef, &box.base );
		}
		{
			b3Vec3 offset = { 0.0f, 8.0f, -50.0f };
			b3BoxHull box = b3MakeOffsetBoxHull( 50.0f, 8.0f, 1.0f, offset );
			b3CreateHullShape( groundId, &shapeDef, &box.base );
		}
		{
			b3Vec3 offset = { 0.0f, 8.0f, 50.0f };
			b3BoxHull box = b3MakeOffsetBoxHull( 50.0f, 8.0f, 1.0f, offset );
			b3CreateHullShape( groundId, &shapeDef, &box.base );
		}
	}
	{
		b3HullData* rockHull = b3CreateRock( 1.5f );

		int count = BENCHMARK_DEBUG ? 2 : 24;
		float height = 24.0f;
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		for ( int Y = 0; Y < count; ++Y )
		{
			for ( int X = 0; X <= 20; ++X )
			{
				for ( int Z = 0; Z <= 20; ++Z )
				{
					bodyDef.position.x = -40.0f + 4.0f * X;
					bodyDef.position.y = 4.0f * Y + height + 1.0f;
					bodyDef.position.z = -40.0f + 4.0f * Z;
					b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
					b3CreateHullShape( bodyId, &shapeDef, rockHull );
				}
			}
		}

		b3DestroyHull( rockHull );
	}

	g_junkyardData.radius = 35.0f;
	float mHeight = 24.0f;

	b3HullData* hull = b3CreateCylinder( mHeight, 4.0f, 0.0f, 16 );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_kinematicBody;
	bodyDef.position = (b3Pos){ g_junkyardData.radius, 0.0f, 0.0f };
	g_junkyardData.pusherId = b3CreateBody( worldId, &bodyDef );
	g_junkyardData.degrees = 0.0f;
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( g_junkyardData.pusherId, &shapeDef, hull );
	b3DestroyHull( hull );
}

void StepJunkyard( b3WorldId worldId, int stepCount )
{
	(void)worldId;
	(void)stepCount;

	float timeStep = 1.0f / 60.0f;
	const float omega = -6.0f;
	g_junkyardData.degrees += omega * timeStep;
	b3CosSin cs = b3ComputeCosSin( g_junkyardData.degrees * B3_PI / 180.0f );
	float r = g_junkyardData.radius;
	b3Pos targetPos = { r * cs.cosine, 0.0f, r * cs.sine };
	b3WorldTransform target = { .p = targetPos, .q = b3Quat_identity };
	b3Body_SetTargetTransform( g_junkyardData.pusherId, target, timeStep, false );
}

// Huge pile of large convexes, ported from PEEL. Each convex is the hull of 32 random points on a
// sphere. A fixed LCG seed makes the hull identical across runs so results compare directly.

// PEEL's BasicRandom, kept verbatim so the point set matches the original
typedef struct ConvexPileRandom
{
	unsigned int state;
} ConvexPileRandom;

static unsigned int NextConvexPileRandom( ConvexPileRandom* rng )
{
	rng->state = rng->state * 2147001325u + 715136305u;
	return rng->state;
}

// Float in [-0.5, 0.5]
static float ConvexPileRandomFloat( ConvexPileRandom* rng )
{
	return (float)( NextConvexPileRandom( rng ) & 0xffff ) / 65535.0f - 0.5f;
}

// Uniform random direction, rejection sampled inside the unit sphere then pushed to the surface
static b3Vec3 UnitRandomPoint( ConvexPileRandom* rng )
{
	b3Vec3 point;
	float lengthSq;
	do
	{
		point.x = ConvexPileRandomFloat( rng );
		point.y = ConvexPileRandomFloat( rng );
		point.z = ConvexPileRandomFloat( rng );
		lengthSq = b3Dot( point, point );
	} while ( lengthSq > 0.25f );

	return b3Normalize( point );
}

void CreateConvexPile( b3WorldId worldId )
{
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Pos){ 0.0f, -1.0f, 0.0f };
		b3BodyId groundId = b3CreateBody( worldId, &bodyDef );

		b3BoxHull box = b3MakeBoxHull( 250.0f, 1.0f, 250.0f );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		g_groundShapeId = b3CreateHullShape( groundId, &shapeDef, &box.base );
	}

	int countX = 8;
	int countZ = 8;
	int layers = BENCHMARK_DEBUG ? 10 : 80;
	float amplitude = 2.0f;
	int pointCount = 32;
	float scatter = 2.0f * amplitude;

	// Hull around random points on a sphere of radius amplitude
	assert( pointCount <= 64 );
	b3Vec3 points[64];
	ConvexPileRandom rng = { 42 };
	for ( int i = 0; i < pointCount; ++i )
	{
		points[i] = b3MulSV( amplitude, UnitRandomPoint( &rng ) );
	}

	b3HullData* convex = b3CreateHull( points, pointCount, pointCount );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3ShapeDef shapeDef = b3DefaultShapeDef();

	// Grid tall enough to collapse into a pile
	for ( int layer = 0; layer < layers; ++layer )
	{
		for ( int z = 0; z < countZ; ++z )
		{
			for ( int x = 0; x < countX; ++x )
			{
				float posX = ( (float)x - 0.5f * (float)countX ) * scatter;
				float posZ = ( (float)z - 0.5f * (float)countZ ) * scatter;
				float posY = amplitude + 2.0f * amplitude * (float)layer;

				bodyDef.position = (b3Pos){ posX, posY, posZ };
				b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
				b3CreateHullShape( bodyId, &shapeDef, convex );
			}
		}
	}

	b3DestroyHull( convex );
}

// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "box3d/box3d.h"
#include "determinism.h"
#include "metal_wheel1_hulls.h"
#include "test_macros.h"

#include <stdio.h>
#include <stdlib.h>

#ifdef BOX3D_PROFILE
	#include <tracy/TracyC.h>
#else
	#define TracyCFrameMark
#endif

// Double precision accumulates body positions in double, so the settle/sleep step and the
// state hash differ from the float build. Both modes are internally deterministic.
#if defined( BOX3D_DOUBLE_PRECISION )
#define EXPECTED_SLEEP_STEP 301
#define EXPECTED_HASH 0xE4844A97
#define EXPECTED_WHEEL_HASH 0xEC7CDF3B
#else
#define EXPECTED_SLEEP_STEP 269
#define EXPECTED_HASH 0x50313037
#define EXPECTED_WHEEL_HASH 0x1A105A82
#endif

static int SingleMultithreadingTest( int workerCount )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = workerCount;

	b3WorldId worldId = b3CreateWorld( &worldDef );

	FallingRagdollData data = CreateFallingRagdolls( worldId );

	float timeStep = 1.0f / 60.0f;

	int stepLimit = 500;
	for ( int i = 0; i < stepLimit; ++i )
	{
		int subStepCount = 4;
		b3World_Step( worldId, timeStep, subStepCount );
		TracyCFrameMark;

		bool done = UpdateFallingRagdolls( worldId, &data );
		if ( done )
		{
			break;
		}
	}

	b3DestroyWorld( worldId );

	if ( data.sleepStep != EXPECTED_SLEEP_STEP || data.hash != EXPECTED_HASH )
	{
		printf( "  workers=%d sleepStep=%d hash=0x%08X\n", workerCount, data.sleepStep, data.hash );
	}

	ENSURE( data.sleepStep == EXPECTED_SLEEP_STEP );
	ENSURE( data.hash == EXPECTED_HASH );

	DestroyFallingRagdolls( &data );

	return 0;
}

// Test multithreaded determinism.
static int MultithreadingTest( void )
{
	for ( int workerCount = 1; workerCount < 6; ++workerCount )
	{
		int result = SingleMultithreadingTest( workerCount );
		ENSURE( result == 0 );
	}

	return 0;
}

// Test cross platform determinism.
static int CrossPlatformTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	FallingRagdollData data = CreateFallingRagdolls( worldId );

	float timeStep = 1.0f / 60.0f;

	bool done = false;
	while ( done == false )
	{
		int subStepCount = 4;
		b3World_Step( worldId, timeStep, subStepCount );
		TracyCFrameMark;

		done = UpdateFallingRagdolls( worldId, &data );
	}

	if ( data.sleepStep != EXPECTED_SLEEP_STEP || data.hash != EXPECTED_HASH )
	{
		printf( "  cross-platform sleepStep=%d hash=0x%08X\n", data.sleepStep, data.hash );
	}

	ENSURE( data.sleepStep == EXPECTED_SLEEP_STEP );
	ENSURE( data.hash == EXPECTED_HASH );

	DestroyFallingRagdolls( &data );

	b3DestroyWorld( worldId );

	return 0;
}

// Step the wheel stack and hash the body transforms.
static uint32_t RunWheelStackHash( int workerCount, int stepCount )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = workerCount;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = ( b3Pos ){ 0.0f, -1.0f, 0.0f };
		b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
		b3BoxHull box = b3MakeBoxHull( 10.0f, 1.0f, 10.0f );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateHullShape( groundId, &shapeDef, &box.base );
	}

	b3HullData* hulls[s_metalWheel1HullCount];
	for ( int h = 0; h < s_metalWheel1HullCount; ++h )
	{
		const WheelHullSpan span = s_metalWheel1Hulls[h];
		hulls[h] = b3CreateHull( &s_metalWheel1Verts[span.offset], span.count, span.count );
	}

	const float height = 0.171f;
	const float spacing = height + 0.006f;
	const float startY = 0.5f * height + 0.004f;
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = 0.6f;

	const int wheelCount = 10;
	b3BodyId wheelBodies[10];
	for ( int i = 0; i < wheelCount; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = ( b3Pos ){ 0.0f, startY + i * spacing, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		for ( int h = 0; h < s_metalWheel1HullCount; ++h )
		{
			b3CreateHullShape( bodyId, &shapeDef, hulls[h] );
		}
		wheelBodies[i] = bodyId;
	}
	for ( int h = 0; h < s_metalWheel1HullCount; ++h )
	{
		b3DestroyHull( hulls[h] );
	}

	float timeStep = 1.0f / 60.0f;
	for ( int i = 0; i < stepCount; ++i )
	{
		b3World_Step( worldId, timeStep, 8 );
	}

	uint32_t hash = B3_HASH_INIT;
	for ( int i = 0; i < wheelCount; ++i )
	{
		b3WorldTransform xf = b3Body_GetTransform( wheelBodies[i] );
		hash = b3Hash( hash, (uint8_t*)( &xf ), sizeof( b3WorldTransform ) );
	}

	b3DestroyWorld( worldId );
	return hash;
}

// Compound contacts must be deterministic: identical hash at every worker count and against the reference.
static int WheelStackDeterminismTest( void )
{
	uint32_t base = RunWheelStackHash( 1, 200 );

	for ( int workerCount = 2; workerCount <= 4; ++workerCount )
	{
		uint32_t hash = RunWheelStackHash( workerCount, 200 );
		if ( hash != base )
		{
			printf( "  wheel stack: workers=%d hash=0x%08X base=0x%08X\n", workerCount, hash, base );
		}
		ENSURE( hash == base );
	}

	if ( base != EXPECTED_WHEEL_HASH )
	{
		printf( "  wheel stack cross-platform: hash=0x%08X expected=0x%08X\n", base, EXPECTED_WHEEL_HASH );
	}
	ENSURE( base == EXPECTED_WHEEL_HASH );

	return 0;
}

int DeterminismTest( void )
{
	RUN_SUBTEST( MultithreadingTest );
	RUN_SUBTEST( CrossPlatformTest );
	RUN_SUBTEST( WheelStackDeterminismTest );

	return 0;
}

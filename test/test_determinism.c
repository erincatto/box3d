// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "box3d/box3d.h"
#include "determinism.h"
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
#define EXPECTED_SLEEP_STEP 297
#define EXPECTED_HASH 0x27FF38C1
#else
#define EXPECTED_SLEEP_STEP 308
#define EXPECTED_HASH 0x1E5EDD79
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

int DeterminismTest( void )
{
	RUN_SUBTEST( MultithreadingTest );
	RUN_SUBTEST( CrossPlatformTest );

	return 0;
}

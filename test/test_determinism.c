// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "box3d/box3d.h"
#include "determinism.h"
#include "stability.h"
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
#define RAGDOLL_SLEEP_STEP 272
#define RAGDOLL_HASH 0xBF43ECD7
#define WAVE_PILE_SLEEP_STEP 291
#define WAVE_PILE_HASH 0x8B6E2CD2
#define QUERY_SPAWN_SLEEP_STEP 242
#define QUERY_SPAWN_HASH 0x1737F5BC
#define QUERY_SPAWN_HIT_COUNT 59
#define QUERY_SPAWN_QUERY_HASH 0x31F090DC
#define MESH_DROP_SLEEP_STEP 251
#define MESH_DROP_HASH 0x465381C5
#else
#define RAGDOLL_SLEEP_STEP 286
#define RAGDOLL_HASH 0x0211682A
#define WAVE_PILE_SLEEP_STEP 282
#define WAVE_PILE_HASH 0xFC161D3D
#define QUERY_SPAWN_SLEEP_STEP 242
#define QUERY_SPAWN_HASH 0xB9F993A5
#define QUERY_SPAWN_HIT_COUNT 59
#define QUERY_SPAWN_QUERY_HASH 0xB9D3863D
#define MESH_DROP_SLEEP_STEP 251
#define MESH_DROP_HASH 0xE58C7240
#endif

// The goldens above pin exact values for the default four point manifold. A build that
// overrides B3_MAX_MANIFOLD_POINTS produces a different contact set, so those checks drop
// out and only the run to run agreement below applies.
#if B3_MAX_MANIFOLD_POINTS == 4
#define ENSURE_GOLDEN( condition ) ENSURE( condition )
#else
#define ENSURE_GOLDEN( condition ) ( (void)0 )
#endif

typedef struct DeterminismResult
{
	int sleepStep;
	uint32_t hash;
	int queryHitCount;
	uint32_t queryHash;
	bool seeded;
} DeterminismResult;

// Every worker count has to reproduce the first run exactly. Scenarios without queries leave
// the query fields zero in both the reference and the result.
static int EnsureRepeatable( DeterminismResult* reference, DeterminismResult result )
{
	if ( reference->seeded == false )
	{
		result.seeded = true;
		*reference = result;
		return 0;
	}

	ENSURE( result.sleepStep == reference->sleepStep );
	ENSURE( result.hash == reference->hash );
	ENSURE( result.queryHitCount == reference->queryHitCount );
	ENSURE( result.queryHash == reference->queryHash );

	return 0;
}

static int SingleMultithreadingTest( int workerCount, DeterminismResult* reference )
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

	if ( data.sleepStep != RAGDOLL_SLEEP_STEP || data.hash != RAGDOLL_HASH )
	{
		printf( "  workers=%d sleepStep=%d hash=0x%08X\n", workerCount, data.sleepStep, data.hash );
	}

	ENSURE_GOLDEN( data.sleepStep == RAGDOLL_SLEEP_STEP );
	ENSURE_GOLDEN( data.hash == RAGDOLL_HASH );
	ENSURE( EnsureRepeatable( reference, (DeterminismResult){ .sleepStep = data.sleepStep, .hash = data.hash } ) == 0 );

	DestroyFallingRagdolls( &data );

	return 0;
}

// Test multithreaded determinism.
static int MultithreadingTest( void )
{
	DeterminismResult reference = { 0 };
	for ( int workerCount = 1; workerCount < 6; ++workerCount )
	{
		int result = SingleMultithreadingTest( workerCount, &reference );
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

	if ( data.sleepStep != RAGDOLL_SLEEP_STEP || data.hash != RAGDOLL_HASH )
	{
		printf( "  cross-platform sleepStep=%d hash=0x%08X\n", data.sleepStep, data.hash );
	}

	ENSURE_GOLDEN( data.sleepStep == RAGDOLL_SLEEP_STEP );
	ENSURE_GOLDEN( data.hash == RAGDOLL_HASH );

	DestroyFallingRagdolls( &data );

	b3DestroyWorld( worldId );

	return 0;
}

static int SingleWavePileTest( int workerCount, DeterminismResult* reference )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = workerCount;

	b3WorldId worldId = b3CreateWorld( &worldDef );

	WavePileData data = CreateWavePile( worldId );

	float timeStep = 1.0f / 60.0f;

	// Rolling resistance must put the pile to sleep within 500 steps
	bool done = false;
	for ( int i = 0; i < 500 && done == false; ++i )
	{
		int subStepCount = 4;
		b3World_Step( worldId, timeStep, subStepCount );
		TracyCFrameMark;

		done = UpdateWavePile( worldId, &data );
	}

	b3DestroyWorld( worldId );

	if ( data.sleepStep != WAVE_PILE_SLEEP_STEP || data.hash != WAVE_PILE_HASH )
	{
		printf( "  wave pile workers=%d sleepStep=%d hash=0x%08X\n", workerCount, data.sleepStep, data.hash );
	}

	ENSURE( done == true );
	ENSURE_GOLDEN( data.sleepStep == WAVE_PILE_SLEEP_STEP );
	ENSURE_GOLDEN( data.hash == WAVE_PILE_HASH );
	ENSURE( EnsureRepeatable( reference, (DeterminismResult){ .sleepStep = data.sleepStep, .hash = data.hash } ) == 0 );

	DestroyWavePile( &data );

	return 0;
}

// Test multithreaded determinism of a mixed convex pile on a wave height field.
static int WavePileTest( void )
{
	DeterminismResult reference = { 0 };
	for ( int workerCount = 1; workerCount <= 4; ++workerCount )
	{
		int result = SingleWavePileTest( workerCount, &reference );
		ENSURE( result == 0 );
	}

	return 0;
}

static int SingleQuerySpawnTest( int workerCount, DeterminismResult* reference )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = workerCount;

	b3WorldId worldId = b3CreateWorld( &worldDef );

	QuerySpawnData data = CreateQuerySpawn( worldId );

	float timeStep = 1.0f / 60.0f;

	bool done = false;
	for ( int i = 0; i < 1000 && done == false; ++i )
	{
		int subStepCount = 4;
		b3World_Step( worldId, timeStep, subStepCount );
		TracyCFrameMark;

		done = UpdateQuerySpawn( worldId, &data );
	}

	b3DestroyWorld( worldId );

	if ( data.sleepStep != QUERY_SPAWN_SLEEP_STEP || data.hash != QUERY_SPAWN_HASH || data.queryHitCount != QUERY_SPAWN_HIT_COUNT ||
		 data.queryHash != QUERY_SPAWN_QUERY_HASH )
	{
		printf( "  query spawn workers=%d sleepStep=%d hash=0x%08X hits=%d queryHash=0x%08X\n", workerCount, data.sleepStep,
				data.hash, data.queryHitCount, data.queryHash );
	}

	ENSURE( done == true );
	ENSURE( data.spawnCount == QUERY_SPAWN_COUNT );
	ENSURE_GOLDEN( data.sleepStep == QUERY_SPAWN_SLEEP_STEP );
	ENSURE_GOLDEN( data.hash == QUERY_SPAWN_HASH );
	ENSURE_GOLDEN( data.queryHitCount == QUERY_SPAWN_HIT_COUNT );
	ENSURE_GOLDEN( data.queryHash == QUERY_SPAWN_QUERY_HASH );
	ENSURE( EnsureRepeatable( reference, (DeterminismResult){ .sleepStep = data.sleepStep,
															  .hash = data.hash,
															  .queryHitCount = data.queryHitCount,
															  .queryHash = data.queryHash } ) == 0 );

	DestroyQuerySpawn( &data );

	return 0;
}

// Test determinism of world queries by feeding their results back into the simulation.
static int QuerySpawnTest( void )
{
	DeterminismResult reference = { 0 };
	for ( int workerCount = 1; workerCount <= 4; ++workerCount )
	{
		int result = SingleQuerySpawnTest( workerCount, &reference );
		ENSURE( result == 0 );
	}

	return 0;
}

static int SingleMeshDropTest( int workerCount, DeterminismResult* reference )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = workerCount;

	b3WorldId worldId = b3CreateWorld( &worldDef );

	MeshDropData data = CreateMeshDrop( worldId, b3Pos_zero );

	float timeStep = 1.0f / 60.0f;

	bool done = false;
	for ( int i = 0; i < 400 && done == false; ++i )
	{
		int subStepCount = 4;
		b3World_Step( worldId, timeStep, subStepCount );
		TracyCFrameMark;

		done = UpdateMeshDrop( worldId, &data );
	}

	b3DestroyWorld( worldId );

	if ( data.sleepStep != MESH_DROP_SLEEP_STEP || data.hash != MESH_DROP_HASH )
	{
		printf( "  mesh drop workers=%d sleepStep=%d hash=0x%08X\n", workerCount, data.sleepStep, data.hash );
	}

	ENSURE( done == true );
	ENSURE_GOLDEN( data.sleepStep == MESH_DROP_SLEEP_STEP );
	ENSURE_GOLDEN( data.hash == MESH_DROP_HASH );
	ENSURE( EnsureRepeatable( reference, (DeterminismResult){ .sleepStep = data.sleepStep, .hash = data.hash } ) == 0 );

	DestroyMeshDrop( &data );

	return 0;
}

// Test continuous collision determinism. Thin fast boxes need CCD against the wave mesh.
// The scene is large, so only the single threaded and widest schedules run.
static int MeshDropTest( void )
{
	DeterminismResult reference = { 0 };
	int workerCounts[2] = { 1, 4 };
	for ( int i = 0; i < 2; ++i )
	{
		int result = SingleMeshDropTest( workerCounts[i], &reference );
		ENSURE( result == 0 );
	}

	return 0;
}

int DeterminismTest( void )
{
	RUN_SUBTEST( MultithreadingTest );
	RUN_SUBTEST( CrossPlatformTest );
	RUN_SUBTEST( WavePileTest );
	RUN_SUBTEST( QuerySpawnTest );
	RUN_SUBTEST( MeshDropTest );

	return 0;
}

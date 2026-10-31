// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Headless soak of the Continuous / Mesh Drop sample. Each generation drops 1024 small bodies on a
// wave mesh and steps until every body sleeps. A generation fails if a body falls through the mesh
// or the pile is still awake at the step limit. Tunnels are rare, so the soak also counts bodies that
// sink into the wave past a few depth levels and can compare those counts against a baseline.
// Generations run in parallel, one single threaded world per thread. The printed seed reproduces a
// generation in the sample.

#define _CRT_SECURE_NO_WARNINGS

#include "core.h"
#include "mesh_drop.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_REPORTED_FAILURES 32
#define MAX_THREADS 64
#define MAX_BASELINE_ROWS 64
#define BASELINE_ROW_SIZE 512

typedef enum FailureKind
{
	e_tunnel,
	e_awake,
} FailureKind;

typedef struct Failure
{
	FailureKind kind;
	MeshDropShape shape;
	int generation;
	uint32_t seed;
	int step;
	int index;
	b3Pos position;
} Failure;

typedef struct ShapeStats
{
	int generationCount;
	int tunnelCount;
	int awakeCount;
	int depthCounts[MESH_DROP_DEPTH_LEVEL_COUNT];

	float maxDepth;
	uint32_t maxSeed;
	int maxGeneration;
	int maxIndex;
	int maxStep;
} ShapeStats;

typedef struct Soak
{
	b3Mutex* mutex;
	b3MeshData* groundMesh;
	b3HullData* cylinder;
	MeshDropSettings settings;
	uint32_t baseSeed;
	int generationCount;
	int nextGeneration;

	// -1 rotates through the shapes
	int shapeFilter;

	bool fixedSeed;

	int completedCount;
	int failureCount;
	long long totalSteps;
	int maxSteps;
	ShapeStats stats[e_meshDropShapeCount];
	Failure failures[MAX_REPORTED_FAILURES];
} Soak;

static void RecordFailure( Soak* soak, const Failure* failure )
{
	b3LockMutex( soak->mutex );
	if ( soak->failureCount < MAX_REPORTED_FAILURES )
	{
		soak->failures[soak->failureCount] = *failure;
	}
	soak->failureCount += 1;
	b3UnlockMutex( soak->mutex );

	const char* kind = failure->kind == e_tunnel ? "TUNNEL" : "AWAKE";
	printf( "%s: generation %d, %s, seed %u, step %d, body (%d, %d) at (%.2f, %.2f, %.2f)\n", kind, failure->generation,
			g_meshDropShapeNames[failure->shape], failure->seed, failure->step, failure->index / MESH_DROP_GRID_COUNT,
			failure->index % MESH_DROP_GRID_COUNT, (float)failure->position.x, (float)failure->position.y,
			(float)failure->position.z );
}

// Mirrors MeshDrop::Generate and the auto generate step loop. Returns the step count.
static int RunGeneration( Soak* soak, int generation, uint32_t seed, MeshDropShape shape, b3BodyId* bodies,
						  MeshDropDepth* depth, int* failureKind )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;

	// The world registry is global and not thread safe
	b3LockMutex( soak->mutex );
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3UnlockMutex( soak->mutex );

	CreateMeshDropGround( worldId, soak->groundMesh );

	MeshDropSettings settings = soak->settings;
	settings.shape = shape;
	CreateMeshDropBodies( worldId, settings, seed, soak->cylinder, bodies );
	ResetMeshDropDepth( depth, settings, soak->cylinder );

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;
	int maxSteps = GetMeshDropStepLimit( shape );
	float tunnelHeight = GetMeshDropTunnelHeight();
	int step = 0;
	bool failed = false;

	while ( failed == false )
	{
		b3World_Step( worldId, timeStep, subStepCount );
		UpdateMeshDropDepth( worldId, depth );
		step += 1;

		for ( int i = 0; i < MESH_DROP_BODY_COUNT; ++i )
		{
			b3Pos center = b3Body_GetWorldCenter( bodies[i] );
			if ( center.y < tunnelHeight )
			{
				Failure failure = { e_tunnel, shape, generation, seed, step, i, center };
				RecordFailure( soak, &failure );
				*failureKind = e_tunnel;
				failed = true;
				break;
			}
		}

		if ( b3World_GetBodyEvents( worldId ).moveCount == 0 )
		{
			break;
		}

		if ( failed == false && step > maxSteps )
		{
			int awakeIndex = 0;
			for ( int i = 0; i < MESH_DROP_BODY_COUNT; ++i )
			{
				if ( b3Body_IsAwake( bodies[i] ) )
				{
					awakeIndex = i;
					break;
				}
			}

			b3Pos center = b3Body_GetWorldCenter( bodies[awakeIndex] );
			Failure failure = { e_awake, shape, generation, seed, step, awakeIndex, center };
			RecordFailure( soak, &failure );
			*failureKind = e_awake;
			failed = true;
		}
	}

	b3LockMutex( soak->mutex );
	b3DestroyWorld( worldId );
	b3UnlockMutex( soak->mutex );

	return step;
}

static void SoakWorker( void* context )
{
	Soak* soak = context;
	b3BodyId* bodies = malloc( MESH_DROP_BODY_COUNT * sizeof( b3BodyId ) );
	MeshDropDepth* depth = malloc( sizeof( MeshDropDepth ) );

	while ( true )
	{
		b3LockMutex( soak->mutex );
		int generation = soak->nextGeneration;
		soak->nextGeneration += 1;
		b3UnlockMutex( soak->mutex );

		if ( generation >= soak->generationCount )
		{
			break;
		}

		uint32_t seed = soak->fixedSeed ? soak->baseSeed : GetMeshDropSeed( soak->baseSeed, generation );
		MeshDropShape shape =
			soak->shapeFilter >= 0 ? (MeshDropShape)soak->shapeFilter : (MeshDropShape)( generation % e_meshDropShapeCount );

		int failureKind = -1;
		int steps = RunGeneration( soak, generation, seed, shape, bodies, depth, &failureKind );

		int depthCounts[MESH_DROP_DEPTH_LEVEL_COUNT];
		GetMeshDropDepthCounts( depth, depthCounts );

		b3LockMutex( soak->mutex );
		soak->completedCount += 1;
		soak->totalSteps += steps;
		soak->maxSteps = b3MaxInt( soak->maxSteps, steps );

		ShapeStats* stats = soak->stats + shape;
		stats->generationCount += 1;
		stats->tunnelCount += failureKind == e_tunnel ? 1 : 0;
		stats->awakeCount += failureKind == e_awake ? 1 : 0;
		for ( int i = 0; i < MESH_DROP_DEPTH_LEVEL_COUNT; ++i )
		{
			stats->depthCounts[i] += depthCounts[i];
		}

		// Ties go to the earlier generation so the report does not depend on thread timing
		if ( depth->maxDepth > stats->maxDepth ||
			 ( depth->maxDepth == stats->maxDepth && generation < stats->maxGeneration ) )
		{
			stats->maxDepth = depth->maxDepth;
			stats->maxSeed = seed;
			stats->maxGeneration = generation;
			stats->maxIndex = depth->maxIndex;
			stats->maxStep = depth->maxStep;
		}

		int completed = soak->completedCount;
		b3UnlockMutex( soak->mutex );

		if ( completed % b3MaxInt( 100, soak->generationCount / 10 ) == 0 )
		{
			printf( "%d generations, %d bodies\n", completed, completed * MESH_DROP_BODY_COUNT );
			fflush( stdout );
		}
	}

	free( depth );
	free( bodies );
}

static const char* GetFilterName( int shapeFilter )
{
	return shapeFilter >= 0 ? g_meshDropShapeNames[shapeFilter] : "all";
}

// A baseline row starts with the run that made it, so one file can hold several runs
static void GetBaselineKey( const Soak* soak, char* key, int size )
{
	snprintf( key, size, "%s,%u,%d,", GetFilterName( soak->shapeFilter ), soak->baseSeed, soak->generationCount );
}

// Replaces the rows of this run and keeps the rest
static void WriteBaseline( const Soak* soak, const char* path )
{
	char key[64];
	GetBaselineKey( soak, key, sizeof( key ) );
	size_t keyLength = strlen( key );

	char* rows = malloc( MAX_BASELINE_ROWS * BASELINE_ROW_SIZE );
	int rowCount = 0;

	FILE* file = fopen( path, "r" );
	if ( file != NULL )
	{
		char line[BASELINE_ROW_SIZE];
		bool header = true;
		while ( fgets( line, sizeof( line ), file ) != NULL )
		{
			if ( header == false && strncmp( line, key, keyLength ) != 0 && rowCount < MAX_BASELINE_ROWS )
			{
				snprintf( rows + rowCount * BASELINE_ROW_SIZE, BASELINE_ROW_SIZE, "%s", line );
				rowCount += 1;
			}

			header = false;
		}

		fclose( file );
	}

	file = fopen( path, "w" );
	if ( file == NULL )
	{
		printf( "cannot write %s\n", path );
		free( rows );
		return;
	}

	fprintf( file, "filter,base,generations,shape,tunnels,awake" );
	for ( int i = 0; i < MESH_DROP_DEPTH_LEVEL_COUNT; ++i )
	{
		fprintf( file, ",depth%g", g_meshDropDepthLevels[i] );
	}
	fprintf( file, "\n" );

	for ( int i = 0; i < rowCount; ++i )
	{
		fputs( rows + i * BASELINE_ROW_SIZE, file );
	}

	free( rows );

	for ( int shape = 0; shape < e_meshDropShapeCount; ++shape )
	{
		const ShapeStats* stats = soak->stats + shape;
		if ( stats->generationCount == 0 )
		{
			continue;
		}

		fprintf( file, "%s,%u,%d,%s,%d,%d", GetFilterName( soak->shapeFilter ), soak->baseSeed, soak->generationCount,
				 g_meshDropShapeNames[shape], stats->tunnelCount, stats->awakeCount );
		for ( int i = 0; i < MESH_DROP_DEPTH_LEVEL_COUNT; ++i )
		{
			fprintf( file, ",%d", stats->depthCounts[i] );
		}
		fprintf( file, "\n" );
	}

	fclose( file );
	printf( "wrote %s\n", path );
}

// Returns 0 when no count grew past its noise band, 1 when one did, 2 when the baseline has no rows for this run
static int CompareBaseline( const Soak* soak, const char* path )
{
	FILE* file = fopen( path, "r" );
	if ( file == NULL )
	{
		printf( "cannot read %s\n", path );
		return 2;
	}

	char line[512];
	if ( fgets( line, sizeof( line ), file ) == NULL )
	{
		fclose( file );
		printf( "empty baseline %s\n", path );
		return 2;
	}

	int result = 0;
	int changedCount = 0;
	int matchedCount = 0;

	while ( fgets( line, sizeof( line ), file ) != NULL )
	{
		char filter[32];
		char shapeName[32];
		unsigned int base;
		int generations;
		int counts[2 + MESH_DROP_DEPTH_LEVEL_COUNT];

		_Static_assert( MESH_DROP_DEPTH_LEVEL_COUNT == 4, "update the baseline row format" );
		int fieldCount = sscanf( line, "%31[^,],%u,%d,%31[^,],%d,%d,%d,%d,%d,%d", filter, &base, &generations, shapeName,
								 counts + 0, counts + 1, counts + 2, counts + 3, counts + 4, counts + 5 );
		if ( fieldCount != 10 )
		{
			continue;
		}

		if ( strcmp( filter, GetFilterName( soak->shapeFilter ) ) != 0 || base != soak->baseSeed ||
			 generations != soak->generationCount )
		{
			continue;
		}

		int shape = -1;
		for ( int i = 0; i < e_meshDropShapeCount; ++i )
		{
			if ( strcmp( shapeName, g_meshDropShapeNames[i] ) == 0 )
			{
				shape = i;
			}
		}

		if ( shape < 0 )
		{
			continue;
		}

		matchedCount += 1;

		const ShapeStats* stats = soak->stats + shape;
		int current[2 + MESH_DROP_DEPTH_LEVEL_COUNT] = { stats->tunnelCount, stats->awakeCount };
		for ( int i = 0; i < MESH_DROP_DEPTH_LEVEL_COUNT; ++i )
		{
			current[2 + i] = stats->depthCounts[i];
		}

		for ( int i = 0; i < 2 + MESH_DROP_DEPTH_LEVEL_COUNT; ++i )
		{
			if ( current[i] == counts[i] )
			{
				continue;
			}

			// Any collision change reshuffles the chaotic pile, so a count moves by about its square root
			// even when nothing got worse. Flag growth past three of those. A count that was zero flags on any
			// growth, since a good build never sinks that deep.
			float band = 3.0f * sqrtf( (float)counts[i] );
			bool regressed = i < 2 ? current[i] > counts[i] : current[i] > counts[i] + band;

			char column[32];
			if ( i < 2 )
			{
				snprintf( column, sizeof( column ), i == 0 ? "tunnels" : "awake" );
			}
			else
			{
				snprintf( column, sizeof( column ), "> %g slop", g_meshDropDepthLevels[i - 2] );
			}

			printf( "  %-9s %-11s %d>%d%s\n", shapeName, column, counts[i], current[i], regressed ? "  GREW" : "" );
			changedCount += 1;
			result = regressed ? 1 : result;
		}
	}

	fclose( file );

	if ( matchedCount == 0 )
	{
		printf( "%s has no rows for -s=%s -b=%u -g=%d\n", path, GetFilterName( soak->shapeFilter ), soak->baseSeed,
				soak->generationCount );
		return 2;
	}

	printf( "compared with %s: %d changed, %s\n", path, changedCount, result == 0 ? "none grew past noise" : "GREW" );
	return result;
}

static const char* MatchValue( const char* arg, const char* shortName, const char* longName )
{
	size_t shortLength = strlen( shortName );
	if ( strncmp( arg, shortName, shortLength ) == 0 )
	{
		return arg + shortLength;
	}

	size_t longLength = strlen( longName );
	if ( strncmp( arg, longName, longLength ) == 0 )
	{
		return arg + longLength;
	}

	return NULL;
}

int main( int argc, char** argv )
{
	int generationCount = 1000;
	int threadCount = 8;
	int shapeFilter = e_meshDropBox;
	uint32_t seed = (uint32_t)b3GetTicks();
	bool fixedSeed = false;
	const char* writePath = NULL;
	const char* comparePath = NULL;
	MeshDropSettings settings = GetDefaultMeshDropSettings();

	for ( int i = 1; i < argc; ++i )
	{
		const char* arg = argv[i];
		const char* value;

		if ( ( value = MatchValue( arg, "-g=", "--generations=" ) ) != NULL )
		{
			generationCount = atoi( value );
		}
		else if ( ( value = MatchValue( arg, "-t=", "--threads=" ) ) != NULL )
		{
			threadCount = b3ClampInt( atoi( value ), 1, MAX_THREADS );
		}
		else if ( ( value = MatchValue( arg, "-s=", "--shape=" ) ) != NULL )
		{
			shapeFilter = -2;
			for ( int k = 0; k < e_meshDropShapeCount; ++k )
			{
				if ( strcmp( value, g_meshDropShapeNames[k] ) == 0 )
				{
					shapeFilter = k;
				}
			}

			if ( strcmp( value, "all" ) == 0 )
			{
				shapeFilter = -1;
			}

			if ( shapeFilter == -2 )
			{
				printf( "unknown shape: %s\n", value );
				return 2;
			}
		}
		else if ( ( value = MatchValue( arg, "-seed=", "--seed=" ) ) != NULL )
		{
			seed = (uint32_t)strtoul( value, NULL, 10 );
			fixedSeed = true;
		}
		else if ( ( value = MatchValue( arg, "-b=", "--base-seed=" ) ) != NULL )
		{
			seed = (uint32_t)strtoul( value, NULL, 10 );
		}
		else if ( ( value = MatchValue( arg, "-a=", "--amplitude=" ) ) != NULL )
		{
			settings.amplitude = (float)atof( value );
		}
		else if ( ( value = MatchValue( arg, "-w=", "--write=" ) ) != NULL )
		{
			writePath = value;
		}
		else if ( ( value = MatchValue( arg, "-c=", "--compare=" ) ) != NULL )
		{
			comparePath = value;
		}
		else
		{
			printf( "Usage: mesh_soak [options]\n"
					"Drops %d bodies per generation on the Mesh Drop wave mesh until they sleep.\n"
					"Fails on a body below y = -2 (tunnel) or a pile awake past the step limit.\n"
					"Counts bodies that sink into the wave deeper than each depth level.\n"
					"  -g, --generations=<int>  generation count (default 1000)\n"
					"  -t, --threads=<int>      worker threads, one world each (default 8)\n"
					"  -s, --shape=<name>       box, capsule, cylinder, sphere or all (default box)\n"
					"  -b, --base-seed=<uint>   base seed, each generation hashes its own seed from it (default random)\n"
					"  --seed=<uint>            run every generation with this exact seed (replay)\n"
					"  -a, --amplitude=<float>  wave amplitude (default 0.5)\n"
					"  -w, --write=<csv>        write the counts as a baseline, replacing only the rows of this run\n"
					"  -c, --compare=<csv>      compare the counts with the baseline rows from the same -s, -b and -g\n"
					"Exit code is 0 when every generation passes and no count grew past noise, 1 otherwise,\n"
					"2 for bad arguments or a baseline from a different run.\n",
					MESH_DROP_BODY_COUNT );
			return 2;
		}
	}

	if ( fixedSeed && ( writePath != NULL || comparePath != NULL ) )
	{
		printf( "a baseline needs a base seed, not --seed\n" );
		return 2;
	}

	Soak soak = { 0 };
	soak.mutex = b3CreateMutex();
	soak.settings = settings;
	soak.groundMesh = CreateMeshDropMesh( settings.amplitude );
	soak.cylinder = CreateMeshDropCylinder();
	soak.baseSeed = seed;
	soak.fixedSeed = fixedSeed;
	soak.generationCount = generationCount;
	soak.shapeFilter = shapeFilter;

	for ( int i = 0; i < e_meshDropShapeCount; ++i )
	{
		soak.stats[i].maxDepth = -FLT_MAX;
		soak.stats[i].maxGeneration = INT32_MAX;
	}

	printf( "mesh soak: %d generations, %d threads, shape %s, %s seed %u\n", generationCount, threadCount,
			GetFilterName( shapeFilter ), fixedSeed ? "fixed" : "base", seed );
	fflush( stdout );

	uint64_t ticks = b3GetTicks();

	b3Thread* threads[MAX_THREADS];
	for ( int i = 0; i < threadCount; ++i )
	{
		threads[i] = b3CreateThread( SoakWorker, &soak, "soak" );
	}

	for ( int i = 0; i < threadCount; ++i )
	{
		b3JoinThread( threads[i] );
	}

	float seconds = 0.001f * b3GetMilliseconds( ticks );
	double averageSteps = soak.completedCount > 0 ? (double)soak.totalSteps / soak.completedCount : 0.0;
	float slop = B3_LINEAR_SLOP;

	printf( "\n%-9s %8s %8s %6s", "shape", "bodies", "tunnels", "awake" );
	for ( int i = 0; i < MESH_DROP_DEPTH_LEVEL_COUNT; ++i )
	{
		char label[32];
		snprintf( label, sizeof( label ), "> %g slop", g_meshDropDepthLevels[i] );
		printf( " %10s", label );
	}
	printf( "   deepest\n" );

	for ( int shape = 0; shape < e_meshDropShapeCount; ++shape )
	{
		const ShapeStats* stats = soak.stats + shape;
		if ( stats->generationCount == 0 )
		{
			continue;
		}

		printf( "%-9s %8d %8d %6d", g_meshDropShapeNames[shape], stats->generationCount * MESH_DROP_BODY_COUNT,
				stats->tunnelCount, stats->awakeCount );
		for ( int i = 0; i < MESH_DROP_DEPTH_LEVEL_COUNT; ++i )
		{
			printf( " %10d", stats->depthCounts[i] );
		}

		if ( stats->maxIndex >= 0 )
		{
			printf( "   %.1f slop, seed %u body (%d, %d) step %d\n", stats->maxDepth / slop, stats->maxSeed,
					stats->maxIndex / MESH_DROP_GRID_COUNT, stats->maxIndex % MESH_DROP_GRID_COUNT, stats->maxStep );
		}
		else
		{
			printf( "\n" );
		}
	}

	printf( "\nsteps to sleep: average %.0f, max %d, time %.1f s\n", averageSteps, soak.maxSteps, seconds );

	int reported = b3MinInt( soak.failureCount, MAX_REPORTED_FAILURES );
	for ( int i = 0; i < reported; ++i )
	{
		Failure* f = soak.failures + i;
		printf( "  %s %s seed %u body (%d, %d)\n", f->kind == e_tunnel ? "tunnel" : "awake", g_meshDropShapeNames[f->shape],
				f->seed, f->index / MESH_DROP_GRID_COUNT, f->index % MESH_DROP_GRID_COUNT );
	}

	int exitCode = soak.failureCount == 0 ? 0 : 1;

	if ( writePath != NULL )
	{
		WriteBaseline( &soak, writePath );
	}

	if ( comparePath != NULL )
	{
		int compareResult = CompareBaseline( &soak, comparePath );
		exitCode = b3MaxInt( exitCode, compareResult );
	}

	printf( exitCode == 0 ? "PASS\n" : "FAIL\n" );

	b3DestroyHull( soak.cylinder );
	b3DestroyMesh( soak.groundMesh );
	b3DestroyMutex( soak.mutex );

	return exitCode;
}

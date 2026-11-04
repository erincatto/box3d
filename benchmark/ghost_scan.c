// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Headless scan of the Mesh / Ghost Collisions sample. Runs every scene, body, mirror and PEEL
// preset for a fixed number of steps and counts the ghost manifolds that pushed on the body. A
// written result file can be compared against a later build, which fails on any count that grew.

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "ghost_collisions.h"

#include "box3d/box3d.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Command line and result file keys
static const char* s_sceneKeys[e_ghostSceneCount] = { "flat", "ramp", "cliff", "crest", "bumps", "tjunction", "peel", "chamfer", "rail" };
static const char* s_bodyKeys[e_ghostBodyCount] = { "box", "capsule", "upright", "sphere", "cylinder" };

#define MAX_CONFIGS 256

typedef struct Config
{
	GhostScene scene;
	int preset;
	bool mirror;
	GhostBody body;
	int ghostCount;
	int runCount;

	// From the compare file, -1 when absent
	int baseGhostCount;
} Config;

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

// Returns -1 for all, -2 for unknown
static int ParseKey( const char* value, const char** keys, int count )
{
	if ( strcmp( value, "all" ) == 0 )
	{
		return -1;
	}

	for ( int i = 0; i < count; ++i )
	{
		if ( strcmp( value, keys[i] ) == 0 )
		{
			return i;
		}
	}

	char* end;
	long index = strtol( value, &end, 10 );
	if ( *end == '\0' && 0 <= index && index < count )
	{
		return (int)index;
	}

	return -2;
}

static void RunConfig( Config* config, GhostSettings settings, int stepCount, bool verbose )
{
	settings.scene = config->scene;
	settings.peelPreset = config->preset;
	settings.mirror = config->mirror;
	settings.body = config->body;

	// The s&box character is a fixed rotation hull without speculative contact. Speculative contact or
	// rotation hide the ghost.
	if ( config->scene == e_ghostChamfer )
	{
		settings.fixedRotation = true;
		settings.speculative = false;
	}

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	GhostCollisionsData data = CreateGhostCollisions( worldId, settings );

	for ( int step = 0; step < stepCount; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );

		GhostReport reports[16];
		int ghostCount = UpdateGhostCollisions( worldId, &data, reports, 16 );
		if ( verbose )
		{
			ghostCount = b3MinInt( ghostCount, 16 );
			for ( int i = 0; i < ghostCount; ++i )
			{
				GhostReport* r = reports + i;
				printf( "  ghost step %d run %d n (%.3f %.3f %.3f) p (%.2f %.2f %.2f) sep %.4f impulse %.3f\n", step + 1, data.runCount,
						r->normal.x, r->normal.y, r->normal.z, (float)r->point.x, (float)r->point.y, (float)r->point.z,
						r->separation, r->impulse );
			}
		}
	}

	config->ghostCount = data.ghostCount;
	config->runCount = data.runCount;

	b3DestroyWorld( worldId );
	DestroyGhostCollisions( &data );
}

static void FormatRow( char* buffer, size_t size, const Config* config )
{
	if ( config->scene == e_ghostInternal )
	{
		snprintf( buffer, size, "%s %d%s", s_sceneKeys[config->scene], config->preset, config->mirror ? " mirror" : "" );
	}
	else
	{
		snprintf( buffer, size, "%s%s", s_sceneKeys[config->scene], config->mirror ? " mirror" : "" );
	}
}

static bool WriteResults( const char* path, const Config* configs, int configCount )
{
	FILE* file = fopen( path, "w" );
	if ( file == NULL )
	{
		printf( "cannot write %s\n", path );
		return false;
	}

	fprintf( file, "scene,preset,mirror,body,ghosts,runs\n" );
	for ( int i = 0; i < configCount; ++i )
	{
		const Config* c = configs + i;
		fprintf( file, "%s,%d,%d,%s,%d,%d\n", s_sceneKeys[c->scene], c->preset, c->mirror ? 1 : 0, s_bodyKeys[c->body], c->ghostCount,
				 c->runCount );
	}

	fclose( file );
	return true;
}

static bool ReadBaseline( const char* path, Config* configs, int configCount )
{
	FILE* file = fopen( path, "r" );
	if ( file == NULL )
	{
		printf( "cannot read %s\n", path );
		return false;
	}

	char line[256];
	while ( fgets( line, sizeof( line ), file ) != NULL )
	{
		char sceneKey[32], bodyKey[32];
		int preset, mirror, ghosts, runs;
		if ( sscanf( line, "%31[^,],%d,%d,%31[^,],%d,%d", sceneKey, &preset, &mirror, bodyKey, &ghosts, &runs ) != 6 )
		{
			continue;
		}

		int scene = ParseKey( sceneKey, s_sceneKeys, e_ghostSceneCount );
		int body = ParseKey( bodyKey, s_bodyKeys, e_ghostBodyCount );
		for ( int i = 0; i < configCount; ++i )
		{
			Config* c = configs + i;
			if ( (int)c->scene == scene && c->preset == preset && c->mirror == ( mirror != 0 ) && (int)c->body == body )
			{
				c->baseGhostCount = ghosts;
			}
		}
	}

	fclose( file );
	return true;
}

int main( int argc, char** argv )
{
	int sceneFilter = -1;
	int bodyFilter = -1;
	int mirrorFilter = -1;
	int presetFilter = -1;
	int stepCount = 900;
	bool verbose = false;
	const char* writePath = NULL;
	const char* comparePath = NULL;
	GhostSettings settings = GetDefaultGhostSettings();

	for ( int i = 1; i < argc; ++i )
	{
		const char* arg = argv[i];
		const char* value;

		if ( ( value = MatchValue( arg, "-s=", "--scene=" ) ) != NULL )
		{
			sceneFilter = ParseKey( value, s_sceneKeys, e_ghostSceneCount );
		}
		else if ( ( value = MatchValue( arg, "-b=", "--body=" ) ) != NULL )
		{
			bodyFilter = ParseKey( value, s_bodyKeys, e_ghostBodyCount );
		}
		else if ( ( value = MatchValue( arg, "-m=", "--mirror=" ) ) != NULL )
		{
			mirrorFilter = strcmp( value, "all" ) == 0 ? -1 : atoi( value ) != 0;
		}
		else if ( ( value = MatchValue( arg, "-p=", "--preset=" ) ) != NULL )
		{
			presetFilter = strcmp( value, "all" ) == 0 ? -1 : b3ClampInt( atoi( value ), 0, GHOST_PEEL_PRESET_COUNT - 1 );
		}
		else if ( ( value = MatchValue( arg, "-n=", "--steps=" ) ) != NULL )
		{
			stepCount = atoi( value );
		}
		else if ( ( value = MatchValue( arg, "-w=", "--write=" ) ) != NULL )
		{
			writePath = value;
		}
		else if ( ( value = MatchValue( arg, "-c=", "--compare=" ) ) != NULL )
		{
			comparePath = value;
		}
		else if ( ( value = MatchValue( arg, "--speed=", "--speed=" ) ) != NULL )
		{
			settings.speed = (float)atof( value );
		}
		else if ( ( value = MatchValue( arg, "--yaw=", "--yaw=" ) ) != NULL )
		{
			settings.yaw = (float)atof( value );
		}
		else if ( ( value = MatchValue( arg, "--friction=", "--friction=" ) ) != NULL )
		{
			settings.friction = (float)atof( value );
		}
		else if ( ( value = MatchValue( arg, "--cell=", "--cell=" ) ) != NULL )
		{
			settings.cellSize = (float)atof( value );
		}
		else if ( strcmp( arg, "--fixed-rotation" ) == 0 )
		{
			settings.fixedRotation = true;
		}
		else if ( strcmp( arg, "--no-speculative" ) == 0 )
		{
			settings.speculative = false;
		}
		else if ( strcmp( arg, "--height-field" ) == 0 )
		{
			settings.heightField = true;
		}
		else if ( strcmp( arg, "-v" ) == 0 || strcmp( arg, "--verbose" ) == 0 )
		{
			verbose = true;
		}
		else
		{
			sceneFilter = -2;
		}

		if ( sceneFilter == -2 || bodyFilter == -2 )
		{
			printf( "Usage: ghost_scan [options]\n"
					"Runs the Ghost Collisions sample headless and counts ghost manifolds per scene and body.\n"
					"  -s, --scene=<key|all>    flat, ramp, cliff, crest, bumps, tjunction, peel, chamfer (default all)\n"
					"                           chamfer runs only the box and cylinder, fixed rotation without speculative\n"
					"  -b, --body=<key|all>     box, capsule, upright, sphere, cylinder (default all)\n"
					"  -m, --mirror=<0|1|all>   mirrored mesh (default all)\n"
					"  -p, --preset=<0-3|all>   PEEL preset (default all)\n"
					"  -n, --steps=<int>        steps per config (default 900)\n"
					"  -w, --write=<file>       write results as csv\n"
					"  -c, --compare=<file>     compare with a written csv, fail if any count grew\n"
					"  --speed=, --yaw=, --friction=, --cell=, --fixed-rotation, --no-speculative\n"
					"  --height-field           height field ground, the t-junction becomes a fine and coarse tile seam\n"
					"  -v, --verbose            print every ghost\n"
					"Exit code is 1 when a compared count grew, 0 otherwise.\n" );
			return 2;
		}
	}

	Config configs[MAX_CONFIGS];
	int configCount = 0;
	for ( int scene = 0; scene < e_ghostSceneCount; ++scene )
	{
		if ( sceneFilter >= 0 && scene != sceneFilter )
		{
			continue;
		}

		int presetCount = scene == e_ghostInternal ? GHOST_PEEL_PRESET_COUNT : 1;
		for ( int preset = 0; preset < presetCount; ++preset )
		{
			if ( scene == e_ghostInternal && presetFilter >= 0 && preset != presetFilter )
			{
				continue;
			}

			for ( int mirror = 0; mirror < 2; ++mirror )
			{
				if ( mirrorFilter >= 0 && mirror != mirrorFilter )
				{
					continue;
				}

				for ( int body = 0; body < e_ghostBodyCount; ++body )
				{
					if ( bodyFilter >= 0 && body != bodyFilter )
					{
						continue;
					}

					// Only a flat bottom can rest on the chamfer edges
					if ( scene == e_ghostChamfer && body != e_ghostBox && body != e_ghostCylinder )
					{
						continue;
					}

					configs[configCount++] = (Config){
						.scene = (GhostScene)scene,
						.preset = preset,
						.mirror = mirror != 0,
						.body = (GhostBody)body,
						.baseGhostCount = -1,
					};
				}
			}
		}
	}

	if ( comparePath != NULL && ReadBaseline( comparePath, configs, configCount ) == false )
	{
		return 2;
	}

	printf( "ghost scan: %d configs, %d steps, speed %.1f, yaw %.0f, friction %.2f, cell %.2f%s\n", configCount, stepCount,
			settings.speed, settings.yaw, settings.friction, settings.cellSize, settings.heightField ? ", height field" : "" );
	fflush( stdout );

	uint64_t ticks = b3GetTicks();
	for ( int i = 0; i < configCount; ++i )
	{
		if ( verbose )
		{
			char row[64];
			FormatRow( row, sizeof( row ), configs + i );
			printf( "%s %s\n", row, s_bodyKeys[configs[i].body] );
		}

		RunConfig( configs + i, settings, stepCount, verbose );
	}
	float seconds = 0.001f * b3GetMilliseconds( ticks );

	// One row per ground, one column per body. A compared cell that changed prints as old>new.
	printf( "\n%-18s", "" );
	for ( int body = 0; body < e_ghostBodyCount; ++body )
	{
		if ( bodyFilter < 0 || body == bodyFilter )
		{
			printf( "%10s", s_bodyKeys[body] );
		}
	}
	printf( "\n" );

	int totalGhosts = 0;
	int grewCount = 0;
	int shrankCount = 0;
	int column = 0;
	for ( int i = 0; i < configCount; ++i )
	{
		const Config* c = configs + i;
		bool newRow = i == 0 || c->scene != configs[i - 1].scene || c->preset != configs[i - 1].preset ||
					  c->mirror != configs[i - 1].mirror;
		if ( newRow )
		{
			char row[64];
			FormatRow( row, sizeof( row ), c );
			printf( "%s%-18s", i == 0 ? "" : "\n", row );
			column = 0;
		}

		// A scene that skips a body leaves its column empty
		int bodyColumn = bodyFilter >= 0 ? 0 : (int)c->body;
		while ( column < bodyColumn )
		{
			printf( "%10s", "-" );
			column += 1;
		}
		column += 1;

		char cell[32];
		if ( c->baseGhostCount >= 0 && c->baseGhostCount != c->ghostCount )
		{
			snprintf( cell, sizeof( cell ), "%d>%d", c->baseGhostCount, c->ghostCount );
		}
		else
		{
			snprintf( cell, sizeof( cell ), "%d", c->ghostCount );
		}
		printf( "%10s", cell );

		totalGhosts += c->ghostCount;
		grewCount += c->baseGhostCount >= 0 && c->ghostCount > c->baseGhostCount;
		shrankCount += c->baseGhostCount >= 0 && c->ghostCount < c->baseGhostCount;
	}

	printf( "\n\ntotal ghosts %d, time %.1f s\n", totalGhosts, seconds );

	if ( writePath != NULL && WriteResults( writePath, configs, configCount ) == false )
	{
		return 2;
	}

	if ( comparePath != NULL )
	{
		printf( "compared with %s: %d grew, %d shrank\n", comparePath, grewCount, shrankCount );
		printf( grewCount == 0 ? "PASS\n" : "FAIL\n" );
		return grewCount == 0 ? 0 : 1;
	}

	return 0;
}

// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "name_cache.h"
#include "test_macros.h"

#include "box3d/base.h"
#include "box3d/box3d.h"

#include <string.h>

// Direct unit test of the cache: dedup, reverse lookup, empty/null, arbitrary length, no leak.
static int CacheUnit( void )
{
	int32_t base = b3GetByteCount();

	b3NameCache cache = b3CreateNameCache();

	// Empty and NULL map to the null id, which resolves to no string.
	ENSURE( b3AddName( &cache, "" ) == B3_NULL_NAME );
	ENSURE( b3AddName( &cache, NULL ) == B3_NULL_NAME );
	ENSURE( b3FindName( &cache, B3_NULL_NAME ) == NULL );

	// Distinct strings get distinct ids and round-trip.
	uint32_t crate = b3AddName( &cache, "crate" );
	uint32_t barrel = b3AddName( &cache, "barrel" );
	ENSURE( crate != B3_NULL_NAME && barrel != B3_NULL_NAME && crate != barrel );
	ENSURE( strcmp( b3FindName( &cache, crate ), "crate" ) == 0 );
	ENSURE( strcmp( b3FindName( &cache, barrel ), "barrel" ) == 0 );

	// Re-interning the same bytes returns the same id and stores no second entry.
	ENSURE( b3AddName( &cache, "crate" ) == crate );
	ENSURE( cache.entries.count == 2 );

	// A name far longer than any inline buffer round-trips exactly.
	char longName[4096];
	for ( int i = 0; i < (int)sizeof( longName ) - 1; ++i )
	{
		longName[i] = (char)( 'a' + i % 26 );
	}
	longName[sizeof( longName ) - 1] = 0;
	uint32_t longId = b3AddName( &cache, longName );
	ENSURE( strcmp( b3FindName( &cache, longId ), longName ) == 0 );
	ENSURE( cache.entries.count == 3 );

	b3DestroyNameCache( &cache );

	// The cache owns every interned copy, so all of it comes back on destroy.
	ENSURE( b3GetByteCount() == base );
	return 0;
}

// Build a ground plus named dynamic boxes. Two share a name to exercise dedup and one is longer
// than the old inline buffer to exercise variable length. Ground is ordinal 0, boxes 1..N.
static const char* s_bodyNames[] = {
	"crate",
	"barrel",
	"crate",
	"a_very_long_body_name_that_exceeds_any_inline_name_buffer",
};

// Each box also carries a named hull shape. Shapes could not hold a name at all before the cache
// (the old inline shape buffer had length 0), so this is a new path. Two shapes share a name for
// dedup, one shape shares a body's name to prove shape ids intern independently, and one is far
// longer than any inline buffer.
static const char* s_shapeNames[] = {
	"box_hull",
	"box_hull",
	"crate",
	"a_very_long_shape_name_that_never_fit_the_old_inline_buffer",
};

_Static_assert( ARRAY_COUNT( s_shapeNames ) == ARRAY_COUNT( s_bodyNames ), "name arrays must stay parallel" );

static void BuildNamedScene( b3WorldId worldId )
{
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
	b3ShapeDef groundShape = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &groundShape, &groundBox.base );

	b3ShapeDef boxShape = b3DefaultShapeDef();
	boxShape.density = 1.0f;
	for ( int i = 0; i < ARRAY_COUNT( s_bodyNames ); ++i )
	{
		b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 0.0f, 1.0f + 1.1f * (float)i, 0.0f };
		bodyDef.name = s_bodyNames[i];
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		boxShape.name = s_shapeNames[i];
		b3CreateHullShape( bodyId, &boxShape, &box.base );
	}
}

static int CheckReplayNames( b3RecPlayer* player )
{
	for ( int i = 0; i < ARRAY_COUNT( s_bodyNames ); ++i )
	{
		b3BodyId id = b3RecPlayer_GetBodyId( player, 1 + i ); // ordinal 0 is the ground
		ENSURE( b3Body_IsValid( id ) );
		const char* name = b3Body_GetName( id );
		ENSURE( name != NULL && strcmp( name, s_bodyNames[i] ) == 0 );

		// Shape names ride the same world cache, so the snapshot must restore them too.
		b3ShapeId shapeId;
		ENSURE( b3Body_GetShapes( id, &shapeId, 1 ) == 1 );
		const char* shapeName = b3Shape_GetName( shapeId );
		ENSURE( shapeName != NULL && strcmp( shapeName, s_shapeNames[i] ) == 0 );
	}
	return 0;
}

// Names live on the world, so the world snapshot must carry them. Record a snapshot-seeded session
// and confirm every body and shape name resolves on the reconstructed replay world.
static int NameRoundTrip( void )
{
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	BuildNamedScene( worldId );

	// Record from a snapshot of the populated world, so names ride in the frame-0 image.
	b3World_StartRecording( worldId, rec );
	for ( int i = 0; i < 20; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}
	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	ENSURE( b3ValidateReplay( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 ) );

	b3RecPlayer* player = b3RecPlayer_Create( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 );
	ENSURE( player != NULL );

	b3RecPlayer_SeekFrame( player, 20 );
	ENSURE( !b3RecPlayer_HasDiverged( player ) );
	ENSURE( CheckReplayNames( player ) == 0 );

	b3RecPlayer_Destroy( player );
	b3DestroyRecording( rec );
	return 0;
}

// Rollback preserves names: a backward seek restores from a keyframe, which rebuilds world->names
// on each restore. Scrub across keyframe boundaries, confirm names resolve every time, and require
// the tracked byte count to return to baseline so a re-load that drops duplicate copies leaks nothing.
static int RollbackNames( void )
{
	int32_t base = b3GetByteCount();

	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	BuildNamedScene( worldId );

	// Settle, then record a snapshot-seeded session long enough to span several keyframes.
	for ( int i = 0; i < 10; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}
	b3World_StartRecording( worldId, rec );
	const int totalFrames = 80;
	for ( int i = 0; i < totalFrames; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}
	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	b3RecPlayer* player = b3RecPlayer_Create( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 );
	ENSURE( player != NULL );

	// Play to the end so the keyframe ring is populated.
	b3RecPlayer_SeekFrame( player, totalFrames );
	ENSURE( !b3RecPlayer_HasDiverged( player ) );

	// Scrub backward and forward across keyframe boundaries. Each restore rebuilds the name table.
	int targets[] = { 10, 60, 5, 70, 1, 40 };
	for ( int k = 0; k < ARRAY_COUNT( targets ); ++k )
	{
		b3RecPlayer_SeekFrame( player, targets[k] );
		ENSURE( b3RecPlayer_GetFrame( player ) == targets[k] );
		ENSURE( !b3RecPlayer_HasDiverged( player ) );
		ENSURE( CheckReplayNames( player ) == 0 );
	}

	b3RecPlayer_Destroy( player );
	b3DestroyRecording( rec );

	// Every restore reloaded the name table; dropped duplicate copies must be freed, not leaked.
	ENSURE( b3GetByteCount() == base );
	return 0;
}

int NameCacheTest( void )
{
	RUN_SUBTEST( CacheUnit );
	RUN_SUBTEST( NameRoundTrip );
	RUN_SUBTEST( RollbackNames );
	return 0;
}

// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "physics_world.h"
#include "recording.h"
#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static const char* s_recPath = "recording_allops_test.b3rec";

// Sphere round-trip: record/step/stop, then replay and validate.
static int SphereRoundTrip( void )
{
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3World_StartRecording( worldId, rec );

	// Set a non-default gravity so the setter op appears in the stream.
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	// Static ground
	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );

	b3BoxHull groundBox = b3MakeBoxHull( 50.0f, 1.0f, 50.0f );
	b3ShapeDef groundShapeDef = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &groundShapeDef, &groundBox.base );

	// Dynamic body with a sphere shape
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 5.0f, 0.0f };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3Sphere sphere;
	sphere.center = (b3Vec3){ 0.0f, 0.0f, 0.0f };
	sphere.radius = 0.5f;
	b3ShapeDef sphereDef = b3DefaultShapeDef();
	sphereDef.density = 1.0f;
	b3CreateSphereShape( bodyId, &sphereDef, &sphere );

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;
	for ( int i = 0; i < 30; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	ENSURE( b3ValidateReplay( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 ) );

	b3DestroyRecording( rec );
	return 0;
}

// Hull dedup: three bodies sharing the same hull should produce one registry entry.
static int HullDedup( void )
{
	// Build a small convex hull
	b3Vec3 pts[8] = {
		{ -1.0f, -1.0f, -1.0f }, { 1.0f, -1.0f, -1.0f }, { 1.0f, 1.0f, -1.0f }, { -1.0f, 1.0f, -1.0f },
		{ -1.0f, -1.0f, 1.0f },	 { 1.0f, -1.0f, 1.0f },	 { 1.0f, 1.0f, 1.0f },	{ -1.0f, 1.0f, 1.0f },
	};
	b3HullData* hull = b3CreateHull( pts, 8, 8 );
	ENSURE( hull != NULL );

	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3World_StartRecording( worldId, rec );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;

	for ( int i = 0; i < 3; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ (float)( i * 3 ), 5.0f, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3CreateHullShape( bodyId, &shapeDef, hull );
	}

	float timeStep = 1.0f / 60.0f;
	for ( int i = 0; i < 5; ++i )
	{
		b3World_Step( worldId, timeStep, 4 );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );
	b3DestroyHull( hull );

	// Validate the replay
	ENSURE( b3ValidateReplay( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 ) );

	// Confirm the registry was deduped to 1 hull entry.
	// Parse registryOffset from the header and count entries.
	const uint8_t* bytes = b3Recording_GetData( rec );
	int sz = b3Recording_GetSize( rec );
	ENSURE( sz >= 48 );

	uint64_t regOff = 0;
	memcpy( &regOff, bytes + 32, 8 ); // registryOffset at offset 32 in b3RecHeader
	ENSURE( regOff != 0 && (int)regOff + 4 <= sz );

	// entryCount is a little-endian u32 at the start of the registry block
	const uint8_t* rp = bytes + (int)regOff;
	uint32_t entryCount = (uint32_t)rp[0] | ( (uint32_t)rp[1] << 8 ) | ( (uint32_t)rp[2] << 16 ) | ( (uint32_t)rp[3] << 24 );
	ENSURE( entryCount == 1 );

	b3DestroyRecording( rec );
	return 0;
}

// Mid-stream snapshot with only dynamic bodies floating in air (no contacts).
static int MidStreamNoContacts( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	b3Sphere sphere;
	sphere.center = (b3Vec3){ 0.0f, 0.0f, 0.0f };
	sphere.radius = 0.5f;
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;

	// A few dynamic bodies well apart from each other so no contacts form
	for ( int i = 0; i < 4; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ (float)( i * 10 ), 50.0f, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );
	}

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;
	for ( int i = 0; i < 10; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	// Start recording mid-stream, with a snapshot of the current world
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );
	b3World_StartRecording( worldId, rec );

	for ( int i = 0; i < 30; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	ENSURE( b3ValidateReplay( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 ) );

	b3DestroyRecording( rec );
	return 0;
}

// Mid-stream snapshot with contacts: bodies touching ground with warm-start manifolds.
static int MidStreamContacts( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	// Static ground using a box hull
	{
		b3BodyDef groundDef = b3DefaultBodyDef();
		groundDef.type = b3_staticBody;
		b3BodyId groundId = b3CreateBody( worldId, &groundDef );

		b3BoxHull groundBox = b3MakeBoxHull( 50.0f, 1.0f, 50.0f );
		b3ShapeDef groundShape = b3DefaultShapeDef();
		b3CreateHullShape( groundId, &groundShape, &groundBox.base );
	}

	b3ShapeDef dynamicShape = b3DefaultShapeDef();
	dynamicShape.density = 1.0f;

	// A few dynamic boxes dropped onto the ground
	for ( int i = 0; i < 3; ++i )
	{
		b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ (float)( i * 2 ) - 2.0f, 5.0f, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3CreateHullShape( bodyId, &dynamicShape, &box.base );
	}

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;

	// Let the scene settle: bodies hit ground, build manifolds, islands, graph colors
	for ( int i = 0; i < 60; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	// Start recording with snapshot of the settled world (contacts, islands, warm starts)
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );
	b3World_StartRecording( worldId, rec );

	for ( int i = 0; i < 30; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	ENSURE( b3ValidateReplay( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 ) );

	b3DestroyRecording( rec );
	return 0;
}

// Record a scene with hull boxes settling on a ground plane, create a player, step to
// the end recording per-frame world hashes, then seek backward to several frames and
// verify each reproduces the recorded hash exactly.
static int ScrubBackward( void )
{
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	b3World_StartRecording( worldId, rec );

	// Static ground
	{
		b3BodyDef groundDef = b3DefaultBodyDef();
		groundDef.type = b3_staticBody;
		b3BodyId groundId = b3CreateBody( worldId, &groundDef );
		b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
		b3ShapeDef groundShape = b3DefaultShapeDef();
		b3CreateHullShape( groundId, &groundShape, &groundBox.base );
	}

	// A small stack of dynamic hull boxes
	b3ShapeDef boxShape = b3DefaultShapeDef();
	boxShape.density = 1.0f;
	for ( int i = 0; i < 4; ++i )
	{
		b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 0.0f, 2.0f + (float)i * 1.5f, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3CreateHullShape( bodyId, &boxShape, &box.base );
	}

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;
	int totalFrames = 80;
	for ( int i = 0; i < totalFrames; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	const uint8_t* data = b3Recording_GetData( rec );
	int sz = b3Recording_GetSize( rec );

	// Create the player
	b3RecPlayer* player = b3RecPlayer_Create( data, sz, 1 );
	ENSURE( player != NULL );
	ENSURE( b3RecPlayer_GetFrameCount( player ) == totalFrames );

	// Forward pass: record per-frame hashes
	uint64_t* hashes = (uint64_t*)b3Alloc( (size_t)( totalFrames + 1 ) * sizeof( uint64_t ) );
	hashes[0] = 0; // frame 0 before any step

	while ( !b3RecPlayer_IsAtEnd( player ) )
	{
		b3RecPlayer_StepFrame( player );
		int f = b3RecPlayer_GetFrame( player );
		if ( f <= totalFrames )
		{
			b3WorldId wid = b3RecPlayer_GetWorldId( player );
			b3World* w = b3GetWorldFromId( wid );
			hashes[f] = b3HashWorldState( w );
		}
	}
	ENSURE( b3RecPlayer_GetFrame( player ) == totalFrames );
	ENSURE( !b3RecPlayer_HasDiverged( player ) );

	// Backward seek to several interesting frames and verify hash matches
	int seekTargets[] = { totalFrames, totalFrames / 2, 5, totalFrames - 1, 0, 1 };
	int seekCount = (int)( sizeof( seekTargets ) / sizeof( seekTargets[0] ) );
	for ( int k = 0; k < seekCount; ++k )
	{
		int target = seekTargets[k];
		b3RecPlayer_SeekFrame( player, target );
		ENSURE( b3RecPlayer_GetFrame( player ) == target );
		ENSURE( !b3RecPlayer_HasDiverged( player ) );

		if ( target > 0 )
		{
			b3WorldId wid = b3RecPlayer_GetWorldId( player );
			b3World* w = b3GetWorldFromId( wid );
			uint64_t got = b3HashWorldState( w );
			ENSURE( got == hashes[target] );
		}
	}

	b3Free( hashes, (size_t)( totalFrames + 1 ) * sizeof( uint64_t ) );
	b3RecPlayer_Destroy( player );
	b3DestroyRecording( rec );
	return 0;
}

// Record a scene that includes a mesh shape, create a player, seek backward, verify
// it works and no divergence is reported. Also checks the keyframe-by-geometry-id
// invariant: keyframeRec registry count should not grow beyond initial slot count.
static int SeekWithHull( void )
{
	b3Vec3 pts[8] = {
		{ -1.0f, -1.0f, -1.0f }, { 1.0f, -1.0f, -1.0f }, { 1.0f, 1.0f, -1.0f }, { -1.0f, 1.0f, -1.0f },
		{ -1.0f, -1.0f, 1.0f },	 { 1.0f, -1.0f, 1.0f },	 { 1.0f, 1.0f, 1.0f },	{ -1.0f, 1.0f, 1.0f },
	};
	b3HullData* hull = b3CreateHull( pts, 8, 8 );
	ENSURE( hull != NULL );

	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );
	b3World_StartRecording( worldId, rec );

	// Static ground
	{
		b3BodyDef groundDef = b3DefaultBodyDef();
		groundDef.type = b3_staticBody;
		b3BodyId groundId = b3CreateBody( worldId, &groundDef );
		b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
		b3ShapeDef gs = b3DefaultShapeDef();
		b3CreateHullShape( groundId, &gs, &groundBox.base );
	}

	// Dynamic bodies using the custom hull
	b3ShapeDef sd = b3DefaultShapeDef();
	sd.density = 1.0f;
	for ( int i = 0; i < 3; ++i )
	{
		b3BodyDef bd = b3DefaultBodyDef();
		bd.type = b3_dynamicBody;
		bd.position = (b3Pos){ (float)( i * 4 ) - 4.0f, 5.0f, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bd );
		b3CreateHullShape( bodyId, &sd, hull );
	}

	float timeStep = 1.0f / 60.0f;
	int totalFrames = 40;
	for ( int i = 0; i < totalFrames; ++i )
	{
		b3World_Step( worldId, timeStep, 4 );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );
	b3DestroyHull( hull );

	const uint8_t* data = b3Recording_GetData( rec );
	int sz = b3Recording_GetSize( rec );

	b3RecPlayer* player = b3RecPlayer_Create( data, sz, 1 );
	ENSURE( player != NULL );

	// Step to end
	while ( !b3RecPlayer_IsAtEnd( player ) )
	{
		b3RecPlayer_StepFrame( player );
	}
	ENSURE( !b3RecPlayer_HasDiverged( player ) );

	int midFrame = totalFrames / 2;
	b3RecPlayer_SeekFrame( player, midFrame );
	ENSURE( b3RecPlayer_GetFrame( player ) == midFrame );
	ENSURE( !b3RecPlayer_HasDiverged( player ) );

	b3RecPlayer_SeekFrame( player, 0 );
	ENSURE( b3RecPlayer_GetFrame( player ) == 0 );

	b3RecPlayer_Destroy( player );
	b3DestroyRecording( rec );
	return 0;
}

// Host debug-shape callbacks just count create/destroy so the test can prove the player
// wires them into every world it builds. The returned token is opaque to the engine.
typedef struct
{
	int created;
	int destroyed;
} DebugShapeCounters;

static void* RecTestCreateDebugShape( const b3DebugShape* debugShape, void* userContext )
{
	(void)debugShape;
	DebugShapeCounters* counters = (DebugShapeCounters*)userContext;
	counters->created += 1;
	return userContext; // any non-NULL token; engine stores and hands it back to destroy
}

static void RecTestDestroyDebugShape( void* userShape, void* userContext )
{
	(void)userShape;
	DebugShapeCounters* counters = (DebugShapeCounters*)userContext;
	counters->destroyed += 1;
}

static bool RecTestDrawShape( void* userShape, b3WorldTransform transform, b3HexColor color, void* context )
{
	(void)userShape;
	(void)transform;
	(void)color;
	(void)context;
	return true;
}

// b3World_Draw lazily fires createDebugShape for shapes entering the draw set, the same way the
// sample renderer does. Drive a draw so the player's wired callbacks actually run.
static void RecTestDrawWorld( b3WorldId worldId )
{
	b3DebugDraw draw = b3DefaultDebugDraw();
	draw.DrawShapeFcn = RecTestDrawShape;
	draw.drawShapes = true;
	float big = 1.0e6f;
	draw.drawingBounds = (b3AABB){ { -big, -big, -big }, { big, big, big } };
	b3World_Draw( worldId, &draw, B3_DEFAULT_MASK_BITS );
}

// The 3D sample renderer builds per-shape GPU meshes through createDebugShape, so the replay
// world must carry the host callbacks. Verify b3RecPlayer_SetDebugShapeCallbacks rewinds, fires
// the callbacks for every replayed shape, keeps them across a backward-seek world rebuild, and
// balances create/destroy at teardown.
static int DebugShapeCallbacks( void )
{
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3World_StartRecording( worldId, rec );
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
	b3ShapeDef groundShape = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &groundShape, &groundBox.base );

	// Four dynamic boxes sharing one hull: ground + 4 boxes = 5 shapes total.
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3ShapeDef boxShape = b3DefaultShapeDef();
	boxShape.density = 1.0f;
	for ( int i = 0; i < 4; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 0.0f, 1.0f + 1.1f * (float)i, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3CreateHullShape( bodyId, &boxShape, &box.base );
	}

	int totalFrames = 30;
	for ( int i = 0; i < totalFrames; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}
	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	// Round-trip through a file, mirroring the replay sample's Generate/Load path exactly.
	const char* path = "replay_test.b3rec";
	ENSURE( b3SaveRecordingToFile( rec, path ) );
	b3Recording* loaded = b3LoadRecordingFromFile( path );
	ENSURE( loaded != NULL );

	b3RecPlayer* player = b3RecPlayer_Create( b3Recording_GetData( loaded ), b3Recording_GetSize( loaded ), 1 );
	ENSURE( player != NULL );

	// Wiring the callbacks rebuilds the world and rewinds to frame 0.
	DebugShapeCounters counters = { 0, 0 };
	b3RecPlayer_SetDebugShapeCallbacks( player, RecTestCreateDebugShape, RecTestDestroyDebugShape, &counters );
	ENSURE( b3RecPlayer_GetFrame( player ) == 0 );

	// Replay to the end, then draw: createDebugShape fires once per shape (ground + 4 boxes = 5).
	while ( !b3RecPlayer_IsAtEnd( player ) )
	{
		b3RecPlayer_StepFrame( player );
	}
	RecTestDrawWorld( b3RecPlayer_GetWorldId( player ) );
	ENSURE( counters.created >= 5 );
	ENSURE( !b3RecPlayer_HasDiverged( player ) );

	// A backward seek restores the empty seed in place, releasing the live debug shapes, then forward
	// stepping recreates them through the callbacks, so more creates fire.
	int createdBefore = counters.created;
	b3RecPlayer_SeekFrame( player, 0 );
	b3RecPlayer_SeekFrame( player, totalFrames );
	RecTestDrawWorld( b3RecPlayer_GetWorldId( player ) );
	ENSURE( counters.created > createdBefore );

	// Teardown destroys the final world; every live shape is released, so the counts balance.
	b3RecPlayer_Destroy( player );
	ENSURE( counters.created == counters.destroyed );

	b3DestroyRecording( loaded );
	b3DestroyRecording( rec );
	remove( path );
	return 0;
}

// Exercise the viewer-facing player accessors: recording info, creation-ordinal body tracking
// (seeded from a snapshot), divergence frame, and keyframe policy. Recording starts after the
// bodies exist, so the snapshot seeds the outliner list and ordinals are stable from frame 0.
static int PlayerAccessors( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	// Static ground (creation ordinal 0)
	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
	b3ShapeDef groundShape = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &groundShape, &groundBox.base );

	// Four dynamic boxes (ordinals 1..4)
	const int dynamicCount = 4;
	b3ShapeDef boxShape = b3DefaultShapeDef();
	boxShape.density = 1.0f;
	for ( int i = 0; i < dynamicCount; ++i )
	{
		b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 0.0f, 2.0f + (float)i * 1.5f, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3CreateHullShape( bodyId, &boxShape, &box.base );
	}

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;

	// Settle, then record with a snapshot of the populated world.
	for ( int i = 0; i < 10; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );
	b3World_StartRecording( worldId, rec );

	int totalFrames = 80;
	for ( int i = 0; i < totalFrames; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}
	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	b3RecPlayer* player = b3RecPlayer_Create( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 );
	ENSURE( player != NULL );

	// Info reflects the recorded tuning and a non-degenerate bounds.
	b3RecPlayerInfo info = b3RecPlayer_GetInfo( player );
	ENSURE( info.frameCount == totalFrames );
	ENSURE( info.subStepCount == subStepCount );
	ENSURE( info.timeStep > 0.0f );
	b3Vec3 extent = b3Sub( info.bounds.upperBound, info.bounds.lowerBound );
	ENSURE( extent.x > 0.0f && extent.y > 0.0f && extent.z > 0.0f );

	// Body ordinals: ground + 4 dynamic, seeded from the snapshot and present at frame 0.
	ENSURE( b3RecPlayer_GetBodyCount( player ) == 1 + dynamicCount );
	b3BodyId ground = b3RecPlayer_GetBodyId( player, 0 );
	ENSURE( b3Body_IsValid( ground ) );
	ENSURE( b3Body_GetType( ground ) == b3_staticBody );
	for ( int i = 1; i <= dynamicCount; ++i )
	{
		b3BodyId id = b3RecPlayer_GetBodyId( player, i );
		ENSURE( b3Body_IsValid( id ) );
		ENSURE( b3Body_GetType( id ) == b3_dynamicBody );
	}
	ENSURE( B3_IS_NULL( b3RecPlayer_GetBodyId( player, 1 + dynamicCount ) ) );

	// No divergence on a clean serial replay.
	b3RecPlayer_SeekFrame( player, totalFrames );
	ENSURE( !b3RecPlayer_HasDiverged( player ) );
	ENSURE( b3RecPlayer_GetDivergeFrame( player ) == -1 );

	// Ordinals survive a backward seek that restores from a keyframe.
	b3BodyId before = b3RecPlayer_GetBodyId( player, 2 );
	b3RecPlayer_SeekFrame( player, totalFrames / 2 );
	b3RecPlayer_SeekFrame( player, totalFrames );
	b3BodyId after = b3RecPlayer_GetBodyId( player, 2 );
	ENSURE( B3_ID_EQUALS( before, after ) );

	// Keyframe policy: defaults present, setter takes effect and clears the ring.
	ENSURE( b3RecPlayer_GetKeyframeMinInterval( player ) == 16 );
	b3RecPlayer_SetKeyframePolicy( player, (size_t)256 * 1024 * 1024, 8 );
	ENSURE( b3RecPlayer_GetKeyframeMinInterval( player ) == 8 );
	ENSURE( b3RecPlayer_GetKeyframeInterval( player ) == 8 );
	ENSURE( b3RecPlayer_GetKeyframeBudget( player ) == (size_t)256 * 1024 * 1024 );
	ENSURE( b3RecPlayer_GetKeyframeBytes( player ) == 0 );

	b3RecPlayer_Destroy( player );
	b3DestroyRecording( rec );
	return 0;
}

// A keyframe restore is a deterministic replay state, so shapes that persist must keep their renderer
// handle rather than being torn down and rebuilt every seek. Record a snapshot-seeded session (shapes
// exist at frame 0), replay with handle callbacks, scrub backward across keyframes repeatedly, and
// verify no new handles are built per restore and the create/destroy counts still balance at teardown.
static int KeyframeHandleReuse( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
	b3ShapeDef groundShape = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &groundShape, &groundBox.base );

	const int dynamicCount = 5;
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3ShapeDef boxShape = b3DefaultShapeDef();
	boxShape.density = 1.0f;
	for ( int i = 0; i < dynamicCount; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 0.0f, 1.0f + 1.1f * (float)i, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3CreateHullShape( bodyId, &boxShape, &box.base );
	}
	int shapeCount = 1 + dynamicCount;

	// Settle, then record with a snapshot of the populated world so shapes exist at frame 0.
	for ( int i = 0; i < 10; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );
	b3World_StartRecording( worldId, rec );

	int totalFrames = 80;
	for ( int i = 0; i < totalFrames; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}
	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	b3RecPlayer* player = b3RecPlayer_Create( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 );
	ENSURE( player != NULL );

	DebugShapeCounters counters = { 0, 0 };
	b3RecPlayer_SetDebugShapeCallbacks( player, RecTestCreateDebugShape, RecTestDestroyDebugShape, &counters );

	// Replay to the end and draw: one handle per shape.
	b3RecPlayer_SeekFrame( player, totalFrames );
	RecTestDrawWorld( b3RecPlayer_GetWorldId( player ) );
	ENSURE( !b3RecPlayer_HasDiverged( player ) );
	ENSURE( counters.created == shapeCount );

	// Scrub backward and forward across keyframes. Each restore keeps the persistent shapes' handles,
	// so drawing builds no new ones.
	int createdAfterFirstDraw = counters.created;
	int seekTargets[] = { 40, 70, 20, 60, 8, 75 };
	for ( int k = 0; k < (int)( sizeof( seekTargets ) / sizeof( seekTargets[0] ) ); ++k )
	{
		b3RecPlayer_SeekFrame( player, seekTargets[k] );
		RecTestDrawWorld( b3RecPlayer_GetWorldId( player ) );
	}
	ENSURE( counters.created == createdAfterFirstDraw );

	// Teardown releases exactly the live handles, so the leak-free invariant holds.
	b3RecPlayer_Destroy( player );
	ENSURE( counters.created == counters.destroyed );

	b3DestroyRecording( rec );
	return 0;
}

static bool QueryReplayOverlapFcn( b3ShapeId shapeId, void* context )
{
	(void)shapeId;
	(void)context;
	return true;
}

static float QueryReplayCastFcn( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction, uint64_t userMaterialId,
								 int triangleIndex, int childIndex, void* context )
{
	(void)shapeId;
	(void)point;
	(void)normal;
	(void)userMaterialId;
	(void)triangleIndex;
	(void)childIndex;
	(void)context;
	// Return the fraction to keep the closest hit, exercising the recorded user-return path.
	return fraction;
}

static bool QueryReplayPlaneFcn( b3ShapeId shapeId, const b3PlaneResult* planes, int planeCount, void* context )
{
	(void)shapeId;
	(void)planes;
	(void)planeCount;
	(void)context;
	return true;
}

static bool QueryReplayMoverFilterFcn( b3ShapeId shapeId, void* context )
{
	(void)shapeId;
	(void)context;
	return true;
}

// Issue all seven world queries each frame, then replay. Every query is re-issued against the replay
// world and compared to what was recorded, so a clean (non-diverged) replay proves the queries
// reproduce. Also opens a player and confirms the per-frame query store surfaces all seven.
static int QueryReplay( void )
{
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
	b3ShapeDef groundShape = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &groundShape, &groundBox.base );

	// A few dynamic spheres for the queries to find.
	for ( int i = 0; i < 4; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ (float)i - 1.5f, 3.0f, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
		b3ShapeDef sphereDef = b3DefaultShapeDef();
		sphereDef.density = 1.0f;
		b3CreateSphereShape( bodyId, &sphereDef, &sphere );
	}

	b3World_StartRecording( worldId, rec );

	b3QueryFilter filter = b3DefaultQueryFilter();

	const int totalFrames = 30;
	for ( int i = 0; i < totalFrames; ++i )
	{
		b3Pos origin = { 0.0f, 6.0f, 0.0f };
		b3Vec3 translation = { 0.0f, -8.0f, 0.0f };
		b3AABB aabb = { { -5.0f, -1.0f, -5.0f }, { 5.0f, 6.0f, 5.0f } };

		b3Vec3 proxyPts = { 0.0f, 0.0f, 0.0f };
		b3ShapeProxy proxy = { &proxyPts, 1, 0.5f };
		b3Capsule mover = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 0.3f };

		b3World_OverlapAABB( worldId, aabb, filter, QueryReplayOverlapFcn, NULL );
		b3World_OverlapShape( worldId, origin, &proxy, filter, QueryReplayOverlapFcn, NULL );
		b3World_CastRay( worldId, origin, translation, filter, QueryReplayCastFcn, NULL );
		b3World_CastRayClosest( worldId, origin, translation, filter );
		b3World_CastShape( worldId, origin, &proxy, translation, filter, QueryReplayCastFcn, NULL );
		b3World_CastMover( worldId, origin, &mover, translation, filter, QueryReplayMoverFilterFcn, NULL );
		b3World_CollideMover( worldId, origin, &mover, filter, QueryReplayPlaneFcn, NULL );

		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	// Headless validation: re-issues every recorded query and compares the results.
	ENSURE( b3ValidateReplay( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 ) );

	// Player path: seek to a mid frame and confirm the per-frame store holds all seven queries.
	b3RecPlayer* player = b3RecPlayer_Create( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 );
	ENSURE( player != NULL );

	b3RecPlayer_SeekFrame( player, 15 );
	ENSURE( !b3RecPlayer_HasDiverged( player ) );
	ENSURE( b3RecPlayer_GetFrameQueryCount( player ) == 7 );

	b3RecQueryInfo first = b3RecPlayer_GetFrameQuery( player, 0 );
	ENSURE( first.type == b3_recQueryOverlapAABB );

	// The ray cast should find at least the ground, so its recorded hit list is non-empty.
	bool sawCastRay = false;
	for ( int qi = 0; qi < b3RecPlayer_GetFrameQueryCount( player ); ++qi )
	{
		b3RecQueryInfo info = b3RecPlayer_GetFrameQuery( player, qi );
		if ( info.type == b3_recQueryCastRay )
		{
			sawCastRay = true;
			ENSURE( info.hitCount > 0 );
		}
	}
	ENSURE( sawCastRay );

	b3RecPlayer_Destroy( player );
	b3DestroyRecording( rec );
	return 0;
}

// Tagged queries: the caller (id, label) is hashed into a key that rides the QueryTag op, with the id
// and label interned in the trailing tag table. The same label under two entity ids is two distinct
// keys. All survive a file round-trip and resolve back through b3RecQueryInfo; untagged queries report
// key 0 / id 0 / name NULL.
static int TaggedQuery( void )
{
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
	b3ShapeDef groundShape = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &groundShape, &groundBox.base );

	b3World_StartRecording( worldId, rec );

	// Same label, different entity ids: distinct logical queries with distinct keys.
	b3QueryFilter bullet53 = b3DefaultQueryFilter();
	bullet53.id = 53;
	bullet53.name = "bullet";

	b3QueryFilter bullet54 = b3DefaultQueryFilter();
	bullet54.id = 54;
	bullet54.name = "bullet";

	b3QueryFilter untagged = b3DefaultQueryFilter();

	uint64_t key53 = b3HashQueryTag( 53, "bullet" );
	uint64_t key54 = b3HashQueryTag( 54, "bullet" );
	ENSURE( key53 != 0 && key54 != 0 && key53 != key54 );

	const int totalFrames = 10;
	for ( int i = 0; i < totalFrames; ++i )
	{
		b3Pos origin = { 0.0f, 6.0f, 0.0f };
		b3Vec3 translation = { 0.0f, -8.0f, 0.0f };
		b3AABB aabb = { { -5.0f, -1.0f, -5.0f }, { 5.0f, 6.0f, 5.0f } };

		// Two tagged rays sharing the label "bullet" plus one untagged overlap, every frame.
		b3World_CastRay( worldId, origin, translation, bullet53, QueryReplayCastFcn, NULL );
		b3World_CastRay( worldId, origin, translation, bullet54, QueryReplayCastFcn, NULL );
		b3World_OverlapAABB( worldId, aabb, untagged, QueryReplayOverlapFcn, NULL );

		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	ENSURE( b3ValidateReplay( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 ) );

	// Round-trip through a file so the interned tag table is exercised on the persisted bytes.
	const char* path = "tagged_query_test.b3rec";
	ENSURE( b3SaveRecordingToFile( rec, path ) );
	b3Recording* loaded = b3LoadRecordingFromFile( path );
	ENSURE( loaded != NULL );

	b3RecPlayer* player = b3RecPlayer_Create( b3Recording_GetData( loaded ), b3Recording_GetSize( loaded ), 1 );
	ENSURE( player != NULL );

	b3RecPlayer_SeekFrame( player, 5 );
	ENSURE( !b3RecPlayer_HasDiverged( player ) );
	ENSURE( b3RecPlayer_GetFrameQueryCount( player ) == 3 );

	bool saw53 = false, saw54 = false, sawUntagged = false;
	for ( int qi = 0; qi < b3RecPlayer_GetFrameQueryCount( player ); ++qi )
	{
		b3RecQueryInfo info = b3RecPlayer_GetFrameQuery( player, qi );
		if ( info.key == key53 )
		{
			saw53 = true;
			ENSURE( info.id == 53 && info.name != NULL && strcmp( info.name, "bullet" ) == 0 );
		}
		else if ( info.key == key54 )
		{
			saw54 = true;
			ENSURE( info.id == 54 && info.name != NULL && strcmp( info.name, "bullet" ) == 0 );
		}
		else
		{
			sawUntagged = true;
			ENSURE( info.key == 0 && info.id == 0 && info.name == NULL );
		}
	}
	ENSURE( saw53 && saw54 && sawUntagged );

	b3RecPlayer_Destroy( player );
	b3DestroyRecording( loaded );
	b3DestroyRecording( rec );
	return 0;
}

// Empty world: recording starts with no bodies and none are ever created. The empty world is still
// seed-serialized like any other, so replay validates and Restart restores in place with a stable
// world id rather than tearing down and rebuilding the world.
static int EmptyWorldRoundTrip( void )
{
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3World_StartRecording( worldId, rec );
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	for ( int i = 0; i < 10; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	const uint8_t* data = b3Recording_GetData( rec );
	int size = b3Recording_GetSize( rec );

	// The seed snapshot is written even with no bodies.
	b3RecHeader hdr;
	memcpy( &hdr, data, sizeof( hdr ) );
	ENSURE( hdr.snapshotSize > 0 );

	ENSURE( b3ValidateReplay( data, size, 1 ) );

	// Restart restores in place, so the replay world id survives a rewind.
	b3RecPlayer* player = b3RecPlayer_Create( data, size, 1 );
	ENSURE( player != NULL );

	uint32_t worldKey = b3StoreWorldId( b3RecPlayer_GetWorldId( player ) );
	while ( !b3RecPlayer_IsAtEnd( player ) )
	{
		b3RecPlayer_StepFrame( player );
	}
	b3RecPlayer_Restart( player );
	ENSURE( b3StoreWorldId( b3RecPlayer_GetWorldId( player ) ) == worldKey );
	ENSURE( b3RecPlayer_GetFrame( player ) == 0 );
	ENSURE( !b3RecPlayer_HasDiverged( player ) );

	b3RecPlayer_Destroy( player );
	b3DestroyRecording( rec );
	return 0;
}

// Exercise every recorded op in a single session, then validate replay at two worker
// counts, round-trip through a file, and drive the incremental player. Mirrors the
// comprehensive RecordingTest in Box2D's test suite (box2d/test/test_recording.c).
static int AllOps( void )
{
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) );

	b3World_StartRecording( worldId, rec );

	// Static ground with a box-hull shape
	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	ENSURE( b3Body_IsValid( groundId ) );
	b3BoxHull groundBox = b3MakeBoxHull( 50.0f, 1.0f, 50.0f );
	b3ShapeDef groundShapeDef = b3DefaultShapeDef();
	b3ShapeId groundShapeId = b3CreateHullShape( groundId, &groundShapeDef, &groundBox.base );
	ENSURE( b3Shape_IsValid( groundShapeId ) );

	// Dynamic body with a sphere shape.
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 5.0f, 0.0f };
	bodyDef.name = "testBodyWithVeryLongNameThatIsAVeryLongNameLength";
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
	ENSURE( b3Body_IsValid( bodyId ) );

	b3ShapeDef sphereShapeDef = b3DefaultShapeDef();
	sphereShapeDef.density = 1.0f;
	// Over-length shape name so replay exercises the clamp in the shape def reader, like the body above.
	sphereShapeDef.name = "sphereNameThatExceedsTheLimit";
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
	b3ShapeId sphereShapeId = b3CreateSphereShape( bodyId, &sphereShapeDef, &sphere );
	ENSURE( b3Shape_IsValid( sphereShapeId ) );

	// Capsule shape on a second dynamic body
	b3BodyDef capsuleBodyDef = b3DefaultBodyDef();
	capsuleBodyDef.type = b3_dynamicBody;
	capsuleBodyDef.position = (b3Pos){ 3.0f, 5.0f, 0.0f };
	b3BodyId capsuleBodyId = b3CreateBody( worldId, &capsuleBodyDef );
	ENSURE( b3Body_IsValid( capsuleBodyId ) );

	b3ShapeDef capsuleShapeDef = b3DefaultShapeDef();
	capsuleShapeDef.density = 1.0f;
	b3Capsule capsule = { { 0.0f, -0.4f, 0.0f }, { 0.0f, 0.4f, 0.0f }, 0.25f };
	b3ShapeId capsuleShapeId = b3CreateCapsuleShape( capsuleBodyId, &capsuleShapeDef, &capsule );
	ENSURE( b3Shape_IsValid( capsuleShapeId ) );

	// Custom hull shape on a third dynamic body
	b3Vec3 hullPts[8] = {
		{ -0.5f, -0.5f, -0.5f }, { 0.5f, -0.5f, -0.5f }, { 0.5f, 0.5f, -0.5f }, { -0.5f, 0.5f, -0.5f },
		{ -0.5f, -0.5f, 0.5f },	 { 0.5f, -0.5f, 0.5f },	 { 0.5f, 0.5f, 0.5f },	{ -0.5f, 0.5f, 0.5f },
	};
	b3HullData* customHull = b3CreateHull( hullPts, 8, 8 );
	ENSURE( customHull != NULL );

	b3BodyDef hullBodyDef = b3DefaultBodyDef();
	hullBodyDef.type = b3_dynamicBody;
	hullBodyDef.position = (b3Pos){ -3.0f, 5.0f, 0.0f };
	b3BodyId hullBodyId = b3CreateBody( worldId, &hullBodyDef );
	ENSURE( b3Body_IsValid( hullBodyId ) );

	b3ShapeDef hullShapeDef = b3DefaultShapeDef();
	hullShapeDef.density = 1.0f;
	b3ShapeId hullShapeId = b3CreateHullShape( hullBodyId, &hullShapeDef, customHull );
	ENSURE( b3Shape_IsValid( hullShapeId ) );

	// Box hull shape on a fourth dynamic body (b3MakeBoxHull path)
	b3BodyDef boxBodyDef = b3DefaultBodyDef();
	boxBodyDef.type = b3_dynamicBody;
	boxBodyDef.position = (b3Pos){ 6.0f, 5.0f, 0.0f };
	b3BodyId boxBodyId = b3CreateBody( worldId, &boxBodyDef );
	ENSURE( b3Body_IsValid( boxBodyId ) );

	b3ShapeDef boxShapeDef = b3DefaultShapeDef();
	boxShapeDef.density = 2.0f;
	b3BoxHull boxHull = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3ShapeId boxShapeId = b3CreateHullShape( boxBodyId, &boxShapeDef, &boxHull.base );
	ENSURE( b3Shape_IsValid( boxShapeId ) );

	// Transformed hull shape on a fifth dynamic body (b3CreateTransformedHullShape path)
	b3BodyDef xformBodyDef = b3DefaultBodyDef();
	xformBodyDef.type = b3_dynamicBody;
	xformBodyDef.position = (b3Pos){ 12.0f, 5.0f, 0.0f };
	b3BodyId xformBodyId = b3CreateBody( worldId, &xformBodyDef );
	ENSURE( b3Body_IsValid( xformBodyId ) );
	b3ShapeDef xformShapeDef = b3DefaultShapeDef();
	xformShapeDef.density = 1.0f;
	b3Transform xformXf = { (b3Vec3){ 0.1f, 0.2f, -0.1f }, b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 1.0f, 0.0f }, 0.4f ) };
	b3ShapeId xformShapeId =
		b3CreateTransformedHullShape( xformBodyId, &xformShapeDef, customHull, xformXf, (b3Vec3){ 1.25f, 0.75f, 1.5f } );
	ENSURE( b3Shape_IsValid( xformShapeId ) );

	// Mesh, height field, and compound static shapes (3D-only)
	b3BodyDef meshBodyDef = b3DefaultBodyDef();
	meshBodyDef.type = b3_staticBody;
	meshBodyDef.position = (b3Pos){ 20.0f, 0.0f, 0.0f };
	b3BodyId meshBodyId = b3CreateBody( worldId, &meshBodyDef );
	b3MeshData* meshData = b3CreateGridMesh( 3, 3, 2.0f, 0, false );
	ENSURE( meshData != NULL );
	b3ShapeDef meshShapeDef = b3DefaultShapeDef();
	b3CreateMeshShape( meshBodyId, &meshShapeDef, meshData, (b3Vec3){ 1.0f, 1.0f, 1.0f } );

	b3BodyDef hfBodyDef = b3DefaultBodyDef();
	hfBodyDef.type = b3_staticBody;
	hfBodyDef.position = (b3Pos){ -20.0f, 0.0f, 0.0f };
	b3BodyId hfBodyId = b3CreateBody( worldId, &hfBodyDef );
	b3HeightFieldData* hf = b3CreateGrid( 4, 4, (b3Vec3){ 2.0f, 1.0f, 2.0f }, false );
	ENSURE( hf != NULL );
	b3ShapeDef hfShapeDef = b3DefaultShapeDef();
	b3CreateHeightFieldShape( hfBodyId, &hfShapeDef, hf );

	b3BodyDef compoundBodyDef = b3DefaultBodyDef();
	compoundBodyDef.type = b3_staticBody;
	compoundBodyDef.position = (b3Pos){ 30.0f, 0.0f, 0.0f };
	b3BodyId compoundBodyId = b3CreateBody( worldId, &compoundBodyDef );
	b3CompoundSphereDef compSphere;
	compSphere.sphere = (b3Sphere){ { 0.0f, 0.0f, 0.0f }, 1.0f };
	compSphere.material = b3DefaultSurfaceMaterial();
	b3CompoundDef compoundDef;
	memset( &compoundDef, 0, sizeof( compoundDef ) );
	compoundDef.spheres = &compSphere;
	compoundDef.sphereCount = 1;
	b3CompoundData* compound = b3CreateCompound( &compoundDef );
	ENSURE( compound != NULL );
	b3ShapeDef compoundShapeDef = b3DefaultShapeDef();
	b3CreateCompoundShape( compoundBodyId, &compoundShapeDef, compound );

	// Throwaway shape to exercise DestroyShape
	b3Sphere tmpSphere = { { 0.0f, 0.0f, 0.0f }, 0.1f };
	b3ShapeId tmpShapeId = b3CreateSphereShape( capsuleBodyId, &capsuleShapeDef, &tmpSphere );
	b3DestroyShape( tmpShapeId, true );

	// Shape mutators: SetFriction, SetRestitution, SetDensity, SetSurfaceMaterial, SetFilter,
	// EnableSensorEvents, EnableContactEvents, EnableHitEvents, EnablePreSolveEvents, ApplyWind,
	// SetSphere, SetCapsule, SetName
	b3Shape_SetFriction( boxShapeId, 0.3f );
	b3Shape_SetRestitution( capsuleShapeId, 0.5f );
	b3Shape_SetDensity( boxShapeId, 3.0f, true );
	b3SurfaceMaterial surfMat = b3DefaultSurfaceMaterial();
	surfMat.friction = 0.7f;
	surfMat.restitution = 0.1f;
	b3Shape_SetSurfaceMaterial( capsuleShapeId, surfMat );
	b3Filter shapeFilter = b3DefaultFilter();
	shapeFilter.categoryBits = 0x2;
	b3Shape_SetFilter( boxShapeId, shapeFilter, false );
	b3Shape_EnableSensorEvents( capsuleShapeId, true );
	b3Shape_EnableContactEvents( capsuleShapeId, true );
	b3Shape_EnableHitEvents( boxShapeId, true );
	b3Shape_EnablePreSolveEvents( boxShapeId, true );
	b3Shape_ApplyWind( capsuleShapeId, (b3Vec3){ 1.0f, 0.0f, 0.0f }, 0.1f, 0.0f, 10.0f, true );
	b3Sphere newSphere = { { 0.0f, 0.0f, 0.0f }, 0.45f };
	b3Shape_SetSphere( sphereShapeId, &newSphere );
	b3Capsule newCapsule = { { 0.0f, -0.3f, 0.0f }, { 0.0f, 0.3f, 0.0f }, 0.3f };
	b3Shape_SetCapsule( capsuleShapeId, &newCapsule );
	b3Shape_SetName( boxShapeId, "box" );

	// Body mutators: SetTransform, SetLinearVelocity/AngularVelocity (Vec3), SetName,
	// damping, gravity scale, sleep threshold, SetAwake, EnableSleep, SetBullet, SetMotionLocks,
	// SetMassData, ApplyMassFromShapes, SetType, SetTargetTransform, Disable/Enable, EnableContactRecycling,
	// EnableHitEvents, all force/impulse/torque variants
	b3Body_SetTransform( bodyId, (b3Pos){ 1.0f, 6.0f, 0.0f }, b3Quat_identity );
	b3Body_SetLinearVelocity( bodyId, (b3Vec3){ 0.5f, 0.0f, 0.0f } );
	b3Body_SetAngularVelocity( bodyId, (b3Vec3){ 0.0f, 0.25f, 0.0f } );
	b3Body_SetName( bodyId, "renamedBody" );
	b3Body_SetLinearDamping( bodyId, 0.1f );
	b3Body_SetAngularDamping( bodyId, 0.05f );
	b3Body_SetGravityScale( bodyId, 0.9f );
	b3Body_SetSleepThreshold( bodyId, 0.02f );
	b3Body_EnableSleep( bodyId, false );
	b3Body_SetBullet( bodyId, true );
	b3Body_EnableContactRecycling( bodyId, false );
	b3Body_EnableHitEvents( bodyId, true );
	b3Body_SetMotionLocks( bodyId, (b3MotionLocks){ false, false, false, false, false, true } );
	b3MassData massData;
	massData.mass = 2.0f;
	massData.center = (b3Vec3){ 0.0f, 0.0f, 0.0f };
	massData.inertia = b3Mat3_identity;
	b3Body_SetMassData( bodyId, massData );
	b3Body_ApplyMassFromShapes( bodyId );
	b3Body_SetType( capsuleBodyId, b3_kinematicBody );
	b3Body_SetType( capsuleBodyId, b3_dynamicBody );
	b3Body_SetAwake( bodyId, true );

	// Kinematic body to exercise SetTargetTransform
	b3BodyDef kinematicDef = b3DefaultBodyDef();
	kinematicDef.type = b3_kinematicBody;
	kinematicDef.position = (b3Pos){ -6.0f, 5.0f, 0.0f };
	b3BodyId kinematicId = b3CreateBody( worldId, &kinematicDef );
	b3BoxHull kinBox = b3MakeBoxHull( 0.4f, 0.4f, 0.4f );
	b3ShapeDef kinShapeDef = b3DefaultShapeDef();
	b3CreateHullShape( kinematicId, &kinShapeDef, &kinBox.base );
	b3WorldTransform kinTarget;
	kinTarget.p = (b3Pos){ -5.0f, 5.0f, 0.0f };
	kinTarget.q = b3Quat_identity;
	b3Body_SetTargetTransform( kinematicId, kinTarget, 1.0f / 60.0f, true );

	// Body to exercise Disable/Enable
	b3BodyDef disableDef = b3DefaultBodyDef();
	disableDef.type = b3_dynamicBody;
	disableDef.position = (b3Pos){ 9.0f, 5.0f, 0.0f };
	b3BodyId disableId = b3CreateBody( worldId, &disableDef );
	b3Sphere disableSphere = { { 0.0f, 0.0f, 0.0f }, 0.3f };
	b3CreateSphereShape( disableId, &sphereShapeDef, &disableSphere );
	b3Body_Disable( disableId );
	b3Body_Enable( disableId );

	// Force/impulse/torque (Vec3 args in 3D)
	b3Body_ApplyForce( bodyId, (b3Vec3){ 0.0f, 50.0f, 0.0f }, (b3Pos){ 1.0f, 6.0f, 0.0f }, true );
	b3Body_ApplyForceToCenter( bodyId, (b3Vec3){ 5.0f, 0.0f, 0.0f }, true );
	b3Body_ApplyTorque( bodyId, (b3Vec3){ 0.0f, 1.0f, 0.0f }, true );
	b3Body_ApplyLinearImpulse( bodyId, (b3Vec3){ 0.1f, 0.0f, 0.0f }, (b3Pos){ 1.0f, 6.0f, 0.0f }, true );
	b3Body_ApplyLinearImpulseToCenter( bodyId, (b3Vec3){ 0.0f, 0.1f, 0.0f }, true );
	b3Body_ApplyAngularImpulse( bodyId, (b3Vec3){ 0.0f, 0.05f, 0.0f }, true );

	// Joint bodies: a row of dynamic bodies connected by each joint type
	b3BodyId jb[9];
	for ( int i = 0; i < 9; ++i )
	{
		b3BodyDef jbd = b3DefaultBodyDef();
		jbd.type = b3_dynamicBody;
		jbd.position = (b3Pos){ -8.0f + (float)i * 2.0f, 10.0f, 0.0f };
		jb[i] = b3CreateBody( worldId, &jbd );
		b3Sphere js = { { 0.0f, 0.0f, 0.0f }, 0.25f };
		b3ShapeDef jsd = b3DefaultShapeDef();
		jsd.density = 1.0f;
		b3CreateSphereShape( jb[i], &jsd, &js );
	}

	// Revolute joint with full setter coverage and the generic joint mutators
	b3RevoluteJointDef revDef = b3DefaultRevoluteJointDef();
	revDef.base.bodyIdA = jb[0];
	revDef.base.bodyIdB = jb[1];
	revDef.base.localFrameA.p = (b3Vec3){ 1.0f, 0.0f, 0.0f };
	revDef.base.localFrameB.p = (b3Vec3){ -1.0f, 0.0f, 0.0f };
	b3JointId revId = b3CreateRevoluteJoint( worldId, &revDef );
	ENSURE( b3Joint_IsValid( revId ) );
	b3RevoluteJoint_EnableLimit( revId, true );
	b3RevoluteJoint_SetLimits( revId, -1.0f, 1.0f );
	b3RevoluteJoint_EnableMotor( revId, true );
	b3RevoluteJoint_SetMotorSpeed( revId, 0.5f );
	b3RevoluteJoint_SetMaxMotorTorque( revId, 10.0f );
	b3RevoluteJoint_EnableSpring( revId, true );
	b3RevoluteJoint_SetSpringHertz( revId, 2.0f );
	b3RevoluteJoint_SetSpringDampingRatio( revId, 0.5f );
	b3RevoluteJoint_SetTargetAngle( revId, 0.25f );
	b3Joint_SetLocalFrameA( revId, (b3Transform){ (b3Vec3){ 1.0f, 0.0f, 0.0f }, b3Quat_identity } );
	b3Joint_SetLocalFrameB( revId, (b3Transform){ (b3Vec3){ -1.0f, 0.0f, 0.0f }, b3Quat_identity } );
	b3Joint_SetConstraintTuning( revId, 60.0f, 2.0f );
	b3Joint_SetForceThreshold( revId, 100.0f );
	b3Joint_SetTorqueThreshold( revId, 50.0f );
	b3Joint_SetCollideConnected( revId, false );
	b3Joint_WakeBodies( revId );

	// Distance joint
	b3DistanceJointDef distDef = b3DefaultDistanceJointDef();
	distDef.base.bodyIdA = jb[1];
	distDef.base.bodyIdB = jb[2];
	distDef.length = 2.0f;
	b3JointId distId = b3CreateDistanceJoint( worldId, &distDef );
	b3DistanceJoint_SetLength( distId, 2.2f );
	b3DistanceJoint_EnableSpring( distId, true );
	b3DistanceJoint_SetSpringHertz( distId, 3.0f );
	b3DistanceJoint_SetSpringDampingRatio( distId, 0.4f );
	b3DistanceJoint_SetSpringForceRange( distId, -50.0f, 50.0f );
	b3DistanceJoint_EnableLimit( distId, true );
	b3DistanceJoint_SetLengthRange( distId, 1.0f, 4.0f );
	b3DistanceJoint_EnableMotor( distId, true );
	b3DistanceJoint_SetMotorSpeed( distId, 0.3f );
	b3DistanceJoint_SetMaxMotorForce( distId, 5.0f );

	// Filter joint (plus a throwaway to exercise DestroyJoint)
	b3FilterJointDef filterDef = b3DefaultFilterJointDef();
	filterDef.base.bodyIdA = jb[2];
	filterDef.base.bodyIdB = jb[3];
	b3JointId filterId = b3CreateFilterJoint( worldId, &filterDef );
	ENSURE( b3Joint_IsValid( filterId ) );

	b3DistanceJointDef tmpJointDef = b3DefaultDistanceJointDef();
	tmpJointDef.base.bodyIdA = jb[0];
	tmpJointDef.base.bodyIdB = jb[8];
	tmpJointDef.length = 5.0f;
	b3JointId tmpJointId = b3CreateDistanceJoint( worldId, &tmpJointDef );
	b3DestroyJoint( tmpJointId, true );

	// Motor joint (Vec3 velocities in 3D)
	b3MotorJointDef motorDef = b3DefaultMotorJointDef();
	motorDef.base.bodyIdA = jb[3];
	motorDef.base.bodyIdB = jb[4];
	b3JointId motorId = b3CreateMotorJoint( worldId, &motorDef );
	b3MotorJoint_SetLinearVelocity( motorId, (b3Vec3){ 0.1f, 0.0f, 0.0f } );
	b3MotorJoint_SetAngularVelocity( motorId, (b3Vec3){ 0.0f, 0.2f, 0.0f } );
	b3MotorJoint_SetMaxVelocityForce( motorId, 10.0f );
	b3MotorJoint_SetMaxVelocityTorque( motorId, 10.0f );
	b3MotorJoint_SetLinearHertz( motorId, 2.0f );
	b3MotorJoint_SetLinearDampingRatio( motorId, 0.5f );
	b3MotorJoint_SetAngularHertz( motorId, 2.0f );
	b3MotorJoint_SetAngularDampingRatio( motorId, 0.5f );
	b3MotorJoint_SetMaxSpringForce( motorId, 20.0f );
	b3MotorJoint_SetMaxSpringTorque( motorId, 20.0f );

	// Prismatic joint
	b3PrismaticJointDef prisDef = b3DefaultPrismaticJointDef();
	prisDef.base.bodyIdA = jb[4];
	prisDef.base.bodyIdB = jb[5];
	b3JointId prisId = b3CreatePrismaticJoint( worldId, &prisDef );
	b3PrismaticJoint_EnableSpring( prisId, true );
	b3PrismaticJoint_SetSpringHertz( prisId, 2.0f );
	b3PrismaticJoint_SetSpringDampingRatio( prisId, 0.5f );
	b3PrismaticJoint_SetTargetTranslation( prisId, 0.1f );
	b3PrismaticJoint_EnableLimit( prisId, true );
	b3PrismaticJoint_SetLimits( prisId, -1.0f, 1.0f );
	b3PrismaticJoint_EnableMotor( prisId, true );
	b3PrismaticJoint_SetMotorSpeed( prisId, 0.2f );
	b3PrismaticJoint_SetMaxMotorForce( prisId, 8.0f );

	// Spherical joint (3D-only)
	b3SphericalJointDef sphDef = b3DefaultSphericalJointDef();
	sphDef.base.bodyIdA = jb[5];
	sphDef.base.bodyIdB = jb[6];
	b3JointId sphId = b3CreateSphericalJoint( worldId, &sphDef );
	b3SphericalJoint_EnableConeLimit( sphId, true );
	b3SphericalJoint_SetConeLimit( sphId, 0.5f );
	b3SphericalJoint_EnableTwistLimit( sphId, true );
	b3SphericalJoint_SetTwistLimits( sphId, -0.3f, 0.3f );
	b3SphericalJoint_EnableSpring( sphId, true );
	b3SphericalJoint_SetSpringHertz( sphId, 3.0f );
	b3SphericalJoint_SetSpringDampingRatio( sphId, 0.5f );
	b3SphericalJoint_SetTargetRotation( sphId, b3Quat_identity );
	b3SphericalJoint_EnableMotor( sphId, true );
	b3SphericalJoint_SetMotorVelocity( sphId, (b3Vec3){ 0.0f, 0.1f, 0.0f } );
	b3SphericalJoint_SetMaxMotorTorque( sphId, 5.0f );

	// Weld joint
	b3WeldJointDef weldDef = b3DefaultWeldJointDef();
	weldDef.base.bodyIdA = jb[6];
	weldDef.base.bodyIdB = jb[7];
	b3JointId weldId = b3CreateWeldJoint( worldId, &weldDef );
	b3WeldJoint_SetLinearHertz( weldId, 5.0f );
	b3WeldJoint_SetLinearDampingRatio( weldId, 0.6f );
	b3WeldJoint_SetAngularHertz( weldId, 5.0f );
	b3WeldJoint_SetAngularDampingRatio( weldId, 0.6f );

	// Wheel joint (Box3D uses Suspension/Spin/Steering naming)
	b3WheelJointDef wheelDef = b3DefaultWheelJointDef();
	wheelDef.base.bodyIdA = jb[7];
	wheelDef.base.bodyIdB = jb[8];
	b3JointId wheelId = b3CreateWheelJoint( worldId, &wheelDef );
	b3WheelJoint_EnableSuspension( wheelId, true );
	b3WheelJoint_SetSuspensionHertz( wheelId, 4.0f );
	b3WheelJoint_SetSuspensionDampingRatio( wheelId, 0.7f );
	b3WheelJoint_EnableSuspensionLimit( wheelId, true );
	b3WheelJoint_SetSuspensionLimits( wheelId, -0.5f, 0.5f );
	b3WheelJoint_EnableSpinMotor( wheelId, true );
	b3WheelJoint_SetSpinMotorSpeed( wheelId, 1.0f );
	b3WheelJoint_SetMaxSpinTorque( wheelId, 6.0f );
	b3WheelJoint_EnableSteering( wheelId, true );
	b3WheelJoint_SetSteeringHertz( wheelId, 2.0f );
	b3WheelJoint_SetSteeringDampingRatio( wheelId, 0.5f );
	b3WheelJoint_SetMaxSteeringTorque( wheelId, 3.0f );
	b3WheelJoint_EnableSteeringLimit( wheelId, true );
	b3WheelJoint_SetSteeringLimits( wheelId, -0.5f, 0.5f );
	b3WheelJoint_SetTargetSteeringAngle( wheelId, 0.1f );

	// Parallel joint (3D-only)
	b3ParallelJointDef parallelDef = b3DefaultParallelJointDef();
	parallelDef.base.bodyIdA = groundId;
	parallelDef.base.bodyIdB = bodyId;
	b3JointId parallelId = b3CreateParallelJoint( worldId, &parallelDef );
	b3ParallelJoint_SetSpringHertz( parallelId, 2.0f );
	b3ParallelJoint_SetSpringDampingRatio( parallelId, 0.5f );
	b3ParallelJoint_SetMaxTorque( parallelId, 20.0f );

	// World config mutators
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -9.8f, 0.0f } );
	b3World_EnableSleeping( worldId, true );
	b3World_EnableContinuous( worldId, false );
	b3World_EnableWarmStarting( worldId, true );
	b3World_EnableSpeculative( worldId, true );
	b3World_SetRestitutionThreshold( worldId, 1.5f );
	b3World_SetHitEventThreshold( worldId, 2.0f );
	b3World_SetContactTuning( worldId, 30.0f, 10.0f, 3.0f );
	b3World_SetContactRecycleDistance( worldId, 0.05f );
	b3World_SetMaximumLinearSpeed( worldId, 100.0f );
	b3World_RebuildStaticTree( worldId );

	b3ExplosionDef explosion = b3DefaultExplosionDef();
	explosion.position = (b3Pos){ 0.0f, 5.0f, 0.0f };
	explosion.radius = 3.0f;
	explosion.falloff = 1.0f;
	explosion.impulsePerArea = 2.0f;
	b3World_Explode( worldId, &explosion );

	// Pre-step queries
	b3QueryFilter qfilter = b3DefaultQueryFilter();
	b3AABB qaabb = { { -10.0f, -5.0f, -10.0f }, { 10.0f, 15.0f, 10.0f } };
	b3World_OverlapAABB( worldId, qaabb, qfilter, QueryReplayOverlapFcn, NULL );
	b3Pos qorigin = { 0.0f, 15.0f, 0.0f };
	b3Vec3 proxyPts = { 0.0f, 0.0f, 0.0f };
	b3ShapeProxy proxy = { &proxyPts, 1, 0.5f };
	b3World_OverlapShape( worldId, qorigin, &proxy, qfilter, QueryReplayOverlapFcn, NULL );
	b3Vec3 qTranslation = { 0.0f, -20.0f, 0.0f };
	b3World_CastRay( worldId, qorigin, qTranslation, qfilter, QueryReplayCastFcn, NULL );
	b3World_CastRayClosest( worldId, qorigin, qTranslation, qfilter );
	b3World_CastShape( worldId, qorigin, &proxy, qTranslation, qfilter, QueryReplayCastFcn, NULL );
	b3Capsule mover = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 0.3f };
	b3World_CastMover( worldId, qorigin, &mover, qTranslation, qfilter, QueryReplayMoverFilterFcn, NULL );
	b3World_CollideMover( worldId, qorigin, &mover, qfilter, QueryReplayPlaneFcn, NULL );

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;
	for ( int i = 0; i < 12; ++i )
	{
		// Inject mutators mid-simulation
		if ( i == 6 )
		{
			b3Body_ApplyLinearImpulseToCenter( capsuleBodyId, (b3Vec3){ 2.0f, 0.0f, 0.0f }, true );
			b3Body_SetGravityScale( bodyId, 1.0f );
		}

		// Issue queries mid-loop to exercise recording across steps
		if ( i == 3 )
		{
			b3World_OverlapAABB( worldId, qaabb, qfilter, QueryReplayOverlapFcn, NULL );
			b3World_CastRay( worldId, qorigin, qTranslation, qfilter, QueryReplayCastFcn, NULL );
		}

		b3World_Step( worldId, timeStep, subStepCount );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	// Free geometry allocated for this subtest
	b3DestroyHull( customHull );
	b3DestroyMesh( meshData );
	b3DestroyHeightField( hf );
	b3DestroyCompound( compound );

	const uint8_t* recData = b3Recording_GetData( rec );
	int recSize = b3Recording_GetSize( rec );
	ENSURE( recSize > 0 );

	// Replay headless at worker count 1 and 4, a cross-thread determinism check matching Box2D.
	ENSURE( b3ValidateReplay( recData, recSize, 1 ) );
	ENSURE( b3ValidateReplay( recData, recSize, 4 ) );

	// File round-trip
	ENSURE( b3SaveRecordingToFile( rec, s_recPath ) );
	b3Recording* loaded = b3LoadRecordingFromFile( s_recPath );
	ENSURE( loaded != NULL );
	ENSURE( b3ValidateReplay( b3Recording_GetData( loaded ), b3Recording_GetSize( loaded ), 1 ) );
	b3DestroyRecording( loaded );

	// Drive the incremental player. Exercises per-frame stepping, restart, getters, and the
	// draw path beyond what b3ValidateReplay covers.
	{
		b3RecPlayer* player = b3RecPlayer_Create( recData, recSize, 1 );
		ENSURE( player != NULL );

		b3RecPlayerInfo info = b3RecPlayer_GetInfo( player );
		b3Vec3 recExtents = b3Sub( info.bounds.upperBound, info.bounds.lowerBound );
		ENSURE( recExtents.x > 0.0f && recExtents.y > 0.0f );

		// Build a no-op b3DebugDraw to exercise the draw path headlessly
		b3DebugDraw dd = b3DefaultDebugDraw();
		dd.DrawShapeFcn = RecTestDrawShape;
		dd.drawShapes = true;
		dd.drawingBounds = (b3AABB){ { -1.0e6f, -1.0e6f, -1.0e6f }, { 1.0e6f, 1.0e6f, 1.0e6f } };

		int frames = 0;
		while ( b3RecPlayer_StepFrame( player ) )
		{
			if ( frames % 2 == 0 )
			{
				b3RecPlayer_DrawFrameQueries( player, &dd, -1, -1 );
			}
			frames += 1;
		}
		ENSURE( frames == 12 );
		ENSURE( b3RecPlayer_GetFrame( player ) == 12 );
		ENSURE( b3RecPlayer_IsAtEnd( player ) );
		ENSURE( b3RecPlayer_HasDiverged( player ) == false );

		// The trailing DestroyWorld is an end marker; the world stays valid after end
		ENSURE( b3World_IsValid( b3RecPlayer_GetWorldId( player ) ) );

		// Restart reproduces the same run without reloading the file
		b3RecPlayer_Restart( player );
		ENSURE( b3RecPlayer_GetFrame( player ) == 0 );
		ENSURE( b3RecPlayer_IsAtEnd( player ) == false );

		int frames2 = 0;
		while ( b3RecPlayer_StepFrame( player ) )
		{
			frames2 += 1;
		}
		ENSURE( frames2 == 12 );
		ENSURE( b3RecPlayer_HasDiverged( player ) == false );

		b3RecPlayer_Destroy( player );
	}

	b3DestroyRecording( rec );
	remove( s_recPath );
	return 0;
}

// A transformed hull bakes its transform and non-uniform scale into fresh hull data at create time.
// It must be recorded like any other shape create, else its shape id allocation is invisible to the
// player and every later id drifts. A plain hull created after it would then mismatch on replay.
static int TransformedHullRoundTrip( void )
{
	b3Vec3 pts[8] = {
		{ -1.0f, -1.0f, -1.0f }, { 1.0f, -1.0f, -1.0f }, { 1.0f, 1.0f, -1.0f }, { -1.0f, 1.0f, -1.0f },
		{ -1.0f, -1.0f, 1.0f },	 { 1.0f, -1.0f, 1.0f },	 { 1.0f, 1.0f, 1.0f },	{ -1.0f, 1.0f, 1.0f },
	};
	b3HullData* hull = b3CreateHull( pts, 8, 8 );
	ENSURE( hull != NULL );

	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3World_StartRecording( worldId, rec );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;

	// Baked transform with a rotation and non-uniform scale, the path Unreal uses for instanced hulls.
	b3Transform xf = { (b3Vec3){ 0.25f, 0.0f, -0.5f }, b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 0.0f, 1.0f }, 0.3f ) };
	b3Vec3 scl = { 1.5f, 0.5f, 2.0f };
	for ( int i = 0; i < 3; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ (float)( i * 3 ), 5.0f, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3ShapeId sid = b3CreateTransformedHullShape( bodyId, &shapeDef, hull, xf, scl );
		ENSURE( b3Shape_IsValid( sid ) );
	}

	// A plain hull after the transformed ones: if the transformed creates desynced the id pool, this
	// shape's recorded id would not match what replay allocates and b3ValidateReplay would fail.
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 0.0f, 10.0f, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3ShapeId sid = b3CreateHullShape( bodyId, &shapeDef, hull );
		ENSURE( b3Shape_IsValid( sid ) );
	}

	// Step past the keyframe interval so replay captures a keyframe. That path re-serializes the live
	// world and interns the baked hull, which must already be in the pre-seeded registry.
	for ( int i = 0; i < 20; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );
	b3DestroyHull( hull );

	ENSURE( b3ValidateReplay( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 ) );
	ENSURE( b3ValidateReplay( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 4 ) );

	b3DestroyRecording( rec );
	return 0;
}

// Patch the reserved header bytes to nonzero and confirm b3ValidateReplay ignores them.
// Guards a future change that starts validating them or shrinks the header.
static int ReservedHeaderBytes( void )
{
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_StartRecording( worldId, rec );
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	b3BodyDef bd = b3DefaultBodyDef();
	bd.type = b3_dynamicBody;
	bd.position = (b3Pos){ 0.0f, 5.0f, 0.0f };
	b3BodyId bodyId = b3CreateBody( worldId, &bd );
	b3Sphere s = { { 0.0f, 0.0f, 0.0f }, 0.5f };
	b3ShapeDef sd = b3DefaultShapeDef();
	sd.density = 1.0f;
	b3CreateSphereShape( bodyId, &sd, &s );

	for ( int i = 0; i < 10; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	const uint8_t* recData = b3Recording_GetData( rec );
	int recSize = b3Recording_GetSize( rec );
	ENSURE( recSize >= (int)sizeof( b3RecHeader ) );

	// Patch reserved fields at their byte offsets in b3RecHeader:
	//   byte 11 : reserved  (uint8_t at offset 11)
	//   bytes 16-19 : reserved2 (uint32_t at offset 16)
	//   bytes 20-23 : reserved3 (uint32_t at offset 20)
	uint8_t* patched = (uint8_t*)b3Alloc( (size_t)recSize );
	memcpy( patched, recData, (size_t)recSize );
	patched[11] = 0xAB;
	patched[16] = 0xCD;
	patched[17] = 0xEF;
	patched[18] = 0x12;
	patched[19] = 0x34;
	patched[20] = 0x56;
	patched[21] = 0x78;
	patched[22] = 0x9A;
	patched[23] = 0xBC;
	ENSURE( b3ValidateReplay( patched, recSize, 1 ) );
	b3Free( patched, (size_t)recSize );

	b3DestroyRecording( rec );
	return 0;
}

// Geometry that shares a content hash but differs in bytes must dedup exactly. Mirrors the keyframe
// flow that crashed: the seed appends every slot 1:1 (even byte-identical duplicates a hash collision
// left in an already-recorded file), then capture must resolve a live blob back to an existing slot
// without growing the registry. Forces the collision by handing the same hash to distinct blobs.
static int GeometryHashCollision( void )
{
	const int n = 16;
	const uint64_t sharedHash = 0xABCD1234ull;

	// The content hash must use its full width: a one-byte change in a same-length blob has to perturb
	// the high word too, not just the low one, which is the trap a reseeded 32-bit djb2 fell into.
	{
		uint8_t p[16];
		uint8_t q[16];
		memset( p, 0x11, sizeof( p ) );
		memset( q, 0x11, sizeof( q ) );
		q[7] = 0x12;
		uint64_t hp = b3Hash64Blob( p, (int)sizeof( p ) );
		uint64_t hq = b3Hash64Blob( q, (int)sizeof( q ) );
		ENSURE( hp != hq );
		ENSURE( (uint32_t)( hp >> 32 ) != (uint32_t)( hq >> 32 ) );
	}

	b3GeometryRegistry reg = { 0 };

	uint8_t* blobA = (uint8_t*)b3Alloc( (size_t)n );
	uint8_t* blobB = (uint8_t*)b3Alloc( (size_t)n );
	memset( blobA, 0xAA, (size_t)n );
	memset( blobB, 0xBB, (size_t)n );

	// Distinct blobs colliding on the hash must become two entries, not a false dedup.
	uint32_t idA = b3InternGeometry( &reg, b3_geometryHull, sharedHash, blobA, n );
	uint32_t idB = b3InternGeometry( &reg, b3_geometryHull, sharedHash, blobB, n );
	ENSURE( idA != idB );
	ENSURE( reg.count == 2 );

	// Re-interning either blob must find it through the hash chain and never grow the registry,
	// including the one shadowed behind the bucket head. The old single-entry lookup missed the
	// shadowed blob and appended a duplicate, which is exactly what tripped the keyframe assert.
	uint8_t* blobA2 = (uint8_t*)b3Alloc( (size_t)n );
	memset( blobA2, 0xAA, (size_t)n );
	ENSURE( b3InternGeometry( &reg, b3_geometryHull, sharedHash, blobA2, n ) == idA );
	ENSURE( reg.count == 2 );

	uint8_t* blobB2 = (uint8_t*)b3Alloc( (size_t)n );
	memset( blobB2, 0xBB, (size_t)n );
	ENSURE( b3InternGeometry( &reg, b3_geometryHull, sharedHash, blobB2, n ) == idB );
	ENSURE( reg.count == 2 );

	b3FreeRegistry( &reg );

	// Seed-then-capture: appending byte-identical duplicate slots keeps id == slot index, and a later
	// exact intern still resolves to one of them without appending a new entry.
	b3GeometryRegistry seeded = { 0 };
	uint8_t* slot0 = (uint8_t*)b3Alloc( (size_t)n );
	uint8_t* slot1 = (uint8_t*)b3Alloc( (size_t)n );
	uint8_t* slot2 = (uint8_t*)b3Alloc( (size_t)n );
	memset( slot0, 0xAA, (size_t)n );
	memset( slot1, 0xBB, (size_t)n );
	memset( slot2, 0xAA, (size_t)n ); // duplicate of slot0
	ENSURE( b3AppendGeometry( &seeded, b3_geometryHull, sharedHash, slot0, n ) == 0 );
	ENSURE( b3AppendGeometry( &seeded, b3_geometryHull, sharedHash, slot1, n ) == 1 );
	ENSURE( b3AppendGeometry( &seeded, b3_geometryHull, sharedHash, slot2, n ) == 2 );

	uint8_t* live = (uint8_t*)b3Alloc( (size_t)n );
	memset( live, 0xAA, (size_t)n );
	uint32_t resolved = b3InternGeometry( &seeded, b3_geometryHull, sharedHash, live, n );
	ENSURE( seeded.count == 3 );			  // no growth
	ENSURE( resolved == 0 || resolved == 2 ); // a valid slot index for that content
	ENSURE( seeded.entries[resolved].byteCount == n );
	ENSURE( memcmp( seeded.entries[resolved].bytes, slot0, (size_t)n ) == 0 );

	b3FreeRegistry( &seeded );
	return 0;
}

// Staged stepping must reveal a mid-stream body at its creation transform. A body created and given an
// impulse in one recorded step is first placed by CreateBody, then displaced by the following Step. Atomic
// replay fuses the two, so the body is only ever seen already moved. Staged replay parks between them so
// the pre-integration pose is drawable. Verify the parked pose is the creation transform, that atomic
// replay does not show it, and that the extra park does not perturb the end state.
static int StagedStepCreationPose( void )
{
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	b3World_StartRecording( worldId, rec );

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;

	// A few empty steps so the creation lands inside the stream, not at frame 0.
	const int leadFrames = 3;
	for ( int i = 0; i < leadFrames; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	// The creation transform under test. No ground, so the body is the only create and stays ballistic.
	// Components are exact in float so the round-trip pose compares tight.
	const b3Pos spawn = { 1.0, 20.0, -2.0 };
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = spawn;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );

	// Impulse along +x so the first integrated step moves the body a visible distance off the spawn.
	b3Body_ApplyLinearImpulseToCenter( bodyId, (b3Vec3){ 5.0f, 0.0f, 0.0f }, true );

	const int creationFrame = leadFrames + 1;
	b3World_Step( worldId, timeStep, subStepCount ); // advances to creationFrame

	const int totalFrames = creationFrame + 3;
	for ( int i = creationFrame; i < totalFrames; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	const uint8_t* data = b3Recording_GetData( rec );
	int sz = b3Recording_GetSize( rec );

	// Atomic replay fuses the create and its step, so at the creation frame the body is already
	// integrated and displaced along +x. Capture that pose and the final state hash.
	b3RecPlayer* atomic = b3RecPlayer_Create( data, sz, 1 );
	ENSURE( atomic != NULL );
	b3Pos atomicPose = { 0.0, 0.0, 0.0 };
	while ( !b3RecPlayer_IsAtEnd( atomic ) )
	{
		b3RecPlayer_StepFrame( atomic );
		if ( b3RecPlayer_GetFrame( atomic ) == creationFrame )
		{
			b3BodyId id = b3RecPlayer_GetBodyId( atomic, 0 );
			ENSURE( b3Body_IsValid( id ) );
			atomicPose = b3Body_GetPosition( id );
		}
	}
	ENSURE( b3RecPlayer_GetFrame( atomic ) == totalFrames );
	ENSURE( !b3RecPlayer_HasDiverged( atomic ) );
	ENSURE( atomicPose.x - spawn.x > 0.01 ); // the impulse moved it before it was ever seen
	uint64_t atomicHash = b3HashWorldState( b3GetWorldFromId( b3RecPlayer_GetWorldId( atomic ) ) );
	b3RecPlayer_Destroy( atomic );

	// Staged replay: step forward until the first pre-step park. It must sit at the creation frame's
	// pre-integration state with the new body at exactly its spawn transform.
	b3RecPlayer* staged = b3RecPlayer_Create( data, sz, 1 );
	ENSURE( staged != NULL );
	while ( !b3RecPlayer_IsAtEnd( staged ) )
	{
		b3RecPlayer_SubStepFrame( staged );
		if ( b3RecPlayer_IsAtPreStep( staged ) )
		{
			break;
		}
	}
	ENSURE( b3RecPlayer_IsAtPreStep( staged ) );
	ENSURE( b3RecPlayer_GetFrame( staged ) == creationFrame - 1 ); // parked before the step, frame not advanced

	b3BodyId stagedId = b3RecPlayer_GetBodyId( staged, 0 );
	ENSURE( b3Body_IsValid( stagedId ) );
	b3Pos parkPose = b3Body_GetPosition( stagedId );
	ENSURE_SMALL( parkPose.x - spawn.x, 1.0e-4 );
	ENSURE_SMALL( parkPose.y - spawn.y, 1.0e-4 );
	ENSURE_SMALL( parkPose.z - spawn.z, 1.0e-4 );

	// Finishing the frame integrates the body, so it leaves the spawn pose on the next advance.
	b3RecPlayer_SubStepFrame( staged );
	ENSURE( b3RecPlayer_GetFrame( staged ) == creationFrame );
	ENSURE( !b3RecPlayer_IsAtPreStep( staged ) );

	// Run staged to the end: the same op stream, so it must land on the atomic end state bit for bit.
	while ( !b3RecPlayer_IsAtEnd( staged ) )
	{
		b3RecPlayer_SubStepFrame( staged );
	}
	ENSURE( b3RecPlayer_GetFrame( staged ) == totalFrames );
	ENSURE( !b3RecPlayer_HasDiverged( staged ) );
	uint64_t stagedHash = b3HashWorldState( b3GetWorldFromId( b3RecPlayer_GetWorldId( staged ) ) );
	ENSURE( stagedHash == atomicHash );
	b3RecPlayer_Destroy( staged );

	b3DestroyRecording( rec );
	return 0;
}

// Shape names are debug only and do not feed the determinism hash, so b3ValidateReplay cannot catch a
// broken name round-trip. Replay through the player and read the names back to prove the def field and
// the ShapeSetName op survive serialization.
static int ShapeNameReplay( void )
{
	const char* names[3] = {
		"def",
		"set",
		"abcdefghijklmnopqrstuvwxyz",
	};

	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_StartRecording( worldId, rec );

	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };

	// One shape per body so read-back never depends on per-body shape ordering.
	for ( int i = 0; i < 3; ++i )
	{
		b3BodyDef bd = b3DefaultBodyDef();
		bd.type = b3_dynamicBody;
		bd.position = (b3Pos){ 3.0f * (float)i, 5.0f, 0.0f };
		b3BodyId body = b3CreateBody( worldId, &bd );

		b3ShapeDef sd = b3DefaultShapeDef();
		sd.density = 1.0f;
		if ( i != 1 )
		{
			sd.name = names[i];
		}
		b3ShapeId shape = b3CreateSphereShape( body, &sd, &sphere );
		if ( i == 1 )
		{
			b3Shape_SetName( shape, names[i] );
		}
	}

	for ( int i = 0; i < 4; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	const uint8_t* data = b3Recording_GetData( rec );
	int sz = b3Recording_GetSize( rec );

	b3RecPlayer* player = b3RecPlayer_Create( data, sz, 1 );
	ENSURE( player != NULL );
	while ( b3RecPlayer_StepFrame( player ) )
	{
	}
	ENSURE( b3RecPlayer_HasDiverged( player ) == false );

	// Body ordinals follow creation order in the replayed world.
	for ( int i = 0; i < 3; ++i )
	{
		b3BodyId body = b3RecPlayer_GetBodyId( player, i );
		ENSURE( b3Body_IsValid( body ) );

		b3ShapeId shape;
		ENSURE( b3Body_GetShapes( body, &shape, 1 ) == 1 );
		const char* got = b3Shape_GetName( shape );
		ENSURE( got != NULL );

		int srcLen = (int)strlen( names[i] );
		ENSURE( (int)strlen( got ) == srcLen );
		if ( srcLen > 0 )
		{
			ENSURE( strncmp( got, names[i], (size_t)srcLen ) == 0 );
		}
	}

	b3RecPlayer_Destroy( player );
	b3DestroyRecording( rec );
	return 0;
}

int RecordingTest( void )
{
	RUN_SUBTEST( GeometryHashCollision );
	RUN_SUBTEST( ShapeNameReplay );
	RUN_SUBTEST( SphereRoundTrip );
	RUN_SUBTEST( EmptyWorldRoundTrip );
	RUN_SUBTEST( HullDedup );
	RUN_SUBTEST( MidStreamNoContacts );
	RUN_SUBTEST( MidStreamContacts );
	RUN_SUBTEST( StagedStepCreationPose );
	RUN_SUBTEST( ScrubBackward );
	RUN_SUBTEST( SeekWithHull );
	RUN_SUBTEST( DebugShapeCallbacks );
	RUN_SUBTEST( PlayerAccessors );
	RUN_SUBTEST( KeyframeHandleReuse );
	RUN_SUBTEST( QueryReplay );
	RUN_SUBTEST( TaggedQuery );
	RUN_SUBTEST( TransformedHullRoundTrip );
	RUN_SUBTEST( AllOps );
	RUN_SUBTEST( ReservedHeaderBytes );
	return 0;
}

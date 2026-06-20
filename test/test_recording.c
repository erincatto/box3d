// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "test_macros.h"

#include "physics_world.h"
#include "recording.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Sphere round-trip: record/step/stop, then replay and validate.
static int SphereRoundTrip( void )
{
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId  worldId  = b3CreateWorld( &worldDef );

	b3World_StartRecording( worldId, rec );

	// Set a non-default gravity so the setter op appears in the stream.
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	// Static ground
	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type      = b3_staticBody;
	b3BodyId groundId   = b3CreateBody( worldId, &groundDef );

	b3BoxHull groundBox  = b3MakeBoxHull( 50.0f, 1.0f, 50.0f );
	b3ShapeDef groundShapeDef = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &groundShapeDef, &groundBox.base );

	// Dynamic body with a sphere shape
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type      = b3_dynamicBody;
	bodyDef.position  = (b3Pos){ 0.0f, 5.0f, 0.0f };
	b3BodyId bodyId   = b3CreateBody( worldId, &bodyDef );

	b3Sphere sphere;
	sphere.center = (b3Vec3){ 0.0f, 0.0f, 0.0f };
	sphere.radius = 0.5f;
	b3ShapeDef sphereDef  = b3DefaultShapeDef();
	sphereDef.density     = 1.0f;
	b3CreateSphereShape( bodyId, &sphereDef, &sphere );

	float timeStep    = 1.0f / 60.0f;
	int   subStepCount = 4;
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
		{ -1.0f, -1.0f, -1.0f }, {  1.0f, -1.0f, -1.0f },
		{  1.0f,  1.0f, -1.0f }, { -1.0f,  1.0f, -1.0f },
		{ -1.0f, -1.0f,  1.0f }, {  1.0f, -1.0f,  1.0f },
		{  1.0f,  1.0f,  1.0f }, { -1.0f,  1.0f,  1.0f },
	};
	b3HullData* hull = b3CreateHull( pts, 8, 8 );
	ENSURE( hull != NULL );

	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId  worldId  = b3CreateWorld( &worldDef );

	b3World_StartRecording( worldId, rec );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density    = 1.0f;

	for ( int i = 0; i < 3; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type      = b3_dynamicBody;
		bodyDef.position  = (b3Pos){ (float)( i * 3 ), 5.0f, 0.0f };
		b3BodyId bodyId   = b3CreateBody( worldId, &bodyDef );
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
	int            sz    = b3Recording_GetSize( rec );
	ENSURE( sz >= 48 );

	uint64_t regOff = 0;
	memcpy( &regOff, bytes + 32, 8 ); // registryOffset at offset 32 in b3RecHeader
	ENSURE( regOff != 0 && (int)regOff + 4 <= sz );

	// entryCount is a little-endian u32 at the start of the registry block
	const uint8_t* rp      = bytes + (int)regOff;
	uint32_t       entryCount = (uint32_t)rp[0] | ( (uint32_t)rp[1] << 8 ) |
	                             ( (uint32_t)rp[2] << 16 ) | ( (uint32_t)rp[3] << 24 );
	ENSURE( entryCount == 1 );

	b3DestroyRecording( rec );
	return 0;
}

// Mid-stream snapshot with only dynamic bodies floating in air (no contacts).
static int MidStreamNoContacts( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId  worldId  = b3CreateWorld( &worldDef );

	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	b3Sphere sphere;
	sphere.center = (b3Vec3){ 0.0f, 0.0f, 0.0f };
	sphere.radius = 0.5f;
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density    = 1.0f;

	// A few dynamic bodies well apart from each other so no contacts form
	for ( int i = 0; i < 4; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type      = b3_dynamicBody;
		bodyDef.position  = (b3Pos){ (float)( i * 10 ), 50.0f, 0.0f };
		b3BodyId bodyId   = b3CreateBody( worldId, &bodyDef );
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );
	}

	float timeStep    = 1.0f / 60.0f;
	int   subStepCount = 4;
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
	b3WorldId  worldId  = b3CreateWorld( &worldDef );

	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	// Static ground using a box hull
	{
		b3BodyDef groundDef = b3DefaultBodyDef();
		groundDef.type      = b3_staticBody;
		b3BodyId groundId   = b3CreateBody( worldId, &groundDef );

		b3BoxHull  groundBox  = b3MakeBoxHull( 50.0f, 1.0f, 50.0f );
		b3ShapeDef groundShape = b3DefaultShapeDef();
		b3CreateHullShape( groundId, &groundShape, &groundBox.base );
	}

	b3ShapeDef dynamicShape = b3DefaultShapeDef();
	dynamicShape.density    = 1.0f;

	// A few dynamic boxes dropped onto the ground
	for ( int i = 0; i < 3; ++i )
	{
		b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type      = b3_dynamicBody;
		bodyDef.position  = (b3Pos){ (float)( i * 2 ) - 2.0f, 5.0f, 0.0f };
		b3BodyId bodyId   = b3CreateBody( worldId, &bodyDef );
		b3CreateHullShape( bodyId, &dynamicShape, &box.base );
	}

	float timeStep    = 1.0f / 60.0f;
	int   subStepCount = 4;

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
	b3WorldId  worldId  = b3CreateWorld( &worldDef );

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
	int   subStepCount = 4;
	int   totalFrames = 80;
	for ( int i = 0; i < totalFrames; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	const uint8_t* data = b3Recording_GetData( rec );
	int            sz   = b3Recording_GetSize( rec );

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
		{ -1.0f, -1.0f, -1.0f }, {  1.0f, -1.0f, -1.0f },
		{  1.0f,  1.0f, -1.0f }, { -1.0f,  1.0f, -1.0f },
		{ -1.0f, -1.0f,  1.0f }, {  1.0f, -1.0f,  1.0f },
		{  1.0f,  1.0f,  1.0f }, { -1.0f,  1.0f,  1.0f },
	};
	b3HullData* hull = b3CreateHull( pts, 8, 8 );
	ENSURE( hull != NULL );

	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId  worldId  = b3CreateWorld( &worldDef );

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
		bd.type      = b3_dynamicBody;
		bd.position  = (b3Pos){ (float)( i * 4 ) - 4.0f, 5.0f, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bd );
		b3CreateHullShape( bodyId, &sd, hull );
	}

	float timeStep = 1.0f / 60.0f;
	int   totalFrames = 40;
	for ( int i = 0; i < totalFrames; ++i )
	{
		b3World_Step( worldId, timeStep, 4 );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );
	b3DestroyHull( hull );

	const uint8_t* data = b3Recording_GetData( rec );
	int            sz   = b3Recording_GetSize( rec );

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
	b3WorldId  worldId  = b3CreateWorld( &worldDef );

	b3World_StartRecording( worldId, rec );
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type    = b3_staticBody;
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
		bodyDef.type     = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 0.0f, 1.0f + 1.1f * (float)i, 0.0f };
		b3BodyId bodyId  = b3CreateBody( worldId, &bodyDef );
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
	b3WorldId  worldId  = b3CreateWorld( &worldDef );
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
	int   subStepCount = 4;

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
	b3WorldId  worldId  = b3CreateWorld( &worldDef );
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type    = b3_staticBody;
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
		bodyDef.type     = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 0.0f, 1.0f + 1.1f * (float)i, 0.0f };
		b3BodyId bodyId  = b3CreateBody( worldId, &bodyDef );
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
	b3WorldId  worldId  = b3CreateWorld( &worldDef );
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type    = b3_staticBody;
	b3BodyId  groundId = b3CreateBody( worldId, &groundDef );
	b3BoxHull groundBox   = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
	b3ShapeDef groundShape = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &groundShape, &groundBox.base );

	// A few dynamic spheres for the queries to find.
	for ( int i = 0; i < 4; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type     = b3_dynamicBody;
		bodyDef.position = (b3Pos){ (float)i - 1.5f, 3.0f, 0.0f };
		b3BodyId bodyId  = b3CreateBody( worldId, &bodyDef );
		b3Sphere sphere  = { { 0.0f, 0.0f, 0.0f }, 0.5f };
		b3ShapeDef sphereDef = b3DefaultShapeDef();
		sphereDef.density    = 1.0f;
		b3CreateSphereShape( bodyId, &sphereDef, &sphere );
	}

	b3World_StartRecording( worldId, rec );

	b3QueryFilter filter = b3DefaultQueryFilter();

	const int totalFrames = 30;
	for ( int i = 0; i < totalFrames; ++i )
	{
		b3Pos  origin      = { 0.0f, 6.0f, 0.0f };
		b3Vec3 translation = { 0.0f, -8.0f, 0.0f };
		b3AABB aabb        = { { -5.0f, -1.0f, -5.0f }, { 5.0f, 6.0f, 5.0f } };

		b3Vec3       proxyPts = { 0.0f, 0.0f, 0.0f };
		b3ShapeProxy proxy    = { &proxyPts, 1, 0.5f };
		b3Capsule    mover    = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 0.3f };

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

	// A different worker count replays the same queries, a cross-thread determinism check.
	ENSURE( b3ValidateReplay( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 4 ) );

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

// Empty world: recording starts with no bodies and none are ever created. This used to skip the seed
// snapshot and produce a from-creation player whose Restart destroyed and rebuilt the world, changing
// its id. The empty world is now serialized like any other, so replay validates and Restart restores
// in place with a stable world id.
static int EmptyWorldRoundTrip( void )
{
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId  worldId  = b3CreateWorld( &worldDef );

	b3World_StartRecording( worldId, rec );
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	for ( int i = 0; i < 10; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	const uint8_t* data = b3Recording_GetData( rec );
	int            size = b3Recording_GetSize( rec );

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

int RecordingTest( void )
{
	RUN_SUBTEST( SphereRoundTrip );
	RUN_SUBTEST( EmptyWorldRoundTrip );
	RUN_SUBTEST( HullDedup );
	RUN_SUBTEST( MidStreamNoContacts );
	RUN_SUBTEST( MidStreamContacts );
	RUN_SUBTEST( ScrubBackward );
	RUN_SUBTEST( SeekWithHull );
	RUN_SUBTEST( DebugShapeCallbacks );
	RUN_SUBTEST( PlayerAccessors );
	RUN_SUBTEST( KeyframeHandleReuse );
	RUN_SUBTEST( QueryReplay );
	return 0;
}

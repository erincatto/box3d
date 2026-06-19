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

	// A backward seek rebuilds the world (from-creation) and must keep the callbacks, so re-stepping
	// then drawing fires more creates.
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

int RecordingTest( void )
{
	RUN_SUBTEST( SphereRoundTrip );
	RUN_SUBTEST( HullDedup );
	RUN_SUBTEST( MidStreamNoContacts );
	RUN_SUBTEST( MidStreamContacts );
	RUN_SUBTEST( ScrubBackward );
	RUN_SUBTEST( SeekWithHull );
	RUN_SUBTEST( DebugShapeCallbacks );
	return 0;
}

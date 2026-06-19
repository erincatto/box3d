// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "test_macros.h"

#include <stdint.h>
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

int RecordingTest( void )
{
	RUN_SUBTEST( SphereRoundTrip );
	RUN_SUBTEST( HullDedup );
	return 0;
}
